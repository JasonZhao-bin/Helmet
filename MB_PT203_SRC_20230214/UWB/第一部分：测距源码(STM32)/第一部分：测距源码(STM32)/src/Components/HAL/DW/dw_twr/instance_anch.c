/*! ----------------------------------------------------------------------------
 *  @file    instance_anch.c
 *  @brief   Decawave anchor application state machine and for TREK demo
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <stdbool.h> 
 
#include "uwb.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "deca_regs.h"

#include "instance.h"

// -------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------
void anch_prepare_anc2tag_response(unsigned int tof_index, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame, uint32 uTimeStamp);
void anch_enable_rx(uint32 dlyTime);


/**
 * 功能：基站 重新打开接受  this function re-enables RX ...
 *
 */   
void anch_no_timeout_rx_reenable(void)
{
	dwt_setrxtimeout(0); //reconfigure the timeout
	dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
}

/**
 * 功能：接受Rx超时  function to process RX timeout event
 *
 */   
void anch_process_RX_timeout(instance_data_t *inst)
{
    //inst->rxTimeouts ++ ;

    if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
    {
		//only enable receiver when not using double buffering
		inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
		dwt_setrxtimeout(0);
		inst->wait4ack = 0 ; //clear the flag,
    }
}


/**
 * 功能：基站 设置延迟接受打开时间  this function either enables the receiver (delayed)
 *
 */    
void anch_enable_rx(uint32 dlyTime)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	//subtract preamble duration (because when instructing delayed TX the time is the time of SFD,
	//however when doing delayed RX the time is RX on time)
	dwt_setdelayedtrxtime(dlyTime - inst->preambleDuration32h) ;
	if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
	{
		//if the delayed RX failed - time has passed - do immediate enable
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy*2); //reconfigure the timeout before enable
		//longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //restore the timeout for next RX enable
		//inst->lateRX++;
	}
}


/**
 * 功能：基站 重新开启接受或者发送Resp信息  this function either re-enables the receiver (delayed or immediate) or transmits the response frame
 *
 */  
uint8 anch_txresponse_or_rx_reenable(void)
{
	uint8 typePend = DWT_SIG_DW_IDLE;
	int sendResp = 0;
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	//if remainingRespToRx == 0 - all the expected responses have been received (or timed out or errors instead)
	if(inst->remainingRespToRx == 0) //go back to RX without timeout - ranging has finished. (wait for Final but no timeout)
	{
		dwt_setrxtimeout(inst->fwto4FinalFrame_sy*2); //reconfigure the timeout for the final
		inst->wait4final = WAIT4TAGFINAL;
	}

	if((inst->remainingRespToRx + inst->shortAdd_idx) == NUM_EXPECTED_RESPONSES)
	{
		sendResp = 1;
	}

	//configure delayed reply time (this is incremented for each received frame) it is timed from Poll rx time
	inst->delayedTRXTime32h += inst->fixedReplyDelayAnc32h;

	//sendResp为1,则发送Resp数据包
	if(sendResp == 1)
	{
		//response is expected
		inst->wait4ack = DWT_RESPONSE_EXPECTED; //re has/will be re-enabled

		dwt_setdelayedtrxtime(inst->delayedTRXTime32h) ;
		if(dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED))
		{
//		    _dbg_printf("Resp fault\n");
			//if TX has failed - we need to re-enable RX for the next response or final reception...
			dwt_setrxaftertxdelay(0);
			inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
			//inst->lateTX++;

			if(inst->remainingRespToRx == 0) //not expecting any more responses - enable RX
			{
				inst->delayedTRXTime32h = (inst->tagPollRxTime >> 8) + inst->pollTx2FinalTxDelay;
			}
			else
			{
				inst->delayedTRXTime32h += 2*(inst->fixedReplyDelayAnc32h); //to take into account W4R
			}
			anch_enable_rx(inst->delayedTRXTime32h);
			typePend = DWT_SIG_RX_PENDING ;
		}
		else
		{
			inst->delayedTRXTime32h += inst->fixedReplyDelayAnc32h; //to take into account W4R
			typePend = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
			inst->timeofTx = portGetTickCnt();
			inst->monitor = 1;
		}
	}
    //sendResp为0,则继续保持接受
	else
	{
		if(inst->remainingRespToRx == 0) //not expecting any more responses - enable RX
		{
			inst->delayedTRXTime32h = (inst->tagPollRxTime >> 8) + (inst->pollTx2FinalTxDelay >> 8);
			dwt_setdelayedtrxtime(inst->delayedTRXTime32h - inst->preambleDuration32h) ;
			//dwt_setdelayedtrxtime(inst->delayedReplyTime);
			if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
			{
				anch_no_timeout_rx_reenable();
			}
		}
		else
		{
			anch_enable_rx(inst->delayedTRXTime32h);
		}

		typePend = DWT_SIG_RX_PENDING ;
	}
	//if time to send a response
	return typePend;
}



/**
 * 功能：基站 重新开启接受或者回复发送Resp信息  this function prepares and writes the anchor to tag response frame into the TX buffer
 *
 */ 
void anch_prepare_anc2tag_response(unsigned int tof_idx, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame, uint32 uTimeStamp)
{
	uint16 frameLength = 0;
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	int tagSleepCorrection_ms = 0;

	inst->psduLength = frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
	memcpy(&inst->msg_f.destAddr[0], &frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
	inst->msg_f.sourceAddr[0] = inst->eui64[0];
	inst->msg_f.sourceAddr[1] = inst->eui64[1];
	// TOF距离(上次测距)写入到Resp数据包中 
	memcpy(&(inst->msg_f.messageData[TOFR]), &inst->tof[tof_idx], 4);
	inst->msg_f.messageData[TOFRN] = inst->rangeNumA[tof_idx]; //get the previous range number

	inst->rangeNumA[tof_idx] = 0; //clear after copy above...
	inst->rangeNum = frame[POLL_RNUM+fcode_index] ;
	inst->msg_f.seqNum = inst->frameSN++;

	//we have our range - update the own mask entry...
	if(inst->tof[tof_idx] != INVALID_TOF) //check the last ToF entry is valid and copy into the current array
	{
		inst->rxResponseMask = (0x1 << inst->shortAdd_idx);
		inst->tofArray[inst->shortAdd_idx] = inst->tof[tof_idx];
	}
	else	//reset response mask
	{
		inst->tofArray[inst->shortAdd_idx] = INVALID_TOF ;
		inst->rxResponseMask = 0;	//reset the mask of received responses when rx poll
	}
	//设置基站发送Resp包后延迟打开接受时间
	dwt_setrxaftertxdelay(inst->ancRespRxDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

	//如果为基站,负责修正标签的下次测距时间
	if(inst->gatewayAnchor)
	{
		int error = 0;
		int currentSlotTime = 0;
		int expectedSlotTime = 0;
		//find the time in the current superframe
		currentSlotTime = uTimeStamp % inst->sframePeriod_ms;

		//this is the slot time the poll should be received in (Mask 0x07 for the 8 MAX tags we support in TREK)
		expectedSlotTime = tof_idx * inst->slotDuration_ms; //

		//error = expectedSlotTime - currentSlotTime
		error = expectedSlotTime - currentSlotTime;

		if(error < (-(inst->sframePeriod_ms>>1))) //if error is more negative than 0.5 period, add whole period to give up to 1.5 period sleep
		{
			tagSleepCorrection_ms = (inst->sframePeriod_ms + error);
		}
		else //the minimum Sleep time will be 0.5 period
		{
			tagSleepCorrection_ms = error;
		}
		inst->msg_f.messageData[RES_TAG_SLP0] = tagSleepCorrection_ms & 0xFF ;
		inst->msg_f.messageData[RES_TAG_SLP1] = (tagSleepCorrection_ms >> 8) & 0xFF;
	}
	else
	{
		tagSleepCorrection_ms = 0;
		inst->msg_f.messageData[RES_TAG_SLP0] = 0 ;
		inst->msg_f.messageData[RES_TAG_SLP1] = 0 ;
	}
	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)

    //基站 赋值透传命令至Resp数据包
    if(inst->trasport_cmd.send_enable == true)
    {
        inst->msg_f.messageData[RESP_TRSPORT_ADDR0] = inst->trasport_cmd.send_addr & 0xff;
        inst->msg_f.messageData[RESP_TRSPORT_ADDR1] = (inst->trasport_cmd.send_addr >> 8) & 0xff;

        inst->msg_f.messageData[RESP_TRSPORT_LEN] = inst->trasport_cmd.send_cmdlen;
        memcpy(&inst->msg_f.messageData[RESP_TRSPORT], inst->trasport_cmd.send_cmd, inst->trasport_cmd.send_cmdlen);
        inst->trasport_cmd.send_enable = false;
    }
    else 
    {
        inst->msg_f.messageData[RESP_TRSPORT_ADDR0] = 0xfe;
        inst->msg_f.messageData[RESP_TRSPORT_ADDR1] = 0xff;
    }

	//设置发送Resp数据长度
	dwt_writetxfctrl(frameLength, 0, 1);
    //将Resp发送数据写入dw1000准备发送
	dwt_writetxdata(frameLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data
}


/**
 * 功能：基站 错误处理函数，立即打开接受操作 this function handles frame error event, it will either signal timeout or re-enable the receiver
 *
 */  
void anch_handle_error_unknownframe_timeout(event_data_t dw_event)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	/*Note: anchor can be involved in anchor-to-anchor ranging ANCHOR_RNG mode or just ANCHOR mode (ranging with tags)
	 * In the former case if the anchor is initiating ranging exchange and after timeout/error
	 * it needs to send a final message or re-enable receiver to receiver more responses.
	 * In the latter case if the anchor get an error or timeout it may want to send response to the tag or re-enable the receiver.
	 */
	dw_event.type = 0;
	dw_event.rxLength = 0;

	//check if the anchor has received any of the responses from other anchors
	//it's timed out (re-enable rx or tx final)
	if(inst->remainingRespToRx >= 0) //if it was expecting responses - then decrement count
	{
		inst->remainingRespToRx--;
	}

	switch(inst->mode)
	{
		case ANCHOR:
		{
			//check firstly if involved in the ranging exchange with a tag
			if(inst->twrMode == RESPONDER_T)
			{
				if(inst->wait4final == WAIT4TAGFINAL)
				{
					inst->twrMode = LISTENER ;
					inst->wait4final = 0;

					dw_event.typePend = DWT_SIG_DW_IDLE;
					instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
				}
				else
				{
					dw_event.typePend = anch_txresponse_or_rx_reenable();

					//if rx error or timeout and then is sending response or in idle
					//report timeout to application
					if(dw_event.typePend != DWT_SIG_RX_PENDING)
					{
						instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
					}
				}
				break;
			}
		}
		default: //e.g. listener - got error, and not participating in ranging - ignore event
		{
			dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
		}
		break;
	}

	/*if(inst->remainingRespToRx == 0)
	{
		inst->remainingRespToRx = -1;
	}*/
}

/**
 * 功能：基站 接受超时回调函数 this is the receive timeout event callback handler
 *
 */ 
void rx_to_cb_anch(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
   	anch_handle_error_unknownframe_timeout(dw_event);
}

/**
 * 功能：基站 接受错误回调函数 this is the receive error event callback handler
 *
 */  
void rx_err_cb_anch(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
    anch_handle_error_unknownframe_timeout(dw_event);
}


/**
 * 功能：基站 接受成功回调函数
 *
 */  
void rx_ok_cb_anch(const dwt_cb_data_t *rxd)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	uint8 srcAddr_index = 0;
	event_data_t dw_event;

	//数据接受时间(stm系统时间ms) 
    dw_event.uTimeStamp = portGetTickCnt();

    //if we got a frame with a good CRC - RX OK
    {
 		dw_event.rxLength = rxd->datalength;

		//验证接受数据帧frame control
		if((rxd->fctrl[0] == 0x41) //no auto-ACK
				&&
				((rxd->fctrl[1] & 0xCC) == 0x88)) //short address
		{

			fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
			srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
			rxd_event = DWT_SIG_RX_OKAY;
		}
		else
		{
			rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
		}

        //数据受时间(dw1000 RX时间戳)
        dwt_readrxtimestamp(rxTimeStamp) ;
        //读取dw1000接受数据
        dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
        instance_seteventtime(&dw_event, rxTimeStamp);

        dw_event.type = 0; //type will be added as part of adding to event queue
		dw_event.typePend = DWT_SIG_DW_IDLE;


        //检验数据帧格式
        if(rxd_event == DWT_SIG_RX_OKAY)
        {
            if((dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL && rxd->datalength == (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
             ||(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_ANCH_RESP && rxd->datalength == (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
             ||(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_FINAL && rxd->datalength == (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
                )
             {
                //系统中正确的数据             
             }
             else
             {
                rxd_event = SIG_RX_UNKNOWN;
             }
        }


        //frame control验证成功
		if(rxd_event == DWT_SIG_RX_OKAY) //Process good/known frame types
		{
			uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];
			//基站没有开启帧过滤，所以进行软过滤
			if((dw_event.msgu.rxmsg_ss.panID[0] != (inst->panID & 0xff)) ||
					(dw_event.msgu.rxmsg_ss.panID[1] != (inst->panID >> 8)))
			{
//			    _dbg_printf("recv error panid\n");
				anch_handle_error_unknownframe_timeout(dw_event);
				return;
			}
                    
            //检查是否为TWR测距帧以及透传DTU帧
			switch(dw_event.msgu.frame[fcode_index])
			{
				//标签发送Poll数据 			
				case RTLS_DEMO_MSG_TAG_POLL:
				{
					//if ANCHOR_RNG ignore tag polls
					//source address is used as index so has to be < MAX_TAG_LIST_SIZE or frame is ignored
					if((inst->mode == ANCHOR) && (sourceAddress < MAX_TAG_LIST_SIZE))
					{
						inst->twrMode = RESPONDER_T ;
						inst->wait4final = 0;

						//预备Resp发送数据包
						anch_prepare_anc2tag_response(sourceAddress, srcAddr_index, fcode_index, &dw_event.msgu.frame[0], dw_event.uTimeStamp);

                        //设置接受超时时间
						dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //reconfigure the timeout for response

						inst->delayedTRXTime32h = dw_event.timeStamp32h ;
						inst->remainingRespToRx = NUM_EXPECTED_RESPONSES; //set number of expected responses to 3 (from other anchors)

                        //判断1.发送Resp 2.继续接受Resp
						dw_event.typePend = anch_txresponse_or_rx_reenable();

						inst->tof[sourceAddress] = INVALID_TOF; //clear ToF ..

					}
					else //not participating in this exchange - ignore this frame
					{
						anch_handle_error_unknownframe_timeout(dw_event);
						return;
					}
				}
				break;

				//基站发送Resp数据 
				case RTLS_DEMO_MSG_ANCH_RESP:
				{
					if(inst->twrMode == RESPONDER_T)
					{
						// are participating in this TWR exchange - need to check if time to send response or go back to RX
						// got a response... (check if we got a Poll with the same range number as in this response)
						inst->remainingRespToRx--; //reduce number of expected responses (as we have just received one)

						//send a response or re-enable rx
						dw_event.typePend = anch_txresponse_or_rx_reenable();
					}
					else //not participating in this exchange - ignore this frame
					{
						anch_no_timeout_rx_reenable();
						return;
					}
				}
				break;
                
                //标签发送Final数据
				case RTLS_DEMO_MSG_TAG_FINAL:
                {            
					if((inst->twrMode == RESPONDER_T)&&(inst->mode == ANCHOR))
					{
					    memset(&dw_event.rx_diag, 0, sizeof(dwt_rxdiag_t));
                        dwt_readdiagnostics(&dw_event.rx_diag); //基站通过final数据包读取UWB信号参数
                        
						anch_no_timeout_rx_reenable();  // turn RX on, without delay
						dw_event.typePend = DWT_SIG_RX_PENDING ;
						inst->twrMode = LISTENER ;
						inst->wait4final = 0;
						break;
					}
                }

                //DTU透传数据
                case RTLS_DEMO_MSG_DTU:
                {
                }
                break;
                    
				default:  //ignore this frame
				{
					anch_no_timeout_rx_reenable();
					return;
				}
				break;

			}
            //写入事件
            instance_putevent(dw_event, rxd_event);

			//inst->rxMsgCount++;
		}
		else //if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx (got unknown frame type)
		{
			anch_handle_error_unknownframe_timeout(dw_event);
		}
	}
}
// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine for anchor application
//
// -------------------------------------------------------------------------------------------------------------------
//

/**
 * 功能：基站 检查设备状态
 *
 */
int check_state_change(uint8 event)
{
	int state = TA_RXE_WAIT ;
	//the response has been sent - await TX done event
	if(event == DWT_SIG_TX_PENDING)
	{
		state = TA_TX_WAIT_CONF;                // wait confirmation
	}
	//already re-enabled the receiver
	else if (event == DWT_SIG_RX_PENDING)
	{
		//stay in RX wait for next frame...
		//RX is already enabled...
		state = TA_RX_WAIT_DATA ;              // wait for next frame
	}
	else //the DW1000 is idle (re-enable from the application level)
	{
		//stay in RX change state and re-enable RX for next frame...
		state = TA_RXE_WAIT ;
	}
	return state ;
}


/**
 * 功能：计算TOF数据
 *
 */  
int32 calc_tof(uint8 *messageData, uint64 anchorRespTxTime, uint64 tagFinalRxTime, uint64 tagPollRxTime, uint8 shortAdd_idx)
{
	int64 Rb, Da, Ra, Db ;
	uint64 tagFinalTxTime  = 0;
	uint64 tagPollTxTime  = 0;
	uint64 anchorRespRxTime  = 0;

	double RaRbxDaDb = 0;
	double RbyDb = 0;
	double RayDa = 0;
	int32 tof;

	uint8 index = RRXT0 + 5*(shortAdd_idx);

	// times measured at Tag extracted from the message buffer
	// extract 40bit times
	memcpy(&tagPollTxTime, &(messageData[PTXT]), 5);
	memcpy(&anchorRespRxTime, &(messageData[index]), 5);
	memcpy(&tagFinalTxTime, &(messageData[FTXT]), 5);

	// poll response round trip delay time is calculated as
	// (anchorRespRxTime - tagPollTxTime) - (anchorRespTxTime - tagPollRxTime)
	Ra = (int64)((anchorRespRxTime - tagPollTxTime) & MASK_40BIT);
	Db = (int64)((anchorRespTxTime - tagPollRxTime) & MASK_40BIT);

	// response final round trip delay time is calculated as
	// (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
	Rb = (int64)((tagFinalRxTime - anchorRespTxTime) & MASK_40BIT);
	Da = (int64)((tagFinalTxTime - anchorRespRxTime) & MASK_40BIT);

	RaRbxDaDb = (((double)Ra))*(((double)Rb))
	- (((double)Da))*(((double)Db));

	RbyDb = ((double)Rb + (double)Db);

	RayDa = ((double)Ra + (double)Da);

	tof = (int32) ( RaRbxDaDb/(RbyDb + RayDa) );

	return tof;
}


/**
 * 功能：基站 运行主循环函数
 *
 */ 
int anch_app_run(instance_data_t *inst)
{
	int instDone = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR

    switch (inst->testAppState)
    {
         case TA_INIT : //状态:初始化
            switch (inst->mode)
            {
                case ANCHOR:
                {
                    //设置EUI64
                    memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
                    dwt_seteui(inst->eui64);
                    //设置panid用于帧过滤
                    dwt_setpanid(inst->panID);

                    //设置16bit短地址用于帧过滤
#ifdef HW_8Anc                    
                    inst->shortAdd_idx = (inst->instanceAddress16 & (MAX_ANCHOR_LIST_SIZE-1)) ;
#else
                    inst->shortAdd_idx = (inst->instanceAddress16 & 0x3) ;
#endif
                    dwt_setaddress16(inst->instanceAddress16);

                	//主基站判断
                	if(inst->instanceAddress16 == GATEWAY_ANCHOR_ADDR)
                	{
                		inst->gatewayAnchor = TRUE;
                	}

					//取消帧过滤
                	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //allow data, ack frames;

                	//立刻打开接受
					dwt_setrxaftertxdelay(0);
                    //切换状态为TA_RXE_WAIT
                    inst->testAppState = TA_RXE_WAIT ;

                    //设置接受超时为0
                    dwt_setrxtimeout(0);
                    //填充空中数据包
                    instance_config_frameheader_16bit(inst);
                }
                break;
                default:
                break;
            }
            break; // end case TA_INIT
            
        case TA_TX_WAIT_CONF ://状态:等待发送完成
            {
                //读取事件
    			event_data_t* dw_event = instance_getevent(11);

				//等待事件[发送成功类型]
                if(dw_event->type != DWT_SIG_TX_DONE)
                {
    			    instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    break;
                }

                instDone = INST_NOT_DONE_YET;

                {
    				inst->txu.txTimeStamp = dw_event->timeStamp;
    	            if(inst->previousState == TA_TXRESPONSE_SENT_TORX)
    	            {
    	            	inst->previousState = TA_TXRESPONSE_WAIT_SEND ;
    	            }
                    inst->testAppState = TA_RXE_WAIT ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll

                    message = 0;
                    //fall into the next case (turn on the RX)
                }
            }

            //break ; // end case TA_TX_WAIT_CONF


        case TA_RXE_WAIT : //状态:等待读取 
            {
                if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
                {
                	if(dwt_read16bitoffsetreg(0x19,1) != 0x0505)
                	{
                		//turn RX on
                	  	dwt_rxenable(DWT_START_RX_IMMEDIATE) ;  // turn RX on, without delay
                	}
                }
                else
                {
                    inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
                }

                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
                inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

                // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
                if(message == 0) break;
            }

        case TA_RX_WAIT_DATA :     //状态:读取数据成功                                                                   // Wait RX data
            switch (message)
            {
                //读取有效数据帧
                case DWT_SIG_RX_OKAY :
                    {
    					event_data_t* dw_event = instance_getevent(15); //get and clear this event
    					uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
    					uint8  dstAddr[8] = {0,0,0,0,0,0,0,0};
                        int fcode = 0;
    					uint8 tof_idx  = 0;
    					int tag_index = 0;
    					uint8 *messageData;

    					memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
    					memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ss.destAddr[0]), ADDR_BYTE_SIZE_S);
    					fcode = dw_event->msgu.rxmsg_ss.messageData[FCODE];
    					messageData = &dw_event->msgu.rxmsg_ss.messageData[0];

#ifdef HW_8Anc
                        tof_idx = srcAddr[0] & (MAX_ANCHOR_LIST_SIZE - 1) ;
#else
    					tof_idx = srcAddr[0] & 0x3 ;
#endif

    					//解析测距数据帧
    					switch(fcode)
    					{
        					//接受   标签发送的Poll包
    						case RTLS_DEMO_MSG_TAG_POLL:
    						{ 		
                                /*基站接受 标签Poll数据包传输的透传指令*/
                                uint16 trasport_addr = messageData[POLL_TRSPORT_ADDR0] | (messageData[POLL_TRSPORT_ADDR0] << 8);
                                if(trasport_addr == inst->instanceAddress16 || trasport_addr == 0xffff)
                                {
                                    inst->trasport_cmd.recv_enable = true;
                                    inst->trasport_cmd.recv_addr = srcAddr[0] + (((uint16) srcAddr[1]) << 8);
                                    inst->trasport_cmd.recv_cmdlen = messageData[POLL_TRSPORT_LEN];
                                    memcpy(inst->trasport_cmd.recv_cmd, &messageData[POLL_TRSPORT], inst->trasport_cmd.recv_cmdlen);
                                }


                            
    							//source address is used as index so has to be < MAX_TAG_LIST_SIZE or frame is ignored
    							//this check is done in the callback so the event here will have good address range
    							tag_index = srcAddr[0] + (((uint16) srcAddr[1]) << 8);
    							inst->rangeNumA[tag_index] = messageData[POLL_RNUM]; //when anchor receives a poll, we need to remember the new range number

    							inst->tagPollRxTime = dw_event->timeStamp ; //save Poll's Rx time

    							//check if need to change state
    							inst->testAppState = check_state_change(dw_event->typePend) ;
    							if(inst->testAppState == TA_TX_WAIT_CONF)
    							{
    								inst->previousState = TA_TXRESPONSE_SENT_POLLRX ;    //wait for TX confirmation of sent response
    							}
    						}
    						break; //RTLS_DEMO_MSG_TAG_POLL

                            //接受   基站发送的Resp包
    						case RTLS_DEMO_MSG_ANCH_RESP:
    						{
    							uint8 currentRangeNum = (messageData[TOFRN] + 1); //current = previous + 1

    							//the response has been sent - await TX done event
    							if(dw_event->typePend == DWT_SIG_TX_PENDING) //anchor received response from anchor ID - 1 so is sending it's response now back to tag
    							{
    								inst->testAppState = TA_TX_WAIT_CONF;                // wait confirmation
    								inst->previousState = TA_TXRESPONSE_SENT_RESPRX ;    //wait for TX confirmation of sent response
    							}
    							//already re-enabled the receiver
    							else if(dw_event->typePend == DWT_SIG_RX_PENDING)
    							{
    								// stay in TA_RX_WAIT_DATA - receiver is already enabled.
    							}
    							//DW1000 idle re-enable RX
    							else
    							{
    								inst->testAppState = TA_RXE_WAIT ; // wait for next frame
    							}

    							if(currentRangeNum == inst->rangeNum) //these are the previous ranges...
    							{
    								//copy the ToF and put into array (array holds last 4 ToFs)
    								memcpy(&inst->tofArray[tof_idx], &(messageData[TOFR]), 4);

    								//check if the ToF is valid, this makes sure we only report valid ToFs
    								//e.g. consider the case of reception of response from anchor a1 (we are anchor a2)
    								//if a1 got a Poll with previous Range number but got no Final, then the response will have
    								//the correct range number but the range will be INVALID_TOF
    								if(inst->tofArray[tof_idx] != INVALID_TOF)
    								{
    									inst->rxResponseMask |= (0x1 << (tof_idx));
    								}
    							}
    							else //mark as invalid (clear the array)
    							{
    								if(inst->tofArray[tof_idx] != INVALID_TOF)
    								{
    									inst->tofArray[tof_idx] = INVALID_TOF;
    								}
    							}

    						}
    						break; //RTLS_DEMO_MSG_ANCH_RESP

                            //接受   标签发送Final包
    						case RTLS_DEMO_MSG_TAG_FINAL:   
    						{						
//                               port_tx_msg("recv final\n", strlen("recv final\n"));
    							uint64 tof = INVALID_TOF;
    							uint8 validResp = messageData[VRESP];
    							//source address is used as index so has to be < MAX_TAG_LIST_SIZE or frame is ignored
    							//this check is done in the callback so the event here will have good address range
    							tag_index = srcAddr[0] + (((uint16) srcAddr[1]) << 8);

    							if(inst->rangeNumA[tag_index] != messageData[POLL_RNUM]) //Final's range number needs to match Poll's or else discard this message
    							{
    								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
    								break;
    							}

    							//if we got the final, maybe the tag did not get our response, so
    							//we can use other anchors responses/ToF if there are any.. and output..
    							//but we cannot calculate new range
    							if(((validResp & (0x1<<(inst->shortAdd_idx))) != 0))
    							{
    								tof = calc_tof(messageData, inst->txu.anchorRespTxTime, dw_event->timeStamp, inst->tagPollRxTime, inst->shortAdd_idx);
    							}

    							//tag to anchor ranging
    							inst->newRangeAncAddress = inst->instanceAddress16;
    							inst->delayedTRXTime32h = 0 ;
    							inst->newRangeTagAddress = tag_index ;
    							//time-of-flight
    							inst->tof[tag_index] = tof;
    							//calculate all tag - anchor ranges... and report
    							inst->newRange = instance_calc_ranges(&inst->tofArray[0], MAX_ANCHOR_LIST_SIZE, TOF_REPORT_T2A, &inst->rxResponseMask);
    							inst->rxResponseMaskReport = inst->rxResponseMask; //copy the valid mask to report
    							inst->rxResponseMask = 0;
    							//we have our range - update the own mask entry...
    							if(tof != INVALID_TOF) //check the last ToF entry is valid and copy into the current array
    							{
    								inst->rxResponseMask = (0x1 << inst->shortAdd_idx);
    								inst->tofArray[inst->shortAdd_idx] = tof;
    							}
    							inst->newRangeTime = dw_event->uTimeStamp ;
                                inst->rx_diag = dw_event->rx_diag;
                                
    							instance_set_antennadelays(); //this will update the antenna delay if it has changed
    							instance_set_txpower(); // configure TX power if it has changed

    							if(dw_event->typePend != DWT_SIG_RX_PENDING)
    							{
    								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
    							}
    							//else stay in TA_RX_WAIT_DATA - receiver is already enabled.

    						}
    						break; //RTLS_DEMO_MSG_TAG_FINAL

                            //接受   设备发送的DTU包
                            case RTLS_DEMO_MSG_DTU: 
                            {

                            }
                            break; //RTLS_DEMO_MSG_DTU
                            
    						default:
    						{
    							//only enable receiver when not using double buffering
    							inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
    							dwt_setrxaftertxdelay(0);

    						}
    						break;
    					} //end switch (fcode)
                    }
    				break ; //end of DWT_SIG_RX_OKAY

                case DWT_SIG_RX_TIMEOUT :
                	{
                		event_data_t* dw_event = instance_getevent(17); //get and clear this event

                		//Anchor can time out and then need to send response - so will be in TX pending
                		if(dw_event->typePend == DWT_SIG_TX_PENDING)
                		{
                			inst->testAppState = TA_TX_WAIT_CONF;              // wait confirmation
                			inst->previousState = TA_TXRESPONSE_SENT_TORX ;    // wait for TX confirmation of sent response
                		}
                		else if(dw_event->typePend == DWT_SIG_DW_IDLE) //if timed out and back in receive then don't process as timeout
    					{
                			anch_process_RX_timeout(inst);
                			instDone = INST_NOT_DONE_YET;
    					}
                		//else if RX_PENDING then wait for next RX event...
    					message = 0; //clear the message as we have processed the event
                	}
                    break ;

				default :
                    {
                        if(message)
                        {
                        	instance_getevent(20); //get and clear this event
                        }

                    	if(instDone == INST_NOT_DONE_YET) instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    }
                    break;

            }
            break ; // end case TA_RX_WAIT_DATA
        default:
            break;
    } // end switch on testAppState

    return instDone;
} // end testapprun_anch()

// -------------------------------------------------------------------------------------------------------------------
/**
 * 功能：基站 运行主函数
 *
 */
int anch_run(void)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
    int done = INST_NOT_DONE_YET;

	while(done == INST_NOT_DONE_YET)
	{
		done = anch_app_run(inst) ; // run the communications application
	}

    return 0 ;
}
/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
