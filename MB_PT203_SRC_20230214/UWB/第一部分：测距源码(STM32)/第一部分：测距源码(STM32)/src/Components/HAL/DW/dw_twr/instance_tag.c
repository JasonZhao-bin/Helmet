/*! ----------------------------------------------------------------------------
 *  @file    instance_tag.c
 *  @brief   Decawave tag application state machine for TREK demo
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
#include "OSAL_Comdef.h"
// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

/**
 * ���ܣ���ǩ �����ӳٽ��ܴ�ʱ�� this function either enables the receiver (delayed)
 *
 */  
void tag_enable_rx(uint32 dlyTime)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	//subtract preamble duration (because when instructing delayed TX the time is the time of SFD,
	//however when doing delayed RX the time is RX on time)
	dwt_setdelayedtrxtime(dlyTime - inst->preambleDuration32h) ;
	if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
	{
		//if the delayed RX failed - time has passed - do immediate enable
		//led_on(LED_PC9);
		dwt_setpreambledetecttimeout(0); //clear preamble timeout as RX is turned on early/late
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy*2); //reconfigure the timeout before enable
		//longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_setpreambledetecttimeout(PTO_PACS); //configure preamble timeout
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //restore the timeout for next RX enable
		//inst->lateRX++;
		//led_off(LED_PC9);
	}

}

/**
 * ���ܣ���ǩ ���ܳ�ʱ function to process RX timeout event
 *
 */ 
void tag_process_rx_timeout(instance_data_t *inst)
{
#if (TAG_HASTO_RANGETO_A0 == 0)
	if(inst->rxResponseMask == 0) //if any response have been received send a Final else go to SLEEP
	{
  		inst->instToSleep = TRUE ; //set sleep to TRUE so that tag will go to DEEP SLEEP before next ranging attempt
		inst->testAppState = TA_TXE_WAIT ;
		inst->nextState = TA_TXPOLL_WAIT_SEND ;
	}
#else

	//if tag times out - no response (check if we are to send a final)
	//send the final only if it has received response from anchor 0
	if((inst->previousState == TA_TXPOLL_WAIT_SEND)
			&& ((inst->rxResponseMask & 0x1) == 0)
			)
	{
		inst->instToSleep = TRUE ; //set sleep to TRUE so that tag will go to DEEP SLEEP before next ranging attempt
		inst->testAppState = TA_TXE_WAIT ;
		inst->nextState = TA_TXPOLL_WAIT_SEND ;
	}
#endif
	else if (inst->previousState == TA_TXFINAL_WAIT_SEND) //got here from main (error sending final - handle as timeout)
	{
		dwt_forcetrxoff();	//this will clear all events
		inst->instToSleep = TRUE ;
		// initiate the re-transmission of the poll that was not responded to
		inst->testAppState = TA_TXE_WAIT ;
		inst->nextState = TA_TXPOLL_WAIT_SEND ;
	}
	else //send the final
	{
		// initiate the transmission of the final
		inst->testAppState = TA_TXE_WAIT ;
		inst->nextState = TA_TXFINAL_WAIT_SEND ;
	}
}


/**
 * ���ܣ���ǩ ��ʹ�����ǩ���վ���ʱ this function is only used for tag when ranging to other anchors
 *
 */
uint8 tag_rx_reenable(uint16 sourceAddress, uint8 error)
{
	uint8 type_pend = DWT_SIG_DW_IDLE;

#ifdef HW_8Anc    
	uint8 anc = sourceAddress & (MAX_ANCHOR_LIST_SIZE - 1);
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	switch(anc)
	{
		//if we got Response from anchor 3 - this is the last expected response - send the final
		case 7:
			type_pend = DWT_SIG_DW_IDLE;
			break;

		//if we got response from anchor 0, 1, or 2 - go back to wait for next anchor's response
		//if we got response from 0, then still expecting 3, so remainingRespToRx set to 3
		case 0:
		case 1:
		case 2:
        case 3:
		case 4:
		case 5:
        case 6:            
		default:
			if(inst->remainingRespToRx > 0) //can get here as result of error frame so need to check
			{
				//can't use anc address as this is an error frame, so just re-enable TO based on remainingRespToRx count
				if(error == 0)
				{
					switch (anc)
					{
						case 0:
							inst->remainingRespToRx = 7; //expecting 3 more responses
							break;
						case 1:
							inst->remainingRespToRx = 6; //expecting 2 more responses
							break;
						case 2:
							inst->remainingRespToRx = 5; //expecting 1 more response
							break;
						case 3:
							inst->remainingRespToRx = 4; //expecting 3 more responses
							break;
						case 4:
							inst->remainingRespToRx = 3; //expecting 2 more responses
							break;
						case 5:
							inst->remainingRespToRx = 2; //expecting 1 more response
							break;
						case 6:
							inst->remainingRespToRx = 1; //expecting 3 more responses
							break;
					}
				}
				//Poll sent at tagPollTxTime_32bit
				//1st response is delayTime + fixedReplyDelayAnc32h - preambleDuration_32MSBs
				//2nd is delayTime + fixedReplyDelayAnc32h - preambleDuration_32MSBs + fixedReplyDelayAnc32h
				tag_enable_rx(inst->tagPollTxTime32h +
						(MAX_ANCHOR_LIST_SIZE-inst->remainingRespToRx+1)*(inst->fixedReplyDelayAnc32h));

				type_pend = DWT_SIG_RX_PENDING ;
			}
			else //finished waiting for responses - no responses left to receive... send a final
			{
				type_pend = DWT_SIG_DW_IDLE; //report timeout - send the final if due to be sent
			}
			break;

	}
#else
    uint8 anc = sourceAddress & 0x3;
    instance_data_t* inst = instance_get_local_structure_ptr(0);

    switch(anc)
    {
        //if we got Response from anchor 3 - this is the last expected response - send the final
        case 3:
            type_pend = DWT_SIG_DW_IDLE;
            break;

        //if we got response from anchor 0, 1, or 2 - go back to wait for next anchor's response
        //if we got response from 0, then still expecting 3, so remainingRespToRx set to 3
        case 0:
        case 1:
        case 2:
        default:
            if(inst->remainingRespToRx > 0) //can get here as result of error frame so need to check
            {
                //can't use anc address as this is an error frame, so just re-enable TO based on remainingRespToRx count
                if(error == 0)
                {
                    switch (anc)
                    {
                        case 0:
                            inst->remainingRespToRx = 3; //expecting 3 more responses
                            break;
                        case 1:
                            inst->remainingRespToRx = 2; //expecting 2 more responses
                            break;
                        case 2:
                            inst->remainingRespToRx = 1; //expecting 1 more response
                            break;
                    }
                }
                //Poll sent at tagPollTxTime_32bit
                //1st response is delayTime + fixedReplyDelayAnc32h - preambleDuration_32MSBs
                //2nd is delayTime + fixedReplyDelayAnc32h - preambleDuration_32MSBs + fixedReplyDelayAnc32h
                tag_enable_rx(inst->tagPollTxTime32h +
                        (MAX_ANCHOR_LIST_SIZE-inst->remainingRespToRx+1)*(inst->fixedReplyDelayAnc32h));

                type_pend = DWT_SIG_RX_PENDING ;
            }
            else //finished waiting for responses - no responses left to receive... send a final
            {
                type_pend = DWT_SIG_DW_IDLE; //report timeout - send the final if due to be sent
            }
            break;

    }


#endif


	return type_pend;
}

/**
 * ���ܣ���ǩ ���������������򿪽��ܲ��� this function handles frame error event, it will either signal timeout or re-enable the receiver
 *
 */
void tag_handle_error_unknownframe(event_data_t dw_event)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	if(inst->twrMode != GREETER)
	{
		//re-enable the receiver (after error frames as we are not using auto re-enable
		//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
		inst->remainingRespToRx--; //got something (need to reduce timeout (for remaining responses))

		dw_event.typePend = tag_rx_reenable(0, 1); //check if receiver will be re-enabled or it's time to send the final
	}
	else
	{
		dw_event.typePend = DWT_SIG_DW_IDLE; //in GREETER mode only waiting for 1 frame
	}

	dw_event.type = 0;
	dw_event.rxLength = 0;

	instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
}

/**
 * ���ܣ���ǩ ���ܳ�ʱ�ص����� this is the receive timeout event callback handler
 *
 */ 
void rx_to_cb_tag(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
    tag_handle_error_unknownframe(dw_event);
}

/**
 * ���ܣ���ǩ ���ܴ���ص����� this is the receive error event callback handler
 *
 */ 
void rx_err_cb_tag(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
    tag_handle_error_unknownframe(dw_event);
}

/**
 * ���ܣ���ǩ ���ܳɹ��ص�����
 *
 */ 
void rx_ok_cb_tag(const dwt_cb_data_t *rxd)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	uint8 srcAddr_index = 0;
	event_data_t dw_event;

	//������ʱ���(stmϵͳʱ��ms) 
    dw_event.uTimeStamp = portGetTickCnt();

    //if we got a frame with a good CRC - RX OK
    {
 		dw_event.rxLength = rxd->datalength;

		//��֤��������֡frame control
		if(rxd->fctrl[0] == 0x41)
		{
			if((rxd->fctrl[1] & 0xCC) == 0x88) //short address
			{
				fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
				srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
				rxd_event = DWT_SIG_RX_OKAY;
			}
			else
			{
				rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
			}
		}
		else
		{
			rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
		}

        //������ʱ��(dw1000 RXʱ���)
        dwt_readrxtimestamp(rxTimeStamp) ;
        //��ȡdw1000��������
        dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
        instance_seteventtime(&dw_event, rxTimeStamp);

        dw_event.type = 0; //type will be added as part of adding to event queue
		dw_event.typePend = DWT_SIG_DW_IDLE;


        //��������֡��ʽ
        if(rxd_event == DWT_SIG_RX_OKAY)
        {
            if((dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL && rxd->datalength == (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
             ||(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_ANCH_RESP && rxd->datalength == (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
             ||(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_FINAL && rxd->datalength == (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
                )
             {
                //ϵͳ����ȷ������             
             }
             else
             {
                rxd_event = SIG_RX_UNKNOWN;
             }
        }


        //frame control��֤�ɹ�
		if(rxd_event == DWT_SIG_RX_OKAY) 
		{
			uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];

			switch(dw_event.msgu.frame[fcode_index])
			{
				//��վ����Resp���� 
				case RTLS_DEMO_MSG_ANCH_RESP:
				{
					if(inst->twrMode == INITIATOR)
					{
						//if tag is involved in the ranging exchange expecting responses
						uint8 index ;
						inst->remainingRespToRx--; //got 1 more response or other RX frame - need to reduce timeout (for next response)
						dw_event.typePend = tag_rx_reenable(sourceAddress, 0); //remainingRespToRx decremented above...

#ifdef HW_8Anc
                        index = RRXT0 + 5*(sourceAddress & (MAX_ANCHOR_LIST_SIZE - 1));
                        inst->rxResponseMask |= (0x1 << (sourceAddress & (MAX_ANCHOR_LIST_SIZE - 1))); //add anchor ID to the mask

#else
						index = RRXT0 + 5*(sourceAddress & 0x3);
                        inst->rxResponseMask |= (0x1 << (sourceAddress & 0x3)); //add anchor ID to the mask
#endif
						// д��ʱ���(���ܻ�վResp)��Final���ݰ���
						memcpy(&(inst->msg_f.messageData[index]), rxTimeStamp, 5);
						break;
					}
				}
                //DTU͸������
                case RTLS_DEMO_MSG_DTU:
                {
                }
                break;
                
				//��ǩ����Poll����/Final���� 
				case RTLS_DEMO_MSG_TAG_POLL:
				case RTLS_DEMO_MSG_TAG_FINAL:
				default:
				//tag should ignore any other frames - only receive responses
				{
					tag_handle_error_unknownframe(dw_event);
					//inst->rxMsgCount++;
					return;
				}
			}
            //д���¼�
            instance_putevent(dw_event, rxd_event);

			//inst->rxMsgCount++;
		}
		else //if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx (got unknown frame type)
		{
			tag_handle_error_unknownframe(dw_event);
		}
	}
}
// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine for tag application
//
// -------------------------------------------------------------------------------------------------------------------
/**
 * ���ܣ���ǩ ������ѭ������
 *
 */ 
int tag_app_run(instance_data_t *inst)
{
	int instDone = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR

    switch (inst->testAppState)
    {
        case TA_INIT :  //״̬:��ʼ��
            switch (inst->mode)
            {
                case TAG:
                {
                	uint16 sleep_mode = 0;

                    //����֡����
                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); // allow data, ack frames;
                    //����EUI64
                    inst->eui64[0] += inst->instanceAddress16; //so switch 5,6,7 can be used to emulate more tags
                    dwt_seteui(inst->eui64);
                    //����panid����֡����
                    dwt_setpanid(inst->panID);  

                    memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
                    //set source address
                    inst->newRangeTagAddress = inst->instanceAddress16 ;
                    //����16bit�̵�ַ����֡����
                    dwt_setaddress16(inst->instanceAddress16);

                    //Start off by Sleeping 1st -> set instToSleep to TRUE
                    inst->nextState = TA_TXPOLL_WAIT_SEND;
                    inst->testAppState = TA_TXE_WAIT;
                    inst->instToSleep = TRUE ;
                    inst->tagSleepTime_ms = inst->tagPeriod_ms ;

                    inst->rangeNum = 0;
                    inst->tagSleepCorrection_ms = 0;

                    sleep_mode = (DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

					if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
						sleep_mode |= DWT_LOADOPSET;

#if (DEEP_SLEEP == 1)
                    //���û��Ѳ���
				    dwt_configuresleep(sleep_mode, DWT_WAKE_WK|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif
                    //���������ݰ�
				    instance_config_frameheader_16bit(inst);    
				    inst->instanceWakeTime_ms = portGetTickCnt();
                }
                break;
                default:
                break;
            }
            break; // end case TA_INIT

        case TA_SLEEP_DONE ://״̬:˯��(�ȴ�����)
            {
                //��ȡ�¼�
            	event_data_t* dw_event = instance_getevent(10); 
    			//�ȴ��¼�[��ʱ����]����dw1000 
    			if (dw_event->type != DWT_SIG_RX_TIMEOUT)       
    			{
    				// if no pause and no wake-up timeout continu waiting for the sleep to be done.
                    instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
                    break;
                }

                instDone = INST_NOT_DONE_YET;
                inst->instToSleep = FALSE ;
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
    			inst->instanceWakeTime_ms = portGetTickCnt(); // Record the time count when we wake-up
#if (DEEP_SLEEP == 1)
                {
                    //����dw1000�豸 wake up device from low power mode
                	port_wakeup_dw1000_fast();               

                    //ʹ��dw1000  LED���� this is platform dependent - only program if DW EVK/EVB
                    dwt_setleds(1);

                    //���������ӳٲ��� MP bug - TX antenna delay needs reprogramming as it is not preserved (only RX)
                    dwt_settxantennadelay(inst->txAntennaDelay) ;

                    //����EUI set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
    				dwt_seteui(inst->eui64);
                }
#else
                Sleep(3); //to approximate match the time spent in the #if above
#endif
                //�������߲���(������������仯)
                instance_set_antennadelays(); //this will update the antenna delay if it has changed
                //���÷��书�ʲ���(������������仯)
                instance_set_txpower(); //configure TX power if it has changed
           }
           break;

        case TA_TXE_WAIT ://״̬:�����ɽ���˯��/���δ��ɼ������� 
            //if we are scheduled to go to sleep before next transmission then sleep first.
        	if((inst->nextState == TA_TXPOLL_WAIT_SEND)
                    && (inst->instToSleep)  //go to sleep before sending the next poll/ starting new ranging exchange
                    )
            {
            	inst->rangeNum++; //increment the range number before going to sleep
                //the app should put chip into low power state and wake up after tagSleepTime_ms time...
                //the app could go to *_IDLE state and wait for uP to wake it up...
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //don't sleep here but kick off the Sleep timer countdown
                inst->testAppState = TA_SLEEP_DONE;

                {
#if (DEEP_SLEEP == 1)
                	//dw1000����˯��ģʽ,����ua���� put device into low power mode
                	dwt_entersleep(); // go to sleep
#endif
					if(inst->rxResponseMask != 0)
					{
						//��ǩ���ܾ���ֵ֪ͨ�ϱ� DW1000 gone to sleep - report the received range
						inst->newRange = instance_calc_ranges(&inst->tofArray[0], MAX_ANCHOR_LIST_SIZE, TOF_REPORT_T2A, &inst->rxResponseMask);
						inst->rxResponseMaskReport = inst->rxResponseMask;
						inst->rxResponseMask = 0;
						inst->newRangeTime = portGetTickCnt() ;
					}
                }
            }
            else //proceed to configuration and transmission of a frame
            {
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
            }
            break ; // end case TA_TXE_WAIT

        case TA_TXPOLL_WAIT_SEND ://״̬:����Poll���ݰ� 
            {
                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum; //copy new range number
            	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_TAG_POLL; //message function code (specifies if message is a poll, response or other...)

                //��ǩ ��ֵ͸��������Poll���ݰ�
                if(inst->trasport_cmd.send_enable == true)
                {
                    inst->msg_f.messageData[POLL_TRSPORT_ADDR0] = inst->trasport_cmd.send_addr & 0xff;
                    inst->msg_f.messageData[POLL_TRSPORT_ADDR1] = (inst->trasport_cmd.send_addr >> 8) & 0xff;
                    inst->msg_f.messageData[POLL_TRSPORT_LEN] = inst->trasport_cmd.send_cmdlen;
                    memcpy(&inst->msg_f.messageData[POLL_TRSPORT], inst->trasport_cmd.send_cmd, inst->trasport_cmd.send_cmdlen);
                    inst->trasport_cmd.send_enable = false;   
                }
                else 
                {
                    inst->msg_f.messageData[POLL_TRSPORT_ADDR0] = 0xfe;
                    inst->msg_f.messageData[POLL_TRSPORT_ADDR1] = 0xff;
                }


                inst->psduLength = (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
                inst->msg_f.seqNum = inst->frameSN++; //copy sequence number and then increment
                inst->msg_f.sourceAddr[0] = inst->instanceAddress16 & 0xff; //inst->eui64[0]; //copy the address
                inst->msg_f.sourceAddr[1] = (inst->instanceAddress16>>8) & 0xff; //inst->eui64[1]; //copy the address
            	inst->msg_f.destAddr[0] = 0xff;  //set the destination address (broadcast == 0xffff)
            	inst->msg_f.destAddr[1] = 0xff;  //set the destination address (broadcast == 0xffff)

                //��Poll��������д��dw1000׼������  
                dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	//write the frame data
				//���ñ�ǩ����Poll�����ӳٴ򿪽���ʱ�� set the delayed rx on time (the response message will be sent after this delay (from A0))
				dwt_setrxaftertxdelay((uint32)inst->tagRespRxDelay_sy);  // units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
                //���ܻ�վRESP����
				inst->remainingRespToRx = MAX_ANCHOR_LIST_SIZE; //expecting 4 responses
				//���ý��ܳ�ʱʱ��
				dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy);  //configure the RX FWTO
				//����ǰ������ܳ�ʱʱ��
				dwt_setpreambledetecttimeout(PTO_PACS); //configure preamble timeout
                //���ܻ�վResp��־λ
				inst->rxResponseMask = 0;	//reset/clear the mask of received responses when tx poll

				inst->wait4ack = DWT_RESPONSE_EXPECTED; //response is expected - automatically enable the receiver

                //���÷���Poll���ݳ���
				dwt_writetxfctrl(inst->psduLength, 0, 1); //write frame control
                //�����豸ͨ��ģʽ
				inst->twrMode = INITIATOR;
                //�����򿪷���
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); //transmit the frame

                inst->testAppState = TA_TX_WAIT_CONF ;  // wait confirmation
                inst->previousState = TA_TXPOLL_WAIT_SEND ;
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)

            }
            break;

        case TA_TXFINAL_WAIT_SEND ://״̬:����Final���ݰ�
            {
            	//the final has the same range number as the poll (part of the same ranging exchange)
                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNum;
                //the mask is sent so the anchors know whether the response RX time is valid
				inst->msg_f.messageData[VRESP] = inst->rxResponseMask;
            	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_TAG_FINAL; //message function code (specifies if message is a poll, response or other...)
                inst->psduLength = (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
                inst->msg_f.seqNum = inst->frameSN++;
                //��Final��������д��dw1000׼������ 
				dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

				inst->wait4ack = 0; //clear the flag not using wait for response as this message ends the ranging exchange
                //���÷���final����,���ӳٷ���
				if(instance_send_delayed_frame(inst, DWT_START_TX_DELAYED))
                {
                    uint8_t buf[64];
                    sprintf(buf, "instance_send_delayed_frame fault :%02X", inst->rxResponseMask);
                    port_tx_msg(buf, strlen(buf));
                    
                    // initiate the re-transmission
					inst->testAppState = TA_TXE_WAIT ; //go to TA_TXE_WAIT first to check if it's sleep time
					inst->nextState = TA_TXPOLL_WAIT_SEND ;
					inst->instToSleep = TRUE ;
                    break; //exit this switch case...
                }
                else
                {
                    inst->testAppState = TA_TX_WAIT_CONF;   // wait confirmation
                }

				inst->previousState = TA_TXFINAL_WAIT_SEND;
				inst->instToSleep = TRUE ;
            	instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
            }
            break;


        case TA_TX_WAIT_CONF ://״̬:�ȴ��������
            {
                //��ȡ�¼�
				event_data_t* dw_event = instance_getevent(11); //get and clear this event

				//�ȴ��¼�[���ͳɹ�����]
                if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
                {
				    instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    break;
                }

                instDone = INST_NOT_DONE_YET;

                if(inst->previousState == TA_TXFINAL_WAIT_SEND)
                {
                   	inst->testAppState = TA_TXE_WAIT ;
                   	inst->nextState = TA_TXPOLL_WAIT_SEND ;
                    break;
                }
                else
                {
					inst->txu.txTimeStamp = dw_event->timeStamp;
					inst->tagPollTxTime32h = dw_event->timeStamp32h;

                    //֮ǰ״̬Ϊ����Poll�������final���ݰ�
					if(inst->previousState == TA_TXPOLL_WAIT_SEND)
					{
		                uint64 tagCalculatedFinalTxTime ;
		                // Embed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
		                tagCalculatedFinalTxTime =  (inst->txu.txTimeStamp + inst->pollTx2FinalTxDelay) & MASK_TXDTS;

		                inst->delayedTRXTime32h = tagCalculatedFinalTxTime >> 8; //high 32-bits
		                // Calculate Time Final message will be sent and write this field of Final message
		                // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
		                // zeroing its low 9 bits, and then having the TX antenna delay added
		                // getting antenna delay from the device and add it to the Calculated TX Time
		                tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txAntennaDelay;
		                tagCalculatedFinalTxTime &= MASK_40BIT;

		                // Write Calculated TX time field of Final message
						memcpy(&(inst->msg_f.messageData[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);
		                // Write Poll TX time field of Final message
						memcpy(&(inst->msg_f.messageData[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);

					}

                    inst->testAppState = TA_RX_WAIT_DATA ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll

                    message = 0;
                    //fall into the next case (turn on the RX)
                }

            }

            //break ; // end case TA_TX_WAIT_CONF

        case TA_RX_WAIT_DATA :  //״̬:��ȡ���ݳɹ� 

            switch (message)
            {
				//��ȡ��Ч����֡
				case DWT_SIG_RX_OKAY :
                    {
                        //��ȡ�¼�
    					event_data_t* dw_event = instance_getevent(15); //get and clear this event
    					uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
    					uint8  dstAddr[8] = {0,0,0,0,0,0,0,0};
                        int fcode = 0;
    					uint8 tof_idx  = 0;
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

    					//�����������֡
    					switch(fcode)
    					{
    					    //�������Resp֡
    						case RTLS_DEMO_MSG_ANCH_RESP:
    						{
    							uint8 currentRangeNum = (messageData[TOFRN] + 1); //current = previous + 1

                                /*��ǩ���� ��վResp���ݰ������͸��ָ��*/
                                uint16 trasport_addr = messageData[RESP_TRSPORT_ADDR0] | (messageData[RESP_TRSPORT_ADDR0] << 8);
                                if(trasport_addr == inst->instanceAddress16 || trasport_addr == 0xffff)
                                {
                                    inst->trasport_cmd.recv_enable = true;
                                    inst->trasport_cmd.recv_addr = srcAddr[0] + (((uint16) srcAddr[1]) << 8);
                                    inst->trasport_cmd.recv_cmdlen = messageData[RESP_TRSPORT_LEN];
                                    memcpy(inst->trasport_cmd.recv_cmd, &messageData[RESP_TRSPORT], inst->trasport_cmd.recv_cmdlen);
                                    
                                }


                                //����վ��ȡУ����һ�����ʱ��϶����
    							if(GATEWAY_ANCHOR_ADDR == (srcAddr[0] | ((uint32)(srcAddr[1] << 8)))) //if response from gateway then use the correction factor
    							{
    								// int sleepCorrection = (int16) (((uint16) messageData[RES_TAG_SLP1] << 8) + messageData[RES_TAG_SLP0]);
    								// casting received bytes to int because this is a signed correction -0.5 periods to +1.5 periods
    								inst->tagSleepCorrection_ms = (int16) (((uint16) messageData[RES_TAG_SLP1] << 8) + messageData[RES_TAG_SLP0]);
    								inst->tagSleepRnd_ms = 0; // once we have initial response from Anchor #0 the slot correction acts and we don't need this anymore
    							}

    							if(dw_event->typePend == DWT_SIG_RX_PENDING)
    							{
    								// stay in TA_RX_WAIT_DATA - receiver is already enabled, waiting for next response.
    							}
    							//����final���ݰ��ж� DW1000 idle - send the final
    							else //if(dw_event->type_pend == DWT_SIG_DW_IDLE)
    							{
#if (TAG_HASTO_RANGETO_A0 == 1)
    								if(inst->rxResponseMask & 0x1)//if this is tag and A0's response received send the final
#endif
    								{
    									inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final
    								}
#if (TAG_HASTO_RANGETO_A0 == 1)
    								else //go to sleep
    								{
    									inst->testAppState = TA_TXE_WAIT ; //go to TA_TXE_WAIT first to check if it's sleep time
    									inst->nextState = TA_TXPOLL_WAIT_SEND ;
    									inst->instToSleep = TRUE;
    								}
#endif
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
    									inst->rxResponseMask |= (0x1 << tof_idx);
    								}

    							}
    							else
    							{
    								if(inst->tofArray[tof_idx] != INVALID_TOF)
    								{
    									inst->tofArray[tof_idx] = INVALID_TOF;
    								}
    							}
    						}
    						break; //RTLS_DEMO_MSG_ANCH_RESP

                            //�������͸��֡
                            case RTLS_DEMO_MSG_DTU:
                            {
                            }
                            break;

    						default:
    						{
    							tag_process_rx_timeout(inst); //if unknown message process as timeout
    						}
    						break;
    					} //end switch (fcode)

                    }
				    break ; //end of DWT_SIG_RX_OKAY


                case DWT_SIG_RX_TIMEOUT :
                	{
                	    //��ȡ�¼�
                		event_data_t* dw_event = instance_getevent(17); //get and clear this event

                		//Anchor can time out and then need to send response - so will be in TX pending
                		if(dw_event->typePend == DWT_SIG_TX_PENDING)
                		{
                			inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                			inst->previousState = TA_TXRESPONSE_SENT_TORX ;    //wait for TX confirmation of sent response
                		}
                		else if(dw_event->typePend == DWT_SIG_DW_IDLE) //if timed out and back in receive then don't process as timeout
						{
                			tag_process_rx_timeout(inst);
						}
                		//else if RX_PENDING then wait for next RX event...
						message = 0; //clear the message as we have processed the event
                	}
                    break ;

                default :
                    {
                        if(message) // == DWT_SIG_TX_DONE)
                        {
                        	instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                        }

                    	if(instDone == INST_NOT_DONE_YET) instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    }
                    break;

            }
            break ; // end case TA_RX_WAIT_DATA
        default:
                //printf("\nERROR - invalid state %d - what is going on??\n", inst->testAppState) ;
            break;
    } // end switch on testAppState

    return instDone;
} // end testapprun_tag()

// -------------------------------------------------------------------------------------------------------------------
/**
 * ���ܣ���ǩ ����������
 *
 */
int tag_run(void)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
    int done = INST_NOT_DONE_YET;

    while(done == INST_NOT_DONE_YET)
	{
		done = tag_app_run(inst) ; // run the communications application
	}

    //��ɱ��β�࣬��������ͬʱ������ʱ����
    if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //tag has finished the ranging exchange and needs to configure sleep time
    {
		int32 nextPeriod ;

		//������һ�����ڻ���ʱ��            next period will be a positive number because correction is -0.5 to +1.5 periods, (and tagSleepTime_ms is the period)
		nextPeriod = inst->tagSleepRnd_ms + inst->tagSleepTime_ms + inst->tagSleepCorrection_ms;

		inst->nextWakeUpTime_ms = (uint32) nextPeriod ; //set timeout time, CAST the positive period to UINT for correct wrapping.
		inst->tagSleepCorrection_ms = 0; //clear the correction
		inst->instanceTimerEn = 1; //start timer
    }

    //�����ʱ���Ƿ�ʱ����ʱ���ѽ�����һ�β�� check if timer has expired
    if(inst->instanceTimerEn == 1)
    {
		if((portGetTickCnt() - inst->instanceWakeTime_ms) > inst->nextWakeUpTime_ms)
		{
			event_data_t dw_event;
			inst->instanceTimerEn = 0;
			dw_event.rxLength = 0;
			dw_event.type = 0;
			//д���¼�[��ʱ����]
			instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
		}
    }
    return 0 ;
}

/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
