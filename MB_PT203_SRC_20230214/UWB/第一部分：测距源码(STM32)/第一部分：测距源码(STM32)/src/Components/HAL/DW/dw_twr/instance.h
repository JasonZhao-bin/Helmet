/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>	
	
#include "uwb.h"
#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* NOTES on TREK compile/build time features/options ***********************************************************
*******************************************************************************************************************/
//���˯�ߺ�
#define DEEP_SLEEP (1)         //  To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode while it is
//waiting for start of next ranging exchange

//����У����
#define CORRECT_RANGE_BIAS  (1) // Compensate for small bias due to uneven accumulator growth at close up high power

//��ǩ����Final��������
#define TAG_HASTO_RANGETO_A0 (0)// if set to 1 then tag will only send the Final if the Response from A0 has been received
/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1				  // ʵ������(��ǩ/��վ) one instance (tag or anchor - controlling one DW1000)
#define SPEED_OF_LIGHT      (299702547.0)     // ���ڿ������ٶ� in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // dw1000 40bit������ DW1000 counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  // dw1000 ʱ������Ե�9λ The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

//! callback events
#define DWT_SIG_RX_NOERR            0
#define DWT_SIG_TX_DONE             1       // ���ͳɹ����� Frame has been sent
#define DWT_SIG_RX_OKAY             2       // ���ܳɹ����� Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // ���ܴ������� Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // ���ܳ�ʱ���� Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE          6       // δʹ�� ACK frame has been sent (as a result of auto-ACK)
#define DWT_SIG_RX_BLINK			7		// δʹ�� Received ISO EUI 64 blink message
#define DWT_SIG_RX_PHR_ERROR        8       // δʹ�� Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // δʹ�� Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // δʹ�� Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT       11      // δʹ�� Got preamble detection timeout (no preamble detected)

#define DWT_SIG_TX_PENDING          12      // ׼���������� TX is pending
#define DWT_SIG_TX_ERROR            13      // δʹ�� TX failed
#define DWT_SIG_RX_PENDING          14      // ׼���������� RX has been re-enabled
#define DWT_SIG_DW_IDLE             15      // �������� DW radio is in IDLE (no TX or RX pending)

#define SIG_RX_UNKNOWN			    99		// ����δ֪����֡���� Received an unknown frame

//֡������
#define RTLS_DEMO_MSG_TAG_POLL              (0x81)          // ��ǩ����Poll�� Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x70)          // ��վ����Resp�� Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x82)          // ��ǩ����Final�� Tag final massage back to Anchor
#define RTLS_DEMO_MSG_DTU                   (0x93)          // �豸����DTU�� ͸��ʹ��


//lengths including the Decaranging Message Function Code byte

//absolute length = 11 +
//֡��Ϣ����
#define TAG_POLL_MSG_LEN                    2+19			// FunctionCode(1), Range Num (1), trasport addr(2) trasport cmdlen(1) trasport cmd(16)
#define ANCH_RESPONSE_MSG_LEN               8+19            // FunctionCode(1), Sleep Correction Time (2), Measured_TOF_Time(4), Range Num (1) (previous), trasport addr(2) trasport cmdlen(1) trasport cmd(16)

#ifdef HW_8Anc
#define TAG_FINAL_MSG_LEN                   53              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
															// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5),
															// Resp4_RxTime(5), Resp5_RxTime(5), Resp6_RxTime(5), Resp7_RxTime(5), Final_TxTime(5), Valid Response Mask (1)

#else
#define TAG_FINAL_MSG_LEN                   33              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
															// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)

#endif

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for one 16-bit address and one 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

#define BLINK_FRAME_CONTROL_BYTES       (1)
#define BLINK_FRAME_SEQ_NUM_BYTES       (1)
#define BLINK_FRAME_CRC					(FRAME_CRC)
#define BLINK_FRAME_SOURCE_ADDRESS      (ADDR_BYTE_SIZE_L)
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes
#define BLINK_FRAME_LEN_BYTES           (BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC)

//��ǩ����
#define MAX_TAG_LIST_SIZE				(32)

//��վ����
#define MAX_ANCHOR_LIST_SIZE			(8) //this is limited to 4 in this application
#define NUM_EXPECTED_RESPONSES			(7) //e.g. MAX_ANCHOR_LIST_SIZE - 1

//����վ��ַ
#define GATEWAY_ANCHOR_ADDR				(0x8000)


#define WAIT4TAGFINAL					2

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//3���¼��궨��
#define INST_DONE_WAIT_FOR_NEXT_EVENT   	1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        		//which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               	0   //this signifies that the instance is still processing the current event

//֡��Ϣƫ����

//Final���ݰ��������
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                2				// Poll TX time
#define RRXT0                               7				// A0 Response RX time
#define RRXT1                               12				// A1 Response RX time
#define RRXT2                               17				// A2 Response RX time
#define RRXT3                               22				// A3 Response RX time
#ifdef HW_8Anc
	#define RRXT4                           27				// A4 Response RX time
	#define RRXT5                           32				// A5 Response RX time
	#define RRXT6                           37				// A6 Response RX time	
	#define RRXT7                           42				// A7 Response RX time	

	#define FTXT							47				// Final TX time		
	#define VRESP                           52				// Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
#else
	#define FTXT                            27				// Final TX time
	#define VRESP                           32				// Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
#endif

//Resp���ݰ��������
#define RES_TAG_SLP0                        1               // Response tag sleep correction LSB
#define RES_TAG_SLP1                        2               // Response tag sleep correction MSB
#define TOFR                                3				// ToF (n-1) 4 bytes
#define TOFRN								7				// range number 1 byte
#define RESP_TRSPORT_ADDR0                  8               // ��վ͸������Ŀ���ַ LSB
#define RESP_TRSPORT_ADDR1                  9               // ��վ͸������Ŀ���ַ MSB
#define RESP_TRSPORT_LEN                    10              // ��վ͸���������ݳ���
#define RESP_TRSPORT                        11              // ��վ͸�����ݴ��λ��(11:27),�ܼ�16�ֽ�

//Poll���ݰ��������
#define POLL_RNUM                           1               // Poll message range number
#define POLL_TRSPORT_ADDR0                  2               // ��ǩ͸������Ŀ���ַ LSB
#define POLL_TRSPORT_ADDR1                  3               // ��ǩ͸������Ŀ���ַ MSB
#define POLL_TRSPORT_LEN                    4               // ��ǩ͸�����ݳ���
#define POLL_TRSPORT                        5               // ��ǩ͸�����ݴ��λ��(5:21),�ܼ�16�ֽ�

#define DW_RX_ON_DELAY          (16) //us - the DW RX has 16 us RX on delay before it will receive any data

//this it the delay used for configuring the receiver on delay (wait for response delay)
//NOTE: this RX_RESPONSE_TURNAROUND is dependent on the microprocessor and code optimisations
//#define RX_RESPONSE_TURNAROUND  (300) //this takes into account any turnaround/processing time (reporting received poll and sending the response)
#define RX_RESPONSE1_TURNAROUND_6M81 	(220)//(300) //takes about 100 us for response to come back
#define RX_RESPONSE1_TURNAROUND_110K 	(300)//(300) //takes about 100 us for response to come back



#define PTO_PACS				(3)	 //tag will use PTO to reduce power consumption (if no response coming stop RX)

//Tag will range to 3 or 4 anchors
//Each ranging exchange will consist of minimum of 3 messages (Poll, Response, Final)
//and a maximum of 6 messages (Poll, Response x 4, Final)
//Thus the ranging exchange will take either 28 ms for 110 kbps and 5 ms for 6.81 Mbps.
//NOTE: the above times are for 110k rate with 64 symb non-standard SFD and 1024 preamble length


//Tag to Anchor Ranging
//1. Tag sends a Poll
//2. A0 responds (delayed response) after fixedReplyDelayAnc1, A0 will re-enable its receiver automatically (by using WAIT4RESP)
//3. A1, A2, A3 re-enble the receiver to receive A0's response
//4. A1 responds and will re-enable its receiver automatically (by using WAIT4RESP)
//5. A2, A3 re-enble the receiver to receive A1's response
//6. A2 responds and will re-enable its receiver automatically (by using WAIT4RESP)
//7. A0, A3 re-enable the receiver to receive A2's response
//9. A3 responds and will re-enable its receiver automatically (by using WAIT4RESP)
//10. A0, A1, A2, A3 - all receive the Final from the tag


typedef enum instanceModes{TAG, ANCHOR, NUM_MODES} INST_MODE;
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above
//Anchor_Rng = the anchor (assumes a tag function) and ranges to another anchor - used in Anchor to Anchor TWR for auto positioning function


// instance sending a poll (starting TWR) is INITIATOR
// instance which receives a poll (and will be involved in the TWR) is RESPONDER
// instance which does not receive a poll (default state) will be a LISTENER - will send no responses
typedef enum instanceTWRModes{INITIATOR,  RESPONDER_T, LISTENER, GREETER, ATWR_MODES} ATWR_MODE;


//��ȡTOF���ݺ�
#define TOF_REPORT_NUL 0
#define TOF_REPORT_T2A 1

//��ЧTOF���ݺ�
#define INVALID_TOF (0xABCDFFFF)

//�豸״̬ö��
typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1 - state in which the instance will enter sleep (if ranging finished) or proceed to transmit a message
    TA_TXPOLL_WAIT_SEND,        //2 - configuration and sending of Poll message
    TA_TXFINAL_WAIT_SEND,       //3 - configuration and sending of Final message
    TA_TXRESPONSE_WAIT_SEND,    //4 - a place holder - response is sent from call back
    TA_TX_WAIT_CONF,            //5 - confirmation of TX done message

    TA_RXE_WAIT,                //6
    TA_RX_WAIT_DATA,            //7

    TA_SLEEP_DONE,               //8
    TA_TXRESPONSE_SENT_POLLRX,   //9
    TA_TXRESPONSE_SENT_RESPRX,   //10
    TA_TXRESPONSE_SENT_TORX,     //11
    TA_TXRESPONSE_SENT_APOLLRX,  //12
    TA_TXRESPONSE_SENT_ARESPRX   //13

} INST_STATES;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

//֡��ʽ1�ṹ��(Դ����ַ��Ŀ�곤��ַ)
typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

//֡��ʽ2�ṹ��(Դ�̵�ַ��Ŀ��̵�ַ)
typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

//֡��ʽ3�ṹ��(Դ�̵�ַ��Ŀ�곤��ַ)
typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

//֡��ʽ4�ṹ��(Դ����ַ��Ŀ��̵�ַ)
typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

//�ŵ����ýṹ��
typedef struct
{
    uint8 channelNumber ;       // �ŵ� valid range is 1 to 11
    uint8 preambleCode ;        // ǰ���� 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // �����ظ�Ƶ�� NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // �������� DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // preambleLength values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;             // pacSize
    uint8 nsSFD ;               // 0:�Ǳ�׼SFD, 1:��׼SFD 
    uint16 sfdTO;               // SFD ��ʱ !< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;

typedef struct
{
    uint16 slotDuration_ms ;    //���������� slot duration (time for 1 tag to range to 4 anchors)
    uint16 numSlots ; 		    //������ number of slots in one superframe (number of tags supported)
    uint16 sfPeriod_ms ;	    //����֡���� superframe period in ms
    uint16 tagPeriod_ms ; 	    //=����֡���� the time during which tag ranges to anchors and then sleeps, should be same as FRAME PERIOD so that tags don't interfere
    uint16 pollTxToFinalTxDly_us ; //Poll->Final�����ӳ� response delay time (Poll to Final delay)
} sfConfig_t ;

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

//size of the event queue, in this application there should be at most 2 unprocessed events,
//i.e. if there is a transmission with wait for response then the TX callback followed by RX callback could be executed
//in turn and the event queued up before the instance processed the TX event.
#define MAX_EVENT_NUMBER (4)

//�¼��ṹ��
typedef struct
{
	uint8  type;			    // �¼����� event type - if 0 there is no event in the queue
	uint8  typePend;	        // ���ù����¼� set if there is a pending event (i.e. DW is not in IDLE (TX/RX pending)
	uint16 rxLength ;		    // ���ܳ��� length of RX data (does not apply to TX events)

	uint64 timeStamp ;		    // dw1000 40bitʱ��� last timestamp (Tx or Rx) - 40 bit DW1000 time

	uint32 timeStamp32l ;		// dw1000ʱ�����32bit last tx/rx timestamp - low 32 bits of the 40 bit DW1000 time
	uint32 timeStamp32h ;		// dw1000ʱ����32bit last tx/rx timestamp - high 32 bits of the 40 bit DW1000 time

	uint32 uTimeStamp ;			  //32 bit system counter (ms) - STM32 tick time (at time of IRQ)

	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
	}msgu;

    dwt_rxdiag_t rx_diag;       //uwb�źŲ���
    
	//uint8 gotit;			//stores the instance function which processed the event (used for debug)
}event_data_t ;

// TX power and PG delay configuration structure
// ���书�����ýṹ��
typedef struct {
    uint8 pgDelay;

    //TX POWER
    //31:24     BOOST_0.125ms_PWR
    //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    //7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32 txPwr[2]; //
}tx_struct;

//
typedef struct
{
    INST_MODE mode;				    //�豸ģʽ(��ǩ/��վ) //instance mode (tag or anchor)
    ATWR_MODE twrMode;              //�豸ͨ��ģʽ
    INST_STATES testAppState ;		//�豸��ǰ״̬	//state machine - current state
    INST_STATES nextState ;			//�豸��һ��״̬	//state machine - next state
    INST_STATES previousState ;		//�豸��һ��״̬	//state machine - previous state

	//configuration structures
	dwt_config_t    configData ;	//dw1000�ŵ����� //DW1000 channel configuration
	dwt_txconfig_t  configTX ;		//dw1000����������� //DW1000 TX power configuration
	uint16			txAntennaDelay ;//dw1000���������ӳ� //DW1000 TX antenna delay
	uint16			rxAntennaDelay ;//dw1000���������ӳ� //DW1000 RX antenna delay
	uint32 			txPower ;		//���͹��� //DW1000 TX power
	uint8 txPowerChanged ;			//���书�ʸ����ж�λ //power has been changed - update the register on next TWR exchange
	uint8 antennaDelayChanged;		//�����ӳٸ����ж�λ //antenna delay has been changed - update the register on next TWR exchange

	uint16 instanceAddress16;       //�豸��ַ //contains tag/anchor 16 bit address

	//timeouts and delays
	int32 tagPeriod_ms;             //��ǩ����(���+˯��) // in ms, tag ranging + sleeping period
	int32 tagSleepTime_ms;          //��ǩ˯��ʱ�� //in milliseconds - defines the nominal Tag sleep time period
	int32 tagSleepRnd_ms;           //��ǩ��ʱ��      //add an extra slot duration to sleep time to avoid collision before getting synced by anchor 0

	//this is the delay used for the delayed transmit
	uint64 pollTx2FinalTxDelay ;    //����Poll->����Final�ӳ�ʱ�� //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit)
	uint32 fixedReplyDelayAnc32h ;  //�̶���Resplyʱ���32bit //this is a delay used for calculating delayed TX/delayed RX on time (units: 32bit of 40bit DW time)
	uint32 preambleDuration32h ;    //ǰ�������ʱ�� //preamble duration in device time (32 MSBs of the 40 bit time)
	uint32 tagRespRxDelay_sy ;      //��ǩ��������ӳٴ�ʱ��dwt_setrxaftertxdelay() //TX to RX delay time when tag is awaiting response message an another anchor
	uint32 ancRespRxDelay_sy ;      //��վ��������ӳٴ�ʱ��dwt_setrxaftertxdelay() //TX to RX delay time when anchor is awaiting response message an another anchor

	int fwto4RespFrame_sy ;         //dwt_setrxtimeout ����Resp��ʱ this is a frame wait timeout used when awaiting reception of Response frames (used by both tag/anchor)
	int fwto4FinalFrame_sy ;        //dwt_setrxtimeout����Final��ʱ this is a frame wait timeout used when awaiting reception of Final frames
	uint32 delayedTRXTime32h;		//dwt_setdelayedtrxtime�ӳٷ��͸�32bit time at which to do delayed TX or delayed RX (note TX time is time of SFD, RX time is RX on time)

    //message structure used for holding the data of the frame to transmit before it is written to the DW1000
	srd_msg_dsss msg_f ;            //�������ݽṹ��   // ranging message frame with 16-bit addresses

	//Tag function address/message configuration
	uint8   shortAdd_idx ;			//�̵�ַ���� 	// device's 16-bit address low byte (used as index into arrays [0 - 3])
	uint8   eui64[8];				//64λ����ַ device's EUI 64-bit address
	uint16  psduLength ;			//���Ϳ������ݳ��� used for storing the TX frame length
    uint8   frameSN;				//���Ϳ����������к� modulo 256 frame sequence number - it is incremented for each new frame transmission
	uint16  panID ;					//���Ϳ�������Panid panid used in the frames

	//64 bit timestamps
	//union of TX timestamps
	union {
		uint64 txTimeStamp ;		//����ʱ���    // last tx timestamp
		uint64 tagPollTxTime ;		//��ǩ����Pollʱ���    // tag's poll tx timestamp
	    uint64 anchorRespTxTime ;	//��վ����Respʱ���   // anchor's reponse tx timestamp
	}txu;
	uint32 tagPollTxTime32h ;       //��ǩ����Pollʱ�����32bit
	uint64 tagPollRxTime ;          //��վ����Pollʱ��� // receive time of poll message


	//application control parameters
	uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
	uint8   wait4final ;

	uint8   instToSleep;			//�������˯���ж�λ // if set the instance will go to sleep before sending the blink/poll message
    uint8	instanceTimerEn;		//��ǩ������ʱ���ж�λ // enable/start a timer
    uint32	instanceWakeTime_ms;    //��ǩ����ʱ�� // micro time at which the tag was waken up
    uint32  nextWakeUpTime_ms;		//��ǩ����ʱ�� micro time at which to wake up tag

	uint8   rxResponseMask;			//����Resp��־λ bit mask - bit 0 = received response from anchor ID = 0, bit 1 from anchor ID = 1 etc...
	uint8   rxResponseMaskReport;   //����Resp�ϱ���־λ this will be set before outputting range reports to signify which are valid
	uint8	rangeNum;				//���Seq incremented for each sequence of ranges (each slot)
	uint8	rangeNumA[MAX_TAG_LIST_SIZE];//��վ��Ӧÿ����ǩ���Seq				// array which holds last range number from each tag

    int8    remainingRespToRx ;		//ʣ��Resp����������־λ how many responses remain to be received (in current ranging exchange)

	uint16  sframePeriod_ms;		//����֡���� // superframe period in ms
	uint16  slotDuration_ms;		//������ slot duration in ms
	uint16  numSlots;               //������
	int32   tagSleepCorrection_ms;  //��ǩ����ʱ������ tag's sleep correction to keep it in it's assigned slot

    //diagnostic counters/data, results and logging
    uint32 tof[MAX_TAG_LIST_SIZE];  //��һ�ξ���ֵ���� //this is an array which holds last ToF from particular tag (ID 0-(MAX_TAG_LIST_SIZE-1))

    //this is an array which holds last ToF to each anchor it should
    uint32 tofArray[MAX_ANCHOR_LIST_SIZE]; //��ǩ��n����վ����ֵ contain 4 ToF to 4 anchors all relating to same range number sequence

    dwt_rxdiag_t rx_diag;           //uwb�źŲ���(��վͨ�����ܱ�ǩ��Final�� ��ȡ�źŲ���)
    //debug counters
    //int txMsgCount; //number of transmitted messages
	//int rxMsgCount; //number of received messages
    //int rxTimeouts ; //number of received timeout events
	//int lateTX; //number of "LATE" TX events
	//int lateRX; //number of "LATE" RX events


	//ranging counters
    int longTermRangeCount ;        //���в����������ֵ //total number of ranges

    int newRange;			        //�µľ��� �ж�λ//flag set when there is a new range to report TOF_REPORT_A2A or TOF_REPORT_T2A
    int newRangeAncAddress;         //�µľ��� ��վ��ַ//last 4 bytes of anchor address - used for printing/range output display
    int newRangeTagAddress;         //�µľ��� ��ǩ��ַ//last 4 bytes of tag address - used for printing/range output display
    int newRangeTime;               //�µľ��� ʱ�����

    uint8 gatewayAnchor ;           //�Ƿ�Ϊ����վ�ж�λ//set to TRUE = 1 if anchor address == GATEWAY_ANCHOR_ADDR

	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER];//�¼���Ϣ //this holds any TX/RX events and associated message data
    uint8 dweventIdxOut;            //�¼���ȡ������־λ
    uint8 dweventIdxIn;             //�¼�д��������־λ
	uint8 dweventPeek;              //�¼��鿴
	uint8 monitor;      
	uint32 timeofTx ;

	uint8 smartPowerEn;         


    struct{
        //����͸�����ݲ���
		uint16 send_addr;			//�û�͸����ַ    0xffff �㲥, 0x8xxx��վ��ַ 0x0xxx��ǩ��ַ
        uint8 send_cmd[16];         //�û�͸������
        uint8 send_cmdlen;          //�û�͸�����ݳ���
        bool  send_enable;          //�û�͸������ʹ��

        //����͸������
        uint16 recv_addr;
        uint8 recv_cmd[16];         
        uint8 recv_cmdlen;
        bool  recv_enable;
        
    }trasport_cmd;
} instance_data_t ;

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and the range from given Time of Flight
int instance_calculate_rangefromTOF(int idx, uint32 tofx);

void instance_cleardisttable(int idx);

int instance_get_idist_mm(int idx);
int instance_get_idistraw_mm(int idx);
uint8 instance_validranges(void);

int instance_get_rnum(void);
int instance_get_rnuma(int idx);
int instance_get_lcount(void);

int instance_newrangeancadd(void);
int instance_newrangetagadd(void);
int instance_newrange(void);
int instance_newrangetim(void);
dwt_rxdiag_t instance_newrangerx_diag(void);
void instance_newtrasport_cmd(uint8_t *buf);


int instance_calc_ranges(uint32 *array, uint16 size, int reportRange, uint8* mask);

// clear the status/ranging data
void instance_clearcounts(void) ;

void instance_cleardisttableall();
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

// Call init, then call config, then call run.
// initialise the instance (application) structures and DW1000 device
int instance_init(int role);
// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config, sfConfig_t *sfconfig) ;

// configure the MAC address
void instance_set_16bit_address(uint16 address) ;
void instance_config_frameheader_16bit(instance_data_t *inst);

void tag_process_rx_timeout(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int tag_run(void) ;
int anch_run(void) ;       // returns indication of status report change

// configure TX/RX callback functions that are called from DW1000 ISR
void rx_ok_cb_tag(const dwt_cb_data_t *cb_data);
void rx_to_cb_tag(const dwt_cb_data_t *cb_data);
void rx_err_cb_tag(const dwt_cb_data_t *cb_data);
//void tx_conf_cb_tag(const dwt_cb_data_t *cb_data);

void rx_ok_cb_anch(const dwt_cb_data_t *cb_data);
void rx_to_cb_anch(const dwt_cb_data_t *cb_data);
void rx_err_cb_anch(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);

void instance_set_replydelay(int delayms);

// set/get the instance roles e.g. Tag/Anchor
// done though instance_init void instance_set_role(int mode) ;                //
int instance_get_role(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for DW1000)
uint32 instance_readdeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

int instance_send_delayed_frame(instance_data_t *inst, int delayedTx);

uint64 instance_convert_usec_to_devtimeu (double microsecu);


void instance_seteventtime(event_data_t *dw_event, uint8* timeStamp);

int instance_peekevent(void);

void instance_putevent(event_data_t newevent, uint8 etype);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

// configure the antenna delays
void instance_config_antennadelays(uint16 tx, uint16 rx);
void instance_set_antennadelays(void);
uint16 instance_get_txantdly(void);
uint16 instance_get_rxantdly(void);

// configure the TX power
void instance_config_txpower(uint32 txpower);
void instance_set_txpower(void);
int instance_starttxtest(int framePeriod);


instance_data_t* instance_get_local_structure_ptr(unsigned int x);

#ifdef __cplusplus
}
#endif

#endif
