#include <string.h>

#include "Generic.h"
#include "Generic_cmd.h"
#include "OSAL.h"

#include "hal_usart.h"
//#include "hal_usb.h"
#include "hal_flash.h"


enum UART_FRAME{
	UART_FRAME_JS = 1,
};


static QUEUE uart_queue; 

#ifdef HAL_USART1
#define USART1_RX_ISR_MAX_LEN  		64 
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
static uint16_t usart1_rx_sta = 0 ;//����״̬���	 
static uint8_t usart1_rx_buf[USART1_RX_ISR_MAX_LEN] = { 0 };	 
#endif


enum Module_AT_CMD{
  /*ϵͳ����*/
	Module_AT_CMD_VER =0x00,		//��ȡ�汾
	Module_AT_CMD_RTOKEN,			//�ָ�����ģʽ
	Module_AT_CMD_RESET,			//��λ
	Module_AT_CMD_SW,				//��������
	Module_AT_CMD_SW_R,				//������ȡ
	
	Module_AT_CMD_ERR = 0xff,
};

int App_Module_CMD_Queue_Init(void)
{
#ifdef HAL_USART1
	memset(usart1_rx_buf,0,sizeof(usart1_rx_buf));
#endif 
	
	osal_CreateQueue(&uart_queue, QUEUE_MSG_MAX);
}
													
static uint8_t Check_cmd(uint8_t* RXbuff)
{
	uint8_t i=0;
	if(strstr((char *)RXbuff,"AT+VER?") != NULL)	//�汾��ѯ
		return Module_AT_CMD_VER;		

	if(strstr((char *)RXbuff,"AT+RTOKEN") != NULL)	
		return Module_AT_CMD_RTOKEN;	
	
	if(strstr((char *)RXbuff,"AT+RSET") != NULL)
		return Module_AT_CMD_RESET;	

	if(strstr((char *)RXbuff,"AT+SW=") != NULL)
	{
		for(i=6;i<14;i++)
		{
			if(RXbuff[i]!='1' && RXbuff[i]!='0')
			{
				return Module_AT_CMD_ERR;
			}
		}						
		if(RXbuff[13]=='0')
			return Module_AT_CMD_SW;			
	}		

	if(strstr((char *)RXbuff,"AT+SW_R?") != NULL)
		return Module_AT_CMD_SW_R;
	
	return Module_AT_CMD_ERR;
}
	

int App_Module_Analyze_AT(uint8_t *buf)
{
	int ret = FALSE;
	bool reset = FALSE;
	uint8_t buf_rsp[64];	
	uint8_t r_tmp[10];
	uint32_t sys_para_tmp = 0;

	memset(r_tmp, 0, sizeof(r_tmp));
	memset(buf_rsp, 0, sizeof(buf_rsp));
	switch(Check_cmd(buf))
	{
		case Module_AT_CMD_VER:
				sprintf(buf_rsp, "OK+VER=soft:%s,hard:%s\r\n", uwb_software_ver, uwb_hardware_ver);						
				port_tx_msg(buf_rsp,strlen(buf_rsp));		
			break;
		case Module_AT_CMD_RTOKEN:
				sys_para.flag = 0x0000;
				App_Module_Sys_Write_NVM();
		
				sprintf(buf_rsp, "OK+RTOKEN\r\n");
				port_tx_msg(buf_rsp,strlen(buf_rsp));			
				reset = TRUE;
			break;
		case Module_AT_CMD_RESET:		
				sprintf(buf_rsp, "OK+RSET\r\n");
				port_tx_msg(buf_rsp,strlen(buf_rsp));			
				reset = TRUE;
			break;
		case Module_AT_CMD_SW:			//���û�������
			{
                sys_para_tmp |= ((buf[6]-'0') << 11);
                sys_para_tmp |= ((buf[7]-'0') << 10);
                sys_para_tmp |= ((buf[8]-'0') << 9);
                sys_para_tmp |= ((buf[9]-'0') << 8);
				sys_para_tmp |= ((buf[10]-'0') << 3);
				sys_para_tmp |= ((buf[11]-'0') << 2);	
				sys_para_tmp |= ((buf[12]-'0') << 1);
				sys_para_tmp |= ((buf[13]-'0') << 0);			
				sys_para.device_switch = sys_para_tmp;

				App_Module_Sys_Write_NVM();
				sprintf(buf_rsp, "OK+SW=%s\r\n", &buf[6]);	
				port_tx_msg(buf_rsp,strlen(buf_rsp));		

			}
			break;	
		case Module_AT_CMD_SW_R:		//��ȡ��������
			{
				r_tmp[0] = ((sys_para.device_switch >> 11) & 0x00000001) + '0';
				r_tmp[1] = ((sys_para.device_switch >> 10) & 0x00000001) + '0';
				r_tmp[2] = ((sys_para.device_switch >> 9) & 0x00000001) + '0';
				r_tmp[3] = ((sys_para.device_switch >> 8) & 0x00000001) + '0';
				r_tmp[4] = ((sys_para.device_switch >> 3) & 0x00000001) + '0';
				r_tmp[5] = ((sys_para.device_switch >> 2) & 0x00000001) + '0';
				r_tmp[6] = ((sys_para.device_switch >> 1) & 0x00000001) + '0';
				r_tmp[7] = ((sys_para.device_switch >> 0) & 0x00000001) + '0';			
				sprintf(buf_rsp, "OK+SW_R=%s\r\n", r_tmp);	
				port_tx_msg(buf_rsp,strlen(buf_rsp));		

			}
			break;					
		
		case Module_AT_CMD_ERR:
			break;	
			
		default:
			break;														
	}

	if(reset == TRUE){
		//�����������������δ���ͣ�����
		while(sys_uart_dma_buf.uart_dma_index != 0)
		{
			HalDelay_nMs(100);
			port_tx_msg(NULL, 0);
		}					
		NVIC_SystemReset();// ��λ
	}

	
	return ret;
}

/*
int App_Module_Process_USB_CMD(void)
{
	int ret = 0;
	uint8_t USB_RxBuff[64];
	memset(USB_RxBuff, 0, sizeof(USB_RxBuff));

	if(HalUsbRead(USB_RxBuff, sizeof(USB_RxBuff)) != 0)
	{
		//�ж��Ƿ�ΪATָ��
		if(memcmp(USB_RxBuff,"AT+",3) == 0)
		{
			App_Module_Analyze_AT(USB_RxBuff);
		}
		else
		{
			command_parser(USB_RxBuff);
		}
	}
	return ret;
}
*/
int App_Module_Process_USART_CMD(void)
{
	Message msg;
	if(osal_Dequeue(&uart_queue, &msg) == TRUE)
	{
		//�ж��Ƿ�ΪATָ��
		if(memcmp(msg.buf,"AT+",3) == 0)
		{	
			App_Module_Analyze_AT(msg.buf);
		}
		else
		{		
			command_parser(msg.buf);
		}
	}
}

/*
 *	����:���ڷ��ͺ���1
 *	�β�:buf(��������)��len(�������鳤��)
 *
 */
int App_Module_Uart_Send(uint8_t *buf, uint16_t len)
{
	int ret = 0;
	if((sys_uart_dma_buf.uart_dma_index + len) < sizeof(sys_uart_dma_buf.uart_dma_tx_tmp))
	{
		/*��ֵ��������*/
		memcpy(sys_uart_dma_buf.uart_dma_tx_tmp + sys_uart_dma_buf.uart_dma_index, buf, len);					
		sys_uart_dma_buf.uart_dma_index += len;							
	}
	else		
	{
//		while(1);		
	}
	
	if(sys_uart_dma_buf.uart_dma_report == true)
	{
		/*��ֵ��������*/
		memcpy(sys_uart_dma_buf.uart_dma_tx, sys_uart_dma_buf.uart_dma_tx_tmp, sys_uart_dma_buf.uart_dma_index);
		USART1_SendBuffer(sys_uart_dma_buf.uart_dma_tx, sys_uart_dma_buf.uart_dma_index, false);		
		sys_uart_dma_buf.uart_dma_report = false;
		sys_uart_dma_buf.uart_dma_index = 0;			
	}

	//HalUsbWrite(buf, len);
	return ret;
}

/*
 *	����:���ڷ��ͺ���2
 *	�β�:buf(��������)��len(�������鳤��)
 *
 */
void port_tx_msg(uint8_t *buf, uint16_t len)
{
	App_Module_Uart_Send(buf, len);
}

static uint8_t get_Xor_CRC(uint8_t *bytes, int offset, int len) 
{
	uint8_t xor_crc = 0;
    int i;
    for (i = 0; i < len; i++) {
		xor_crc ^= bytes[offset + i];
    }
	    
    return xor_crc;
}  


int App_Module_format_conver_uint8(General_t msg, uint8_t *buf)
{
	int index = 0;
	memcpy(buf,"CmdM:4", 6);			//CmdM:5
	index += 6;
	buf[index++] = sizeof(General_t);
	
	buf[index++] = BREAK_UINT32(msg.timer, 0);
	buf[index++] = BREAK_UINT32(msg.timer, 1);
	buf[index++] = BREAK_UINT32(msg.timer, 2);
	buf[index++] = BREAK_UINT32(msg.timer, 3);		

	buf[index++] = LO_UINT16(msg.tagid);
	buf[index++] = HI_UINT16(msg.tagid);	

	buf[index++] = LO_UINT16(msg.ancid);
	buf[index++] = HI_UINT16(msg.ancid);
	
	buf[index++] = msg.seq;	
	buf[index++] = msg.mask;		

	
	//��ǩԭʼ��������
	for(int i = 0; i < 8; i++)
	{
		buf[index++] = BREAK_UINT32(msg.rawrange[i], 0);
		buf[index++] = BREAK_UINT32(msg.rawrange[i], 1);
		buf[index++] = BREAK_UINT32(msg.rawrange[i], 2);
		buf[index++] = BREAK_UINT32(msg.rawrange[i], 3);			
	}

	//��ǩ�˲���������
	buf[index++] = msg.kalman_enable;
	for(int i = 0; i < 8; i++)
	{
		buf[index++] = BREAK_UINT32(msg.kalmanrange[i], 0);
		buf[index++] = BREAK_UINT32(msg.kalmanrange[i], 1);
		buf[index++] = BREAK_UINT32(msg.kalmanrange[i], 2);
		buf[index++] = BREAK_UINT32(msg.kalmanrange[i], 3);			
	}	
	buf[index++] = msg.pos_enable;
	buf[index++] = msg.dimen;
	buf[index++] = msg.ancmask;
	buf[index++] = msg.Coord_valid;
	//��ǩx������
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.x, 0);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.x, 1);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.x, 2);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.x, 3);			
	//��ǩy������
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.y, 0);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.y, 1);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.y, 2);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.y, 3);	
	//��ǩz������
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.z, 0);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.z, 1);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.z, 2);
	buf[index++] = BREAK_UINT32(*(int *)&msg.pos_loc.z, 3);		
	
	buf[index++] = get_Xor_CRC(buf, 6, sizeof(General_t));
	buf[index++] = '\r';
	buf[index++] = '\n';
	
	return (sizeof(General_t) + 6+2+1+1);
}

static uint8_t usart1_recv_msg_handler(uint8_t ch)
{
	uint8_t ret = FALSE;
	if((usart1_rx_sta & 0x8000)==0)//����δ���
	{
		if(usart1_rx_sta & 0x4000)//���յ���0x0d
		{
			if(ch!=0x0a)
				usart1_rx_sta=0;//���մ���,���¿�ʼ
			else{ 
				usart1_rx_sta|=0x8000;//��������� 
				ret = TRUE;
			}
		}
		else 									//��û�յ�0X0D
		{	
			if(ch==0x0d)
				usart1_rx_sta|=0x4000;
			else{
				usart1_rx_buf[ usart1_rx_sta & 0x3FFF] = ch ;
				usart1_rx_sta++;
				if(usart1_rx_sta > (USART1_RX_ISR_MAX_LEN-1) )
					usart1_rx_sta=0;//�������ݴ���,���¿�ʼ����	  
			}		 
		}
	} 

	return ret;
}

void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC5) != RESET)//DMAͨ��5������ɣ������յ�������USART1_RX[]��
	{
		if(usart1_recv_msg_handler(USART1_DMA_RX_BYTE))
		{
			//д�����
			Message msg;
			memset(&msg, 0, sizeof(msg));
			msg.flag = UART_FRAME_JS;				
			msg.len = usart1_rx_sta & 0x3FFF ;
			memcpy(msg.buf, usart1_rx_buf, msg.len);

			osal_Enqueue(&uart_queue, msg);

			//����ṹ��
			usart1_rx_sta = 0;
			memset(usart1_rx_buf, 0, sizeof(usart1_rx_buf));

		}
	}
	DMA_ClearITPendingBit(DMA1_IT_TC5); 	 //���DMAͨ��5������ɱ�־λ
}

void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4))      
	{    
		sys_uart_dma_buf.uart_dma_report = TRUE;
		DMA_ClearFlag(DMA1_FLAG_TC4); //���ȫ���жϱ�־
	}		
}

