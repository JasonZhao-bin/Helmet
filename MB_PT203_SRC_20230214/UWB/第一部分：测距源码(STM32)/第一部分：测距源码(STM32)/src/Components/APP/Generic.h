#ifndef __APP_GENERIC_H
#define __APP_GENERIC_H

#ifdef __cplusplus
extern "C"
{
#endif
	
/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/	
#include <stdbool.h>

#include "Generic_CMD.h"
#include "OSAL_Comdef.h"

#include "hal_led.h"


	
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/
#define IS_FISRT_POEWRON(x) ((x) != 0xAAAA)

//����汾
#define uwb_software_ver "v03_00_004"
#ifdef HW_PA
	#define uwb_hardware_ver "v01_00_000(PA)"
#else
	#define uwb_hardware_ver "v01_00_000(xPA)"
#endif
								
/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
typedef struct
{
    bool  is_kalman_filter; //�Ƿ񿨶����˲�����
    float kalman_Q;         //�������˲�����ֵQ
    float kalman_R;         //�������˲�����ֵR
    
	float para_a;			//У��ϵ��a
	float para_b;			//У��ϵ��b
	
    int   AntennaDelay;     //����/���������ӳٲ���
}dist_t;

typedef struct
{
    bool    is_loc;         //�Ƿ�λ
    uint8_t loc_dimen;      //��λά��

    struct{
        bool enable;         //��վ�Ƿ���붨λ
        float x;
        float y;
        float z;
    }Anc_coord[8];
}loc_t;


typedef struct
{
	uint32_t flag;
	uint32_t start_count;

	uint32_t HardFault_error_bit;
	uint32_t MemManage_error_bit;
	uint32_t BusFault_error_bit;
	uint32_t UsageFault_error_bit;

	uint32_t device_switch;	

    dist_t dist;             			//������
    loc_t  loc;              			//��λ����
    uint8_t device_max_tag_list_size;	//����ǩ����
    uint8_t cmd_mode;      				//����ģʽѡ��0:�ַ�����ʽ 1:Hex��ʽ    
	uint16_t reserver;					//����
} System_Para_t; 		

typedef struct
{
	uint8_t uart_dma_tx[2048];
	uint8_t uart_dma_tx_tmp[2048];	
	int uart_dma_report;
	int uart_dma_index;
}System_uart_dma_t;

enum Sys_mode_led_blink
{
	Sys_mode_led_blink_invalid 		= 0,
		
	/*��˸һ������ */
	Sys_mode_led_blink_fast_time 	= 2,	//0.2s
	Sys_mode_led_blink_slow_time 	= 10,	//1s
};

typedef enum
{
	/*����ģʽ ��*/
	Sys_Operate_Mode_INVAILD 		= 0x0000,		//��Ч
	Sys_Operate_Mode_CFG_ING 		= 0x0001,		//������
	Sys_Operate_Mode_WORK_DONE		= 0x0002,		//������	
}Sys_Work_Mode;

typedef enum
{
    Sys_Commond_Mode_Char_RawDist = 0,				//Json��ʽ���ԭʼ����ֵ
    Sys_Commond_Mode_Char_KalmanDist =1,			//Json��ʽ��������������˲�ֵ
    Sys_Commond_Mode_Char_Pos =2,					//Json��ʽ�����λ����
    Sys_Commond_Mode_Char_Sign = 3,					//Json��ʽ����ź�ǿ��ֵ
    Sys_Commond_Mode_Hex  = 4,						//������λ��(v3.0.0)				
    Sys_Commond_Mode_Char_Test  = 5					//������һ�汾��λ��(v2.0.0)
}Sys_Commond_Mode;


enum Sys_Dimension
{
	Sys_Dimension_Invalid = 0,
	Sys_Dimension_Two = 1,
	Sys_Dimension_Three = 2
};

/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/
extern System_Para_t sys_para;
extern System_uart_dma_t sys_uart_dma_buf;




/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	
extern void App_Module_Init(void);	

extern void App_Module_Sys_Work_Mode_Event(Sys_Work_Mode Mode);

extern void App_Module_Sys_Deal_UART_USB_CMD_Event(bool flag);

extern void App_Modelu_Sys_Deal_IO_LED_Event(uint16_t count);

extern void App_Module_Sys_IO_Led_Mode_Set(Sys_Work_Mode Mode);

extern void App_Module_Sys_Write_NVM(void);

extern void App_Module_Sys_Read_NVM(void);

/*��ʾUWB�豸ģʽ*/
extern void App_Module_UWB_Mode_Display(uint16_t s1switch);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif//__APP_GENERIC_H

