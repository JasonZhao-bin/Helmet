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

//软件版本
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
    bool  is_kalman_filter; //是否卡尔曼滤波距离
    float kalman_Q;         //卡尔曼滤波距离值Q
    float kalman_R;         //卡尔曼滤波距离值R
    
	float para_a;			//校正系数a
	float para_b;			//校正系数b
	
    int   AntennaDelay;     //发送/接受天线延迟参数
}dist_t;

typedef struct
{
    bool    is_loc;         //是否定位
    uint8_t loc_dimen;      //定位维度

    struct{
        bool enable;         //基站是否参与定位
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

    dist_t dist;             			//测距参数
    loc_t  loc;              			//定位参数
    uint8_t device_max_tag_list_size;	//最大标签数量
    uint8_t cmd_mode;      				//命令模式选择0:字符串格式 1:Hex格式    
	uint16_t reserver;					//保留
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
		
	/*闪烁一次周期 */
	Sys_mode_led_blink_fast_time 	= 2,	//0.2s
	Sys_mode_led_blink_slow_time 	= 10,	//1s
};

typedef enum
{
	/*工作模式 主*/
	Sys_Operate_Mode_INVAILD 		= 0x0000,		//无效
	Sys_Operate_Mode_CFG_ING 		= 0x0001,		//配置中
	Sys_Operate_Mode_WORK_DONE		= 0x0002,		//工作中	
}Sys_Work_Mode;

typedef enum
{
    Sys_Commond_Mode_Char_RawDist = 0,				//Json格式输出原始距离值
    Sys_Commond_Mode_Char_KalmanDist =1,			//Json格式输出卡尔曼距离滤波值
    Sys_Commond_Mode_Char_Pos =2,					//Json格式输出定位数据
    Sys_Commond_Mode_Char_Sign = 3,					//Json格式输出信号强度值
    Sys_Commond_Mode_Hex  = 4,						//适配上位机(v3.0.0)				
    Sys_Commond_Mode_Char_Test  = 5					//适配上一版本上位机(v2.0.0)
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

/*显示UWB设备模式*/
extern void App_Module_UWB_Mode_Display(uint16_t s1switch);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif
#endif//__APP_GENERIC_H

