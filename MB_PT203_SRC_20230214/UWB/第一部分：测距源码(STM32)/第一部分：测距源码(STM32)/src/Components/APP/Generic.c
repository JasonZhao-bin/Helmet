#include <string.h>
#include <stdio.h>
#include <math.h>

#include "Generic.h"
#include "cmd.h"
#include "instance.h"

#include "hal_timer.h"
#include "hal_flash.h"


System_uart_dma_t sys_uart_dma_buf ={
	.uart_dma_tx = 0,
	.uart_dma_report = TRUE,
	.uart_dma_index = 0
};


//全局结构体
System_Para_t sys_para = {
	.start_count = 0x0000
};

/*
 *	开机打印设备数据
 *
 */
void App_Module_sys_para_debug(void)
{
	_dbg_printf("*******************UWB Module 系统参数*******************\r\n");
	_dbg_printf("   UWB Module 编译时间 %s %s\n",__TIME__,__DATE__);	

	_dbg_printf("   UWB Module 软件版本号：%s\r\n"  ,uwb_software_ver);
	_dbg_printf("   UWB Module 硬件版本号：%s\r\n"  ,uwb_hardware_ver);

	_dbg_printf("   UWB Module 设备模式: %08X 基站容量:%d 标签容量:%d 最大标签容量:%d\r\n",sys_para.device_switch, MAX_ANCHOR_LIST_SIZE, sys_para.device_max_tag_list_size, MAX_TAG_LIST_SIZE);			

	_dbg_printf("   UWB Module 标志位：%02X\r\n"    ,sys_para.flag);	
	_dbg_printf("   UWB Module 开机次数：%d\r\n"  ,sys_para.start_count);
	_dbg_printf("   UWB Module 错误位1:%d, 错误位2:%d, 错误位3:%d, 错误位4:%d\r\n",
							sys_para.HardFault_error_bit,
							sys_para.MemManage_error_bit,
							sys_para.BusFault_error_bit,
							sys_para.UsageFault_error_bit);	


    _dbg_printf("   UWB Module 命令模式:%d \r\n",sys_para.cmd_mode);                            
    _dbg_printf("   UWB Module 距离参数:启动滤波:%d, 卡尔曼参数Q:%.2f, 卡尔曼参数R:%.2f, 校正系数a:%.4f, 校正系数b:%.2f, 天线延迟:%d\n",
                        sys_para.dist.is_kalman_filter,
                        sys_para.dist.kalman_Q,
                        sys_para.dist.kalman_R,
						sys_para.dist.para_a,
						sys_para.dist.para_b,
                        sys_para.dist.AntennaDelay);                            


    _dbg_printf("   UWB Module 定位参数:启动定位:%d, 定位维度:%d\r\n",sys_para.loc.is_loc, sys_para.loc.loc_dimen);    
    for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++)
    {
        _dbg_printf("              基站%d 坐标[x:%.2f, y:%.2f, z:%.2f] 是否参与定位:%d \n", 
            i,
            sys_para.loc.Anc_coord[i].x, 
            sys_para.loc.Anc_coord[i].y, 
            sys_para.loc.Anc_coord[i].z,
            sys_para.loc.Anc_coord[i].enable); 
    }


	_dbg_printf("\r\n");
	_dbg_printf("*******************UWB Module 系统参数*******************\r\n");
}


/*
 *	设备全局变量初始化
 *
 */
void App_Module_sys_para_Init(void)
{
	memset(&sys_para.flag, 0, sizeof(sys_para));
	
	sys_para.flag = 0xAAAA;
	sys_para.start_count = 0;

	sys_para.HardFault_error_bit	= 0;
	sys_para.MemManage_error_bit = 0 ;
	sys_para.BusFault_error_bit = 0;
	sys_para.UsageFault_error_bit = 0;		

	sys_para.device_switch = 0;

    //测距参数
  sys_para.dist.is_kalman_filter = true;
  sys_para.dist.kalman_Q = 0.018;
  sys_para.dist.kalman_R = 0.542;
	sys_para.dist.para_a = 1.0;
	sys_para.dist.para_b = 0.0;
  sys_para.dist.AntennaDelay = 16418;
    
    //定位参数
    sys_para.loc.is_loc = false;
    sys_para.loc.loc_dimen = Sys_Dimension_Invalid;
    for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++)
    {
        sys_para.loc.Anc_coord[i].enable = false;

        sys_para.loc.Anc_coord[i].x = 0.00; 
        sys_para.loc.Anc_coord[i].y = 0.00; 
        sys_para.loc.Anc_coord[i].z = 0.00;
	}
	
	sys_para.device_max_tag_list_size = 10;
    
	//sys_para.cmd_mode = Sys_Commond_Mode_Char_Test; 
	sys_para.cmd_mode = Sys_Commond_Mode_Hex;
}

  
/*
 *	应用层 写入非易失性数据(主存储区)
 *
 */
void App_Module_Sys_Write_NVM(void)
{	
	//写入主存储区
	while(HalWrite_Flash(PAGE62_ADDR, &sys_para.flag, sizeof(sys_para)/4) == 0)
	{
		HalDelay_nMs(100);
	}
}


/*
 *	应用层 读取非易失性数据
 *
 */
void App_Module_Sys_Read_NVM(void)
{
	HalRead_Flash(PAGE62_ADDR, &sys_para.flag, sizeof(sys_para)/4);     //存储区读取
}

/*
 *	读取flash中配置数据
 *
 */
void App_Module_sys_para_read()
{
    bool is_back2factory = false;

	App_Module_Sys_Read_NVM();
	
	
	if(sys_para.flag != 0xAAAA)
	{
		is_back2factory = true;     
		_dbg_printf("设备首次上电\n");
	}
	else
	{
		sys_para.start_count +=1;     
		App_Module_Sys_Write_NVM();		
	}
	
    /*恢复出厂模式*/
    if(is_back2factory == true)
    {
 		App_Module_sys_para_Init();			
		App_Module_Sys_Write_NVM();			
		NVIC_SystemReset();      
    }
}

/*
 *	应用层 使能操作
 *
 */
void App_Module_Init(void)
{	
	HalDelay_nMs(500);

    //串口队列使能
	App_Module_CMD_Queue_Init();

    //配置读取(flash)
	App_Module_sys_para_read();

    //配置打印
	App_Module_sys_para_debug();
}



/*
 *	应用层 事件模式
 *
 */
void App_Module_Sys_Work_Mode_Event(Sys_Work_Mode Mode)
{
	switch(Mode)
	{
		case Sys_Operate_Mode_CFG_ING:
				App_Module_Sys_Deal_UART_USB_CMD_Event(TRUE);						
			break;
		case Sys_Operate_Mode_WORK_DONE:
				App_Module_Sys_Deal_UART_USB_CMD_Event(FALSE);		
			break;	

		default:
			break;
	}
}


/*
 *	应用层 串口事件
 *
 */
void App_Module_Sys_Deal_UART_USB_CMD_Event(bool flag)
{
	//App_Module_Process_USB_CMD();
	App_Module_Process_USART_CMD();
	if(flag == TRUE)				
		port_tx_msg("Please Send AT Command...\r\n", strlen("Please Send AT Command...\r\n")); 				

	//如果缓存区仍有数据未发送，则发送
	if(sys_uart_dma_buf.uart_dma_index != 0)
	{
		port_tx_msg(NULL, 0);
	}
}

/*
 *	应用层 LED事件
 *
 */
void App_Modelu_Sys_Deal_IO_LED_Event(uint16_t count)
{
	for(int i = 0; i < HAL_LED_ALL; i++)
	{
		if(hal_led[i].mode != HAL_LED_MODE_TOGGLE)
			continue;
		
		if( count % hal_led[i].period  == 0)
		{ 
			if((count / hal_led[i].period) % 2)
				HalLedSet (i, HAL_LED_MODE_ON);
			else
				HalLedSet (i, HAL_LED_MODE_OFF);
		} 		
	} 	
}

/*
 *	应用层 LED模式配置
 *
 */
void App_Module_Sys_IO_Led_Mode_Set(Sys_Work_Mode Mode)
{
	static int val_mode = Sys_Operate_Mode_INVAILD;
	
	if(Mode == val_mode)
	{
		return;
	}
	else
	{	
		val_mode = Mode;
	}
	
	switch(Mode)
	{
		/*LED1*/
		case Sys_Operate_Mode_CFG_ING:	
				HalLed_Mode_Set(HAL_LED1, HAL_LED_MODE_TOGGLE, Sys_mode_led_blink_slow_time);		
			break;
				
		case Sys_Operate_Mode_WORK_DONE:		
				HalLed_Mode_Set(HAL_LED1, HAL_LED_MODE_TOGGLE, Sys_mode_led_blink_fast_time); 	
			break;
		default:
			break;
	}
}

/*
 *	应用层 UWB模式一览
 *
 */
void App_Module_UWB_Mode_Display(uint16_t s1switch)
{
    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));
    strcat(buf, "UWB Module   ");

    sprintf(buf,"ID:%d ",(s1switch & 0x0fe) >> 1);      // 0<ID<128

    if((s1switch & 0x100) == 0)
        strcat(buf, "角色:标签 ");
    else
        strcat(buf, "角色:基站 ");

    if((s1switch & 0x200) == 0)
        strcat(buf, "信道:2 ");
    else
        strcat(buf, "信道:5 ");

    if((s1switch & 0x400) == 0)
        strcat(buf, "速率:110k\r\n");
    else
        strcat(buf, "速率：6.8M\r\n");

    port_tx_msg(buf, strlen(buf));
}


