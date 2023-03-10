/**********************************************************************************
 * @file	main.c
 * @author	久凌电子 
 * @phone	13566273308(久凌电子-唐工)
 * @version V3.0.0
 * @date	2021.10.16
 * @brief 	官改测距源码(使用官方库04.00.06) 运行平台:UWB-S1(参考下列网址)
 * @store	https://item.taobao.com/item.htm?spm=a1z10.5-c.w4002-23565193320.10.6e6c3f96tF7wds&id=572212584700
**********************************************************************************/

#include "stm32f10x.h"

#include "Generic.h"
#include "twr_task.h"
#include "OSAL.h"

#include "hal_drivers.h"

/*******************************************************************************
*******************************************************************************/
void RCC_Configuration_part(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_ClocksTypeDef RCC_ClockFreq;

	/* 将RCC寄存器重新设置为默认值 */
	RCC_DeInit();

	/* 打开外部高速时钟晶振HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* 等待外部高速时钟晶振工作 */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{
		/* 开启Flash预读缓冲功能,时钟起振后使用 */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* 48~72Mhz推荐Latency为2 */
		FLASH_SetLatency(FLASH_Latency_2);

		/* 设置AHB时钟，72MHz HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* 设置告诉APB2时钟，1分频72MHz PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* 设置低速APB1时钟，2分频36MHz PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);
		/*  设置ADC时钟 ADCCLK = PCLK2/4 */
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);

		//设置PLL时钟源及倍频系数 不分频：RCC_PLLSource_HSE_Div1 9倍频：RCC_PLLMul_9
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		/* 打开PLL */
		RCC_PLLCmd(ENABLE);
		/* 等待PLL稳定工作 */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

		/* 选择PLL时钟作为时钟源 */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* 等待时钟源切换，进入稳定状态 */
		while (RCC_GetSYSCLKSource() != 0x08){}
	}

	RCC_GetClocksFreq(&RCC_ClockFreq);
}

/*******************************************************************************
* 函数名  : init
* 描述    : 初始化函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
*******************************************************************************/
void init(void)
{
	
	SystemInit();	//硬件相关可删除

	RCC_Configuration_part();//硬件相关可删除

	Hal_Driver_Init();//硬件相关可删除

	App_Module_Init();
}

/*******************************************************************************
* 函数名  : main
* 描述    : 主函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
********************************************************************************/
int main(void)
{
	init();	

	twr_task();

	for(;;)
	{	
	}
}

