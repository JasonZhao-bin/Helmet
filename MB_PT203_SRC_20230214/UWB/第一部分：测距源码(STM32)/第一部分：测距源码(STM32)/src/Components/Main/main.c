/**********************************************************************************
 * @file	main.c
 * @author	������� 
 * @phone	13566273308(�������-�ƹ�)
 * @version V3.0.0
 * @date	2021.10.16
 * @brief 	�ٸĲ��Դ��(ʹ�ùٷ���04.00.06) ����ƽ̨:UWB-S1(�ο�������ַ)
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

	/* ��RCC�Ĵ�����������ΪĬ��ֵ */
	RCC_DeInit();

	/* ���ⲿ����ʱ�Ӿ���HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* �ȴ��ⲿ����ʱ�Ӿ����� */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{
		/* ����FlashԤ�����幦��,ʱ�������ʹ�� */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* 48~72Mhz�Ƽ�LatencyΪ2 */
		FLASH_SetLatency(FLASH_Latency_2);

		/* ����AHBʱ�ӣ�72MHz HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* ���ø���APB2ʱ�ӣ�1��Ƶ72MHz PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* ���õ���APB1ʱ�ӣ�2��Ƶ36MHz PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);
		/*  ����ADCʱ�� ADCCLK = PCLK2/4 */
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);

		//����PLLʱ��Դ����Ƶϵ�� ����Ƶ��RCC_PLLSource_HSE_Div1 9��Ƶ��RCC_PLLMul_9
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		/* ��PLL */
		RCC_PLLCmd(ENABLE);
		/* �ȴ�PLL�ȶ����� */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

		/* ѡ��PLLʱ����Ϊʱ��Դ */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* �ȴ�ʱ��Դ�л��������ȶ�״̬ */
		while (RCC_GetSYSCLKSource() != 0x08){}
	}

	RCC_GetClocksFreq(&RCC_ClockFreq);
}

/*******************************************************************************
* ������  : init
* ����    : ��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
*******************************************************************************/
void init(void)
{
	
	SystemInit();	//Ӳ����ؿ�ɾ��

	RCC_Configuration_part();//Ӳ����ؿ�ɾ��

	Hal_Driver_Init();//Ӳ����ؿ�ɾ��

	App_Module_Init();
}

/*******************************************************************************
* ������  : main
* ����    : ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
********************************************************************************/
int main(void)
{
	init();	

	twr_task();

	for(;;)
	{	
	}
}

