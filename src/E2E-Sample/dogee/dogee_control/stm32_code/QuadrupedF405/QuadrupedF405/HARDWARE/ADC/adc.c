#include "adc.h"

void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能ADC1通道时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // 使能ADC1时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		 // GPIOA.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	 // 模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			 // 初始化

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);  // ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE); // 复位结束

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;					 // 独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // 两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;		 // DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;					 // 预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;						// 12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;								// 非扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;							// 关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // 禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						// 右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;									// 1个转换在规则序列中 也就是只转换规则序列1
	ADC_Init(ADC1, &ADC_InitStructure);											// ADC初始化
	ADC_Cmd(ADC1, ENABLE);														// 使能指定的ADC1
}

/**************************************************************************
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles); // ADC1,通道,480个周期,提高采样时间可以提高精度
	ADC_SoftwareStartConv(ADC1);									 // 使能指定的ADC1的软件转换启动功能
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
		;								 // 等待转换结束
	return ADC_GetConversionValue(ADC1); // 返回最近一次ADC1规则组的转换结果
}

/**************************************************************************
函数功能：读取电池电压
入口参数：无
返回  值：电池电压 单位MV
**************************************************************************/
int Get_battery_volt(void)
{
	int Volt;											// 电池电压
	Volt = Get_Adc(Battery_Ch) * 3.3 * 11 * 100 / 4096; // 电阻分压，具体根据原理图简单分析可以得到
	return Volt;
}
