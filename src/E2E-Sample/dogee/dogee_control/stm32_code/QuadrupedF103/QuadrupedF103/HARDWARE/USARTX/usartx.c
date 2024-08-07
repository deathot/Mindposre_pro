#include "usartx.h"
SEND_DATA Send_Data;	   // 发送数据的
RECEIVE_DATA Receive_Data; // 接收数据的
extern int Time_count;
/**************************************************************************
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while ((USART2->SR & 0x40) == 0)
		;
}
/**************************************************************************
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能USART时钟
														   // USART_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	  // PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器
	// USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART2, &USART_InitStructure);										// 初始化串口2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);									// 开启串口接受中断
	USART_Cmd(USART2, ENABLE);														// 使能串口2
}
/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
// int USART2_IRQHandler(void)
//{
//	int Usart_Receive;
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
//   return 0;
// }
/**************************************************************************
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while ((USART3->SR & 0x40) == 0)
		;
}
/**************************************************************************
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while ((USART1->SR & 0x40) == 0)
		;
}
/**************************************************************************
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // 需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  // 使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); // 引脚重映射
	// USART_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // C10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	  // PC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器
	// USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART3, &USART_InitStructure);										// 初始化串口3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);									// 开启串口接受中断
	USART_Cmd(USART3, ENABLE);														// 使能串口3
}
/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
// int USART3_IRQHandler(void)
//{

//	u8 Usart_Receive;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //判断是否接收到数据

//  return 0;
//}
/**************************************************************************
 *  函数功能：串口1初始化
 *
 *  入口参数：无
 *
 *  返 回 值：无
 **************************************************************************/
void uart1_init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 使能USART时钟

	// USART_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	  // PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器
	// USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART1, &USART_InitStructure);										// 初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									// 开启串口接受中断
	USART_Cmd(USART1, ENABLE);														// 使能串口1
}
/**************************************************************************
 *  函数功能：串口1接收中断
 *
 *  入口参数：无
 *
 *  返 回 值：无
 **************************************************************************/
// int USART1_IRQHandler(void)
//{
//	u8 Usart_Receive;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收到数据
//		Usart_Receive = USART_ReceiveData(USART1);//读取数据

//		return 0;
//}

/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：
返回  值：检验位
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode)
{
	unsigned char check_sum = 0, k;
	// 发送数据的校验
	if (Mode == 1)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Send_Data.buffer[k];
		}
	// 接收数据的校验
	if (Mode == 0)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Receive_Data.buffer[k];
		}
	return check_sum;
}
