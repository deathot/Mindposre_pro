#include "usartx.h"

u8 USART_Send_Data[24]; // 串口发送数据的数组
u8 Uart3_Receive;		// 串口接收数组
u8 Urxbuf[8];			// 串口接收数组
u8 Receive_Data[11];
volatile unsigned char rx_buffer[100]; // 接收数据缓冲区
volatile unsigned char rx_wr_index;	   // 缓冲写指针
volatile unsigned char RC_Flag;		   // 接收状态标志字节

void sent_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ)); // 100Hz运行频率
		usart_data_transition();						 // 发送数据前先将数据拆分封装好
		usart3_sent_data();								 // 串口3发送数据
	}
}

/**************************实现函数**********************************************
 *功    能:		usart发送一个字节
 *********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while ((USART3->SR & 0x40) == 0)
		;
}

/**************************************************************************
函数功能：串口3初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void usart3_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // 使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART时钟

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3); // GPIO.C10复用为串口3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3); // GPIO.C11复用为串口3

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PC10.11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // 高速50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 // 上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 // 初始化

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器

	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式

	USART_Init(USART3, &USART_InitStructure);	   // 初始化串口
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启串口接受中断
	USART_Cmd(USART3, ENABLE);					   // 使能串口
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{
	u8 com_data;
	// u8 Flag_Move=1;
	short between_x, between_z, between_pose_x, between_pose_y;
	static u8 Count = 0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		// USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		com_data = USART_ReceiveData(USART3); // 读取数据
		if (start_up_15_second == 0)
		{
			USART_ClearFlag(USART3, USART_IT_RXNE);
			return 0;
		}
		Receive_Data[Count] = com_data;
		if (USART_ON_FLAG == 0)
			PS2_ON_Flag = 0, APP_ON_Flag = 0, USART_ON_FLAG = 1, Remote_ON_Flag = 0, Remote_count = 0; // 进入串口3中断，强行进入ROS模式，优先级最高//其他模式关停
		if (com_data == FRAME_HEADER || Count > 0)
			Count++;
		else
			Count = 0;
		if (Count == 11) // 验证数据包的长度
		{
			Count = 0;							// 重新开始接收
			if (Receive_Data[10] == FRAME_TAIL) // 验证数据包的尾部校验信息
			{
				if (Receive_Data[9] == Check_Sum(9, 0)) // 数据校验位计算，模式0是发送数据校验
				{
					if (Check_Sum(9, 0) == FRAME_HEADER) // avoid the influence of the pose data
					{
						// bee_count=3;
						// pose_x=0;
						// pose_y=0;
						// move_x=0;
						// move_z=0;
						// control_transition(100);
						while (fabs(smooth_pose_x) > 0.001 || fabs(smooth_pose_y) > 0.001)
						{
							Smooth_pose(0, 0);
						}
						USART_KEY = 14;
						UpdataFlags(USART_KEY);
						// pose_x=0;
						// pose_y=0;
						// Flag_Move=0;
						USART_ClearFlag(USART3, USART_IT_RXNE);
						return 1;
					}
					else if (Receive_Data[5] == 0 && Receive_Data[6] == 0 && Receive_Data[7] == 0 && Receive_Data[8] == 0)
					{
						USART_KEY = 15;
						UpdataFlags(USART_KEY);
						between_x = (short)(Receive_Data[1] << 8) + Receive_Data[2];
						between_z = (short)(Receive_Data[3] << 8) + Receive_Data[4];

						// pose_x=0;
						// pose_y=0;
						// control_transition(100);
						move_x = (int)between_x * 45 * 0.01;
						move_y = 0;
						move_z = (int)between_z * 25 * 0.01;
						// control_transition(100);
						USART_ClearFlag(USART3, USART_IT_RXNE);
						return 0;
					}
					else
					{
						// control_transition(100);
						USART_KEY = 14;
						UpdataFlags(USART_KEY);
						USART_KEY = 15;
						UpdataFlags(USART_KEY);
						between_pose_x = (short)(Receive_Data[5] << 8) + Receive_Data[6];
						between_pose_y = (short)(Receive_Data[7] << 8) + Receive_Data[8];
						// move_x=0;
						// move_z=0;
						pose_x = (int)between_pose_x;
						pose_y = (int)between_pose_y;
						USART_ClearFlag(USART3, USART_IT_RXNE);
						return 0;
					}

					// if(Flag_Move==1)

					// else{
					// pose_x=0;
					// pose_y=0;
					//}
					// control_transition(100);
				}
			}
		}
	}
	// USART_ClearFlag(USART3, USART_IT_RXNE);
	return 0;
}
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
			check_sum = check_sum ^ USART_Send_Data[k];
		}
	// 接收数据的校验
	if (Mode == 0)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Receive_Data[k];
		}
	return check_sum;
}

/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void usart_data_transition(void)
{
	short power_buff, B_GYRO[3], B_ACCEL[3], between;
	power_buff = Power_Voltage;
	memcpy(B_GYRO, slave_gyro, sizeof(slave_gyro));
	memcpy(B_ACCEL, slave_accel, sizeof(slave_gyro));

	// 这里是做一个角度矫正
	between = -B_GYRO[0];
	B_GYRO[0] = B_GYRO[1];
	B_GYRO[1] = between;
	between = -B_ACCEL[0];
	B_ACCEL[0] = B_ACCEL[1];
	B_ACCEL[1] = between;

	USART_Send_Data[0] = FRAME_HEADER; // 固定帧头
	USART_Send_Data[1] = 0;
	// 机器人三轴速度

	USART_Send_Data[2] = 0;
	USART_Send_Data[3] = 0;
	USART_Send_Data[4] = 0;
	USART_Send_Data[5] = 0;
	USART_Send_Data[6] = 0;
	USART_Send_Data[7] = 0;
	// 三轴角速度
	USART_Send_Data[8] = B_ACCEL[0] >> 8;
	USART_Send_Data[9] = B_ACCEL[0];
	USART_Send_Data[10] = B_ACCEL[1] >> 8;
	USART_Send_Data[11] = B_ACCEL[1];
	USART_Send_Data[12] = B_ACCEL[2] >> 8;
	USART_Send_Data[13] = B_ACCEL[2];
	// 三轴加速度
	USART_Send_Data[14] = B_GYRO[0] >> 8;
	USART_Send_Data[15] = B_GYRO[0];
	USART_Send_Data[16] = B_GYRO[1] >> 8;
	USART_Send_Data[17] = B_GYRO[1];
	USART_Send_Data[18] = B_GYRO[2] >> 8;
	USART_Send_Data[19] = B_GYRO[2];

	USART_Send_Data[20] = power_buff >> 8; // 电池电压高8位
	USART_Send_Data[21] = power_buff;	   // 电池电压低8位

	USART_Send_Data[22] = Check_Sum(22, 1); // 计算校验位
	USART_Send_Data[23] = FRAME_TAIL;		// 固定帧尾
}
/**************************************************************************
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
void usart3_sent_data(void)
{
	unsigned char i = 0;
	for (i = 0; i < 24; i++)
	{
		usart3_send(USART_Send_Data[i]);
	}
}
