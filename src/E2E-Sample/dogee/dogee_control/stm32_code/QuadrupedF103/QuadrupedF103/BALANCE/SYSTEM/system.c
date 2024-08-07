#include "system.h"

unsigned char temp_show;

void systemInit(void)
{
	JTAG_Set(JTAG_SWD_DISABLE); //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);		//=====��SWD�ӿ� �������������SWD�ӿڵ���

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // ����ϵͳ�ж����ȼ�����4
	delay_init();									// ��ʼ����ʱ����
	LED_Init();										// ��ʼ���� LED ���ӵ�Ӳ���ӿ�
	OLED_Init();									// OLED��ʼ��
	KEY_Init();										// ������ʼ��
	CAN1_Mode_Init(1, 2, 3, 6, 0);					//=====CAN��ʼ��
	Encoder_Init_TIM2();							//=====�������ӿ�A��ʼ��
	Encoder_Init_TIM3();							//=====�������ӿ�B��ʼ��
	Encoder_Init_TIM4();							//=====�������ӿ�C��ʼ��
	Encoder_Init_TIM5();							//=====�������ӿ�D��ʼ��
	MiniBalance_Motor_Init();						// ��ʼ�����Ƶ������ת����
	MiniBalance_PWM_Init(7199, 0);					// ��ʼ��PWM 10KHZ�������������
	IIC_Init();										// IIC��ʼ��
	Flag_Mpu6050 = MPU6050_Init();					// MPU6050��ʼ��
}
