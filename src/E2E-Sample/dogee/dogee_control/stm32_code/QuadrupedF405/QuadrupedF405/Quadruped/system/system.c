#include "system.h"

/*��ر�־λ*/
uint8_t armingFlags = 0;													// ����״̬��־λ
u8 APP_ON_Flag = 0, PS2_ON_Flag = 0, USART_ON_FLAG = 0, Remote_ON_Flag = 0; // �������ģʽ��־λ
u8 APP_START_STOP_FLAG = 1;													// APP����ģʽʱ�õ��ı�־λ��1�����㿪����0������ػ�
u8 start_up_15_second = 0;													// �����ϵ翪��15��ȴ���־λ
u8 Remote_key_control_flag = 0, START_COUNT = 0;							// �û�����/��ģң�� �������㿪���͹ػ��ı�־λ /  �û�����/��ģң�� �������㿪�������õ��ı�־λ
u8 RUN_Control = 0;															// Ԥ�˶�״̬���õ��ı�־λ
u8 Flag_Pose, Flag_Direction = 0, Turn_Flag, PID_Send = 0;					// ����ң����صı�־λ
u8 USART_KEY, APP_KEY, Remote_KEY;											// app�����ڡ���ģң�� ����ģʽ�£�ģ���ֱ��ļ�ֵ����

/*�ײ�Ӳ����ʼ��*/
void systemInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // ����ϵͳ�ж����ȼ�����4
	delay_init(168);								//=====��ʼ����ʱ����
	uart1_init(115200);								//=====����1��ʼ������λ��ͨ��
	usart2_init(9600);								//=====����2��ʼ�� �������ӣ���APPͨ��
	usart3_init(115200);							//=====����3��ʼ����ROSͨ��
	LED_Init();										//=====��ʼ��LED�˿�
	OLED_Init();									//=====��ʾ����ʼ��
	Adc_Init();										//=====�ɼ���ص�ѹ���ų�ʼ��
	KEY_Init();										//=====�û�������ʼ��

	IIC_Init();			  //=====IIC��ʼ��
	MPU6050_initialize(); //=====MPU6050��ʼ��
	DMP_Init();			  //=====��ʼ��DMP

	Bee_Init();			 //=====���������ų�ʼ��
	Encoder_Init_TIM2(); //=====��ʼ��������A
	Encoder_Init_TIM3(); //=====��ʼ��������B
	Encoder_Init_TIM4(); //=====��ʼ��������C
	Encoder_Init_TIM5(); //=====��ʼ��������D

	MiniBalance_PWM_Init(8399, 0); //=====��ʼ��PWM 10KHZ�������������
	MiniBalance_Motor_Init();	   //=====��ʼIO�������������

	PS2_Init();	   //=====ps2�����˿ڳ�ʼ��
	PS2_SetInit(); //=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�

	CAN1_Mode_Init(1, 7, 6, 3, CAN_Mode_Normal); //=====CAN��ʼ��   ĩλ0=��ͨģʽ��1=�ػ�ģʽ

	TIM1_Cap_Init(0XFFFF, 72 - 1); //=====��ģң�س�ʼ��

	allPidInit();	   // pid ������ʼ��
	LegPositionInit(); // ������λ�ó�ʼ�����ص��ϵ��ʼ��״̬

	// ������״̬��־λ��0
	DISABLE_ARMING_FLAG(LOCKED);
	DISABLE_ARMING_FLAG(READY);
	DISABLE_ARMING_FLAG(RUNNING);
	DISABLE_ARMING_FLAG(STOP);
	DISABLE_ARMING_FLAG(SHUTDWON);
}
