#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
// #include "usartx.h"
int Flag_Mpu6050;						   // ���MPU6050�Ƿ����������ı�־λ
int Deviation_Count;					   // ���Ư�Ƽ���
short gyro[3], accel[3], sensors;		   // ������ٶ�����������
short Deviation_gyro[3], Original_gyro[3]; // �����Ǿ��� ��ԭʼ����
/**************************************************************************
�������ܣ�MPU6050����
��ڲ�������
����  ֵ����
**************************************************************************/
void MPU6050_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); // ��������100Hz��Ƶ������
		if (Deviation_Count < CONTROL_DELAY)			  // ����ǰ����ȡ���������
		{
			Deviation_Count++;
			memcpy(Deviation_gyro, gyro, sizeof(gyro));
		}
		//================��������������================
		MPU6050_Get_Accelerometer(accel); // ͨ��IIC��ȡ���ٶ���Ϣ
		MPU6050_Get_Gyroscope(gyro);	  // ͨ��IIC��ȡ���ٶ���Ϣ
	}
}
/**************************************************************************
�������ܣ�MPU6050��ʼ��
��ڲ�������
����  ֵ��1����0���� ����һ�㶼��������ַ����ģ�����IIC���Ų��Ե�
**************************************************************************/
unsigned char MPU6050_Init(void)
{
	u8 res;
	MPU6050_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // ��λMPU6050
	delay_ms(200);								 // ��ʾ�ȴ��ȶ�
	MPU6050_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // ����MPU6050
	MPU6050_Set_Gyro_Fsr(1);					 // �����Ǵ�����,���̡�500dps
	MPU6050_Set_Accel_Fsr(0);					 // ���ٶȴ�����,���̡�2g
	MPU6050_Set_Rate(50);						 // ���ò�����50Hz
	MPU6050_Write_Byte(MPU_INT_EN_REG, 0X00);	 // �ر������ж�
	MPU6050_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C��ģʽ�ر�
	MPU6050_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 // �ر�FIFO
	MPU6050_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT���ŵ͵�ƽ��Ч
	res = MPU6050_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) // ����ID��ȷȡ����AD����
	{
		MPU6050_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // ����CLKSEL,PLL X��Ϊ�ο�
		MPU6050_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // ���ٶ��������Ƕ�����
		MPU6050_Set_Rate(50);						 // ���ò�����Ϊ50Hz
	}
	else
		return 1;
	return 0;
}
/**************************************************************************
�������ܣ����������Ǵ����������̷�Χ
��ڲ�����fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
����  ֵ��0,���óɹ�
**************************************************************************/
unsigned char MPU6050_Set_Gyro_Fsr(u8 fsr)
{
	return MPU6050_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // ���������������̷�Χ
}
/**************************************************************************
�������ܣ����ü��ٶȼ������̷�Χ
��ڲ�����fsr:0,��2g;1,��4g;2,��8g;3,��16g
����  ֵ��0,���óɹ�
**************************************************************************/
unsigned char MPU6050_Set_Accel_Fsr(u8 fsr)
{
	return MPU6050_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // ���ü��ٶȴ����������̷�Χ
}
/**************************************************************************
�������ܣ�����MPU6050оƬ�����ֵ�ͨ�˲���
��ڲ�����lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
����  ֵ��0,���óɹ�
**************************************************************************/
unsigned char MPU6050_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU6050_Write_Byte(MPU_CFG_REG, data); // �������ֵ�ͨ�˲���
}
/**************************************************************************
�������ܣ�����MPU6050оƬ�Ĳ�����
��ڲ�����rate:4~1000(Hz)
����  ֵ��0,���óɹ�
**************************************************************************/
unsigned char MPU6050_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU6050_Write_Byte(MPU_SAMPLE_RATE_REG, data); // �������ֵ�ͨ�˲���
	return MPU6050_Set_LPF(rate / 2);					  // �Զ�����LPFΪ�����ʵ�һ��
}
/**************************************************************************
�������ܣ��ɼ������ǵ����� �������Ư�ƴ��������LED״̬
��ڲ�������ŵĵ�ַ
����  ֵ��0,�ɼ��ɹ�
**************************************************************************/
unsigned char MPU6050_Get_Gyroscope(short *gyro)
{
	u8 buf[6], res;
	res = MPU6050_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		if (Deviation_Count < CONTROL_DELAY) // ����ǰ ��ȡ���������
		{
			gyro[0] = (((u16)buf[0] << 8) | buf[1]);
			gyro[1] = (((u16)buf[2] << 8) | buf[3]);
			gyro[2] = (((u16)buf[4] << 8) | buf[5]);
			Led_Count = 1; // LED����
			// Flag_Stop=1;//�رյ��
		}
		else
		{
			// if(Deviation_Count==CONTROL_DELAY)Flag_Stop=0;//ʹ�ܵ��
			Led_Count = 300; // ����ȡ��֮��LED��˸
			// Deviation_Count	=CONTROL_DELAY;	 //�̶�����ֵ
			Original_gyro[0] = (((u16)buf[0] << 8) | buf[1]); // ����ԭʼ�������ڰ����޸����
			Original_gyro[1] = (((u16)buf[2] << 8) | buf[3]);
			Original_gyro[2] = (((u16)buf[4] << 8) | buf[5]);

			gyro[0] = Original_gyro[0] - Deviation_gyro[0]; // ���մ�ȥ�����Ư�Ƶ�����
			gyro[1] = Original_gyro[1] - Deviation_gyro[1];
			gyro[2] = Original_gyro[2] - Deviation_gyro[2];
		}
	}
	return res;
}
/**************************************************************************
�������ܣ��ɼ����ٶȼƵ����ݣ�ֱ��ʹ��
��ڲ�������ŵĵ�ַ
����  ֵ��0,�ɼ��ɹ�
**************************************************************************/
unsigned char MPU6050_Get_Accelerometer(short *accel)
{
	u8 buf[6], res;
	res = MPU6050_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		accel[0] = ((u16)buf[0] << 8) | buf[1];
		accel[1] = ((u16)buf[2] << 8) | buf[3];
		accel[2] = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
}
/**************************************************************************
�������ܣ�IICͨѶ����MPU6050��д����
��ڲ�����addr:������ַ reg:�Ĵ�����ַ len:д�볤�� buf:������
����  ֵ��0����
**************************************************************************/
unsigned char MPU6050_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0); // ����������ַ+д����
	if (IIC_Wait_Ack())				// �ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg); // д�Ĵ�����ַ
	IIC_Wait_Ack();		// �ȴ�Ӧ��
	for (i = 0; i < len; i++)
	{
		IIC_Send_Byte(buf[i]); // ��������
		if (IIC_Wait_Ack())	   // �ȴ�ACK
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}
/**************************************************************************
�������ܣ�IICͨѶ����MPU6050�Ķ�����
��ڲ�����addr:������ַ reg:�Ĵ�����ַ len:д�볤�� buf:������
����  ֵ��0����
**************************************************************************/
unsigned char MPU6050_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0); // ����������ַ+д����
	if (IIC_Wait_Ack())				// �ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg); // д�Ĵ�����ַ
	IIC_Wait_Ack();		// �ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1); // ����������ַ+������
	IIC_Wait_Ack();					// �ȴ�Ӧ��
	while (len)
	{
		if (len == 1)
			*buf = IIC_Read_Byte(0); // ������,����nACK
		else
			*buf = IIC_Read_Byte(1); // ������,����ACK
		len--;
		buf++;
	}
	IIC_Stop(); // ����һ��ֹͣ����
	return 0;
}
/**************************************************************************
�������ܣ�IICͨѶ����MPU6050��дһ���ֽڲ���
��ڲ�����reg:�Ĵ�����ַ data:����
����  ֵ��0����
**************************************************************************/
unsigned char MPU6050_Write_Byte(u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0); // ����������ַ+д����
	if (IIC_Wait_Ack())					// �ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	 // д�Ĵ�����ַ
	IIC_Wait_Ack();		 // �ȴ�Ӧ��
	IIC_Send_Byte(data); // ��������
	if (IIC_Wait_Ack())	 // �ȴ�ACK
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}
/**************************************************************************
�������ܣ�IICͨѶ����MPU6050�Ķ�һ���ֽڲ���
��ڲ�����reg:�Ĵ�����ַ
����  ֵ��0����
**************************************************************************/
unsigned char MPU6050_Read_Byte(u8 reg)
{
	u8 res;
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0); // ����������ַ+д����
	IIC_Wait_Ack();						// �ȴ�Ӧ��
	IIC_Send_Byte(reg);					// д�Ĵ�����ַ
	IIC_Wait_Ack();						// �ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 1); // ����������ַ+������
	IIC_Wait_Ack();						// �ȴ�Ӧ��
	res = IIC_Read_Byte(0);				// ��ȡ����,����nACK
	IIC_Stop();							// ����һ��ֹͣ����
	return res;
}
