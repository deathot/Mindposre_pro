#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
// #include "usartx.h"
int Flag_Mpu6050;						   // 检测MPU6050是否正常启动的标志位
int Deviation_Count;					   // 零点漂移计数
short gyro[3], accel[3], sensors;		   // 三轴加速度陀螺仪数据
short Deviation_gyro[3], Original_gyro[3]; // 陀螺仪静差 和原始数据
/**************************************************************************
函数功能：MPU6050任务
入口参数：无
返回  值：无
**************************************************************************/
void MPU6050_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); // 此任务以100Hz的频率运行
		if (Deviation_Count < CONTROL_DELAY)			  // 开机前，读取陀螺仪零点
		{
			Deviation_Count++;
			memcpy(Deviation_gyro, gyro, sizeof(gyro));
		}
		//================更新陀螺仪数据================
		MPU6050_Get_Accelerometer(accel); // 通过IIC读取加速度信息
		MPU6050_Get_Gyroscope(gyro);	  // 通过IIC读取角速度信息
	}
}
/**************************************************************************
函数功能：MPU6050初始化
入口参数：无
返回  值：1正常0错误 错误一般都是器件地址引起的，或者IIC引脚不对等
**************************************************************************/
unsigned char MPU6050_Init(void)
{
	u8 res;
	MPU6050_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
	delay_ms(200);								 // 演示等待稳定
	MPU6050_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
	MPU6050_Set_Gyro_Fsr(1);					 // 陀螺仪传感器,量程±500dps
	MPU6050_Set_Accel_Fsr(0);					 // 加速度传感器,量程±2g
	MPU6050_Set_Rate(50);						 // 设置采样率50Hz
	MPU6050_Write_Byte(MPU_INT_EN_REG, 0X00);	 // 关闭所有中断
	MPU6050_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU6050_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 // 关闭FIFO
	MPU6050_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	res = MPU6050_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) // 器件ID正确取决于AD引脚
	{
		MPU6050_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
		MPU6050_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
		MPU6050_Set_Rate(50);						 // 设置采样率为50Hz
	}
	else
		return 1;
	return 0;
}
/**************************************************************************
函数功能：设置陀螺仪传感器满量程范围
入口参数：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
返回  值：0,设置成功
**************************************************************************/
unsigned char MPU6050_Set_Gyro_Fsr(u8 fsr)
{
	return MPU6050_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}
/**************************************************************************
函数功能：设置加速度计满量程范围
入口参数：fsr:0,±2g;1,±4g;2,±8g;3,±16g
返回  值：0,设置成功
**************************************************************************/
unsigned char MPU6050_Set_Accel_Fsr(u8 fsr)
{
	return MPU6050_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}
/**************************************************************************
函数功能：设置MPU6050芯片的数字低通滤波器
入口参数：lpf:数字低通滤波频率(Hz)
返回  值：0,设置成功
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
	return MPU6050_Write_Byte(MPU_CFG_REG, data); // 设置数字低通滤波器
}
/**************************************************************************
函数功能：设置MPU6050芯片的采样率
入口参数：rate:4~1000(Hz)
返回  值：0,设置成功
**************************************************************************/
unsigned char MPU6050_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU6050_Write_Byte(MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
	return MPU6050_Set_LPF(rate / 2);					  // 自动设置LPF为采样率的一半
}
/**************************************************************************
函数功能：采集陀螺仪的数据 包括零点漂移处理和设置LED状态
入口参数：存放的地址
返回  值：0,采集成功
**************************************************************************/
unsigned char MPU6050_Get_Gyroscope(short *gyro)
{
	u8 buf[6], res;
	res = MPU6050_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		if (Deviation_Count < CONTROL_DELAY) // 开机前 读取陀螺仪零点
		{
			gyro[0] = (((u16)buf[0] << 8) | buf[1]);
			gyro[1] = (((u16)buf[2] << 8) | buf[3]);
			gyro[2] = (((u16)buf[4] << 8) | buf[5]);
			Led_Count = 1; // LED常亮
			// Flag_Stop=1;//关闭电机
		}
		else
		{
			// if(Deviation_Count==CONTROL_DELAY)Flag_Stop=0;//使能电机
			Led_Count = 300; // 零点读取完之后LED闪烁
			// Deviation_Count	=CONTROL_DELAY;	 //固定计数值
			Original_gyro[0] = (((u16)buf[0] << 8) | buf[1]); // 保存原始数据用于按键修改零点
			Original_gyro[1] = (((u16)buf[2] << 8) | buf[3]);
			Original_gyro[2] = (((u16)buf[4] << 8) | buf[5]);

			gyro[0] = Original_gyro[0] - Deviation_gyro[0]; // 最终带去除零点漂移的数据
			gyro[1] = Original_gyro[1] - Deviation_gyro[1];
			gyro[2] = Original_gyro[2] - Deviation_gyro[2];
		}
	}
	return res;
}
/**************************************************************************
函数功能：采集加速度计的数据，直接使用
入口参数：存放的地址
返回  值：0,采集成功
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
函数功能：IIC通讯用于MPU6050的写操作
入口参数：addr:器件地址 reg:寄存器地址 len:写入长度 buf:数据区
返回  值：0正常
**************************************************************************/
unsigned char MPU6050_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
	if (IIC_Wait_Ack())				// 等待应答
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg); // 写寄存器地址
	IIC_Wait_Ack();		// 等待应答
	for (i = 0; i < len; i++)
	{
		IIC_Send_Byte(buf[i]); // 发送数据
		if (IIC_Wait_Ack())	   // 等待ACK
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}
/**************************************************************************
函数功能：IIC通讯用于MPU6050的读操作
入口参数：addr:器件地址 reg:寄存器地址 len:写入长度 buf:数据区
返回  值：0正常
**************************************************************************/
unsigned char MPU6050_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
	if (IIC_Wait_Ack())				// 等待应答
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg); // 写寄存器地址
	IIC_Wait_Ack();		// 等待应答
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
	IIC_Wait_Ack();					// 等待应答
	while (len)
	{
		if (len == 1)
			*buf = IIC_Read_Byte(0); // 读数据,发送nACK
		else
			*buf = IIC_Read_Byte(1); // 读数据,发送ACK
		len--;
		buf++;
	}
	IIC_Stop(); // 产生一个停止条件
	return 0;
}
/**************************************************************************
函数功能：IIC通讯用于MPU6050的写一个字节操作
入口参数：reg:寄存器地址 data:数据
返回  值：0正常
**************************************************************************/
unsigned char MPU6050_Write_Byte(u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
	if (IIC_Wait_Ack())					// 等待应答
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	 // 写寄存器地址
	IIC_Wait_Ack();		 // 等待应答
	IIC_Send_Byte(data); // 发送数据
	if (IIC_Wait_Ack())	 // 等待ACK
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}
/**************************************************************************
函数功能：IIC通讯用于MPU6050的读一个字节操作
入口参数：reg:寄存器地址
返回  值：0正常
**************************************************************************/
unsigned char MPU6050_Read_Byte(u8 reg)
{
	u8 res;
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
	IIC_Wait_Ack();						// 等待应答
	IIC_Send_Byte(reg);					// 写寄存器地址
	IIC_Wait_Ack();						// 等待应答
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 1); // 发送器件地址+读命令
	IIC_Wait_Ack();						// 等待应答
	res = IIC_Read_Byte(0);				// 读取数据,发送nACK
	IIC_Stop();							// 产生一个停止条件
	return res;
}
