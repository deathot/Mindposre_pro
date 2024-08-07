#include "quadruped.h"

control_t control; /*四足控制参数*/
RelAttitude_t RelAttitude;
TarAttitude_t TarAttitude;
RelRate_t RelRate;
MotorInit_t MotorInit;
TarTragectoryOut_t TarTragectoryOut;
EncoderTarget_t EncoderTarget;

int Time_t = 0, Time2_t = 0;						 // 计算点，用于区分支撑相和摆动相
long int Target_1A, Target_1B, Target_2A, Target_2B; // 八个电机的目标旋转角度
long int Target_3A, Target_3B, Target_4A, Target_4B;

int TarTargectoryLeftX = 0, TarTargectoryRightX = 0; // 四足左边腿和右边腿的控制量
void Quadruped_task(void *pvParameters)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt(); // 获取任务上次唤醒时间，用于执行绝对延时函数
	while (1)
	{

		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ)); // 绝对延时函数，控制任务按照1000Hz频率运行
		//************************************************************************************************************************//
		// 更新编码器的数据
		//************************************************************************************************************************//
		UpdataEncoder();

		//************************************************************************************************************************//
		// 四足机身姿态控制器
		// 实际上姿态控制的原理是通过腿的高度组合，来实现不同的机身姿态
		// PS2左边手柄摇杆/APP的按键模式 可以控制四足静止时水平方向的姿态，摇杆不动时机身为水平
		//************************************************************************************************************************//
		TarAttitude.pitch = constrain((int)-smooth_pose_y, -50, 50); // 控制前倾后蹲
		TarAttitude.roll = constrain((int)-smooth_pose_x, -40, 40);	 // 控制左右侧身

		control.pitch = pidUpdate(&pid[BODY_PITCH], TarAttitude.pitch - RelAttitude.pitch); // 用目标角度减去测量的角度
		control.roll = pidUpdate(&pid[BODY_ROLL], TarAttitude.roll - RelAttitude.roll);		// 对机身姿态做一个PID平衡环

		//************************************************************************************************************************//
		// 四足运动控制器
		//************************************************************************************************************************//
		TarTargectoryLeftX = move_x - constrain(move_z, -55, 55);  // 控制左边的腿
		TarTargectoryRightX = move_x + constrain(move_z, -55, 55); // 控制右边的腿

		TarTargectoryLeftX = constrain(TarTargectoryLeftX, -90, 90);   // 输入控制量限幅
		TarTargectoryRightX = constrain(TarTargectoryRightX, -90, 90); // 输入控制量限幅

		//************************************************************************************************************************//
		// 足端轨迹规划需要用到的时间点，用于区分支撑相和摆动相
		//************************************************************************************************************************//
		if (Time_t == 0)
			Time_t = Tratimes;
		Time_t--;
		if (Time2_t == Tratimes)
			Time2_t = 0;
		Time2_t++;

		//************************************************************************************************************************//
		// 步态控制思路：将一个完整的蹄子运动周期，运动轨迹的每个点划分出来，每个点作为一个目标坐标点去实现
		//************************************************************************************************************************//
		// 由两个电机组成的一条腿，以一条腿为一个整体来控制
		// 根据输入，实时输出腿X和Z轴方向的坐标
		/*************左边的两条腿*****************/
		FootTrajectoryLeg1(TarTargectoryLeftX, -70, Time_t); // 蹄子路径规划函数，其中限制蹄子抬起的最高点为-70，当高度超过70时蹄子响应速度跟不上
		FootTrajectoryLeg2(TarTargectoryLeftX, -70, Time_t);
		/*************右边的两条腿*****************/
		FootTrajectoryLeg3(TarTargectoryRightX, -70, Time2_t);
		FootTrajectoryLeg4(TarTargectoryRightX, -70, Time2_t);

		//************************************************************************************************************************//
		// 预运动状态(READY)  ：在 有和无 运动量状态切换时，增加了一个踏步来过渡，无运动量时四足会静止不踏步
		// 运动或踏步(RUNNING)：四足在无控制量时保持踏步 （PS2模式才可用）
		// 站立不动           ：强制四足瞬间停止运动进入静止态（READY , RUNNING均可用）
		// z轴方向的坐标需要加上一个基础的支撑值（因为z坐标=0时四足的腿是收起的状态），例如：-180，-150
		//************************************************************************************************************************//
		if (ARMING_FLAG(READY)) // 预运动状态
		{
			if (RUN_Control == 0) // 没有收到控制量，此时四足静止
			{
				LegTarPosition.Leg1X = 0;
				LegTarPosition.Leg1Z = -180 + control.pitch - control.roll; // 高度环+轨迹坐标+机身姿态平衡（左右平衡和前后平衡）

				LegTarPosition.Leg2X = 0;
				LegTarPosition.Leg2Z = -180 + control.pitch + control.roll;

				LegTarPosition.Leg3X = 0;
				LegTarPosition.Leg3Z = -180 - control.pitch - control.roll;

				LegTarPosition.Leg4X = 0;
				LegTarPosition.Leg4Z = -180 - control.pitch + control.roll;
			}
			else if (RUN_Control == 1) // 接收到控制量后先踏步让机身保持稳定
			{
				LegTarPosition.Leg1X = 0;															 // x轴方向坐标置0，让四足踏步
				LegTarPosition.Leg1Z = -150 + TarTragectoryOut.Leg1Z + control.pitch - control.roll; // 高度环+轨迹坐标+机身姿态平衡（左右平衡和前后平衡）

				LegTarPosition.Leg2X = 0;
				LegTarPosition.Leg2Z = -150 + TarTragectoryOut.Leg2Z + control.pitch + control.roll;

				LegTarPosition.Leg3X = 0;
				LegTarPosition.Leg3Z = -150 + TarTragectoryOut.Leg3Z - control.pitch - control.roll;

				LegTarPosition.Leg4X = 0;
				LegTarPosition.Leg4Z = -150 + TarTragectoryOut.Leg4Z - control.pitch + control.roll;
			}
			else if (RUN_Control == 2) // 机身稳定后再开始运动
			{
				LegTarPosition.Leg1X = TarTragectoryOut.Leg1X;										 // 轨迹坐标
				LegTarPosition.Leg1Z = -150 + TarTragectoryOut.Leg1Z + control.pitch - control.roll; // 高度环+轨迹坐标+机身姿态平衡（左右平衡和前后平衡）

				LegTarPosition.Leg2X = TarTragectoryOut.Leg2X;
				LegTarPosition.Leg2Z = -150 + TarTragectoryOut.Leg2Z + control.pitch + control.roll;

				LegTarPosition.Leg3X = TarTragectoryOut.Leg3X;
				LegTarPosition.Leg3Z = -150 + TarTragectoryOut.Leg3Z - control.pitch - control.roll;

				LegTarPosition.Leg4X = TarTragectoryOut.Leg4X;
				LegTarPosition.Leg4Z = -150 + TarTragectoryOut.Leg4Z - control.pitch + control.roll;
			}
		}

		else if (ARMING_FLAG(RUNNING)) // 踏步或运动（PS2模式可用）
		{
			LegTarPosition.Leg1X = TarTragectoryOut.Leg1X;										 // 轨迹坐标
			LegTarPosition.Leg1Z = -150 + TarTragectoryOut.Leg1Z + control.pitch - control.roll; // 高度环+轨迹坐标+机身姿态平衡（左右平衡和前后平衡）

			LegTarPosition.Leg2X = TarTragectoryOut.Leg2X;
			LegTarPosition.Leg2Z = -150 + TarTragectoryOut.Leg2Z + control.pitch + control.roll;

			LegTarPosition.Leg3X = TarTragectoryOut.Leg3X;
			LegTarPosition.Leg3Z = -150 + TarTragectoryOut.Leg3Z - control.pitch - control.roll;

			LegTarPosition.Leg4X = TarTragectoryOut.Leg4X;
			LegTarPosition.Leg4Z = -150 + TarTragectoryOut.Leg4Z - control.pitch + control.roll;
		}
		else // 站立不动
		{
			LegTarPosition.Leg1X = 0;
			LegTarPosition.Leg1Z = -180 + control.pitch - control.roll; // 待机站立和高度和运动的高度不同

			LegTarPosition.Leg2X = 0;
			LegTarPosition.Leg2Z = -180 + control.pitch + control.roll;

			LegTarPosition.Leg3X = 0;
			LegTarPosition.Leg3Z = -180 - control.pitch - control.roll;

			LegTarPosition.Leg4X = 0;
			LegTarPosition.Leg4Z = -180 - control.pitch + control.roll;
		}

		LegTarPosition.Leg1Z = constrain((int)LegTarPosition.Leg1Z, -230, -100);
		LegTarPosition.Leg2Z = constrain((int)LegTarPosition.Leg2Z, -230, -100);
		LegTarPosition.Leg3Z = constrain((int)LegTarPosition.Leg3Z, -230, -100);
		LegTarPosition.Leg4Z = constrain((int)LegTarPosition.Leg4Z, -230, -100); // 输出腿的高度限幅

		//************************************************************************************************************************//
		// 逆运动学计算，输出每个电机相对的目标输出角度
		//************************************************************************************************************************//
		UpdataTargerAngles(&TargetMotorAngle, &LegTarPosition); // 根据腿的目标坐标输出臂的实际目标角度

		//************************************************************************************************************************//
		// 编码器的数值对应实际的臂的位置
		// 计算方式是用编码器的基准值加上（反方向是减）目标角度
		// 目标角度需要转换成实际对应的编码器的数值，最终得出新的编码器的值。
		//************************************************************************************************************************//
		//================电机实际目标旋转角度================
		Target_1A = MotorInit.M1 - TargetMotorAngle.Leg1f * EncoderOneDegree;
		Target_1B = MotorInit.M2 + (180 - TargetMotorAngle.Leg1b) * EncoderOneDegree;
		Target_2A = MotorInit.M3 - TargetMotorAngle.Leg2f * EncoderOneDegree;
		Target_2B = MotorInit.M4 + (180 - TargetMotorAngle.Leg2b) * EncoderOneDegree;

		Target_3A = MotorInit.M5 - TargetMotorAngle.Leg3f * EncoderOneDegree;
		Target_3B = MotorInit.M6 + (180 - TargetMotorAngle.Leg3b) * EncoderOneDegree;
		Target_4A = MotorInit.M7 - TargetMotorAngle.Leg4f * EncoderOneDegree;
		Target_4B = MotorInit.M8 + (180 - TargetMotorAngle.Leg4b) * EncoderOneDegree;

		MotorOutputChoose();  // 四足 开机/运动/静止/关机 状态检测，并输出对应的目标编码器的值
		PowerOutputControl(); // 主控的四个电机位置环PID计算２赋值PWM

		//************************************************************************************************************************//
		// 将从机控制的四个电机目标位置值等信息发送给从机
		//************************************************************************************************************************//
		CAN_data_transition(); // 将要发送给从机的数据进行封装
		CAN1_SEND();		   // 通过CAN将数据发送出去

		/*******************************************************************************************************************/
		tick++;
	}
}

/**************************************************************************
函数功能：更新解锁标志
入口参数：状态切换值，APP、串口、航模 控制时模拟了PS2控制的键位值来实现同样的效果
返回  值：无
**************************************************************************/
void UpdataFlags(u8 flag_t)
{
	switch (flag_t)
	{
	case 0x01: // select
	{
		ENABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(READY);
		break;
	}
	case 0x04: // 按下手柄的start按键，四足开始站立
	{
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(STOP);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(LOCKED); // 使能解锁
		break;
	}
	case 5:
		break; // 预留
	case 6:
		break; // 预留
	case 7:
		break; // 预留
	case 8:
		break; // 预留

	case 13: // 右-上 手柄的三角形按键 （踏步）
	{
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(STOP);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(RUNNING); // 使能运动
		break;
	}
	case 14: // 右-右  手柄的圆形按键（停止）
	{
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(STOP);
		break;
	}
	case 15: // 右-下 手柄的叉叉按键（预备态）
	{
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(STOP);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(READY);
		break;
	}
	case 16:
	{ // 右-左 手柄的正方形按键 （归位）
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(STOP);
		ENABLE_ARMING_FLAG(SHUTDWON); // 使能归位
		break;
	}
	default:
		break;
	}
}

/**************************************************************************
函数功能：选择每个电机的控制输入
入口参数：无
返回  值：无
此函数说明了四足上电及运行的流程，具体可表示为
上电 EncoderTarget.XX = 0，开始上电的时候，四足静止不动
|
站立，此时代表解锁状态，刚解锁之后，四足站立，
|
踏步/行走 ，代表四足开始运动，可以调节跨步的大小
|
四足关机
**************************************************************************/
u8 flag_idle = 1;
u32 time_start = 0;
void MotorOutputChoose(void)
{
	// 解锁后，机体缓慢站立，
	if (ARMING_FLAG(LOCKED)) // 解锁状态
	{
		if (flag_idle == 1) // 解锁后，首先开始站立
		{
			time_start++;

			EncoderTarget.M1 = -4.0f * time_start; // 由于电机的安装方式，方向取反，下同
			EncoderTarget.M2 = 4.0f * time_start;
			EncoderTarget.M3 = -4.0f * time_start;
			EncoderTarget.M4 = 4.0f * time_start;

			EncoderTarget.M5 = -4.0f * time_start;
			EncoderTarget.M6 = 4.0f * time_start;
			EncoderTarget.M7 = -4.0f * time_start;
			EncoderTarget.M8 = 4.0f * time_start;

			if (time_start >= 4250) // 425*4=17000，可以通过调整参数来改变四足站立和归位过程的速度
			{
				time_start = 0; // 计数置零，站立过程结束
				flag_idle = 2;	// 站立结束标志位
			}
		}

		if (ARMING_FLAG(STOP) && (flag_idle == 2)) // 四足站立停止 无姿态控制
		{
			EncoderTarget.M1 = -17000;
			EncoderTarget.M2 = 17000;
			EncoderTarget.M3 = -17000;
			EncoderTarget.M4 = 17000; // 四足上电开启后，启动站立的目标编码器数值

			EncoderTarget.M5 = -17000;
			EncoderTarget.M6 = 17000;
			EncoderTarget.M7 = -17000;
			EncoderTarget.M8 = 17000;
		}
		else if (ARMING_FLAG(SHUTDWON) && (flag_idle == 2)) // 四足关机，与四足站立过程原理相同
		{
			time_start++;

			EncoderTarget.M1 = -17000 + 4.0f * time_start;
			EncoderTarget.M2 = 17000 - 4.0f * time_start;
			EncoderTarget.M3 = -17000 + 4.0f * time_start;
			EncoderTarget.M4 = 17000 - 4.0f * time_start;

			EncoderTarget.M5 = -17000 + 4.0f * time_start;
			EncoderTarget.M6 = 17000 - 4.0f * time_start;
			EncoderTarget.M7 = -17000 + 4.0f * time_start;
			EncoderTarget.M8 = 17000 - 4.0f * time_start;
			if (time_start >= 4250)
			{
				time_start = 0; // 计数置零，归位过程结束
				flag_idle = 3;	// 归位结束标志位，关机后不可以直接重新开机，需要将控制器复位后才可以重新开机
			}
		}
		else if (flag_idle == 2) // 四足踏步或控制运动
		{
			EncoderTarget.M1 = Target_1A;
			EncoderTarget.M2 = Target_1B;
			EncoderTarget.M3 = Target_2A;
			EncoderTarget.M4 = Target_2B;

			EncoderTarget.M5 = Target_3A;
			EncoderTarget.M6 = Target_3B;
			EncoderTarget.M7 = Target_4A;
			EncoderTarget.M8 = Target_4B;
		}
	}
	else // 上电后但未站立时，此时电机锁住
	{
		EncoderTarget.M1 = 0;
		EncoderTarget.M2 = 0;
		EncoderTarget.M3 = 0;
		EncoderTarget.M4 = 0;

		EncoderTarget.M5 = 0;
		EncoderTarget.M6 = 0;
		EncoderTarget.M7 = 0;
		EncoderTarget.M8 = 0;
	}
}

void LegPositionInit(void)
{
	MotorInit.M1 = -13000; // 给定一个四足的臂的基准值，和上面17000的值不同
	MotorInit.M2 = 13000;  // 是因为站立和运动时四足的机身高度是不同的
	MotorInit.M3 = -13000;
	MotorInit.M4 = 13000;

	MotorInit.M5 = -13000;
	MotorInit.M6 = 13000;
	MotorInit.M7 = -13000;
	MotorInit.M8 = 13000;
}

// void AttitudeControl(control_t *controlT,RelAttitude_t *RelAttitudeT,RelRate_t * RelRateT,TarAttitude_t *TarAttitudeT )
//{
//
//	control.pitch = pidUpdate(&pid[BODY_PITCH],TarAttitudeT->pitch - RelAttitudeT->pitch);
//
//	//control.pitch = pidCaulate(&pid[BODY_PITCH], TarAttitudeT->pitch - RelAttitudeT->pitch, RelRateT->gyrox );
//	control.roll = pidCaulate(&pid[BODY_ROLL], TarAttitudeT->roll - RelAttitudeT->roll, RelRateT->gyroy );
// }

//************************************************************************************************************************//
// 蹄子足迹路径规划算法，详细解析见足迹路径规划解读文档
//************************************************************************************************************************//
void FootTrajectoryLeg1(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg1X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (lam / 2 + 1 / 2) * Ts) + TarX;
		TarTragectoryOut.Leg1Z = TarZ;
	}
	else
	{
		TarTragectoryOut.Leg1X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2;
		TarTragectoryOut.Leg1Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
}
void FootTrajectoryLeg2(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg2X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2 - TarX;
		TarTragectoryOut.Leg2Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
	else
	{
		TarTragectoryOut.Leg2X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (1 - lam) / 2 * Ts);
		TarTragectoryOut.Leg2Z = TarZ;
	}
}
void FootTrajectoryLeg3(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg3X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (lam / 2 + 1 / 2) * Ts) + TarX;
		TarTragectoryOut.Leg3Z = TarZ;
	}
	else
	{
		TarTragectoryOut.Leg3X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2;
		TarTragectoryOut.Leg3Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
}
void FootTrajectoryLeg4(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg4X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2 - TarX;
		TarTragectoryOut.Leg4Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
	else
	{
		TarTragectoryOut.Leg4X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (1 - lam) / 2 * Ts);
		TarTragectoryOut.Leg4Z = TarZ;
	}
}
