#include "Inverse.h"

/*
运动学逆解函数
函数入口参数为X Z的坐标，坐标的正负参加如下坐标系；
函数返回值为电机旋转的角度， 顺时针旋转为正，逆时针旋转为负；
			---------------- +x
			|
			|
			|
			|
			| -z
---------------Ground---------------------
*/
u8 FlagStart;

TargetMotorAngle_t TargetMotorAngle;
SubPositionLeg_t LegTarPosition;

float TarMotorAngle[8];

/*
上电初始化电机的位置，设定四足的离地高度为 InitPositionZ
*/
void MotorPositionInit(void)
{
	LegTarPosition.Leg1X = InitPositionX;
	LegTarPosition.Leg1Z = InitPositionZ;

	LegTarPosition.Leg1X = InitPositionX;
	LegTarPosition.Leg1Z = InitPositionZ;

	LegTarPosition.Leg1X = InitPositionX;
	LegTarPosition.Leg1Z = InitPositionZ;

	LegTarPosition.Leg1X = InitPositionX;
	LegTarPosition.Leg1Z = InitPositionZ;
}
/*************************************************************
//计算电机目标转速
//输入：四足每根腿的目标位置
//输出：四足八个电机的目标角度
**************************************************************/
void UpdataTargerAngles(TargetMotorAngle_t *TarMotorAng_t, SubPositionLeg_t *TarPosition)
{
	TarMotorAng_t->Leg1f = InverseKinematicsLegAngle1(TarPosition->Leg1X, TarPosition->Leg1Z);
	TarMotorAng_t->Leg1b = InverseKinematicsLegAngle2(TarPosition->Leg1X, TarPosition->Leg1Z);

	TarMotorAng_t->Leg2f = InverseKinematicsLegAngle1(TarPosition->Leg2X, TarPosition->Leg2Z);
	TarMotorAng_t->Leg2b = InverseKinematicsLegAngle2(TarPosition->Leg2X, TarPosition->Leg2Z);

	TarMotorAng_t->Leg3f = InverseKinematicsLegAngle1(TarPosition->Leg3X, TarPosition->Leg3Z);
	TarMotorAng_t->Leg3b = InverseKinematicsLegAngle2(TarPosition->Leg3X, TarPosition->Leg3Z);

	TarMotorAng_t->Leg4f = InverseKinematicsLegAngle1(TarPosition->Leg4X, TarPosition->Leg4Z);
	TarMotorAng_t->Leg4b = InverseKinematicsLegAngle2(TarPosition->Leg4X, TarPosition->Leg4Z);
}

/*************************************************************
//运行学分析
**************************************************************/
float InverseKinematicsLegAngle1(float Px_t, float Pz_t)
{
	float angle, length, gamma;

	length = sqrtf(Px_t * Px_t + Pz_t * Pz_t); // 勾股定理
	gamma = atanf(-Pz_t / Px_t);			   // 反正切函数，计算蹄子与地面夹角的弧度值
	if (gamma < 0)
		gamma += 3.14f; // 反三角函数阈值转换，如果是负值加一个Π
	else
		gamma += 0;
	angle = gamma - acosf((length * length + LegUp * LegUp - LegDown * LegDown) / (2 * LegUp * length)); // 余弦定理求出上臂和（length）之间的夹角，相减得出上臂与x轴正方向的夹角
	return angle * 180.0f / 3.14f;																		 // 最终结果弧度转换成角度
}
float InverseKinematicsLegAngle2(float Px_t, float Pz_t)
{
	float angle, length, gamma;

	length = sqrtf(Px_t * Px_t + Pz_t * Pz_t);
	gamma = atanf(-Pz_t / Px_t);
	if (gamma < 0)
		gamma += 3.14f;
	else
		gamma += 0;
	angle = gamma + acosf((length * length + LegUp * LegUp - LegDown * LegDown) / (2 * LegUp * length)); // 根据对称关系，求出另一条上臂与x轴正方向的夹角
	return angle * 180 / 3.14f;
}

///*************************************************************
////
//**************************************************************/
// float InverseKinematicsLegAngle3 (float Px_t, float Pz_t)
//{
// float angle,length,gamma;
//
//	length = sqrtf(Px_t*Px_t+Pz_t*Pz_t); //勾股定理
//  gamma =  atanf(-Pz_t/-Px_t);  //反正切函数，计算蹄子与地面夹角的弧度值
//	if(gamma<0)
//		gamma+=3.14f ; //反三角函数阈值转换，如果是负值加一个Π
//	else
//		gamma +=0;
//	angle =  3.1415f - gamma - acosf ((length*length +LegUp*LegUp - LegDown*LegDown )/(2*LegUp*length)); //余弦定理求出上臂和（length）之间的夹角，相减得出上臂与x轴正方向的夹角
//	return  angle*180.0f/3.14f ; //最终结果弧度转换成角度
//}
// float InverseKinematicsLegAngle4 (float Px_t, float Pz_t)
//{
// float angle,length,gamma;
//
//	length = sqrtf(Px_t*Px_t+Pz_t*Pz_t);
//  gamma =  atanf(-Pz_t/-Px_t);
//		if(gamma<0)
//		gamma+=3.14f;
//	else
//		gamma +=0;
//	angle =  3.1415f -  gamma + acosf ((length*length +LegUp*LegUp - LegDown*LegDown )/(2*LegUp*length));//根据对称关系，求出另一条上臂与x轴正方向的夹角
//	return angle*180/3.14f   ;
//}
