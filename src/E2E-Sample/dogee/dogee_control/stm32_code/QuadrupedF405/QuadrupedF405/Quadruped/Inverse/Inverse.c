#include "Inverse.h"

/*
�˶�ѧ��⺯��
������ڲ���ΪX Z�����꣬����������μ���������ϵ��
��������ֵΪ�����ת�ĽǶȣ� ˳ʱ����תΪ������ʱ����תΪ����
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
�ϵ��ʼ�������λ�ã��趨�������ظ߶�Ϊ InitPositionZ
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
//������Ŀ��ת��
//���룺����ÿ���ȵ�Ŀ��λ��
//���������˸������Ŀ��Ƕ�
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
//����ѧ����
**************************************************************/
float InverseKinematicsLegAngle1(float Px_t, float Pz_t)
{
	float angle, length, gamma;

	length = sqrtf(Px_t * Px_t + Pz_t * Pz_t); // ���ɶ���
	gamma = atanf(-Pz_t / Px_t);			   // �����к������������������нǵĻ���ֵ
	if (gamma < 0)
		gamma += 3.14f; // �����Ǻ�����ֵת��������Ǹ�ֵ��һ����
	else
		gamma += 0;
	angle = gamma - acosf((length * length + LegUp * LegUp - LegDown * LegDown) / (2 * LegUp * length)); // ���Ҷ�������ϱۺͣ�length��֮��ļнǣ�����ó��ϱ���x��������ļн�
	return angle * 180.0f / 3.14f;																		 // ���ս������ת���ɽǶ�
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
	angle = gamma + acosf((length * length + LegUp * LegUp - LegDown * LegDown) / (2 * LegUp * length)); // ���ݶԳƹ�ϵ�������һ���ϱ���x��������ļн�
	return angle * 180 / 3.14f;
}

///*************************************************************
////
//**************************************************************/
// float InverseKinematicsLegAngle3 (float Px_t, float Pz_t)
//{
// float angle,length,gamma;
//
//	length = sqrtf(Px_t*Px_t+Pz_t*Pz_t); //���ɶ���
//  gamma =  atanf(-Pz_t/-Px_t);  //�����к������������������нǵĻ���ֵ
//	if(gamma<0)
//		gamma+=3.14f ; //�����Ǻ�����ֵת��������Ǹ�ֵ��һ����
//	else
//		gamma +=0;
//	angle =  3.1415f - gamma - acosf ((length*length +LegUp*LegUp - LegDown*LegDown )/(2*LegUp*length)); //���Ҷ�������ϱۺͣ�length��֮��ļнǣ�����ó��ϱ���x��������ļн�
//	return  angle*180.0f/3.14f ; //���ս������ת���ɽǶ�
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
//	angle =  3.1415f -  gamma + acosf ((length*length +LegUp*LegUp - LegDown*LegDown )/(2*LegUp*length));//���ݶԳƹ�ϵ�������һ���ϱ���x��������ļн�
//	return angle*180/3.14f   ;
//}
