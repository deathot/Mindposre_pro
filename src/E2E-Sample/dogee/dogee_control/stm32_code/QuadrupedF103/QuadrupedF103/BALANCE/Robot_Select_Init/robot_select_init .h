#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

typedef struct
{
  float WheelSpacing;    // �־�
  float AxleSpacing;     // ���
  float GearRatio;       // ������ٱ�
  int EncoderAccuracy;   // ����������
  float WheelDiameter;   // �־�
  float OmniTurnRadiaus; // ȫ������ת�뾶
} Robot_Parament_InitTypeDef;

// Car_Mode for Mec
// 0:���������������SENIOR_MEC_NO
// 1:�������ְ�ʽ����SENIOR_MEC_BS
// 2:�������ֶ�������SENIOR_MEC_DL
// 3:�������ְ�ʽ���ҳ�����TOP_MEC_BS_18
// 4:�������ְ�ʽ����������TOP_MEC_BS_47
// 5:�������ֶ������ҳ�����TOP_MEC_DL_18

// �����ְ��� ע����һ��
#define SENIOR_MEC_NO_wheelspacing 0.176
#define SENIOR_MEC_BS_wheelspacing 0.252
#define SENIOR_MEC_DL_wheelspacing 0.247
#define TOP_MEC_BS_wheelspacing 0.311
#define TOP_MEC_DL_wheelspacing 0.295
#define TOP_MEC_DL_wheelspacing_Customized 0.446 // ���Ƴߴ�

// ����� ע����һ��
#define SENIOR_MEC_NO_axlespacing 0.156
#define SENIOR_MEC_BS_axlespacing 0.226
#define SENIOR_MEC_DL_axlespacing 0.214
#define TOP_MEC_BS_axlespacing 0.308
#define TOP_MEC_DL_axlespacing 0.201
#define TOP_MEC_DL_axlespacing_Customized 0.401 // ���Ƴߴ�

// ������ٱ�
#define MD36N_5_18 5.18
#define MD36N_27 27
#define MD36N_51 51
#define MD36N_71 71
#define MD60N_18 18
#define MD60N_47 47

// ����������
#define Photoelectric_500 500
#define Hall_13 13

// ������ֱ̥��
#define Mecanum_60 0.060f
#define Mecanum_75 0.075f
#define Mecanum_100 0.100f
#define Mecanum_127 0.127f
#define Mecanum_152 0.152f

// �־�ȫ����ֱ��ϵ��
#define FullDirecion_75 0.075
#define FullDirecion_127 0.127
#define FullDirecion_152 0.152
#define FullDirecion_203 0.203
#define FullDirecion_217 0.217

// ȫ����С����ת�뾶
#define Omni_Turn_Radiaus_164 0.164
#define Omni_Turn_Radiaus_180 0.180
#define Omni_Turn_Radiaus_290 0.290

// ��������Ƶ�� ������Ƶ��
#define EncoderMultiples 4
#define EncoderFrequency 100

#define CONTROL_FREQUENCY 100
#define PI 3.1415f // Բ����

void Robot_Select(void);
#if Mec
void Robot_Init(float wheelspacing, int gearratio, int Accuracy, float tyre_diameter, float axlespacing);
#elif Omni
void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter);
#endif

#endif
