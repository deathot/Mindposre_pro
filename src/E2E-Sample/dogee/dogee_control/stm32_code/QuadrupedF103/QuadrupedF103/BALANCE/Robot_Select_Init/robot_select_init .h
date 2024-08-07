#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

typedef struct
{
  float WheelSpacing;    // 轮距
  float AxleSpacing;     // 轴距
  float GearRatio;       // 电机减速比
  int EncoderAccuracy;   // 编码器精度
  float WheelDiameter;   // 轮径
  float OmniTurnRadiaus; // 全向轮旋转半径
} Robot_Parament_InitTypeDef;

// Car_Mode for Mec
// 0:高配麦轮无轴承座SENIOR_MEC_NO
// 1:高配麦轮摆式悬挂SENIOR_MEC_BS
// 2:高配麦轮独立悬挂SENIOR_MEC_DL
// 3:顶配麦轮摆式悬挂常规型TOP_MEC_BS_18
// 4:顶配麦轮摆式悬挂重载型TOP_MEC_BS_47
// 5:顶配麦轮独立悬挂常规型TOP_MEC_DL_18

// 主动轮半宽度 注意是一半
#define SENIOR_MEC_NO_wheelspacing 0.176
#define SENIOR_MEC_BS_wheelspacing 0.252
#define SENIOR_MEC_DL_wheelspacing 0.247
#define TOP_MEC_BS_wheelspacing 0.311
#define TOP_MEC_DL_wheelspacing 0.295
#define TOP_MEC_DL_wheelspacing_Customized 0.446 // 定制尺寸

// 半轴距 注意是一半
#define SENIOR_MEC_NO_axlespacing 0.156
#define SENIOR_MEC_BS_axlespacing 0.226
#define SENIOR_MEC_DL_axlespacing 0.214
#define TOP_MEC_BS_axlespacing 0.308
#define TOP_MEC_DL_axlespacing 0.201
#define TOP_MEC_DL_axlespacing_Customized 0.401 // 定制尺寸

// 电机减速比
#define MD36N_5_18 5.18
#define MD36N_27 27
#define MD36N_51 51
#define MD36N_71 71
#define MD60N_18 18
#define MD60N_47 47

// 编码器精度
#define Photoelectric_500 500
#define Hall_13 13

// 麦轮轮胎直径
#define Mecanum_60 0.060f
#define Mecanum_75 0.075f
#define Mecanum_100 0.100f
#define Mecanum_127 0.127f
#define Mecanum_152 0.152f

// 轮径全向轮直径系列
#define FullDirecion_75 0.075
#define FullDirecion_127 0.127
#define FullDirecion_152 0.152
#define FullDirecion_203 0.203
#define FullDirecion_217 0.217

// 全向轮小车旋转半径
#define Omni_Turn_Radiaus_164 0.164
#define Omni_Turn_Radiaus_180 0.180
#define Omni_Turn_Radiaus_290 0.290

// 编码器倍频数 编码器频率
#define EncoderMultiples 4
#define EncoderFrequency 100

#define CONTROL_FREQUENCY 100
#define PI 3.1415f // 圆周率

void Robot_Select(void);
#if Mec
void Robot_Init(float wheelspacing, int gearratio, int Accuracy, float tyre_diameter, float axlespacing);
#elif Omni
void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter);
#endif

#endif
