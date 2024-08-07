#include "robot_select_init .h"
// Car_Mode for Mec
// 0:高配麦轮无轴承座SENIOR_MEC_NO
// 1:高配麦轮摆式悬挂SENIOR_MEC_BS
// 2:高配麦轮独立悬挂SENIOR_MEC_DL
// 3:顶配麦轮摆式悬挂常规型TOP_MEC_BS_18
// 4:顶配麦轮摆式悬挂重载型TOP_MEC_BS_47
// 5:顶配麦轮独立悬挂常规型TOP_MEC_DL_18
Robot_Parament_InitTypeDef Robot_Parament; // 初始化机器人参数结构体
void Robot_Select(void)
{
	Divisor_Mode = 4096 / CAR_NUMBER + 5;
	Car_Mode = (int)(Get_adc_Average(CAR_MODE_ADC, 10) / Divisor_Mode); // 采集电位器引脚信息
// Car_Mode=1;
#if Mec
	{
		if (Car_Mode == 0)
			Robot_Init(SENIOR_MEC_NO_wheelspacing, MD36N_27, Photoelectric_500, Mecanum_100, SENIOR_MEC_NO_axlespacing); // 高配麦轮无轴承座SENIOR_MEC_NO
		if (Car_Mode == 1)
			Robot_Init(SENIOR_MEC_BS_wheelspacing, MD36N_27, Photoelectric_500, Mecanum_100, SENIOR_MEC_BS_axlespacing); // 高配麦轮摆式悬挂SENIOR_MEC_BS

		if (Car_Mode == 2)
			Robot_Init(SENIOR_MEC_DL_wheelspacing, MD36N_27, Photoelectric_500, Mecanum_152, SENIOR_MEC_DL_axlespacing); // 高配麦轮独立悬挂SENIOR_MEC_DL
		if (Car_Mode == 3)
			Robot_Init(TOP_MEC_BS_wheelspacing, MD60N_18, Photoelectric_500, Mecanum_152, TOP_MEC_BS_axlespacing); // 顶配麦轮摆式悬挂常规型TOP_MEC_BS_18

		if (Car_Mode == 4)
			Robot_Init(TOP_MEC_BS_wheelspacing, MD60N_47, Photoelectric_500, Mecanum_152, TOP_MEC_BS_axlespacing); // 顶配麦轮摆式悬挂重载型TOP_MEC_BS_47
		if (Car_Mode == 5)
			Robot_Init(TOP_MEC_DL_wheelspacing, MD60N_18, Photoelectric_500, Mecanum_152, TOP_MEC_DL_axlespacing); // 顶配麦轮独立悬挂常规型TOP_MEC_DL_18

		// if (Car_Mode==6)  Robot_Init(TOP_MEC_DL_wheelspacing_Customized,MD60N_18,Photoelectric_500,Mecanum_152,TOP_MEC_DL_axlespacing_Customized);      //定制专用
		if (Car_Mode == 6)
			Robot_Init(SENIOR_MEC_DL_wheelspacing, MD36N_51, Photoelectric_500, Mecanum_152, SENIOR_MEC_DL_axlespacing); // 定制专用
	}
#elif Omni
	{
		if (Car_Mode == 0)
			Robot_Init(Omni_Turn_Radiaus_164, MD36N_5_18, Photoelectric_500, FullDirecion_75); // 高配全向轮三角形极速  SENIOR_OMNI_5_18 0
		if (Car_Mode == 1)
			Robot_Init(Omni_Turn_Radiaus_180, MD36N_27, Photoelectric_500, FullDirecion_127); // 高配全向轮三角形常规  SENIOR_OMNI_27   0
		if (Car_Mode == 2)
			Robot_Init(Omni_Turn_Radiaus_180, MD36N_27, Photoelectric_500, FullDirecion_127); // 高配全向轮圆形常规    SENIOR_OMNI_27   0
		if (Car_Mode == 3)
			Robot_Init(Omni_Turn_Radiaus_180, MD36N_51, Photoelectric_500, FullDirecion_127); // 高配全向轮圆形重载    SENIOR_OMNI_51   1

		if (Car_Mode == 4)
			Robot_Init(Omni_Turn_Radiaus_290, MD60N_18, Photoelectric_500, FullDirecion_127); // 顶配全向轮重载直径127 TOP_OMNI_18      2
		if (Car_Mode == 5)
			Robot_Init(Omni_Turn_Radiaus_290, MD60N_18, Photoelectric_500, FullDirecion_152); // 顶配全向轮重载直径152 TOP_OMNI_18      2
		if (Car_Mode == 6)
			Robot_Init(Omni_Turn_Radiaus_290, MD60N_18, Photoelectric_500, FullDirecion_203); // 顶配全向轮重载直径203 TOP_OMNI_18      2
	}
#endif
}

#if Mec
void Robot_Init(float wheelspacing, int gearratio, int Accuracy, float tyre_diameter, float axlespacing) //
{
	Robot_Parament.WheelSpacing = wheelspacing;														  // 半轮距
	Robot_Parament.GearRatio = gearratio;															  // 电机减速比
	Robot_Parament.EncoderAccuracy = Accuracy;														  // 编码器精度(编码器线数)
	Robot_Parament.WheelDiameter = tyre_diameter;													  // 主动轮轮径
	Robot_Parament.AxleSpacing = axlespacing;														  // 半轴距
	Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio; // 编码器精度
	Wheel_perimeter = Robot_Parament.WheelDiameter * PI;											  // 轮子周长
	Wheel_spacing = Robot_Parament.WheelSpacing;													  // 半轮距
	Axle_spacing = Robot_Parament.AxleSpacing;														  // 半轴距
}

#elif Omni
void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter) //
{
	Robot_Parament.OmniTurnRadiaus = omni_turn_radiaus;												  // 全向轮小车旋转半径
	Robot_Parament.GearRatio = gearratio;															  // 电机减速比
	Robot_Parament.EncoderAccuracy = Accuracy;														  // 编码器精度(编码器线数)
	Robot_Parament.WheelDiameter = tyre_diameter;													  // 主动轮轮径
	Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio; // 编码器精度
	Wheel_perimeter = Robot_Parament.WheelDiameter * PI;											  // 轮子周长
	Omni_turn_radiaus = Robot_Parament.OmniTurnRadiaus;												  // 全向轮小车旋转半径
}
#endif
