#include "robot_select_init .h"
// Car_Mode for Mec
// 0:���������������SENIOR_MEC_NO
// 1:�������ְ�ʽ����SENIOR_MEC_BS
// 2:�������ֶ�������SENIOR_MEC_DL
// 3:�������ְ�ʽ���ҳ�����TOP_MEC_BS_18
// 4:�������ְ�ʽ����������TOP_MEC_BS_47
// 5:�������ֶ������ҳ�����TOP_MEC_DL_18
Robot_Parament_InitTypeDef Robot_Parament; // ��ʼ�������˲����ṹ��
void Robot_Select(void)
{
	Divisor_Mode = 4096 / CAR_NUMBER + 5;
	Car_Mode = (int)(Get_adc_Average(CAR_MODE_ADC, 10) / Divisor_Mode); // �ɼ���λ��������Ϣ
// Car_Mode=1;
#if Mec
	{
		if (Car_Mode == 0)
			Robot_Init(SENIOR_MEC_NO_wheelspacing, MD36N_27, Photoelectric_500, Mecanum_100, SENIOR_MEC_NO_axlespacing); // ���������������SENIOR_MEC_NO
		if (Car_Mode == 1)
			Robot_Init(SENIOR_MEC_BS_wheelspacing, MD36N_27, Photoelectric_500, Mecanum_100, SENIOR_MEC_BS_axlespacing); // �������ְ�ʽ����SENIOR_MEC_BS

		if (Car_Mode == 2)
			Robot_Init(SENIOR_MEC_DL_wheelspacing, MD36N_27, Photoelectric_500, Mecanum_152, SENIOR_MEC_DL_axlespacing); // �������ֶ�������SENIOR_MEC_DL
		if (Car_Mode == 3)
			Robot_Init(TOP_MEC_BS_wheelspacing, MD60N_18, Photoelectric_500, Mecanum_152, TOP_MEC_BS_axlespacing); // �������ְ�ʽ���ҳ�����TOP_MEC_BS_18

		if (Car_Mode == 4)
			Robot_Init(TOP_MEC_BS_wheelspacing, MD60N_47, Photoelectric_500, Mecanum_152, TOP_MEC_BS_axlespacing); // �������ְ�ʽ����������TOP_MEC_BS_47
		if (Car_Mode == 5)
			Robot_Init(TOP_MEC_DL_wheelspacing, MD60N_18, Photoelectric_500, Mecanum_152, TOP_MEC_DL_axlespacing); // �������ֶ������ҳ�����TOP_MEC_DL_18

		// if (Car_Mode==6)  Robot_Init(TOP_MEC_DL_wheelspacing_Customized,MD60N_18,Photoelectric_500,Mecanum_152,TOP_MEC_DL_axlespacing_Customized);      //����ר��
		if (Car_Mode == 6)
			Robot_Init(SENIOR_MEC_DL_wheelspacing, MD36N_51, Photoelectric_500, Mecanum_152, SENIOR_MEC_DL_axlespacing); // ����ר��
	}
#elif Omni
	{
		if (Car_Mode == 0)
			Robot_Init(Omni_Turn_Radiaus_164, MD36N_5_18, Photoelectric_500, FullDirecion_75); // ����ȫ���������μ���  SENIOR_OMNI_5_18 0
		if (Car_Mode == 1)
			Robot_Init(Omni_Turn_Radiaus_180, MD36N_27, Photoelectric_500, FullDirecion_127); // ����ȫ���������γ���  SENIOR_OMNI_27   0
		if (Car_Mode == 2)
			Robot_Init(Omni_Turn_Radiaus_180, MD36N_27, Photoelectric_500, FullDirecion_127); // ����ȫ����Բ�γ���    SENIOR_OMNI_27   0
		if (Car_Mode == 3)
			Robot_Init(Omni_Turn_Radiaus_180, MD36N_51, Photoelectric_500, FullDirecion_127); // ����ȫ����Բ������    SENIOR_OMNI_51   1

		if (Car_Mode == 4)
			Robot_Init(Omni_Turn_Radiaus_290, MD60N_18, Photoelectric_500, FullDirecion_127); // ����ȫ��������ֱ��127 TOP_OMNI_18      2
		if (Car_Mode == 5)
			Robot_Init(Omni_Turn_Radiaus_290, MD60N_18, Photoelectric_500, FullDirecion_152); // ����ȫ��������ֱ��152 TOP_OMNI_18      2
		if (Car_Mode == 6)
			Robot_Init(Omni_Turn_Radiaus_290, MD60N_18, Photoelectric_500, FullDirecion_203); // ����ȫ��������ֱ��203 TOP_OMNI_18      2
	}
#endif
}

#if Mec
void Robot_Init(float wheelspacing, int gearratio, int Accuracy, float tyre_diameter, float axlespacing) //
{
	Robot_Parament.WheelSpacing = wheelspacing;														  // ���־�
	Robot_Parament.GearRatio = gearratio;															  // ������ٱ�
	Robot_Parament.EncoderAccuracy = Accuracy;														  // ����������(����������)
	Robot_Parament.WheelDiameter = tyre_diameter;													  // �������־�
	Robot_Parament.AxleSpacing = axlespacing;														  // �����
	Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio; // ����������
	Wheel_perimeter = Robot_Parament.WheelDiameter * PI;											  // �����ܳ�
	Wheel_spacing = Robot_Parament.WheelSpacing;													  // ���־�
	Axle_spacing = Robot_Parament.AxleSpacing;														  // �����
}

#elif Omni
void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter) //
{
	Robot_Parament.OmniTurnRadiaus = omni_turn_radiaus;												  // ȫ����С����ת�뾶
	Robot_Parament.GearRatio = gearratio;															  // ������ٱ�
	Robot_Parament.EncoderAccuracy = Accuracy;														  // ����������(����������)
	Robot_Parament.WheelDiameter = tyre_diameter;													  // �������־�
	Encoder_precision = EncoderMultiples * Robot_Parament.EncoderAccuracy * Robot_Parament.GearRatio; // ����������
	Wheel_perimeter = Robot_Parament.WheelDiameter * PI;											  // �����ܳ�
	Omni_turn_radiaus = Robot_Parament.OmniTurnRadiaus;												  // ȫ����С����ת�뾶
}
#endif
