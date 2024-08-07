#include "led.h"

/**************************************************************************
�������ܣ�����LED�Ƶ�IO��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void led_task(void *pvParameters)
{
  while (1)
  {
    if (start_up_15_second == 0)
      LED0 = 0; // ��ȡ���������ǰ����LED�ȳ���
    else
      LED0 = ~LED0;  // ��ȡ����������LED�ƿ�ʼ��˸
    vTaskDelay(500); // �����ʱ����
  }
}
