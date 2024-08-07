#include "led.h"

/**************************************************************************
函数功能：控制LED灯的IO初始化
入口参数：无
返回  值：无
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
      LED0 = 0; // 获取陀螺仪零点前主控LED等常亮
    else
      LED0 = ~LED0;  // 获取陀螺仪零点后LED灯开始闪烁
    vTaskDelay(500); // 相对延时函数
  }
}
