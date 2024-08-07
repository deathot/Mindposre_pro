#ifndef __USRATX_H
#define __USRATX_H
#include "sys.h"
#include "system.h"

#define SENT_TASK_PRIO 4  // �������ȼ�
#define SENT_STK_SIZE 256 // �����ջ��С

// USART Receiver buffer
#define RX_BUFFER_SIZE 100 // ���ջ������ֽ���
#define FRAME_HEADER 0x7B  // ���ͺͽ������ݵ�֡ͷ
#define FRAME_TAIL 0x7D    // ���ͺͽ������ݵ�֡β
void sent_task(void *pvParameters);
void usart_data_transition(void);
void usart3_sent_data(void);
void usart3_init(u32 bound);
int USART3_IRQHandler(void);
void usart3_send(u8 data);
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode);
extern u8 Receive_Data[11];
extern u8 USART_Send_Data[24];
#endif
