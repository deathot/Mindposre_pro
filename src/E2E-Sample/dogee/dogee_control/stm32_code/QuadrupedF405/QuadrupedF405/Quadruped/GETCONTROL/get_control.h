#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "system.h"

#define GETCONTROL_TASK_PRIO 4
#define GETCONTROL_STK_SIZE 256
void getcontrol_task(void *pvParameters);
void PS2_control(void);
void Get_RC(void);
void Remote_control(void);
void usart_control(void);
void control_transition(int time);
float target_limit_float(float insert, float low, float high);
void Smooth_pose(float vx, float vz);
float float_abs(float insert);

extern short RC_Velocity;
extern int move_x, move_y, move_z, pose_x, pose_y;
extern float smooth_pose_x, smooth_pose_y;

#define front_behind_distance 340
#define left_right_distance 285

#endif
