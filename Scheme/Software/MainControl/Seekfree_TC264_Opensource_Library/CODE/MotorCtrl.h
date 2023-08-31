#ifndef MotorCtrl_H
#define MotorCtrl_H
/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "common.h"
#include "zf_uart.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_ICM20602.h"
/******************************************************************************/
/*---------------------------------Define---------------------------------*/
/******************************************************************************/
//电机使用的串口
#define uart_motor  UART_3
//速度
#define spd_slow    1000
#define spd_inter   8000
#define spd_fast    13000
//角度 单位 度°
#define deg_1       45
#define deg_5       228
#define deg_10      455
#define deg_30      1365
#define deg_90      4096
#define deg_180     8192
#define deg_360     16384
// 以下变量定义在cpu0_main.c  此处为声明
extern sint32 KP1;
extern sint32 KP2;
extern sint32 KD1;//角速度环PD参数
extern sint32 KD2;//角速度环PD参数
extern sint16 max;
extern sint16 min;
extern uint8 beacon_len;
extern uint16 beacon_dis;
extern vuint8 camera_change;

extern sint16 omiga_c;//转弯角速度，该值实际上是角速度除以某值（某值 = 平均速度/平均转速，但可能并不是轮径，因为转速的量纲未知），
                   //负数表示旋转中心在车左侧
extern sint16 ramp_v;
extern int16 icm_gyro_z,icm_acc_y;
/******************************************************************************/
/*-------------------------------Functions------------------------------------*/
/******************************************************************************/

void MotorStop();
void AngularVelocityLoop(sint16 n_c, sint16 bias);
void SendSpeedCtrlCode(sint16 n_l,sint16 n_r);
void IncrementAngleCtrl(sint16 angle_inc_l,sint16 angle_inc_r);
void SetMotorPID(uint8 I_threshold,uint8 U_threshold,
        float32 angle_Kp,float32 angle_speed,
        float32 speed_Kp,float32 speed_Ki);
void float2uint8(uint8 *buf,float32 data);
void Circling(sint16 n);
void N_CRC16(uint8 *updata,uint8 len);
void Guiding(sint16);
void GyroscopeDampingLoop(sint16 omiga_c);
/*---------------------------------滤波函数------------------------------------*/
sint16 MovingAverageFilter(sint16 target_v);
sint16 WeightedRecursionFilter(sint16 target_v);
#endif
