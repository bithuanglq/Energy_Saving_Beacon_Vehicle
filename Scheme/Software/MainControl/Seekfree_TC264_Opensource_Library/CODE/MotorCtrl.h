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
//���ʹ�õĴ���
#define uart_motor  UART_3
//�ٶ�
#define spd_slow    1000
#define spd_inter   8000
#define spd_fast    13000
//�Ƕ� ��λ �ȡ�
#define deg_1       45
#define deg_5       228
#define deg_10      455
#define deg_30      1365
#define deg_90      4096
#define deg_180     8192
#define deg_360     16384
// ���±���������cpu0_main.c  �˴�Ϊ����
extern sint32 KP1;
extern sint32 KP2;
extern sint32 KD1;//���ٶȻ�PD����
extern sint32 KD2;//���ٶȻ�PD����
extern sint16 max;
extern sint16 min;
extern uint8 beacon_len;
extern uint16 beacon_dis;
extern vuint8 camera_change;

extern sint16 omiga_c;//ת����ٶȣ���ֵʵ�����ǽ��ٶȳ���ĳֵ��ĳֵ = ƽ���ٶ�/ƽ��ת�٣������ܲ������־�����Ϊת�ٵ�����δ֪����
                   //������ʾ��ת�����ڳ����
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
/*---------------------------------�˲�����------------------------------------*/
sint16 MovingAverageFilter(sint16 target_v);
sint16 WeightedRecursionFilter(sint16 target_v);
#endif
