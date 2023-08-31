/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		gtm_pwm
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC212
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-12
 ********************************************************************************************************************/
 
#ifndef _zf_gtm_pwm_h
#define _zf_gtm_pwm_h

#include "common.h"
#include "ifxGtm_PinMap.h"

#define GTM_TOM0_PWM_DUTY_MAX     10000                 //GTM_TOM0 PWM最大占空比  最大占空比越大占空比的步进值越小
#define GTM_TOM1_PWM_DUTY_MAX     10000                 //GTM_TOM1 PWM最大占空比  最大占空比越大占空比的步进值越小
#define GTM_TOM2_PWM_DUTY_MAX     10000                 //GTM_TOM2 PWM最大占空比  最大占空比越大占空比的步进值越小
#define GTM_TOM3_PWM_DUTY_MAX     10000                 //GTM_TOM3 PWM最大占空比  最大占空比越大占空比的步进值越小


//此枚举定义不允许用户修改
typedef enum //枚举串口引脚
{
    TOM0_CH0_P15_5,  TOM0_CH0_P33_10,
    TOM0_CH1_P33_5,  TOM0_CH1_P33_9,
    TOM0_CH2_P10_5,  TOM0_CH2_P33_6,
    TOM0_CH3_P10_6,  TOM0_CH3_P14_0,   TOM0_CH3_P33_7,
    TOM0_CH4_P00_0,  TOM0_CH4_P02_0,   TOM0_CH4_P02_1,   TOM0_CH4_P02_8,  TOM0_CH4_P11_2,  TOM0_CH4_P14_1, TOM0_CH4_P15_1,  TOM0_CH4_P15_2,  TOM0_CH4_P20_8,  TOM0_CH4_P20_9,  TOM0_CH4_P21_6, TOM0_CH4_P33_8,
    TOM0_CH5_P02_3,  TOM0_CH5_P02_2,   TOM0_CH5_P11_3,   TOM0_CH5_P11_6,  TOM0_CH5_P15_3,  TOM0_CH5_P15_5, TOM0_CH5_P20_11, TOM0_CH5_P21_7,  TOM0_CH5_P33_5,  TOM0_CH5_P33_6,
    TOM0_CH6_P02_4,  TOM0_CH6_P02_5,   TOM0_CH6_P11_9,   TOM0_CH6_P11_10, TOM0_CH6_P14_0,  TOM0_CH6_P14_3, TOM0_CH6_P20_12, TOM0_CH6_P20_13, TOM0_CH6_P23_1,  TOM0_CH6_P33_7,  TOM0_CH6_P33_8,
    TOM0_CH7_P02_6,  TOM0_CH7_P02_7,   TOM0_CH7_P11_11,  TOM0_CH7_P11_12, TOM0_CH7_P14_1,  TOM0_CH7_P14_4, TOM0_CH7_P15_0,  TOM0_CH7_P20_8,  TOM0_CH7_P20_14, TOM0_CH7_P33_9,  TOM0_CH7_P33_10,
    TOM0_CH8_P00_0,  TOM0_CH8_P02_0,   TOM0_CH8_P02_8,   TOM0_CH8_P11_2,  TOM0_CH8_P20_12,
    TOM0_CH9_P02_1,  TOM0_CH9_P20_13,
    TOM0_CH10_P02_2, TOM0_CH10_P11_3,  TOM0_CH10_P20_14,
    TOM0_CH11_P02_3, TOM0_CH11_P11_6,  TOM0_CH11_P15_0,
    TOM0_CH12_P02_4, TOM0_CH12_P11_9,  TOM0_CH12_P15_1,
    TOM0_CH13_P02_5, TOM0_CH13_P11_10, TOM0_CH13_P15_2,  TOM0_CH13_P20_9,
    TOM0_CH14_P02_6, TOM0_CH14_P11_11, TOM0_CH14_P15_3,
    TOM0_CH15_P02_7, TOM0_CH15_P11_12, TOM0_CH15_P20_11, TOM0_CH15_P23_1,

    TOM1_CH0_P00_0,  TOM1_CH0_P02_8,   TOM1_CH0_P15_5,   TOM1_CH0_P20_12, TOM1_CH0_P33_10,
    TOM1_CH1_P11_2,  TOM1_CH1_P20_13,  TOM1_CH1_P33_5,   TOM1_CH1_P33_9,
    TOM1_CH2_P11_3,  TOM1_CH2_P20_14,  TOM1_CH2_P33_6,
    TOM1_CH3_P11_6,  TOM1_CH3_P14_0,   TOM1_CH3_P15_0,   TOM1_CH3_P33_7,
    TOM1_CH4_P00_0,  TOM1_CH4_P02_0,   TOM1_CH4_P02_1,   TOM1_CH4_P02_8,  TOM1_CH4_P11_2,  TOM1_CH4_P11_9, TOM1_CH4_P14_1,  TOM1_CH4_P15_1,  TOM1_CH4_P15_2,  TOM1_CH4_P20_8,  TOM1_CH4_P20_9, TOM1_CH4_P21_6, TOM1_CH4_P33_8,
    TOM1_CH5_P02_2,  TOM1_CH5_P02_3,   TOM1_CH5_P11_3,   TOM1_CH5_P11_6,  TOM1_CH5_P11_10, TOM1_CH5_P15_2, TOM1_CH5_P15_3,  TOM1_CH5_P15_5,  TOM1_CH5_P20_11, TOM1_CH5_P21_7,  TOM1_CH5_P33_5, TOM1_CH5_P33_6,
    TOM1_CH6_P02_4,  TOM1_CH6_P02_5,   TOM1_CH6_P11_9,   TOM1_CH6_P11_10, TOM1_CH6_P11_11, TOM1_CH6_P14_0, TOM1_CH6_P14_3,  TOM1_CH6_P15_3,  TOM1_CH6_P20_12, TOM1_CH6_P20_13, TOM1_CH6_P33_7, TOM1_CH6_P33_8,
    TOM1_CH7_P02_6,  TOM1_CH7_P02_7,   TOM1_CH7_P11_11,  TOM1_CH7_P11_12, TOM1_CH7_P14_1,  TOM1_CH7_P14_4, TOM1_CH7_P15_0,  TOM1_CH7_P20_8,  TOM1_CH7_P20_14, TOM1_CH7_P33_9,  TOM1_CH7_P33_10,
    TOM1_CH8_P02_0,
    TOM1_CH9_P02_1,
    TOM1_CH10_P02_2, TOM1_CH10_P10_5,
    TOM1_CH11_P02_3, TOM1_CH11_P10_6,
    TOM1_CH12_P02_4,
    TOM1_CH13_P02_5, TOM1_CH13_P20_9,
    TOM1_CH14_P02_6,
    TOM1_CH15_P02_7, TOM1_CH15_P20_11,
}TOM_PIN_enum;

void gtm_pwm_init(TOM_PIN_enum pwmch, uint32 freq, uint32 duty);
void pwm_duty(TOM_PIN_enum pwmch, uint32 duty);



#endif
