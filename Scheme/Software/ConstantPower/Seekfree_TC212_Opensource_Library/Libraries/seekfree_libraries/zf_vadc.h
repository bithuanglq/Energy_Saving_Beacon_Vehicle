/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		zf_vadc
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC212
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-12
 ********************************************************************************************************************/
 
#ifndef _zf_vadc_h
#define _zf_vadc_h

#include "common.h"



//此枚举定义不允许用户修改
typedef enum    // 枚举ADC编号
{
	ADC_0,
	ADC_1,
}VADCN_enum;

typedef enum  // 枚举ADC通道
{
    //ADC0可选引脚
    ADC0_CH5_A5 = 0*16 + 5,
    ADC0_CH6_A6,
    ADC0_CH7_A7,
    ADC0_CH8_A8,
    ADC0_CH9_A9,
    ADC0_CH10_A10,
    ADC0_CH11_A11,

    //ADC1可选引脚
    ADC1_CH0_A12  = 1*16 + 0,
    ADC1_CH1_A13,
    ADC1_CH2_A14,
    ADC1_CH3_A15,
    ADC1_CH4_A16,
    ADC1_CH5_A17,
    ADC1_CH6_A18,
}VADC_CHN_enum;

//此枚举定义不允许用户修改
typedef enum    // 枚举ADC通道
{
    ADC_8BIT,     //8位分辨率
    ADC_10BIT,    //10位分辨率
    ADC_12BIT,    //12位分辨率
}VADC_RES_enum;

void adc_init(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn);
uint16 adc_convert(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res);
uint16 adc_mean_filter(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res, uint8 count);
uint16 adc_recursive_filter(uint16 adc_value);
uint16 adc_recursive_filter2(uint16 adc_value);
uint16 adc_rc_filter(uint16 adc_value);
uint16 adc_rc_filter2(uint16 adc_value);
#endif
