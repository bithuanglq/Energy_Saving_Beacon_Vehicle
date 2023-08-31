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
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/
 
 
#include "Vadc/Adc/IfxVadc_Adc.h"
#include "zf_assert.h"
#include "zf_vadc.h"
#include "stdio.h"
#include "SEEKFREE_18TFT.h"

#define ADC_SAMPLE_FREQUENCY	10000000//最大10Mhz

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC初始化
//  @param      adcn            选择ADC模块(ADC_0、ADC_1、ADC_2)
//  @param      ch              选择ADC通道
//  @return     void
//  Sample usage:               adc_init(ADC_0, ADC0_CH0_A0);
//-------------------------------------------------------------------------------------------------------------------
void adc_init(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn)
{
	static uint8 mudule_init_flag = 0;
	IfxVadc_Adc vadc;
	IfxVadc_Adc_Group adcGroup;
    IfxVadc_Adc_Config adcConfig;

	IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);

	if(!mudule_init_flag)
	{
		mudule_init_flag = 1;
		IfxVadc_Adc_initModule(&vadc, &adcConfig);

	}
	else
	{
		vadc.vadc = adcConfig.vadc;
	}

	IfxVadc_Adc_GroupConfig adcGroupConfig;
    IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &vadc);

    adcGroupConfig.groupId = (IfxVadc_GroupId)vadc_n;//IfxVadc_GroupId_0;
    adcGroupConfig.master  = adcGroupConfig.groupId;
    adcGroupConfig.arbiter.requestSlotBackgroundScanEnabled = TRUE;
    adcGroupConfig.backgroundScanRequest.autoBackgroundScanEnabled = TRUE;
    adcGroupConfig.backgroundScanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
	adcGroupConfig.inputClass[0].resolution = IfxVadc_ChannelResolution_12bit;
	adcGroupConfig.inputClass[0].sampleTime = 1.0f/ADC_SAMPLE_FREQUENCY;
	adcGroupConfig.inputClass[1].resolution = IfxVadc_ChannelResolution_12bit;
	adcGroupConfig.inputClass[1].sampleTime = 1.0f/ADC_SAMPLE_FREQUENCY;

    IfxVadc_Adc_initGroup(&adcGroup, &adcGroupConfig);

	IfxVadc_Adc_ChannelConfig adcChannelConfig;
	IfxVadc_Adc_Channel       adcChannel;
	IfxVadc_Adc_initChannelConfig(&adcChannelConfig, &adcGroup);

	adcChannelConfig.channelId         = (IfxVadc_ChannelId)(vadc_chn%16);
	adcChannelConfig.resultRegister    = (IfxVadc_ChannelResult)(vadc_chn%16);
	adcChannelConfig.backgroundChannel = TRUE;

	IfxVadc_Adc_initChannel(&adcChannel, &adcChannelConfig);

	unsigned channels = (1 << adcChannelConfig.channelId);
	unsigned mask     = channels;
	IfxVadc_Adc_setBackgroundScan(&vadc, &adcGroup, channels, mask);

	IfxVadc_Adc_startBackgroundScan(&vadc);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC转换一次
//  @param      adcn            选择ADC模块(ADC_0、ADC_1、ADC_2)
//  @param      ch              选择ADC通道
//  @return     void
//  Sample usage:               adc_convert(ADC_0, ADC0_CH0_A0, ADC_12BIT);
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_convert(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res)
{
	Ifx_VADC_RES result;
	uint8 temp;
	do
	{
		result = IfxVadc_getResult(&MODULE_VADC.G[vadc_n], vadc_chn%16);
	} while (!result.B.VF);

	temp = 4 - (vadc_res * 2);
	return((result.U&0x0fff)>>temp);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC均值滤波
//  @param      adcn            选择ADC模块(ADC_0、ADC_1、ADC_2)
//  @param      ch              选择ADC通道
//  @param      count           均值滤波次数
//  @return     void
//  Sample usage:               adc_mean_filter(ADC_0, ADC0_CH0_A0, ADC_12BIT, 5);//采集5次 然后返回平均值
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_mean_filter(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res, uint8 count)
{
    uint8 i;
    uint32 sum;

    ZF_ASSERT(count);//断言次数不能为0

    sum = 0;
    for(i=0; i<count; i++)
    {
        sum += adc_convert(vadc_n, vadc_chn, vadc_res);
    }

    sum = sum/count;
    return (uint16)sum;
}





//卡尔曼滤波相关参数
extern float ADC_OLD_Value=0;
extern float kalman_adc_old=0;
extern float P_k1_k1=0;
extern float Kg = 0;
extern float P_k_k1 = 1;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC卡尔曼滤波一次
//  @param      ADC_Value       采样值或者上一次卡尔曼滤波值
//  @return     uint16          滤波后的值
//  @note           超参数 :Q R 已调好
//  Sample usage:               adc_result = kalman_filter(adc_result);     //卡尔曼滤波迭代
//-------------------------------------------------------------------------------------------------------------------
uint16 kalman_filter(uint16 ADC_Value)
{
    float x_k1_k1,x_k_k1;
    float Z_k;
    static float Q = 0.0500;            //Q越大代表越相信测量值  Q越小代表越相信预测值     Q增大，动态响应变快，收敛稳定性变坏：    Q 500  R 5 测距
    static float R = 5;                 //R增大，动态响应变慢，收敛稳定性变好

    float kalman_adc;

    Z_k = (float)ADC_Value;

//    if (fabs(kalman_adc_old-ADC_Value)>=100)      //一阶滞后  卡尔曼滤波的优化
//    {
//        x_k1_k1= ADC_Value*0.382 + kalman_adc_old*0.618;
//    }else
    {
        x_k1_k1 = kalman_adc_old;
    }
    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = (float)ADC_Value;
    kalman_adc_old = kalman_adc;

    return (uint16)(kalman_adc);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC卡尔曼滤波
//  @param      adcn            选择ADC模块(ADC_0、ADC_1、ADC_2)
//  @param      ch              选择ADC通道
//  @param      count           卡尔曼滤波次数
//  @return     uint16          滤波后的稳态电压值   0~3.3V
//  Sample usage:               adc_kalman(ADC_1, ADC1_CH0_A16, ADC_12BIT, 100000);//Kalman 滤波  迭代 100000 次
//-------------------------------------------------------------------------------------------------------------------
float adc_kalman(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res, uint32 times)
{
    //就用这组参数    Q + R + times
    //A16 接到 2.0V -->  2600
    // 以下代码耗时  104ms
#if 0
    //注释掉 则每次进入该函数都会使用下列参数的历史值
    {   //初始化
        ADC_OLD_Value=0;
        kalman_adc_old=0;
        P_k1_k1=0;
        Kg = 0;
        P_k_k1 = 1;
    }
#endif
    uint16 adc_result;
    adc_result = adc_convert(vadc_n,vadc_chn,vadc_res);
    for(uint32 i=0;i<times;i++)adc_result = kalman_filter(adc_result);     //200 times -->  3.4ms


//    return (float)adc_result*2.0/2600;               //287us 测距
    return (float)adc_result*26.16/2600;               //287us
}


/*---------------------------------惯性滤波-------------------------------*/
extern uint16 last_adc_result = 0;
float coe = 0.800;                  //RC 系数
uint16 adc_rcfilter(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res)
{

    uint16 adc_result;
    adc_result = adc_convert(vadc_n,vadc_chn,vadc_res);
    lcd_showuint16(0,0,adc_result);
    float ans = coe*adc_result + (1-coe)*last_adc_result;


    last_adc_result = adc_result;

    return (uint16)ans;
}


/*---------------------------------加权递推采样----------------------------*/
#ifndef FILTER_N
#define FILTER_N  20
const int sum_coe = (FILTER_N + 1)*FILTER_N/2;
int filter_buff[FILTER_N + 1];
#endif

float adc_Weightedrecursion(VADCN_enum vadc_n, VADC_CHN_enum vadc_chn, VADC_RES_enum vadc_res)
{
    uint8 i;
    int filter_sum = 0;
    filter_buff[FILTER_N] = (int)adc_rcfilter(vadc_n, vadc_chn, vadc_res);
    for(i=0;i<FILTER_N;i++)
    {
        filter_buff[i] = filter_buff[i+1];
        filter_sum += filter_buff[i]*(i+1);
    }
    filter_sum /= sum_coe;

    float Cap_V = 12.0/1210 * filter_sum;
    return Cap_V;
}
