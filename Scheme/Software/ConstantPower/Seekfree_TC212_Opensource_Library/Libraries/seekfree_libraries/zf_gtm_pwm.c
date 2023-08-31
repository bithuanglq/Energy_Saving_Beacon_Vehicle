/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            gtm_pwm
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        tasking v6.3r1
 * @Target core     TC212
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-12
 ********************************************************************************************************************/
 
 

#include "IfxGtm_Tom_Pwm.h"
#include "zf_gtm_pwm.h"


#define CMU_CLK_FREQ           3125000.0f                       //CMU时钟频率


IfxGtm_Tom_ToutMap* gtm_TOM_mux(TOM_PIN_enum TOM_pin)
{
    switch(TOM_pin)
    {
        case TOM0_CH0_P33_10: return &IfxGtm_TOM0_0_TOUT32_P33_10_OUT;
        case TOM0_CH0_P15_5: return &IfxGtm_TOM0_0_TOUT76_P15_5_OUT;

        case TOM0_CH1_P33_5: return &IfxGtm_TOM0_1_TOUT27_P33_5_OUT;
        case TOM0_CH1_P33_9: return &IfxGtm_TOM0_1_TOUT31_P33_9_OUT;

        case TOM0_CH2_P10_5: return &IfxGtm_TOM0_2_TOUT107_P10_5_OUT;
        case TOM0_CH2_P33_6: return &IfxGtm_TOM0_2_TOUT28_P33_6_OUT;

        case TOM0_CH3_P10_6: return &IfxGtm_TOM0_3_TOUT108_P10_6_OUT;
        case TOM0_CH3_P33_7: return &IfxGtm_TOM0_3_TOUT29_P33_7_OUT;
        case TOM0_CH3_P14_0: return &IfxGtm_TOM0_3_TOUT80_P14_0_OUT;

        case TOM0_CH4_P02_0: return &IfxGtm_TOM0_4_TOUT0_P02_0_OUT;
        case TOM0_CH4_P02_1: return &IfxGtm_TOM0_4_TOUT1_P02_1_OUT;
        case TOM0_CH4_P33_8: return &IfxGtm_TOM0_4_TOUT30_P33_8_OUT;
        case TOM0_CH4_P21_6: return &IfxGtm_TOM0_4_TOUT57_P21_6_OUT;
        case TOM0_CH4_P20_8: return &IfxGtm_TOM0_4_TOUT64_P20_8_OUT;
        case TOM0_CH4_P20_9: return &IfxGtm_TOM0_4_TOUT65_P20_9_OUT;
        case TOM0_CH4_P15_1: return &IfxGtm_TOM0_4_TOUT72_P15_1_OUT;
        case TOM0_CH4_P15_2: return &IfxGtm_TOM0_4_TOUT73_P15_2_OUT;
        case TOM0_CH4_P14_1: return &IfxGtm_TOM0_4_TOUT81_P14_1_OUT;
        case TOM0_CH4_P02_8: return &IfxGtm_TOM0_4_TOUT8_P02_8_OUT;
        case TOM0_CH4_P11_2: return &IfxGtm_TOM0_4_TOUT95_P11_2_OUT;
        case TOM0_CH4_P00_0: return &IfxGtm_TOM0_4_TOUT9_P00_0_OUT;

        case TOM0_CH5_P33_5: return &IfxGtm_TOM0_5_TOUT27_P33_5_OUT;
        case TOM0_CH5_P33_6: return &IfxGtm_TOM0_5_TOUT28_P33_6_OUT;
        case TOM0_CH5_P02_2: return &IfxGtm_TOM0_5_TOUT2_P02_2_OUT;
        case TOM0_CH5_P02_3: return &IfxGtm_TOM0_5_TOUT3_P02_3_OUT;
        case TOM0_CH5_P21_7: return &IfxGtm_TOM0_5_TOUT58_P21_7_OUT;
        case TOM0_CH5_P20_11: return &IfxGtm_TOM0_5_TOUT67_P20_11_OUT;
        case TOM0_CH5_P15_3: return &IfxGtm_TOM0_5_TOUT74_P15_3_OUT;
        case TOM0_CH5_P15_5: return &IfxGtm_TOM0_5_TOUT76_P15_5_OUT;
        case TOM0_CH5_P11_3: return &IfxGtm_TOM0_5_TOUT96_P11_3_OUT;
        case TOM0_CH5_P11_6: return &IfxGtm_TOM0_5_TOUT97_P11_6_OUT;

        case TOM0_CH6_P33_7: return &IfxGtm_TOM0_6_TOUT29_P33_7_OUT;
        case TOM0_CH6_P33_8: return &IfxGtm_TOM0_6_TOUT30_P33_8_OUT;
        case TOM0_CH6_P23_1: return &IfxGtm_TOM0_6_TOUT42_P23_1_OUT;
        case TOM0_CH6_P02_4: return &IfxGtm_TOM0_6_TOUT4_P02_4_OUT;
        case TOM0_CH6_P02_5: return &IfxGtm_TOM0_6_TOUT5_P02_5_OUT;
        case TOM0_CH6_P20_12: return &IfxGtm_TOM0_6_TOUT68_P20_12_OUT;
        case TOM0_CH6_P20_13: return &IfxGtm_TOM0_6_TOUT69_P20_13_OUT;
        case TOM0_CH6_P14_0: return &IfxGtm_TOM0_6_TOUT80_P14_0_OUT;
        case TOM0_CH6_P14_3: return &IfxGtm_TOM0_6_TOUT83_P14_3_OUT;
        case TOM0_CH6_P11_9: return &IfxGtm_TOM0_6_TOUT98_P11_9_OUT;
        case TOM0_CH6_P11_10: return &IfxGtm_TOM0_6_TOUT99_P11_10_OUT;

        case TOM0_CH7_P11_11: return &IfxGtm_TOM0_7_TOUT100_P11_11_OUT;
        case TOM0_CH7_P11_12: return &IfxGtm_TOM0_7_TOUT101_P11_12_OUT;
        case TOM0_CH7_P33_9: return &IfxGtm_TOM0_7_TOUT31_P33_9_OUT;
        case TOM0_CH7_P33_10: return &IfxGtm_TOM0_7_TOUT32_P33_10_OUT;
        case TOM0_CH7_P20_8: return &IfxGtm_TOM0_7_TOUT64_P20_8_OUT;
        case TOM0_CH7_P02_6: return &IfxGtm_TOM0_7_TOUT6_P02_6_OUT;
        case TOM0_CH7_P20_14: return &IfxGtm_TOM0_7_TOUT70_P20_14_OUT;
        case TOM0_CH7_P15_0: return &IfxGtm_TOM0_7_TOUT71_P15_0_OUT;
        case TOM0_CH7_P02_7: return &IfxGtm_TOM0_7_TOUT7_P02_7_OUT;
        case TOM0_CH7_P14_1: return &IfxGtm_TOM0_7_TOUT81_P14_1_OUT;
        case TOM0_CH7_P14_4: return &IfxGtm_TOM0_7_TOUT84_P14_4_OUT;

        case TOM0_CH8_P02_0: return &IfxGtm_TOM0_8_TOUT0_P02_0_OUT;
        case TOM0_CH8_P20_12: return &IfxGtm_TOM0_8_TOUT68_P20_12_OUT;
        case TOM0_CH8_P02_8: return &IfxGtm_TOM0_8_TOUT8_P02_8_OUT;
        case TOM0_CH8_P11_2: return &IfxGtm_TOM0_8_TOUT95_P11_2_OUT;
        case TOM0_CH8_P00_0: return &IfxGtm_TOM0_8_TOUT9_P00_0_OUT;

        case TOM0_CH9_P02_1: return &IfxGtm_TOM0_9_TOUT1_P02_1_OUT;
        case TOM0_CH9_P20_13: return &IfxGtm_TOM0_9_TOUT69_P20_13_OUT;

        case TOM0_CH10_P02_2: return &IfxGtm_TOM0_10_TOUT2_P02_2_OUT;
        case TOM0_CH10_P20_14: return &IfxGtm_TOM0_10_TOUT70_P20_14_OUT;
        case TOM0_CH10_P11_3: return &IfxGtm_TOM0_10_TOUT96_P11_3_OUT;

        case TOM0_CH11_P02_3: return &IfxGtm_TOM0_11_TOUT3_P02_3_OUT;
        case TOM0_CH11_P15_0: return &IfxGtm_TOM0_11_TOUT71_P15_0_OUT;
        case TOM0_CH11_P11_6: return &IfxGtm_TOM0_11_TOUT97_P11_6_OUT;

        case TOM0_CH12_P02_4: return &IfxGtm_TOM0_12_TOUT4_P02_4_OUT;
        case TOM0_CH12_P15_1: return &IfxGtm_TOM0_12_TOUT72_P15_1_OUT;
        case TOM0_CH12_P11_9: return &IfxGtm_TOM0_12_TOUT98_P11_9_OUT;

        case TOM0_CH13_P02_5: return &IfxGtm_TOM0_13_TOUT5_P02_5_OUT;
        case TOM0_CH13_P15_2: return &IfxGtm_TOM0_13_TOUT73_P15_2_OUT;
        case TOM0_CH13_P20_9: return &IfxGtm_TOM0_13_TOUT65_P20_9_OUT;
        case TOM0_CH13_P11_10: return &IfxGtm_TOM0_13_TOUT99_P11_10_OUT;

        case TOM0_CH14_P11_11: return &IfxGtm_TOM0_14_TOUT100_P11_11_OUT;
        case TOM0_CH14_P02_6: return &IfxGtm_TOM0_14_TOUT6_P02_6_OUT;
        case TOM0_CH14_P15_3: return &IfxGtm_TOM0_14_TOUT74_P15_3_OUT;

        case TOM0_CH15_P11_12: return &IfxGtm_TOM0_15_TOUT101_P11_12_OUT;
        case TOM0_CH15_P23_1: return &IfxGtm_TOM0_15_TOUT42_P23_1_OUT;
        case TOM0_CH15_P20_11: return &IfxGtm_TOM0_15_TOUT67_P20_11_OUT;
        case TOM0_CH15_P02_7: return &IfxGtm_TOM0_15_TOUT7_P02_7_OUT;

        case TOM1_CH0_P33_10: return &IfxGtm_TOM1_0_TOUT32_P33_10_OUT;
        case TOM1_CH0_P20_12: return &IfxGtm_TOM1_0_TOUT68_P20_12_OUT;
        case TOM1_CH0_P15_5: return &IfxGtm_TOM1_0_TOUT76_P15_5_OUT;
        case TOM1_CH0_P02_8: return &IfxGtm_TOM1_0_TOUT8_P02_8_OUT;
        case TOM1_CH0_P00_0: return &IfxGtm_TOM1_0_TOUT9_P00_0_OUT;

        case TOM1_CH1_P33_5: return &IfxGtm_TOM1_1_TOUT27_P33_5_OUT;
        case TOM1_CH1_P33_9: return &IfxGtm_TOM1_1_TOUT31_P33_9_OUT;
        case TOM1_CH1_P20_13: return &IfxGtm_TOM1_1_TOUT69_P20_13_OUT;
        case TOM1_CH1_P11_2: return &IfxGtm_TOM1_1_TOUT95_P11_2_OUT;

        case TOM1_CH2_P33_6: return &IfxGtm_TOM1_2_TOUT28_P33_6_OUT;
        case TOM1_CH2_P20_14: return &IfxGtm_TOM1_2_TOUT70_P20_14_OUT;
        case TOM1_CH2_P11_3: return &IfxGtm_TOM1_2_TOUT96_P11_3_OUT;

        case TOM1_CH3_P33_7: return &IfxGtm_TOM1_3_TOUT29_P33_7_OUT;
        case TOM1_CH3_P15_0: return &IfxGtm_TOM1_3_TOUT71_P15_0_OUT;
        case TOM1_CH3_P14_0: return &IfxGtm_TOM1_3_TOUT80_P14_0_OUT;
        case TOM1_CH3_P11_6: return &IfxGtm_TOM1_3_TOUT97_P11_6_OUT;
        
        case TOM1_CH4_P02_0: return &IfxGtm_TOM1_4_TOUT0_P02_0_OUT;
        case TOM1_CH4_P02_1: return &IfxGtm_TOM1_4_TOUT1_P02_1_OUT;
        case TOM1_CH4_P33_8: return &IfxGtm_TOM1_4_TOUT30_P33_8_OUT;
        case TOM1_CH4_P21_6: return &IfxGtm_TOM1_4_TOUT57_P21_6_OUT;
        case TOM1_CH4_P20_8: return &IfxGtm_TOM1_4_TOUT64_P20_8_OUT;
        case TOM1_CH4_P20_9: return &IfxGtm_TOM1_4_TOUT65_P20_9_OUT;
        case TOM1_CH4_P15_1: return &IfxGtm_TOM1_4_TOUT72_P15_1_OUT;
        case TOM1_CH4_P15_2: return &IfxGtm_TOM1_4_TOUT73_P15_2_OUT;
        case TOM1_CH4_P14_1: return &IfxGtm_TOM1_4_TOUT81_P14_1_OUT;
        case TOM1_CH4_P02_8: return &IfxGtm_TOM1_4_TOUT8_P02_8_OUT;
        case TOM1_CH4_P11_2: return &IfxGtm_TOM1_4_TOUT95_P11_2_OUT;
        case TOM1_CH4_P11_9: return &IfxGtm_TOM1_4_TOUT98_P11_9_OUT;
        case TOM1_CH4_P00_0: return &IfxGtm_TOM1_4_TOUT9_P00_0_OUT;

        case TOM1_CH5_P33_5: return &IfxGtm_TOM1_5_TOUT27_P33_5_OUT;
        case TOM1_CH5_P33_6: return &IfxGtm_TOM1_5_TOUT28_P33_6_OUT;
        case TOM1_CH5_P02_2: return &IfxGtm_TOM1_5_TOUT2_P02_2_OUT;
        case TOM1_CH5_P02_3: return &IfxGtm_TOM1_5_TOUT3_P02_3_OUT;
        case TOM1_CH5_P21_7: return &IfxGtm_TOM1_5_TOUT58_P21_7_OUT;
        case TOM1_CH5_P20_11: return &IfxGtm_TOM1_5_TOUT67_P20_11_OUT;
        case TOM1_CH5_P15_2: return &IfxGtm_TOM1_5_TOUT73_P15_2_OUT;
        case TOM1_CH5_P15_3: return &IfxGtm_TOM1_5_TOUT74_P15_3_OUT;
        case TOM1_CH5_P15_5: return &IfxGtm_TOM1_5_TOUT76_P15_5_OUT;
        case TOM1_CH5_P11_3: return &IfxGtm_TOM1_5_TOUT96_P11_3_OUT;
        case TOM1_CH5_P11_6: return &IfxGtm_TOM1_5_TOUT97_P11_6_OUT;
        case TOM1_CH5_P11_10: return &IfxGtm_TOM1_5_TOUT99_P11_10_OUT;

        case TOM1_CH6_P11_11: return &IfxGtm_TOM1_6_TOUT100_P11_11_OUT;
        case TOM1_CH6_P33_7: return &IfxGtm_TOM1_6_TOUT29_P33_7_OUT;
        case TOM1_CH6_P33_8: return &IfxGtm_TOM1_6_TOUT30_P33_8_OUT;
        case TOM1_CH6_P02_4: return &IfxGtm_TOM1_6_TOUT4_P02_4_OUT;
        case TOM1_CH6_P02_5: return &IfxGtm_TOM1_6_TOUT5_P02_5_OUT;
        case TOM1_CH6_P20_12: return &IfxGtm_TOM1_6_TOUT68_P20_12_OUT;
        case TOM1_CH6_P20_13: return &IfxGtm_TOM1_6_TOUT69_P20_13_OUT;
        case TOM1_CH6_P15_3: return &IfxGtm_TOM1_6_TOUT74_P15_3_OUT;
        case TOM1_CH6_P14_0: return &IfxGtm_TOM1_6_TOUT80_P14_0_OUT;
        case TOM1_CH6_P14_3: return &IfxGtm_TOM1_6_TOUT83_P14_3_OUT;
        case TOM1_CH6_P11_9: return &IfxGtm_TOM1_6_TOUT98_P11_9_OUT;
        case TOM1_CH6_P11_10: return &IfxGtm_TOM1_6_TOUT99_P11_10_OUT;

        case TOM1_CH7_P11_11: return &IfxGtm_TOM1_7_TOUT100_P11_11_OUT;
        case TOM1_CH7_P11_12: return &IfxGtm_TOM1_7_TOUT101_P11_12_OUT;
        case TOM1_CH7_P33_9: return &IfxGtm_TOM1_7_TOUT31_P33_9_OUT;
        case TOM1_CH7_P33_10: return &IfxGtm_TOM1_7_TOUT32_P33_10_OUT;
        case TOM1_CH7_P20_8: return &IfxGtm_TOM1_7_TOUT64_P20_8_OUT;
        case TOM1_CH7_P02_6: return &IfxGtm_TOM1_7_TOUT6_P02_6_OUT;
        case TOM1_CH7_P20_14: return &IfxGtm_TOM1_7_TOUT70_P20_14_OUT;
        case TOM1_CH7_P15_0: return &IfxGtm_TOM1_7_TOUT71_P15_0_OUT;
        case TOM1_CH7_P02_7: return &IfxGtm_TOM1_7_TOUT7_P02_7_OUT;
        case TOM1_CH7_P14_1: return &IfxGtm_TOM1_7_TOUT81_P14_1_OUT;
        case TOM1_CH7_P14_4: return &IfxGtm_TOM1_7_TOUT84_P14_4_OUT;

        case TOM1_CH8_P02_0: return &IfxGtm_TOM1_8_TOUT0_P02_0_OUT;

        case TOM1_CH9_P02_1: return &IfxGtm_TOM1_9_TOUT1_P02_1_OUT;

        case TOM1_CH10_P10_5: return &IfxGtm_TOM1_10_TOUT107_P10_5_OUT;
        case TOM1_CH10_P02_2: return &IfxGtm_TOM1_10_TOUT2_P02_2_OUT;

        case TOM1_CH11_P10_6: return &IfxGtm_TOM1_11_TOUT108_P10_6_OUT;
        case TOM1_CH11_P02_3: return &IfxGtm_TOM1_11_TOUT3_P02_3_OUT;

        case TOM1_CH12_P02_4: return &IfxGtm_TOM1_12_TOUT4_P02_4_OUT;

        case TOM1_CH13_P02_5: return &IfxGtm_TOM1_13_TOUT5_P02_5_OUT;
        case TOM1_CH13_P20_9: return &IfxGtm_TOM1_13_TOUT65_P20_9_OUT;

        case TOM1_CH14_P02_6: return &IfxGtm_TOM1_14_TOUT6_P02_6_OUT;

        case TOM1_CH15_P20_11: return &IfxGtm_TOM1_15_TOUT67_P20_11_OUT;
        case TOM1_CH15_P02_7: return &IfxGtm_TOM1_15_TOUT7_P02_7_OUT;

        default: return NULL;
    }

}
IfxGtm_Tom_Pwm_Driver g_TOMDriver;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM初始化
//  @param      pwmch       PWM通道号及引脚
//  @param      freq        PWM频率
//  @param      duty        PWM占空比
//  @return     void
//  Sample usage:           gtm_pwm_init(TOM0_CH7_P02_7, 50, 1000);     //TOM 0模块的通道7 使用P02_7引脚输出PWM  PWM频率50HZ  占空比百分之1000/GTM_TOM0_PWM_DUTY_MAX*100
//                          GTM_TOM0_PWM_DUTY_MAX宏定义在zf_gtm_pwm.h  默认为10000
//-------------------------------------------------------------------------------------------------------------------
void gtm_pwm_init(TOM_PIN_enum pwmch, uint32 freq, uint32 duty)
{
    IfxGtm_Tom_Pwm_Config g_TOMConfig;


    IfxGtm_Tom_ToutMap *TOM_channel;
    TOM_channel = gtm_TOM_mux(pwmch);

    switch(TOM_channel->tom)
    {
        case 0: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= GTM_TOM0_PWM_DUTY_MAX); break;
        case 1: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= GTM_TOM1_PWM_DUTY_MAX); break;
        case 2: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= GTM_TOM2_PWM_DUTY_MAX); break;
        case 3: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= GTM_TOM3_PWM_DUTY_MAX); break;
    }

    IfxGtm_enable(&MODULE_GTM);
    if(!(MODULE_GTM.CMU.CLK_EN.B.EN_FXCLK & 0x2))
    {
        IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, CMU_CLK_FREQ);
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CMU_CLK_FREQ);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);
    }


    IfxGtm_Tom_Pwm_initConfig(&g_TOMConfig, &MODULE_GTM);

    g_TOMConfig.tom = TOM_channel->tom;
    g_TOMConfig.tomChannel = TOM_channel->channel;
    g_TOMConfig.period = CMU_CLK_FREQ/freq;
    g_TOMConfig.pin.outputPin = TOM_channel;
    g_TOMConfig.synchronousUpdateEnabled = TRUE;

    switch(TOM_channel->tom)
    {
        case 0: g_TOMConfig.dutyCycle = (uint32)((uint64)duty * g_TOMConfig.period / GTM_TOM0_PWM_DUTY_MAX); break;
        case 1: g_TOMConfig.dutyCycle = (uint32)((uint64)duty * g_TOMConfig.period / GTM_TOM1_PWM_DUTY_MAX); break;
        case 2: g_TOMConfig.dutyCycle = (uint32)((uint64)duty * g_TOMConfig.period / GTM_TOM2_PWM_DUTY_MAX); break;
        case 3: g_TOMConfig.dutyCycle = (uint32)((uint64)duty * g_TOMConfig.period / GTM_TOM3_PWM_DUTY_MAX); break;
    }

    IfxGtm_Tom_Pwm_init(&g_TOMDriver, &g_TOMConfig);
    IfxGtm_Tom_Pwm_start(&g_TOMDriver, TRUE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM占空比设置
//  @param      pwmch       PWM通道号及引脚
//  @param      duty        PWM占空比
//  @return     void
//  Sample usage:           pwm_duty(TOM0_CH7_P02_7, 5000);//设置占空比为百分之5000/GTM_TOM0_PWM_DUTY_MAX*100
//                          GTM_TOM0_PWM_DUTY_MAX宏定义在zf_gtm_pwm.h  默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_duty(TOM_PIN_enum pwmch, uint32 duty)
{
    uint32 period;

    IfxGtm_Tom_ToutMap *TOM_channel;
    TOM_channel = gtm_TOM_mux(pwmch);

    period = IfxGtm_Tom_Ch_getCompareZero(&MODULE_GTM.TOM[TOM_channel->tom], TOM_channel->channel);

    switch(TOM_channel->tom)
    {
        case 0: duty = (uint32)((uint64)duty * period / GTM_TOM0_PWM_DUTY_MAX); break;
        case 1: duty = (uint32)((uint64)duty * period / GTM_TOM1_PWM_DUTY_MAX); break;
        case 2: duty = (uint32)((uint64)duty * period / GTM_TOM2_PWM_DUTY_MAX); break;
        case 3: duty = (uint32)((uint64)duty * period / GTM_TOM3_PWM_DUTY_MAX); break;
    }

    IfxGtm_Tom_Ch_setCompareOneShadow(&MODULE_GTM.TOM[TOM_channel->tom], TOM_channel->channel, duty);
}
