/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中
/*-------------------------双核同步-------------------------*/
extern IfxCpu_mutexLock mutexCpu0InitIsOk = 1;


/*-------------------------摄像头-------------------------*/
extern sint16 bias_sum = 0;
extern vint8  beacon_found;
extern IfxCpu_mutexLock TftIsOk = 1;

/*-------------------------充电采样值-------------------------*/
extern uint16  adc_cap_V = 0;
extern uint16  adc_cap_I = 0;
extern float P = 0;
extern float Cap_V = 0;
extern float Cap_I = 0;
extern vint8  tocharge = 0;//需要充电为1
extern vint8  onbeacon = 0;//在信标灯上


/*-------------------------陀螺仪------------------------------*/
extern int16 icm_gyro_z,icm_acc_x;

/*-------------------------拨码开关------------------------------*/
/*
 * KEY1 --> P11_3   //充电电压
 * KEY2 --> P11_2   //电机转速1
 * KEY3 --> P13_3   //电机转速2
 * KEY4 --> P13_2   //是否首充
 */
extern sint16 target_v;     //电机目标速度
extern uint8 len_threshold;//减速系数
/*-------------------------电机方向环PID-------------------------*/
extern boolean had_pit_interrupt = 1;   //打开PIT中断
extern boolean had_pit_interrupt2 = 0;
extern sint32 KP1 = 20;   //被控量为beacon_bias
extern sint32 KP2 = 20;
extern sint32 KD1 = 20;   //角速度环PD参数
extern sint32 KD2 = 20;
extern sint16 max = 14000;
extern sint16 min = -14000;
/*
 * 速度平均滤波
 * @param n = 8000
 * extern sint32 KP1 = 10;
 * extern sint32 KP2 = 10;
 * extern sint32 KD1 = 20;
 * extern sint32 KD2 = 20;
 * extern sint16 max = 5000;
 * extern sint16 min = -5000;
 *#define len_threshold 12
 *#define FILTER_N_Motor 20
 */
/*速度加权滤波
 * n = 8000 加了n_c反馈
 * extern sint32 KP1 = 15;   //被控量为beacon_bias
extern sint32 KP2 = 10;
extern sint32 KD1 = 30;   //角速度环PD参数
extern sint32 KD2 = 20;
extern sint16 max = 8000;
extern sint16 min = -8000;
uint8 len_threshold = 20;
#define FILTER_N_Motor 10
 */
//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。

//函数申明
void LowVoltageProtect(void);
void JudgingNCharging(void);
void UISampling(void);

int core0_main(void)
{
    get_clk();//获取时钟频率  务必保留
    //用户在此处调用各种初始化函数等
    gpio_init(P20_8, GPO, 1, PUSHPULL);         //beaconfind
    gpio_init(P20_9, GPO, 1, PUSHPULL);         //首充
    gpio_init(P21_4, GPO, 1, PUSHPULL);         //onbeacon
    gpio_init(P21_5, GPO, 1, PUSHPULL);         //tocharge
    uart_init(UART_3,115200,UART3_TX_P21_7,UART3_RX_P21_6);
    adc_init(ADC_0,ADC0_CH1_A1);//电容电压
    adc_init(ADC_0,ADC0_CH7_A7);    //硬件滤波
    adc_init(ADC_0,ADC0_CH2_A2);//充电电流
    adc_init(ADC_0,ADC0_CH4_A4);    //硬件滤波
    adc_init(ADC_0,ADC0_CH13_A13);

    adc_init(ADC_0,ADC0_CH6_A6);
    //拨码开关
    gpio_init(P11_3,GPI,0,PUSHPULL);    //充电电压阈值
    gpio_init(P11_2,GPI,0,PUSHPULL);    //电机转速1
    gpio_init(P13_3,GPI,0,PUSHPULL);    //电机转速2
    gpio_init(P13_2,GPI,0,PUSHPULL);    //是否首充

    icm20602_init_spi();
    lcd_init();
    lcd_clear(WHITE);

    boolean first_charge = 1;   //首充标志位 1代表要首充
    uint16 charge_threshold = 1700; //充电阈值 1700--13.5V  1570 --12.1V    电压为TFT的示数
#if 1   //决赛设置
    //决赛的电压阈值设置
    if(gpio_get(P11_3)==1)charge_threshold = 3200;
    else charge_threshold = 3000;
    //减速系数
    if(gpio_get(P11_2)==1)len_threshold = 30;
    else len_threshold = 0;
    //决赛的转速设置
    if(gpio_get(P13_3)==1)target_v = 8000;
    else target_v = 6000;
    //是否首充
    if(gpio_get(P13_2)==1)first_charge = 1;
    else first_charge = 0;
#else   //拨码开关配置


    //充电电压阈值0
    if(gpio_get(P11_3)==1)charge_threshold = 2900;
    else charge_threshold = 2500;


    //减速系数
    if(gpio_get(P11_2)==1)len_threshold = 30;
    else len_threshold = 0;

    //电机转速
    if(gpio_get(P13_3)==1)target_v = 10000;
    else target_v = 8000;


    //是否首充
    if(gpio_get(P13_2)==1)first_charge = 1;
    else first_charge = 0;

#endif
    //首充LED指示
    if(first_charge)gpio_set(P20_9,0);
    else gpio_set(P20_9,1);

    //停车
//    if(first_charge)
//    {
//        uint8 chargecounter = 0;
//        SendSpeedCtrlCode(0,0);
//        while(1)                    //第一次充电
//        {
//            UISampling();
//            lcd_showstr(10,0,"charging...");
//            if(adc_cap_V >= charge_threshold)chargecounter++;
//            if(chargecounter > 0)break;          //充电完成
//            gpio_set(P21_4,0);
//            systick_delay_ms(STM0,500);     //太短则采样错误
//
//        }
//        gpio_set(P21_4,1);
//        chargecounter = 0; //清零 该变量可用于之后的低压保护
//
//    }


    mt9v03x_init();
    mt9v03x2_init();
    lcd_clear(WHITE);
    lcd_showstr(10,0,"init");
    systick_delay_ms(STM0,500);

    //打开摄像头中断

    eru_enable_interrupt(MT9V03X_VSYNC_PIN);
    eru_enable_interrupt(MT9V03X_PCLK_PIN);



    enableInterrupts();
    //通知CPU1，CPU0初始化完成
    IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
    //注意 从V1.1.6版本之后  printf打印的信息从串口输出具体可以学习库例程6-Printf_Demo
    //注意 从V1.1.6版本之后  printf打印的信息从串口输出具体可以学习库例程6-Printf_Demo
    //注意 从V1.1.6版本之后  printf打印的信息从串口输出具体可以学习库例程6-Printf_Demo
    pit_interrupt_ms(CCU6_1,PIT_CH0,50);

    had_pit_interrupt = 1;   //打开PIT中断

    //定时中断函数序号已修改，WHILE(1) 中需要注意更正

    while (TRUE)
    {
#if 0   //陀螺仪测试
        get_icm20602_gyro_spi();
        get_icm20602_accdata_spi();
        lcd_showint16(0,1,icm_gyro_x);
        lcd_showint16(0,2,icm_gyro_y);
        lcd_showint16(0,3,icm_gyro_z);
        lcd_showint16(0,4,icm_acc_x);
        lcd_showint16(0,5,icm_acc_y);
        lcd_showint16(0,6 ,icm_acc_z);
        sint32 g =  icm_acc_x^2 + icm_acc_y^2 + icm_acc_z^2;
        lcd_showint32(0,7,g,10);
        systick_delay_ms(STM0,100);
#endif
/*----------------------------------直线测试----------------------------*/
#if 0
        SendSpeedCtrlCode(5000,100);
        systick_delay_ms(STM0,100);

#endif

/*----------------------------------华北赛程序----------------------------*/
#if 1
        if(beacon_found == 0 )
        {
            gpio_set(P20_8,1);
            if(had_pit_interrupt)pit_disable_interrupt(CCU6_1,PIT_CH0);
            had_pit_interrupt = 0;
            if(had_pit_interrupt2)pit_disable_interrupt(CCU6_0,PIT_CH0);
            had_pit_interrupt2 = 0;
            Circling(spd_slow);     //自转

        }
        else
        {
            gpio_set(P20_8,0);
            if(!had_pit_interrupt)
            {
                pit_enable_interrupt(CCU6_1,PIT_CH0);
                had_pit_interrupt = 1;
            }
        }
#endif
#if 1   //一次跑完低压保护
        LowVoltageProtect();
#else //中途补电
        JudgingNCharging();
        LowVoltageProtect();
#endif


    }

}


void LowVoltageProtect(void)
{
    static uint8 protectcounter = 0;
    UISampling();
    if(adc_cap_V < 1000)protectcounter++;
    if(protectcounter > 2)
    {
        if(had_pit_interrupt)pit_disable_interrupt(CCU6_1,PIT_CH0);
        had_pit_interrupt = 0;
        if(had_pit_interrupt2)pit_disable_interrupt(CCU6_0,PIT_CH0);
        had_pit_interrupt2 = 0;

        while(1)
        {
            MotorStop();
            gpio_set(P21_4,0);
        }

    }
}
void JudgingNCharging(void)
{
    UISampling();
    if(adc_cap_V < 900)
    {
        gpio_set(P21_5,0);
        tocharge = 1;
    }
    else
    {
        tocharge = 0;
        gpio_set(P21_5,1);
    }


    if(tocharge == 1 && P > 10)
    {
        gpio_set(P21_4,0);
        //if P>certainvalue 关中断，停车
        if(had_pit_interrupt)pit_disable_interrupt(CCU6_1,PIT_CH0);
        had_pit_interrupt = 0;
        if(had_pit_interrupt2)pit_disable_interrupt(CCU6_0,PIT_CH0);
        had_pit_interrupt2 = 0;
        SendSpeedCtrlCode(0,0);           //较大功率位置停车

        P = 0;
        uint8 chargecounter = 0;
        uint8 adccounter = 0;
        while(1)
        {
            UISampling();
            chargecounter++;
            if(adc_cap_V > 1200)adccounter++;
            if(adccounter > 0)break;        //充电完成
            if(chargecounter > 10)break;    //超时
            systick_delay_ms(STM0,500);     //太短则采样错误
        }
        //开中断,清标志位
        chargecounter = 0;
        tocharge = 0;
        gpio_set(P21_4,1);
        if(!had_pit_interrupt)
        {
            pit_enable_interrupt(CCU6_1,PIT_CH0);
            had_pit_interrupt = 1;
        }
    }
}
void UISampling(void)
{
//    adc_cap_V = adc_mean_filter(ADC_0, ADC0_CH7_A7, ADC_12BIT,10);
    adc_cap_V = adc_mean_filter(ADC_0, ADC0_CH6_A6, ADC_12BIT,10);
    adc_cap_I = adc_mean_filter(ADC_0, ADC0_CH4_A4, ADC_12BIT,5);//500mA-237 1A-400 1.5A-560 4A-1375
    Cap_V = (float)adc_cap_V/204;
    Cap_I = (float)adc_cap_I/400;
    P=Cap_V*Cap_I;
#if 1
    lcd_showfloat(0,1,Cap_V,2,2);
    lcd_showstr(50,1,"V");
    lcd_showuint16(100,1,adc_cap_V);
    lcd_showfloat(0,2,Cap_I,2,2);
    lcd_showstr(50,2,"I");
    lcd_showuint16(100,2,adc_cap_I);
    lcd_showfloat(0,3,P,2,2);
    lcd_showstr(50,3,"W");
#endif
}
#pragma section all restore






