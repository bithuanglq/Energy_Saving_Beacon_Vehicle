/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/


#include "headfile.h"
#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��
/*-------------------------˫��ͬ��-------------------------*/
extern IfxCpu_mutexLock mutexCpu0InitIsOk = 1;


/*-------------------------����ͷ-------------------------*/
extern sint16 bias_sum = 0;
extern vint8  beacon_found;
extern IfxCpu_mutexLock TftIsOk = 1;

/*-------------------------������ֵ-------------------------*/
extern uint16  adc_cap_V = 0;
extern uint16  adc_cap_I = 0;
extern float P = 0;
extern float Cap_V = 0;
extern float Cap_I = 0;
extern vint8  tocharge = 0;//��Ҫ���Ϊ1
extern vint8  onbeacon = 0;//���ű����


/*-------------------------������------------------------------*/
extern int16 icm_gyro_z,icm_acc_x;

/*-------------------------���뿪��------------------------------*/
/*
 * KEY1 --> P11_3   //����ѹ
 * KEY2 --> P11_2   //���ת��1
 * KEY3 --> P13_3   //���ת��2
 * KEY4 --> P13_2   //�Ƿ��׳�
 */
extern sint16 target_v;     //���Ŀ���ٶ�
extern uint8 len_threshold;//����ϵ��
/*-------------------------�������PID-------------------------*/
extern boolean had_pit_interrupt = 1;   //��PIT�ж�
extern boolean had_pit_interrupt2 = 0;
extern sint32 KP1 = 20;   //������Ϊbeacon_bias
extern sint32 KP2 = 20;
extern sint32 KD1 = 20;   //���ٶȻ�PD����
extern sint32 KD2 = 20;
extern sint16 max = 14000;
extern sint16 min = -14000;
/*
 * �ٶ�ƽ���˲�
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
/*�ٶȼ�Ȩ�˲�
 * n = 8000 ����n_c����
 * extern sint32 KP1 = 15;   //������Ϊbeacon_bias
extern sint32 KP2 = 10;
extern sint32 KD1 = 30;   //���ٶȻ�PD����
extern sint32 KD2 = 20;
extern sint16 max = 8000;
extern sint16 min = -8000;
uint8 len_threshold = 20;
#define FILTER_N_Motor 10
 */
//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��

//��������
void LowVoltageProtect(void);
void JudgingNCharging(void);
void UISampling(void);

int core0_main(void)
{
    get_clk();//��ȡʱ��Ƶ��  ��ر���
    //�û��ڴ˴����ø��ֳ�ʼ��������
    gpio_init(P20_8, GPO, 1, PUSHPULL);         //beaconfind
    gpio_init(P20_9, GPO, 1, PUSHPULL);         //�׳�
    gpio_init(P21_4, GPO, 1, PUSHPULL);         //onbeacon
    gpio_init(P21_5, GPO, 1, PUSHPULL);         //tocharge
    uart_init(UART_3,115200,UART3_TX_P21_7,UART3_RX_P21_6);
    adc_init(ADC_0,ADC0_CH1_A1);//���ݵ�ѹ
    adc_init(ADC_0,ADC0_CH7_A7);    //Ӳ���˲�
    adc_init(ADC_0,ADC0_CH2_A2);//������
    adc_init(ADC_0,ADC0_CH4_A4);    //Ӳ���˲�
    adc_init(ADC_0,ADC0_CH13_A13);

    adc_init(ADC_0,ADC0_CH6_A6);
    //���뿪��
    gpio_init(P11_3,GPI,0,PUSHPULL);    //����ѹ��ֵ
    gpio_init(P11_2,GPI,0,PUSHPULL);    //���ת��1
    gpio_init(P13_3,GPI,0,PUSHPULL);    //���ת��2
    gpio_init(P13_2,GPI,0,PUSHPULL);    //�Ƿ��׳�

    icm20602_init_spi();
    lcd_init();
    lcd_clear(WHITE);

    boolean first_charge = 1;   //�׳��־λ 1����Ҫ�׳�
    uint16 charge_threshold = 1700; //�����ֵ 1700--13.5V  1570 --12.1V    ��ѹΪTFT��ʾ��
#if 1   //��������
    //�����ĵ�ѹ��ֵ����
    if(gpio_get(P11_3)==1)charge_threshold = 3200;
    else charge_threshold = 3000;
    //����ϵ��
    if(gpio_get(P11_2)==1)len_threshold = 30;
    else len_threshold = 0;
    //������ת������
    if(gpio_get(P13_3)==1)target_v = 8000;
    else target_v = 6000;
    //�Ƿ��׳�
    if(gpio_get(P13_2)==1)first_charge = 1;
    else first_charge = 0;
#else   //���뿪������


    //����ѹ��ֵ0
    if(gpio_get(P11_3)==1)charge_threshold = 2900;
    else charge_threshold = 2500;


    //����ϵ��
    if(gpio_get(P11_2)==1)len_threshold = 30;
    else len_threshold = 0;

    //���ת��
    if(gpio_get(P13_3)==1)target_v = 10000;
    else target_v = 8000;


    //�Ƿ��׳�
    if(gpio_get(P13_2)==1)first_charge = 1;
    else first_charge = 0;

#endif
    //�׳�LEDָʾ
    if(first_charge)gpio_set(P20_9,0);
    else gpio_set(P20_9,1);

    //ͣ��
//    if(first_charge)
//    {
//        uint8 chargecounter = 0;
//        SendSpeedCtrlCode(0,0);
//        while(1)                    //��һ�γ��
//        {
//            UISampling();
//            lcd_showstr(10,0,"charging...");
//            if(adc_cap_V >= charge_threshold)chargecounter++;
//            if(chargecounter > 0)break;          //������
//            gpio_set(P21_4,0);
//            systick_delay_ms(STM0,500);     //̫�����������
//
//        }
//        gpio_set(P21_4,1);
//        chargecounter = 0; //���� �ñ���������֮��ĵ�ѹ����
//
//    }


    mt9v03x_init();
    mt9v03x2_init();
    lcd_clear(WHITE);
    lcd_showstr(10,0,"init");
    systick_delay_ms(STM0,500);

    //������ͷ�ж�

    eru_enable_interrupt(MT9V03X_VSYNC_PIN);
    eru_enable_interrupt(MT9V03X_PCLK_PIN);



    enableInterrupts();
    //֪ͨCPU1��CPU0��ʼ�����
    IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
    //ע�� ��V1.1.6�汾֮��  printf��ӡ����Ϣ�Ӵ�������������ѧϰ������6-Printf_Demo
    //ע�� ��V1.1.6�汾֮��  printf��ӡ����Ϣ�Ӵ�������������ѧϰ������6-Printf_Demo
    //ע�� ��V1.1.6�汾֮��  printf��ӡ����Ϣ�Ӵ�������������ѧϰ������6-Printf_Demo
    pit_interrupt_ms(CCU6_1,PIT_CH0,50);

    had_pit_interrupt = 1;   //��PIT�ж�

    //��ʱ�жϺ���������޸ģ�WHILE(1) ����Ҫע�����

    while (TRUE)
    {
#if 0   //�����ǲ���
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
/*----------------------------------ֱ�߲���----------------------------*/
#if 0
        SendSpeedCtrlCode(5000,100);
        systick_delay_ms(STM0,100);

#endif

/*----------------------------------����������----------------------------*/
#if 1
        if(beacon_found == 0 )
        {
            gpio_set(P20_8,1);
            if(had_pit_interrupt)pit_disable_interrupt(CCU6_1,PIT_CH0);
            had_pit_interrupt = 0;
            if(had_pit_interrupt2)pit_disable_interrupt(CCU6_0,PIT_CH0);
            had_pit_interrupt2 = 0;
            Circling(spd_slow);     //��ת

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
#if 1   //һ�������ѹ����
        LowVoltageProtect();
#else //��;����
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
        //if P>certainvalue ���жϣ�ͣ��
        if(had_pit_interrupt)pit_disable_interrupt(CCU6_1,PIT_CH0);
        had_pit_interrupt = 0;
        if(had_pit_interrupt2)pit_disable_interrupt(CCU6_0,PIT_CH0);
        had_pit_interrupt2 = 0;
        SendSpeedCtrlCode(0,0);           //�ϴ���λ��ͣ��

        P = 0;
        uint8 chargecounter = 0;
        uint8 adccounter = 0;
        while(1)
        {
            UISampling();
            chargecounter++;
            if(adc_cap_V > 1200)adccounter++;
            if(adccounter > 0)break;        //������
            if(chargecounter > 10)break;    //��ʱ
            systick_delay_ms(STM0,500);     //̫�����������
        }
        //���ж�,���־λ
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






