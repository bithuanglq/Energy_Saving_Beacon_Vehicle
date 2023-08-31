/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC212
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-12
 ********************************************************************************************************************/


#include "headfile.h"

extern uint32 time = 0;
extern uint32 last_time = 0;
extern boolean TimeIsOk = 0;
//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��

int core0_main(void)
{
	get_clk();//��ȡʱ��Ƶ��  ��ر���

	//�û��ڴ˴����ø��ֳ�ʼ��������



	gtm_pwm_init(TOM0_CH5_P11_3,20000,1000);       //20khz  10%
	adc_init(ADC_0,ADC0_CH10_A10);  //ci
	adc_init(ADC_1,ADC1_CH0_A12); //cv
    adc_init(ADC_0,ADC0_CH6_A6); //pv
    adc_init(ADC_0,ADC0_CH8_A8); //pi

	gpio_init(P20_8,GPO,1,PUSHPULL); //led1
    gpio_init(P20_9,GPO,1,PUSHPULL); //led2
	gpio_init(P02_3,GPO,0,PUSHPULL); //en2
	gpio_init(P02_2,GPO,0,PUSHPULL); //en1
	gpio_init(P02_1,GPO,1,PUSHPULL); //uvlo
	gpio_init(P02_0,GPO,1,PUSHPULL); //dir
	gpio_init(P02_4,GPI,0,NO_PULL);  //fault

	lcd_init();
	lcd_clear(WHITE);
	lcd_showstr(0,0,"init");

	//start lm5170
	gpio_set(P02_2,1); //en1
    gpio_set(P02_3,1); //en2
    systick_delay_ms(STM0,10);

	enableInterrupts();

	pit_interrupt_ms(CCU6_0,PIT_CH0,5);

	while (TRUE)
	{
		//�û��ڴ˴���д�������
//	    if(TimeIsOk == 1)lcd_showint32(0,1,last_time,8);            //��ʱ
//	    systick_delay_ms(STM0,500);
	}
}



