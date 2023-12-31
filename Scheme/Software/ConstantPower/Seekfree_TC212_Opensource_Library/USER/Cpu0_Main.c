/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC212
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-12
 ********************************************************************************************************************/


#include "headfile.h"

extern uint32 time = 0;
extern uint32 last_time = 0;
extern boolean TimeIsOk = 0;
//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。

int core0_main(void)
{
	get_clk();//获取时钟频率  务必保留

	//用户在此处调用各种初始化函数等



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
		//用户在此处编写任务代码
//	    if(TimeIsOk == 1)lcd_showint32(0,1,last_time,8);            //计时
//	    systick_delay_ms(STM0,500);
	}
}



