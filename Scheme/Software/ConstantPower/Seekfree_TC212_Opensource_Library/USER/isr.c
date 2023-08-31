
 
/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC212
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-12
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
#include "zf_vadc.h"

//动态爬坡表
uint16 cv_table[10]=
{
        3,4,5,6,7,8,9,10,11,1000
};
uint16 ISET_max[10] =
{
        6000,4800,4000,3500,3000,2500,4000,4000,3000,3000
};
uint16 ISET_min[10] =
{
        4000,3000,2000,2000,2000,1500,2000,1500,1000,1000
};


//开环查表法
#define table_num 29
#if 0   //8.3
uint16 cv_table2[table_num]=
{
        697 ,775 ,863 ,937 ,1015,1102,1170,1255,1338,1420,1500,1580,1660,1740,1820,1905,1993,2066,2140,2240,2310,2380,2470,2550,2630,2712,2790,2860,10000
};
uint16 ISET_op[table_num]=
{
        6000,5400,4800,4500,4000,3700,3400,3200,2800,2700,2600,2500,2400,2200,2100,2100,2000,1800,1800,1700,1700,1600,1500,1500,1500,1400,1400,1400,1300
};
#endif

#if 1   //8.5
uint16 cv_table2[table_num]=
{
        697 ,775 ,863 ,937 ,1015,1102,1170,1255,1338,1420,1500,1580,1660,1740,1820,1905,1993,2066,2140,2240,2310,2380,2470,2550,2630,2712,2790,2860,10000
};
uint16 ISET_op[table_num]=
{
        6000,5400,4800,4500,4000,3700,3400,3200,2800,2700,2600,2500,2400,2200,2100,2100,2000,1800,1800,1700,1700,1600,1500,1500,1500,1400,1400,1400,1300
};
#endif


//extern uint32 time;
//extern uint32 last_time;
//extern boolean TimeIsOk;
//PIT中断函数  示例
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);

	//方法选择
	static uint8 method = 1;



//	time++;                                                                 //计时


	/*此处y
	 * TOM0_CH5_P11_3 -> ISET
	 *ADC0_CH7_A7 -> ci
	 *ADC0_CH6_A6 -> pv
	 *ADC0_CH8_A8 -> cv
	 *ADC0_CH5_A5 -> pi
	 */
	if (method == 1)  //方法一  直接拉高
	{
	    //ISET = 1;
	    uint16 ISET = 1000;
	    pwm_duty(TOM0_CH5_P11_3,ISET);


/*----------------------------TFT_PRINT----------------------------*/
#if 0
	    {
	        uint16 cv;
	        uint16 ci;
	        uint16 pv;
	        uint16 pi;
	        ci = adc_mean_filter(ADC_0,ADC0_CH10_A10,ADC_12BIT,5);//4.11--1814
	        cv = adc_mean_filter(ADC_1,ADC1_CH0_A12,ADC_12BIT,5);//12.99--2320
            pv = adc_mean_filter(ADC_0,ADC0_CH6_A6,ADC_12BIT,5);
            pi = adc_mean_filter(ADC_0,ADC0_CH8_A8,ADC_12BIT,5);

	        lcd_showuint16(0,0,cv);
	        lcd_showuint16(0,1,ci);
	        lcd_showuint16(0,2,pv);
	        lcd_showuint16(0,3,pi);
	        lcd_showuint16(0,5,ISET);
	    }
#endif

	}
	else if (method == 2)     //方法二 查表爬坡法
	{
	    //是否打印
	    uint8 print = 0;

	    static uint16 ISET;
	    static uint8 dir = 0;
	    static uint16 len = 50;
	    static uint16 max = 7000;
	    static uint16 min = 1000;
	    static float P_low = 40;    //爬坡功率区间下限
	    static float P_high = 50;   //爬坡功率区间上限
	    uint16 cv;
        cv = adc_mean_filter(ADC_1,ADC1_CH0_A12,ADC_12BIT,5);//12.99--2320
        float cv_f;
        cv_f = 0.0062*cv-1.3136;
        if(print)lcd_showuint16(0,0,cv);
        if(print)lcd_showfloat(0,1,cv_f,2,3);
        gpio_set(P20_9,1);

//        if(last_time==0 && cv>1480 && cv<1520){                               //计时
//            last_time = time;
//            lcd_showint32(0,0,last_time,8);
//        }
        if(cv_f<9) //查表
        {
            //限幅输出
            {
                uint8 i=0;
                for(i=0;i<table_num-1;i++)
                    if(cv<=cv_table2[i])break;
                ISET = ISET_op[i];
                max = ISET_op[i]+1000;
            }
            if(ISET>max)ISET = max;
            if(ISET<min)ISET = min;
            pwm_duty(TOM0_CH5_P11_3,ISET);
        }
        else        //爬坡
        {
//            if(!TimeIsOk && cv>2610 && cv<2650){                                                              //计时
//                TimeIsOk = 1;
//                last_time = time;
//            }
            uint16 pv,pi;
            pv = adc_mean_filter(ADC_0,ADC0_CH6_A6,ADC_12BIT,5);
            pi = adc_mean_filter(ADC_0,ADC0_CH8_A8,ADC_12BIT,5);
            // 拟合
            float pi_f = 0.0086*pi-0.5203;
            float pv_f = 0.0204*pv-2.0114;
            //控制功率区间
            float P;
            P = pi_f*pv_f;
//            if(print)lcd_showfloat(0,0,pv_f,3,3);
//            if(print)lcd_showfloat(0,1,pi_f,3,3);
            if(print)lcd_showfloat(0,3,P,3,3);


            if(P<P_low)
            {
                if(dir==0){
                    dir=1;
                    uint8 i=0;
                    for(i=0;i<table_num-1;i++)
                        if(cv<=cv_table2[i])break;
                    ISET = ISET_op[i];
                    max = ISET_op[i]+1000;
                }
                ISET+=len;
                //限幅输出
                {
                    uint8 i=0;
                    for(i=0;i<table_num-1;i++)
                        if(cv<=cv_table2[i])break;
                    max = ISET_op[i]+1000;
                }
                if(ISET>max){ISET = max;dir=0;}     //走到最大则改变方向
                if(ISET<min)ISET = min;
                pwm_duty(TOM0_CH5_P11_3,ISET);
                if(print)lcd_showuint16(0,5,ISET);
            }
            else if(P<P_high)
            {
                if(dir)ISET+=len;
                else ISET-=len;
                //限幅输出
                {
                    uint8 i=0;
                    for(i=0;i<table_num-1;i++)
                        if(cv<=cv_table2[i])break;
                    max = ISET_op[i]+1000;
                }
                if(ISET>max){ISET = max;dir=0;}    //走到最大则改变方向
                if(ISET<min)ISET = min;
                pwm_duty(TOM0_CH5_P11_3,ISET);
                if(print)lcd_showuint16(0,5,ISET);
            }
            else
            {
                gpio_set(P20_9,0);      //电容处于高压阶段 且 已到达目标功率
                dir=0;  //默认ISET 减小
                uint8 i=0;
                for(i=0;i<table_num-1;i++)
                    if(cv<=cv_table2[i])break;
                ISET = ISET_op[i];
            }
        }
	}

	else if (method == 3)   //方法三  P = U * I = 50W  电容电压较大时 无啸叫方可使用
	{
	    uint16 cv;
	    uint16 min =1000;
	    uint16 max =6000;
        float Set_P = 45; //目标功率
        float Cur_P;
        uint16 Kp=0;
        uint16 Ki=0;
        //滤波采样
        uint16 ci;
        static sint16 last_bias;
        uint16 ISET;

        ci = adc_mean_filter(ADC_0,ADC0_CH10_A10,ADC_12BIT,5);//4.11A--1814
        cv = adc_mean_filter(ADC_1,ADC1_CH0_A12,ADC_12BIT,5);//12.99V--2320
        Cur_P = (ci*4.11/1814) * (cv*12.99/2320);
        sint16 bias = Set_P - Cur_P;

        ISET = Kp*(bias - last_bias) + Ki*(bias);
        if(ISET<min)ISET = min;
        if(ISET>max)ISET = max;
        pwm_duty(TOM0_CH5_P11_3,ISET);   //强制取整

        last_bias = bias;
	}
	else if(method == 4)  //方法四 开环查表法
	{
	    uint16 cv;
	    uint16 max = 6000;
	    uint16 min = 1000;
        //电压低通滤波采样
        cv = adc_mean_filter(ADC_1,ADC1_CH0_A12,ADC_12BIT,5);

	    uint8 i=0;
	    for(i=0;i<table_num-1;i++)
	        if(cv<=cv_table2[i])break;
	    uint16 ISET = ISET_op[i];
#if 0
	    if(i!=0 && i!=(table_num-1))    //分段线性插值 除去数组首尾两点
	    {
	        float percent = (cv_table2[i]-cv)/(float)(cv_table2[i]-cv_table2[i-1]);//thus cv == cv_table2[i]*(1-percent) + cv_table2[i-1]*(percent)
	        ISET = (uint16)(percent*ISET_op[i-1] + (1-percent)*ISET_op[i]);
	    }
#endif
        //限幅
        if(ISET<min)ISET = min;
        else if(ISET>max)ISET = max;
        pwm_duty(TOM0_CH5_P11_3,ISET);

        //PRINT
#if 1
        {

            uint16 cv;
            uint16 ci;
            uint16 pv;
            uint16 pi;
            ci = adc_mean_filter(ADC_0,ADC0_CH10_A10,ADC_12BIT,5);//4.11--1814
            cv = adc_mean_filter(ADC_1,ADC1_CH0_A12,ADC_12BIT,5);//12.99--2320
            pv = adc_mean_filter(ADC_0,ADC0_CH6_A6,ADC_12BIT,5);
            pi = adc_mean_filter(ADC_0,ADC0_CH8_A8,ADC_12BIT,5);

            lcd_showuint16(0,0,cv);
            lcd_showuint16(0,1,ci);
            lcd_showuint16(0,2,pv);
            lcd_showuint16(0,3,pi);
            lcd_showuint16(0,5,ISET);

        }
#endif
	}
	else if(method == 5)  //方法五 遍历
	{
        static boolean dir = 1;
        static sint16 ISET = 0;
        uint16 min = 1000;
        uint16 max = 5000;
        uint16 height = 200;
#if 1
        {
            uint16 cv;
            uint16 ci;
            uint16 pv;
            uint16 pi;
            ci = adc_mean_filter(ADC_0,ADC0_CH10_A10,ADC_12BIT,5);//4.11--1814
            cv = adc_mean_filter(ADC_1,ADC1_CH0_A12,ADC_12BIT,5);//12.99--2320
            pv = adc_mean_filter(ADC_0,ADC0_CH6_A6,ADC_12BIT,5);
            pi = adc_mean_filter(ADC_0,ADC0_CH8_A8,ADC_12BIT,5);

            lcd_showuint16(0,0,cv);
            lcd_showuint16(0,1,ci);
            lcd_showuint16(0,2,pv);
            lcd_showuint16(0,3,pi);
            lcd_showuint16(0,5,ISET);
        }
#endif
        if(max<=ISET || ISET<= min)dir=!dir;
        if(dir)ISET += height;
        else    ISET -= height;

        //限幅
        if(ISET<min)ISET = min;
        else if(ISET>max)ISET = max;
        pwm_duty(TOM0_CH5_P11_3,ISET);

        lcd_showuint16(0,5,ISET);
	}
	else if(method == 6)  //方法六 bang-bang control
	{
        static uint16 last_cv1 = 0;
        static uint16 ISET = 2000;
        uint16 cv;

        cv = adc_mean_filter(ADC_0,ADC0_CH8_A8,ADC_12BIT,5);
        uint16 cv_1 = adc_recursive_filter(cv);


        if(last_cv1 != 0 && ((sint16)last_cv1 - (sint16)cv_1 >= 100 || (sint16)last_cv1 - (sint16)cv_1 <= -100))
        {
            gpio_set(P20_9,0);
            ISET -= 500;
            if(ISET <= 1000)ISET = 1000;
            pwm_duty(TOM0_CH5_P11_3,ISET);
        }
        else
        {
            gpio_set(P20_9,1);
            ISET += 100;
            if(ISET >= 6000)ISET = 6000;
            pwm_duty(TOM0_CH5_P11_3,ISET);
        }

        last_cv1 = cv;

	}



	//正常工作指示灯
	if(gpio_get(P02_4) == 1)gpio_set(P20_8,0);//正常
	else gpio_set(P20_8,1);
}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);

}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);

}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
	enableInterrupts();//开启中断嵌套
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}




IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套

	if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//通道4中断
	{
		CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
		if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_vsync();
        else if (CAMERA_BIN_UART  == camera_type)   ov7725_uart_vsync();
        else if (CAMERA_BIN       == camera_type)   ov7725_vsync();
	}
}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH1_REQ10_P14_3))//通道1中断
	{
		CLEAR_GPIO_FLAG(ERU_CH1_REQ10_P14_3);
	}
}

//由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//	enableInterrupts();//开启中断嵌套
//	if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//	}
//	if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//通道6中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//	}
//}



IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
	if(GET_GPIO_FLAG(ERU_CH3_REQ15_P14_1))//通道3中断
	{
		CLEAR_GPIO_FLAG(ERU_CH3_REQ15_P14_1);


	}
	if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
	{
		CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

	}
}



IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套

	if		(CAMERA_GRAYSCALE == camera_type)	mt9v03x_dma();
	else if (CAMERA_BIN_UART  == camera_type)	ov7725_uart_dma();
	else if	(CAMERA_BIN       == camera_type)	ov7725_dma();
}


//串口中断函数  示例
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart0_handle);
    wireless_uart_callback();
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    if		(CAMERA_GRAYSCALE == camera_type)	mt9v03x_uart_callback();
    else if (CAMERA_BIN_UART  == camera_type)	ov7725_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
	enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}

