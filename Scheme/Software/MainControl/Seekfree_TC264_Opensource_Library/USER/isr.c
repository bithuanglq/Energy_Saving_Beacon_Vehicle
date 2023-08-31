
 
/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ3184284598)
 * @version         查看doc内version文件 版本说明
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
#include "SEEKFREE_ICM20602.h"
//PIT中断函数  示例
extern boolean had_pit_interrupt2;
extern sint16 omiga_c;
extern sint16 target_v = 8000;         //极速

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
    GyroscopeDampingLoop(omiga_c);//omiga_c

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

    //比例引导
    Guiding(target_v);
    if(!had_pit_interrupt2)
    {
        pit_interrupt_ms(CCU6_0,PIT_CH0,20);
        had_pit_interrupt2 = 1;
    }

}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    enableInterrupts();//开启中断嵌套
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}


//由于摄像头一pclk引脚默认占用了 4通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch0_ch4_isr, 0, ERU_CH0_CH4_INT_PRIO)
//{
//    enableInterrupts();//开启中断嵌套
//    if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//通道0中断
//    {
//        CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
//    }
//
//    if(GET_GPIO_FLAG(ERU_CH4_REQ8_P33_7))//通道4中断
//    {
//        CLEAR_GPIO_FLAG(ERU_CH4_REQ8_P33_7);
//    }
//}

IFX_INTERRUPT(eru_ch1_ch5_isr, 0, ERU_CH1_CH5_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//通道1中断
    {
        CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
    }

    if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//通道5中断
    {
        CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
        if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x2_vsync();
    }
}

//由于摄像头二pclk引脚默认占用了 6通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, 0, ERU_CH2_CH6_INT_PRIO)
//{
//  enableInterrupts();//开启中断嵌套
//  if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//  {
//      CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//  }
//  if(GET_GPIO_FLAG(ERU_CH6_REQ12_P11_10))//通道6中断
//  {
//      CLEAR_GPIO_FLAG(ERU_CH6_REQ12_P11_10);
//
//  }
//}



IFX_INTERRUPT(eru_ch3_ch7_isr, 0, ERU_CH3_CH7_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//通道3中断
    {
        CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
        if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_vsync();

    }
    if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
    {
        CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

    }
}






IFX_INTERRUPT(dma_ch5_isr, 0, ERU_DMA_INT_PRIO)         //摄像头1
{
    enableInterrupts();//开启中断嵌套

    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_dma();
}

IFX_INTERRUPT(dma_ch6_isr, 0, ERU_DMA2_INT_PRIO)         //摄像头2
{
    enableInterrupts();//开启中断嵌套

    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x2_dma();
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
}
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头一配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//串口2默认连接到摄像头二配置串口
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    if      (CAMERA_GRAYSCALE == camera_type)   mt9v03x2_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    enableInterrupts();//开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
