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
#pragma section all "cpu1_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中
extern vint8    beacon_found = 0;         //0代表未找到信标灯
extern vuint8   camera_change = 0;        //0代表使用摄像头一  1代表摄像头二    仅作调试 可删除
extern uint8    nearbeacon = 0;           //0代表未靠近信标灯

extern uint16 beacon_area = 0;
extern sint16 beacon_bias = 0;
extern uint8 beacon_len = 0;
extern uint16 beacon_dis = 1000;

extern IfxCpu_mutexLock mutexCpu0InitIsOk;


void core1_main(void)
{
    disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //用户在此处调用各种初始化函数等



    //等待所有核心初始化完毕

    enableInterrupts();

    while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));
    while (TRUE)
    {
        //用户在此处编写任务代码

    }
}



#pragma section all restore

//-----------------------------cpu1流程--------------------------------------------------------------------
//---主函数中通过变量决定开启摄像头一 还是 摄像头二 互补导通
//------中断头文件添加摄像头二 两个外部中断和一个DMA  优先级查见isr_config.h
//------中断源文件添加相应中断服务函数  回调函数见摄像头源文件
//------摄像头头文件注释在最后一行
//------摄像头源文件添加三个函数 初始化摄像头二 场中断和 dma完成中断
//------摄像头二函数本体与摄像头一不同之处   引脚 通道号
//------两个摄像头完成中断逻辑被修改    当摄像头处理完一帧完整图像后关闭当前使能的摄像头（两个外部中断失能  +  一个dma_stop）
//------两个摄像头初始化矛盾问题    没有注意到linked list复用    为摄像头二单独添加一个linked list 即可


//---------------------------------目前代码思路--------------------------
//------找灯时双摄像头交替7ms寻找  从摄像头一开始
//------找到灯后 在中断中关闭另一个摄像头   主函数主循环思路同工程 OneCameraTwoMotor
//------电机方向环需要在定时中断中手动添加
