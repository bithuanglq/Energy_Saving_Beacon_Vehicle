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
#pragma section all "cpu1_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��
extern vint8    beacon_found = 0;         //0����δ�ҵ��ű��
extern vuint8   camera_change = 0;        //0����ʹ������ͷһ  1��������ͷ��    �������� ��ɾ��
extern uint8    nearbeacon = 0;           //0����δ�����ű��

extern uint16 beacon_area = 0;
extern sint16 beacon_bias = 0;
extern uint8 beacon_len = 0;
extern uint16 beacon_dis = 1000;

extern IfxCpu_mutexLock mutexCpu0InitIsOk;


void core1_main(void)
{
    disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //�û��ڴ˴����ø��ֳ�ʼ��������



    //�ȴ����к��ĳ�ʼ�����

    enableInterrupts();

    while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));
    while (TRUE)
    {
        //�û��ڴ˴���д�������

    }
}



#pragma section all restore

//-----------------------------cpu1����--------------------------------------------------------------------
//---��������ͨ������������������ͷһ ���� ����ͷ�� ������ͨ
//------�ж�ͷ�ļ��������ͷ�� �����ⲿ�жϺ�һ��DMA  ���ȼ����isr_config.h
//------�ж�Դ�ļ������Ӧ�жϷ�����  �ص�����������ͷԴ�ļ�
//------����ͷͷ�ļ�ע�������һ��
//------����ͷԴ�ļ������������ ��ʼ������ͷ�� ���жϺ� dma����ж�
//------����ͷ����������������ͷһ��֮ͬ��   ���� ͨ����
//------��������ͷ����ж��߼����޸�    ������ͷ������һ֡����ͼ���رյ�ǰʹ�ܵ�����ͷ�������ⲿ�ж�ʧ��  +  һ��dma_stop��
//------��������ͷ��ʼ��ì������    û��ע�⵽linked list����    Ϊ����ͷ���������һ��linked list ����


//---------------------------------Ŀǰ����˼·--------------------------
//------�ҵ�ʱ˫����ͷ����7msѰ��  ������ͷһ��ʼ
//------�ҵ��ƺ� ���ж��йر���һ������ͷ   ��������ѭ��˼·ͬ���� OneCameraTwoMotor
//------���������Ҫ�ڶ�ʱ�ж����ֶ����
