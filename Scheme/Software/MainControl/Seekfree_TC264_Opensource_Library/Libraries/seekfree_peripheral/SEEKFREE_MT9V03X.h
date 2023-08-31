/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            �����
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        ADS v1.2.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-3-23
 * @note
                    ���߶��壺
                    ------------------------------------
                    ģ��ܽ�                        ��Ƭ���ܽ�
                    SDA(51��RX)              �鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_COF_UART_TX�궨��
                    SCL(51��TX)              �鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_COF_UART_RX�궨��
                    ���ж�(VSY)                �鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_VSYNC_PIN�궨��
                    ���ж�(HREF)               ����û��ʹ�ã���˲�����
                    �����ж�(PCLK)              �鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_PCLK_PIN�궨��
                    ���ݿ�(D0-D7)          �鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_DATA_PIN�궨��
                    ------------------------------------

                    Ĭ�Ϸֱ�����                      188*120
                    Ĭ��FPS                 50֡
 ********************************************************************************************************************/



#ifndef _SEEKFREE_MT9V03X_h
#define _SEEKFREE_MT9V03X_h

#include "common.h"
#include "IfxDma_cfg.h"
#include "zf_uart.h"

//��������ͷ����
#define MT9V03X_W               188                 //ͼ����  ��Χ1-188
#define MT9V03X_H               120                 //ͼ��߶�  ��Χ1-120



//--------------------------------------------------------------------------------------------------
//��������
//--------------------------------------------------------------------------------------------------
#define MT9V03X_COF_UART        UART_1              //��������ͷ��ʹ�õ��Ĵ���
#define MT9V03X_COF_UART_TX     UART1_TX_P02_2
#define MT9V03X_COF_UART_RX     UART1_RX_P02_3

#define MT9V03X_VSYNC_PIN       ERU_CH3_REQ6_P02_0  //���ж����� ��ѡ��Χ�ο�ERU_PIN_enumö�� ��������������ѡ��Ϊͬһ��ͨ���������ǹ����жϵ�ͨ��
                                                    //���糡�ж�ѡ��ERU_CH3 ��ô�������žͲ���ѡ��ERU_CH7����Ϊ3��7���ж��ǹ��õġ�

#define MT9V03X_DATA_PIN        P00_0               //����D0��������  ����D0����ΪP00_0 ��ôD1��ʹ�õ�������ΪP00_1����������
                                                    //�����ò���P00_0��P02_0��P15_0����������Ϊ��������

#define MT9V03X_PCLK_PIN        ERU_CH4_REQ8_P33_7  //��������ʱ������ ��ѡ��Χ�ο�ERU_PIN_enumö�� �����볡�ж�����ѡ��Ϊͬһ��ͨ���������ǹ����жϵ�ͨ��
                                                    //���糡�ж�ѡ��ERU_CH3 ��ô�������žͲ���ѡ��ERU_CH7����Ϊ3��7���ж��ǹ��õġ�

#define MT9V03X_DMA_CH          IfxDma_ChannelId_5  //����ʹ�õ�DMAͨ�� 0-47��ѡ  ͨ����Խ�����ȼ�Խ��



// ---------------------------------------����ͷ��  ������Ҫ����

#define MT9V03X2_COF_UART        UART_2              //��������ͷ��ʹ�õ��Ĵ���
#define MT9V03X2_COF_UART_TX     UART2_TX_P33_9
#define MT9V03X2_COF_UART_RX     UART2_RX_P33_8

#define MT9V03X2_VSYNC_PIN       ERU_CH5_REQ1_P15_8  //���ж����� ��ѡ��Χ�ο�ERU_PIN_enumö�� ��������������ѡ��Ϊͬһ��ͨ���������ǹ����жϵ�ͨ��
                                                    //���糡�ж�ѡ��ERU_CH3 ��ô�������žͲ���ѡ��ERU_CH7����Ϊ3��7���ж��ǹ��õġ�

#define MT9V03X2_DATA_PIN        P15_0               //����D0��������  ����D0����ΪP00_0 ��ôD1��ʹ�õ�������ΪP00_1����������
                                                    //�����ò���P00_0��P02_0��P15_0����������Ϊ��������

#define MT9V03X2_PCLK_PIN        ERU_CH6_REQ12_P11_10 //��������ʱ������ ��ѡ��Χ�ο�ERU_PIN_enumö�� �����볡�ж�����ѡ��Ϊͬһ��ͨ���������ǹ����жϵ�ͨ��
                                                    //���糡�ж�ѡ��ERU_CH3 ��ô�������žͲ���ѡ��ERU_CH7����Ϊ3��7���ж��ǹ��õġ�
#define MT9V03X2_DMA_CH          IfxDma_ChannelId_6  //����ʹ�õ�DMAͨ�� 0-47��ѡ  ͨ����Խ�����ȼ�Խ��

//����ͷ����ö��
typedef enum
{
    INIT = 0,               //����ͷ��ʼ������
    AUTO_EXP,               //�Զ��ع�����
    EXP_TIME,               //�ع�ʱ������
    FPS,                    //����ͷ֡������
    SET_COL,                //ͼ��������
    SET_ROW,                //ͼ��������
    LR_OFFSET,              //ͼ������ƫ������
    UD_OFFSET,              //ͼ������ƫ������
    GAIN,                   //ͼ��ƫ������
    CONFIG_FINISH,          //������λ����Ҫ����ռλ����
    
    COLOR_GET_WHO_AM_I = 0xEF,
    SET_EXP_TIME = 0XF0,    //���������ع�ʱ������
    GET_STATUS,             //��ȡ����ͷ��������
    GET_VERSION,            //�̼��汾������

    SET_ADDR = 0XFE,        //�Ĵ�����ַ����
    SET_DATA                //�Ĵ�����������
}CMD;
     
//cpu0  main.c
extern uint16 beacon_area;
extern sint16 beacon_bias;
extern uint8 beacon_len;
extern uint16 beacon_dis;
extern uint8  nearbeacon;

//cpu1  main.c
extern vint8    beacon_found;
extern vuint8   camera_change;



     
extern uint8    receive[3];         //�����ڽ���������Ϣʱ����  ��ֹ�û�ʹ�øñ���
extern uint8    receive_num;        //�����ڽ���������Ϣʱ����  ��ֹ�û�ʹ�øñ���
extern vuint8   uart_receive_flag;  //�����ڽ���������Ϣʱ����  ��ֹ�û�ʹ�øñ���


extern uint8    mt9v03x_finish_flag;//һ��ͼ��ɼ���ɱ�־λ
extern uint8    mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8    mt9v03x_bin_image[MT9V03X_H][MT9V03X_W];


void mt9v03x_uart_callback(void);
void set_config(UARTN_enum uartn, int16 buff[CONFIG_FINISH-1][2]);
void get_config(UARTN_enum uartn, int16 buff[CONFIG_FINISH-1][2]);
uint16 get_version(UARTN_enum uartn);
uint16 set_exposure_time(UARTN_enum uartn, uint16 light);

void mt9v03x_init(void);
void mt9v03x_vsync(void);
void mt9v03x_dma(void);
uint16 GetOSTU (uint16 width,uint16 height);
uint16 mean_filter(uint16 width,uint16 height);
void sobelAutoThreshold(uint8  imageIn[MT9V03X_H][MT9V03X_W], uint8  imageOut[MT9V03X_H][MT9V03X_W]);
void MedianFilter();
boolean  extract_beacon(uint16 *area,sint16 *bias,uint8 *len,boolean PRINT);//two pass
void    extract_beacon2(uint16 *area,uint16 *mid);
void beacon_xor(uint16 width,uint16 height,uint16 *area,uint16 *mid);

//-----------------------------����ͷ��
void mt9v03x2_init(void);
void mt9v03x2_vsync(void);
void mt9v03x2_dma(void);
void mt9v03x2_uart_callback(void);


//-----------------------------����ͷһ�Ͷ������жϺ����� ������ ͨ���� �ж����ȼ�  linked list ��һ��       ������һ��------------------
uint16 estimate_distance(uint8 beacon_len);//�������
#endif

