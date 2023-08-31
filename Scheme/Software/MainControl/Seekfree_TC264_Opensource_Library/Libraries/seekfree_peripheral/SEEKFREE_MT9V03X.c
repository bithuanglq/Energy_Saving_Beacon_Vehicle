
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


#include "IfxDma.h"
#include "IfxScuEru.h"
#include "zf_stm_systick.h"
#include "zf_gpio.h"
#include "zf_eru.h"
#include "zf_eru_dma.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_18TFT.h"
#include "stdlib.h"

#define maxNum 1000                  //����ǩ��Ԥ������
#define Pi  3.14159
//����4�ֽڶ���
IFX_ALIGN(4) uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8   mt9v03x_bin_image[MT9V03X_H][MT9V03X_W];

uint8   receive[3];
uint8   receive_num = 0;
vuint8  uart_receive_flag;

uint8   link_list_num;
uint8   link_list_num2;

boolean is_first_image = 1; //��һ֡ͼ��


uint16 Parent[maxNum]={0};                //���׽ڵ�����  ���鼯  ����λ��Ч


uint16 label[MT9V03X_H][MT9V03X_W]={{0}};  //��ǩ����  ������������Ϊ��Чֵ      uint8 ����������
uint16 labelarea[maxNum]={0};
uint8 labelleft[maxNum]={0};
uint8 labelright[maxNum]={0};
uint8 labelup[maxNum]={0};
uint8 labeldown[maxNum]={0};

extern boolean slowdown;
extern uint8 mode_dis;
extern uint8 judgecounter;
//��Ҫ���õ�����ͷ������
int16 MT9V03X_CFG[CONFIG_FINISH][2]=
{
        {AUTO_EXP,          0},   //�Զ��ع�����      ��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
        //һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
        {EXP_TIME,          450}, //�ع�ʱ��          ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ
        {FPS,               50},  //ͼ��֡��          ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS
        {SET_COL,           MT9V03X_W}, //ͼ��������        ��Χ1-752     K60�ɼ���������188
        {SET_ROW,           MT9V03X_H}, //ͼ��������        ��Χ1-480
        {LR_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ188 376 752ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
        {UD_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ120 240 480ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
        {GAIN,              32},  //ͼ������          ��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�


        {INIT,              0}    //����ͷ��ʼ��ʼ��
};

//---------------------------����ͷ��
int16 MT9V03X2_CFG[CONFIG_FINISH][2]=
{
        {AUTO_EXP,          0},   //�Զ��ع�����      ��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
        //һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
        {EXP_TIME,          450}, //�ع�ʱ��          ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ
        {FPS,               50},  //ͼ��֡��          ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS
        {SET_COL,           MT9V03X_W}, //ͼ��������        ��Χ1-752     K60�ɼ���������188
        {SET_ROW,           MT9V03X_H}, //ͼ��������        ��Χ1-480
        {LR_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ188 376 752ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
        {UD_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ120 240 480ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
        {GAIN,              32},  //ͼ������          ��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�


        {INIT,              0}    //����ͷ��ʼ��ʼ��
};

//������ͷ�ڲ���ȡ������������
int16 GET_CFG[CONFIG_FINISH-1][2]=
{
        {AUTO_EXP,          0},   //�Զ��ع�����
        {EXP_TIME,          0},   //�ع�ʱ��
        {FPS,               0},   //ͼ��֡��
        {SET_COL,           0},   //ͼ��������
        {SET_ROW,           0},   //ͼ��������
        {LR_OFFSET,         0},   //ͼ������ƫ����
        {UD_OFFSET,         0},   //ͼ������ƫ����
        {GAIN,              0},   //ͼ������
};

//����ͷ��
int16 GET_CFG2[CONFIG_FINISH-1][2]=
{
        {AUTO_EXP,          0},   //�Զ��ع�����
        {EXP_TIME,          0},   //�ع�ʱ��
        {FPS,               0},   //ͼ��֡��
        {SET_COL,           0},   //ͼ��������
        {SET_ROW,           0},   //ͼ��������
        {LR_OFFSET,         0},   //ͼ������ƫ����
        {UD_OFFSET,         0},   //ͼ������ƫ����
        {GAIN,              0},   //ͼ������
};


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X����ͷ�����жϺ���
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       �˺�����isr.c�� �������жϺ�������
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_uart_callback(void)
{
    while(uart_query(MT9V03X_COF_UART, &receive[receive_num]))
    {
        receive_num++;

        if(1==receive_num && 0XA5!=receive[0])  receive_num = 0;
        if(3 == receive_num)
        {
            receive_num = 0;
            uart_receive_flag = 1;
        }

    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X����ͷ�����жϺ���
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       �˺�����isr.c�� �������жϺ�������
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_uart_callback(void)
{
    while(uart_query(MT9V03X2_COF_UART, &receive[receive_num]))
    {
        receive_num++;

        if(1==receive_num && 0XA5!=receive[0])  receive_num = 0;
        if(3 == receive_num)
        {
            receive_num = 0;
            uart_receive_flag = 1;
        }

    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������ͷ�ڲ�������Ϣ
//  @param      uartn       ѡ��ʹ�õĴ���
//  @param      buff        ����������Ϣ�ĵ�ַ
//  @return     void
//  @since      v1.0
//  Sample usage:           ���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
void set_config(UARTN_enum uartn, int16 buff[CONFIG_FINISH-1][2])
{
    uint16 temp, i;
    uint8  send_buffer[4];

    uart_receive_flag = 0;

    //���ò���  ������ο���������ֲ�
    //��ʼ��������ͷ�����³�ʼ��
    for(i=0; i<CONFIG_FINISH; i++)
    {
        send_buffer[0] = 0xA5;
        send_buffer[1] = (uint8)buff[i][0];
        temp = buff[i][1];
        send_buffer[2] = temp>>8;
        send_buffer[3] = (uint8)temp;
        
        uart_putbuff(uartn,send_buffer,4);
        systick_delay_ms(STM0, 2);
    }
    //�ȴ�����ͷ��ʼ���ɹ�
    while(!uart_receive_flag);
    uart_receive_flag = 0;
    while((0xff != receive[1]) || (0xff != receive[2]));
    //���ϲ��ֶ�����ͷ���õ�����ȫ�����ᱣ��������ͷ��51��Ƭ����eeprom��
    //����set_exposure_time�����������õ��ع����ݲ��洢��eeprom��
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����ͷ�ڲ�������Ϣ
//  @param      uartn       ѡ��ʹ�õĴ���
//  @param      buff        ����������Ϣ�ĵ�ַ
//  @return     void
//  @since      v1.0
//  Sample usage:           ���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
void get_config(UARTN_enum uartn, int16 buff[CONFIG_FINISH-1][2])
{
    uint16 temp, i;
    uint8  send_buffer[4];

    for(i=0; i<CONFIG_FINISH-1; i++)
    {
        send_buffer[0] = 0xA5;
        send_buffer[1] = GET_STATUS;
        temp = buff[i][0];
        send_buffer[2] = temp>>8;
        send_buffer[3] = (uint8)temp;
        
        uart_putbuff(uartn,send_buffer,4);
        
        //�ȴ����ܻش�����
        while(!uart_receive_flag);
        uart_receive_flag = 0;
        
        buff[i][1] = receive[1]<<8 | receive[2];
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����ͷ�̼��汾
//  @param      uartn       ѡ��ʹ�õĴ���
//  @return     void
//  @since      v1.0
//  Sample usage:           ���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
uint16 get_version(UARTN_enum uartn)
{
    uint16 temp;
    uint8  send_buffer[4];
    send_buffer[0] = 0xA5;
    send_buffer[1] = GET_STATUS;
    temp = GET_VERSION;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8)temp;

    uart_putbuff(uartn,send_buffer,4);

    //�ȴ����ܻش�����
    while(!uart_receive_flag);
    uart_receive_flag = 0;

    return ((uint16)(receive[1]<<8) | receive[2]);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������������ͷ�ع�ʱ��
//  @param      uartn       ѡ��ʹ�õĴ���
//  @param      light       �����ع�ʱ��Խ��ͼ��Խ��������ͷ�յ������ݷֱ��ʼ�FPS��������ع�ʱ��������õ����ݹ�����ô����ͷ��������������ֵ
//  @return     uint16      ��ǰ�ع�ֵ������ȷ���Ƿ���ȷд��
//  @since      v1.0
//  Sample usage:           ���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
uint16 set_exposure_time(UARTN_enum uartn, uint16 light)
{
    uint16 temp;
    uint8  send_buffer[4];

    send_buffer[0] = 0xA5;
    send_buffer[1] = SET_EXP_TIME;
    temp = light;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8)temp;

    uart_putbuff(uartn,send_buffer,4);

    //�ȴ����ܻش�����
    while(!uart_receive_flag);
    uart_receive_flag = 0;

    temp = receive[1]<<8 | receive[2];
    return temp;

}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ͷ�ڲ��Ĵ�������д����
//  @param      uartn       ѡ��ʹ�õĴ���
//  @param      addr        ����ͷ�ڲ��Ĵ�����ַ
//  @param      data        ��Ҫд�������
//  @return     uint16      �Ĵ�����ǰ���ݣ�����ȷ���Ƿ�д��ɹ�
//  @since      v1.0
//  Sample usage:           ���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
uint16 set_mt9v03x_reg(UARTN_enum uartn, uint8 addr, uint16 data)
{
    uint16 temp;
    uint8  send_buffer[4];

    send_buffer[0] = 0xA5;
    send_buffer[1] = SET_ADDR;
    temp = addr;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8)temp;

    uart_putbuff(uartn,send_buffer,4);
    systick_delay_ms(STM0, 10);

    send_buffer[0] = 0xA5;
    send_buffer[1] = SET_DATA;
    temp = data;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8)temp;

    uart_putbuff(uartn,send_buffer,4);

    //�ȴ����ܻش�����
    while(!uart_receive_flag);
    uart_receive_flag = 0;

    temp = receive[1]<<8 | receive[2];
    return temp;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X����ͷ��ʼ��
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:   ʹ��FLEXIO�ӿڲɼ�����ͷ
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_init(void)
{
    uint8 i;
    camera_type = CAMERA_GRAYSCALE;//������������ͷ����
    camera_buffer_addr = mt9v03x_image[0];

    boolean interrupt_state = disableInterrupts();

    uart_init (MT9V03X_COF_UART, 9600, MT9V03X_COF_UART_TX, MT9V03X_COF_UART_RX);   //��ʼ������ ��������ͷ
    enableInterrupts();//�����ж�

    //�ȴ�����ͷ�ϵ��ʼ���ɹ� ��ʽ�����֣���ʱ����ͨ����ȡ���õķ�ʽ ��ѡһ
    //systick_delay_ms(STM0, 1000);//��ʱ��ʽ
    get_config(MT9V03X_COF_UART, GET_CFG);//��ȡ���õķ�ʽ

    uart_receive_flag = 0;
    set_config(MT9V03X_COF_UART, MT9V03X_CFG);

    //��ȡ���ñ��ڲ鿴�����Ƿ���ȷ
    get_config(MT9V03X_COF_UART, GET_CFG);

    disableInterrupts();

    //����ͷ�ɼ���ʼ��
    //��ʼ�� ��������
    for(i=0; i<8; i++)
    {
        gpio_init((PIN_enum)(MT9V03X_DATA_PIN+i), GPI, 0, PULLUP);
    }

    link_list_num = eru_dma_init(MT9V03X_DMA_CH, GET_PORT_IN_ADDR(MT9V03X_DATA_PIN), camera_buffer_addr, MT9V03X_PCLK_PIN, FALLING, MT9V03X_W*MT9V03X_H);//�����Ƶ��300M �����ڶ�������������ΪFALLING

    eru_init(MT9V03X_VSYNC_PIN, FALLING);   //��ʼ�����жϣ�������Ϊ�½��ش����ж�


    eru_disable_interrupt(MT9V03X_PCLK_PIN);
    eru_disable_interrupt(MT9V03X_VSYNC_PIN);
    restoreInterrupts(interrupt_state);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X2����ͷ��ʼ��
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:   ʹ��FLEXIO�ӿڲɼ�����ͷ
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_init(void)
{
    uint8 i;
    camera_type = CAMERA_GRAYSCALE;//������������ͷ����
    camera_buffer_addr = mt9v03x_image[0];

    boolean interrupt_state = disableInterrupts();

    uart_init (MT9V03X2_COF_UART, 9600, MT9V03X2_COF_UART_TX, MT9V03X2_COF_UART_RX);   //��ʼ������ ��������ͷ
    enableInterrupts();//�����ж�

    //�ȴ�����ͷ�ϵ��ʼ���ɹ� ��ʽ�����֣���ʱ����ͨ����ȡ���õķ�ʽ ��ѡһ
    //systick_delay_ms(STM0, 1000);//��ʱ��ʽ
    get_config(MT9V03X2_COF_UART, GET_CFG2);//��ȡ���õķ�ʽ

    uart_receive_flag = 0;
    set_config(MT9V03X2_COF_UART, MT9V03X2_CFG);

    //��ȡ���ñ��ڲ鿴�����Ƿ���ȷ
    get_config(MT9V03X2_COF_UART, GET_CFG2);

    disableInterrupts();

    //����ͷ�ɼ���ʼ��
    //��ʼ�� ��������
    for(i=0; i<8; i++)
    {
        gpio_init((PIN_enum)(MT9V03X2_DATA_PIN+i), GPI, 0, PULLUP);
    }

    link_list_num2 = eru_dma2_init(MT9V03X2_DMA_CH, GET_PORT_IN_ADDR(MT9V03X2_DATA_PIN), camera_buffer_addr, MT9V03X2_PCLK_PIN, FALLING, MT9V03X_W*MT9V03X_H);//�����Ƶ��300M �����ڶ�������������ΪFALLING

    eru_init(MT9V03X2_VSYNC_PIN, FALLING);   //��ʼ�����жϣ�������Ϊ�½��ش����ж�

    eru_disable_interrupt(MT9V03X2_PCLK_PIN);
    eru_disable_interrupt(MT9V03X2_VSYNC_PIN);
    restoreInterrupts(interrupt_state);
}

uint8   mt9v03x_finish_flag = 0;    //һ��ͼ��ɼ���ɱ�־λ
uint8   mt9v03x_dma_int_num;    //��ǰDMA�жϴ���
//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X����ͷ���ж�
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               �˺�����isr.c�б�eru��GPIO�жϣ��жϵ���
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_vsync(void)
{
    CLEAR_GPIO_FLAG(MT9V03X_VSYNC_PIN);
    mt9v03x_dma_int_num = 0;
    if(!mt9v03x_finish_flag)//�鿴ͼ�������Ƿ�ʹ����ϣ����δʹ������򲻿�ʼ�ɼ���������ַ��ʳ�ͻ
    {
        if(1 == link_list_num)
        {
            //û�в������Ӵ���ģʽ ��������Ŀ�ĵ�ַ
            DMA_SET_DESTINATION(MT9V03X_DMA_CH, camera_buffer_addr);
        }
        dma_start(MT9V03X_DMA_CH);
    }

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X2����ͷ���ж�
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               �˺�����isr.c�б�eru��GPIO�жϣ��жϵ���
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_vsync(void)
{
    CLEAR_GPIO_FLAG(MT9V03X2_VSYNC_PIN);
    mt9v03x_dma_int_num = 0;
    if(!mt9v03x_finish_flag)//�鿴ͼ�������Ƿ�ʹ����ϣ����δʹ������򲻿�ʼ�ɼ���������ַ��ʳ�ͻ
    {
        if(1 == link_list_num2)
        {
            //û�в������Ӵ���ģʽ ��������Ŀ�ĵ�ַ
            DMA_SET_DESTINATION(MT9V03X2_DMA_CH, camera_buffer_addr);
        }
        dma_start(MT9V03X2_DMA_CH);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X����ͷDMA����ж�
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               �˺�����isr.c�б�dma�жϵ���
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_dma(void)
{
    static uint8 notfound_times = 0;

    CLEAR_DMA_FLAG(MT9V03X_DMA_CH);
    mt9v03x_dma_int_num++;


    if(mt9v03x_dma_int_num >= link_list_num)        //����beacon_found    beacon_bias     beacon_area
    {
        //�ɼ����
        mt9v03x_dma_int_num = 0;
        mt9v03x_finish_flag = 1;//һ��ͼ��Ӳɼ���ʼ���ɼ�������ʱ3.8MS����(50FPS��188*120�ֱ���)
        dma_stop(MT9V03X_DMA_CH);
        //�رճ��ж�
        eru_disable_interrupt(MT9V03X_VSYNC_PIN);
/*-----------------------------------------------��ֵ��-----------------------------------------------*/
        uint32 time;
        systick_start(STM0);
        mean_filter(MT9V03X_W,MT9V03X_H);
//        GetOSTU(MT9V03X_W,MT9V03X_H);        //5ms

//        lcd_displayimage032(mt9v03x_bin_image[0],MT9V03X_W,MT9V03X_H);    //������ӡһ��54ms
        boolean not_found = 0;


/*-----------------------------------------------��ȡ�ű�-----------------------------------------------*/
        not_found = extract_beacon(&beacon_area,&beacon_bias,&beacon_len,0);     //�������ͷͼ������ �������Ϊ����Խ��  ���labelnum�Ƿ񳬹��� maxNum-1  6ms
                                                                                 //���п����Ǵ�ӡ��Ļ����




/*-----------------------------------------------���������-----------------------------------------------*/
        //δ�ҵ��ű�Ʒ���  ���Ҳ��ı�  beacon_area  beacon_bias   beacon_len

        if(!not_found)
        {
            notfound_times++;

            if(notfound_times>2)
            {
                beacon_found = 0;      //�ű�ƶ�ʧ
                mode_dis = 0;
                judgecounter = 0;
            }
        }
        else      //�ҵ��ű��
        {
            notfound_times = 0;
            beacon_found = 1;
            slowdown = 0;
//        lcd_showuint16(0,0,beacon_area);        //�������в���ͬʱ��ӡ����������ͷ����
//        lcd_showint16(0,0,beacon_bias);        //һ��Ҫ�ڿ����ű�Ʋ��ܺ��ӡ
//        systick_delay_ms(STM0,10);             //��ӡint16��Ҫ��ʱ �����ӡ����ֵ���ȶ�
//        lcd_showuint8(0,2,beacon_len);          //��ӡ�����ر����׳�����
        beacon_dis = estimate_distance(beacon_len);
//        lcd_showuint16(0,3,beacon_dis);
        }
//        lcd_showstr(0,1,"f");
        time = systick_getval(STM0);
//        lcd_showint32(0,TFT_Y_MAX/16-1,time,8);



/*-----------------------------------------------����ͷ�л�-----------------------------------------------*/

        //�ر�����ͷ1
        mt9v03x_finish_flag = 0;
#if 1
        if(beacon_found == 0)       //δ�ҵ��ű�����л�����ͷ
        {
            camera_change = 1;
            eru_disable_interrupt(MT9V03X_PCLK_PIN);
            eru_enable_interrupt(MT9V03X2_VSYNC_PIN);
            eru_enable_interrupt(MT9V03X2_PCLK_PIN);
        }
        else                       //�������õ�ǰ����ͷ
        {
            camera_change = 0;
            eru_enable_interrupt(MT9V03X_VSYNC_PIN);
        }
#else
        eru_enable_interrupt(MT9V03X_VSYNC_PIN);    //���Ե�����ͷʱ�� ��ע���ϱ�if else
#endif
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X2����ͷDMA����ж�
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               �˺�����isr.c�б�dma�жϵ���
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_dma(void)
{
    static uint8 notfound_times = 0;

    CLEAR_DMA_FLAG(MT9V03X2_DMA_CH);
    mt9v03x_dma_int_num++;


    if(mt9v03x_dma_int_num >= link_list_num2)        //����beacon_found    beacon_bias     beacon_area
    {
        //�ɼ����
        mt9v03x_dma_int_num = 0;
        mt9v03x_finish_flag = 1;//һ��ͼ��Ӳɼ���ʼ���ɼ�������ʱ3.8MS����(50FPS��188*120�ֱ���)
        dma_stop(MT9V03X2_DMA_CH);
        //�رճ��ж�
        eru_disable_interrupt(MT9V03X2_VSYNC_PIN);
/*-----------------------------------------------��ֵ��-----------------------------------------------*/
        uint32 time;
        systick_start(STM0);
        mean_filter(MT9V03X_W,MT9V03X_H);
//        GetOSTU(MT9V03X_W,MT9V03X_H);        //5ms
//        lcd_displayimage032(mt9v03x_bin_image[0],MT9V03X_W,MT9V03X_H);    //������ӡһ��54ms
        boolean not_found = 0;



/*-----------------------------------------------��ȡ�ű�-----------------------------------------------*/
        not_found = extract_beacon(&beacon_area,&beacon_bias,&beacon_len,0);     //�������ͷͼ������ �������Ϊ����Խ��  ���labelnum�Ƿ񳬹��� maxNum-1  6ms
                                                                                 //���п����Ǵ�ӡ��Ļ����


/*-----------------------------------------------���������-----------------------------------------------*/
        //δ�ҵ��ű�Ʒ���  ���Ҳ��ı�  beacon_area  beacon_bias   beacon_len
        if(!not_found)
        {
            notfound_times++;
            if(notfound_times>2)
            {
                beacon_found = 0;      //�ű�ƶ�
                mode_dis = 0;
                judgecounter = 0;
            }
        }
        else  //�ҵ��ű��
        {
            notfound_times = 0;
            beacon_found = 1;
            slowdown = 0;

//        lcd_showuint16(0,0,beacon_area);        //�������в���ͬʱ��ӡ����������ͷ����
//        lcd_showint16(0,0,beacon_bias);        //һ��Ҫ�ڿ����ű�Ʋ��ܺ��ӡ
//        systick_delay_ms(STM0,10);             //��ӡint16��Ҫ��ʱ �����ӡ����ֵ���ȶ�
//        lcd_showuint8(0,0,beacon_len);          //��ӡ�����ر����׳�����
          beacon_dis = estimate_distance(beacon_len);
//          lcd_showuint16(0,3,beacon_dis);
        }

//        lcd_showstr(0,1,"s");
        time = systick_getval(STM0);
//        lcd_showint32(0,TFT_Y_MAX/16-1,time,8);



/*-----------------------------------------------����ͷ�л�-----------------------------------------------*/
        //�ر�����ͷ2
        mt9v03x_finish_flag = 0;

#if 1
        if(beacon_found == 0)       //δ�ҵ��ű�����л�����ͷ
        {
            camera_change = 0;
            eru_disable_interrupt(MT9V03X2_PCLK_PIN);
            eru_enable_interrupt(MT9V03X_VSYNC_PIN);
            eru_enable_interrupt(MT9V03X_PCLK_PIN);
        }
        else                        //�������õ�ǰ����ͷ
        {
            camera_change = 1;
            eru_enable_interrupt(MT9V03X2_VSYNC_PIN);
        }
#else
        eru_enable_interrupt(MT9V03X2_VSYNC_PIN);    //���Ե�����ͷʱ�� ��ע���ϱ�if else
#endif
    }
}





/*******���
 *
 * ����ֵ�� ��ֵ����ֵ��0~255
 *
 */
uint16 GetOSTU (uint16 width,uint16 height)    //��ά������Ϊ�����Ƿ���У�����
{
    sint16 i, j;
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelshortegralBack = 0;
    uint32 Pixelshortegral = 0;
    sint32 PixelshortegralFore = 0;
    sint32 PixelFore = 0;
    float32 OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    sint16 MinValue, MaxValue;              //���ҶȺ���С�Ҷ�
    uint16 Threshold = 0;
    uint8 HistoGram[256];               //�Ҷ�ֱ��ͼ

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
        {
            HistoGram[mt9v03x_image[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }

    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            if (mt9v03x_image[i][j] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
                mt9v03x_bin_image[i][j] = 255;          //255������1,TFT��ʾ����
            else
                mt9v03x_bin_image[i][j] = 0;
        }
    }

      return Threshold;                        //���������ֵ;
}


/***************ƽ��ֵ�˲�
 *
 *
 * ��ַ���Σ�
 *
 *
 */
uint16 mean_filter(uint16 width,uint16 height)
{
    uint16 Threshold = 0;
    uint32 tv = 0;
    uint8 i,j;

    //�ۼ�
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            tv += mt9v03x_image[i][j];   //�ۼ�
        }
    }
    Threshold =(unsigned short)(tv / height / width);   //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
    Threshold = Threshold + 30;      //�˴���ֵ���ã����ݻ����Ĺ������趨


    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            if (mt9v03x_image[i][j] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
                mt9v03x_bin_image[i][j] = 255;          //255������1,TFT��ʾ����
            else
                mt9v03x_bin_image[i][j] = 0;
        }
    }

      return Threshold;                        //���������ֵ;
}


/*!
 * @brief    ����soble���ؼ�����ӵ�һ���Զ���ֵ���ؼ��
 *
 * @param    imageIn    ��������
 *           imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
 *
 * @return
 *
 * @note
 *
 * @example
 */

void sobelAutoThreshold(uint8  imageIn[MT9V03X_H][MT9V03X_W], uint8  imageOut[MT9V03X_H][MT9V03X_W])
{
    /** ����˴�С */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* ���㲻ͬ�����ݶȷ�ֵ  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                    + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
                    + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = 255;
            }
            else
            {
                imageOut[i][j] = 0;
            }

        }
    }
}



/*-------------------------------------------------�����ͨ���벢�鼯-----------------------------------------------*/




//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���鼯�Ҹ�
//  @return     ��ǩ����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline uint16 find(uint16 x)
{
    uint16 i=x;
    while(Parent[i]!=0)
        i=Parent[i];
    return i;
}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���鼯�ϲ�
//
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline void merge(uint16 x,uint16 y)   //x y  ���ڲ��鼯�ϲ�
{
    if(x==y)return ;
    uint16 i=x;
    uint16 j=y;
    while(Parent[i]!=0)
        i=Parent[i];
    while(Parent[j]!=0)
        j=Parent[j];
    if(i<j)Parent[j]=i;
    else Parent[i]=j;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���鼯����
//  @param      Parent[]ǰn + 1λ����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline void union_find_clear(uint16 n)
{
    if(n==0xffff)n--;
    for(uint16 i=0;i<=n;i++)
        Parent[i]=0;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���鼯�������
//
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline void insert(uint16 left,uint16 up)      //up --> left
{
    if(left==up)return;         //ͬ��ǩ������
    if(Parent[left]!=0)
    {
        uint16 i=left;
        uint16 j=up;
        while(Parent[i]!=0)
            i=Parent[i];
        while(Parent[j]!=0)
            j=Parent[j];
        if(i==j)return ;        //ͬ���򲻲���

        Parent[j]=i;

    }
    else
        Parent[up]=left;        //����Parent[left] = up
                                //������Ϊ���γ�ѭ��
}






/*********��ֵ��ͼ����ȡ�ű����������***********Two-Pass�㷨
 *
 *
   *  ��ַ���Σ��ű����     �ű�����
   * �ҵ��ű�Ʒ���true
 *
 *
 *
 */
boolean  extract_beacon(uint16 *area,sint16 *bias,uint8 *len,boolean PRINT)
{
    uint16 labelnum=0;              //��ȡֵ��Χ 0x0000 ~ 0xfffe  ������Ч  ʵ�����ֵӦ����maxNum - 1
    uint8 i,j;        //iΪ��   jΪ��
    uint16 k;
    uint16 whitepixelnum = 0;

    //��һ��   ������ǩ����
    for(j=1;j<MT9V03X_H-5;j++)           //���ϵ��� ������ ��Χ�ֱ���  1~155  10~178
        for(i=10;i<MT9V03X_W-10;i++)
            if(mt9v03x_bin_image[j][i]==255)
            {
                whitepixelnum++;
                uint8 up = (label[j-1][i]==0)?0:1;      //��һ�����ص� ��ֵ��
                uint16 up_label = label[j-1][i];
                uint8 left = (label[j][i-1]==0)?0:1;    //��һ�����ص�
                uint16 left_label = label[j][i-1];
                                                                    //��������ϵı�ǩ���� ���Էֳ��������
                if(up==0&&left==0)
                {
                    labelnum++;
                    if(labelnum >= maxNum)return FALSE;             //��ǩ��������ǰ����
                    label[j][i]=labelnum;
                    labelarea[labelnum]++;
                    labelleft[labelnum]=i;
                    labelright[labelnum]=i;
                    labelup[labelnum]=j;
                    labeldown[labelnum]=j;
                }
                else if(up==1&&left==0)
                {
                    label[j][i]=up_label;
                    labelarea[up_label]++;
                    labeldown[up_label]=j;
                }
                else if(up==0&&left==1)
                {
                    label[j][i]=left_label;
                    labelarea[left_label]++;
                    labelright[left_label]=i;
                }
                else                                //���϶��б����ȡ��С
                {
                    insert(left_label,up_label);    //���� ���鼯 up-->left
                    if(up_label>left_label)
                    {
                        label[j][i]=left_label;
                        labelarea[left_label]++;
                        labelright[left_label]=i;
                    }
                    else                            //����С�ں͵����������
                    {
                        label[j][i]=up_label;
                        labelarea[up_label]++;
                        labeldown[up_label]=j;
                    }
                }
            }

    //���ع������˳�
    if(whitepixelnum < 5)return FALSE;

    //�ڶ���   �ϲ���ǩ����
    for(k=1;k<=labelnum&&k!=0xffff;k++)  //��ֹlabelnum == 0xffff
    {
        if(Parent[k]!=0)        //���Ǹ��ڵ�
        {
            uint16 p=0;
            p=find(k);          //Ѱ�Ҹ��ڵ�
            labelarea[p]+=labelarea[k];
            labelarea[k]=0;
            if(labelleft[k]<labelleft[p])labelleft[p]=labelleft[k];
            labelleft[k]=0xff;
            if(labelright[k]>labelright[p])labelright[p]=labelright[k];
            labelright[k]=0x00;
            if(labelup[k]<labelup[p])labelup[p]=labelup[k];
            labelup[k]=0xff;
            if(labeldown[k]>labeldown[p])labeldown[p]=labeldown[k];
            labeldown[k]=0x00;
        }
    }



//    ������   Ѱ����������ͨ�� �� ��״ƥ��
    uint16 Smax = 0;
    boolean validlabel = 0;
    uint16 Smax_label=0;
    float diff = 1000;
    for(k=1;k<=labelnum&&k!=0xffff;k++)
        if(labelarea[k]>Smax){
            Smax=labelarea[k];
            Smax_label = k;
        }
//    {
//        if(labelarea[k]!=0&&(labelright[k]<(MT9V03X_W-10))&&(labelleft[k]>10)&&(labelup[k]>5))     //���׽ڵ� �� ���ڲ�����
//    if(labelarea[k]!=0)
//        {
//            validlabel = 1;
//            float temp_diff;
//            temp_diff = (float)(labelright[k] - labelleft[k])*(labeldown[k] - labelup[k])*Pi/4.0;
//            temp_diff = (float)labelarea[k]/temp_diff-1;
//            if(temp_diff<diff){         //��״���ƥ��
//                diff = temp_diff;
//                Smax_label = k;
//            }
//        }
//    }
//    if(!validlabel) return FALSE;   //���������ű�

    uint16 mid;
    mid = ((uint16)labelleft[Smax_label] + (uint16)labelright[Smax_label])>>1;
    *bias = (sint16)(mid - MT9V03X_W/2);
    *len = labelright[Smax_label] - labelleft[Smax_label];
    *area = labelarea[Smax_label];


    if(PRINT)
    {
        for(j=labelup[Smax_label];j<labeldown[Smax_label];j++)lcd_drawpoint(mid*TFT_X_MAX/MT9V03X_W,j,RED);         //���ӻ�����    ��ֱ��
    }
    //���鼯����
    for(j=0;j<MT9V03X_H;j++)
        for(i=0;i<MT9V03X_W;i++)label[j][i]=0;
    for(k=0;k<=labelnum;k++)
    {
        labelarea[k]=0;
        labelleft[k]=0xff;
        labelright[k]=0;
        labelup[k]=0xff;
        labeldown[k]=0;
    }

    for( k=0;k<=labelnum;k++)    //ע��uint16���ֵ
        Parent[k]=0;


    return TRUE;
}






/*********��ֵ��ͼ����ȡ�ű����������***����ɨ�跨
 *
 *
 *��ַ���Σ��ű����     �ű�����
 *
 *
 *
 */
void extract_beacon2(uint16 *area,uint16 *mid)
{

    int16 left = MT9V03X_W-1;
    int16 right = 0;
    uint16 S = 0;

    int16 i,j,k;
    uint8 flag1=0;
    uint8 flag2=0;

    for(j=0;j<MT9V03X_H/2;j++)
    {
        if(mt9v03x_bin_image[j][MT9V03X_W/2] == 255)
        {
            flag1=1;break;
        }
    }


    if(flag1 == 0)  //ͼ�����м�
    {
        for(i=MT9V03X_W/2 - 1;i>=0;i--)     //����ɨ�裬���м�����
            for(j=0;j<MT9V03X_H/2;j++)
            {
                if(mt9v03x_bin_image[j][i] == 255)
                {
                    flag2=1;
                    if(mt9v03x_bin_image[j][i+1]==0 && mt9v03x_bin_image[j][i+2]==0 &&  mt9v03x_bin_image[j][i+3]==0)
                        {
                        uint8 flag3 = 0;
                        uint8 L = (0>(i-3))?0:(i-3);
                        for(k=i-1;k>=L;k--)
                            if(mt9v03x_bin_image[j][k] == 0){flag3=1;break;};
                        if(!flag3)
                            {
                            right = (i>right?i:right);  //�����ҽ�
                            break;  //��һ��
                            }
                        }
                    else if(mt9v03x_bin_image[j][i+1]==255 && mt9v03x_bin_image[j][i+2]==255 &&  mt9v03x_bin_image[j][i+3]==255)
                    {
                        uint8 flag3 = 0;
                        uint8 L = (0>(i-3))?0:(i-3);
                        for(k=i-1;k>=L;k--)
                            if(mt9v03x_bin_image[j][k] == 255){flag3=1;break;};
                        if(!flag3)
                        {
                            left = (i<left)?i:left;     //�������
                            break;  //��һ��
                        }
                    }

                }
            }


        if(!flag2)
        {
          for(i=MT9V03X_W/2 - 1;i<MT9V03X_W;i++)     //����ɨ�裬���м�����
            for(j=0;j<MT9V03X_H/2;j++)
            {
                if(mt9v03x_bin_image[j][i] == 255)
                {
                    if(mt9v03x_bin_image[j][i-1]==0 && mt9v03x_bin_image[j][i-2]==0 && mt9v03x_bin_image[j][i-3]==0)
                    {
                        uint8 flag3 = 0;
                        uint8 R = ((MT9V03X_W-1)>(i+3))?(i+3):(MT9V03X_W-1);
                        for(k=i+1;k<=R;k++)
                            if(mt9v03x_bin_image[j][k]==0){flag3=1;break;};
                        if(!flag3)
                        {
                            left = (i<left)?i:left;     //�������
                            break;  //��һ��
                        }
                    }
                    else if(mt9v03x_bin_image[j][i-1]==255 && mt9v03x_bin_image[j][i-2]==255 && mt9v03x_bin_image[j][i-3]==255)
                    {
                        uint8 flag3 = 0;
                        uint8 R = ((MT9V03X_W-1)>(i+3))?(i+3):(MT9V03X_W-1);
                        for(k=i+1;k<=R;k++)
                            if(mt9v03x_bin_image[j][k] == 255){flag3=1;break;};
                        if(!flag3)
                            {
                            right = (i>right?i:right);  //�����ҽ�
                            break;  //��һ��
                            }
                    }
                }
            }
        }
        if(flag2)lcd_showchar(0,0,'L');
        else lcd_showchar(0,0,'R');
    }
    // �м���ͼ��
    else
    {
        for(i=MT9V03X_W/2;i>=0;i--)
        {
            uint8 flag3=0;
            for(j=0;j<MT9V03X_H/2;j++)
                if(mt9v03x_bin_image[j][i]==255){left=i;flag3=1;break;}
            if(!flag3)break;        //������߽�
        }
        for(i=MT9V03X_W/2;i<MT9V03X_W;i++)
        {
            uint8 flag3=0;
            for(j=0;j<MT9V03X_H/2;j++)
                if(mt9v03x_bin_image[j][i]==255){right=i;flag3=1;break;}
            if(!flag3)break;        //�����ұ߽�
        }
        lcd_showchar(0,0,'M');
    }


    //�������ĺ����
    if(left<right)*mid = ((uint16)left + (uint16)right)/2;
    else *mid = 0;                  //����
    for(i=left;i<=right;i++)
        for(j=0;j<MT9V03X_H/2;j++)
            if(mt9v03x_bin_image[j][i]==255)S++;

    *area = S;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief  ǰ��ͼ�����Ѱ���ű��----------����beacon_found    beacon_bias     beacon_area
//  @return     void
//  @note
//-------------------------------------------------------------------------------------------------------------------

void beacon_xor(uint16 width,uint16 height,uint16 *area,uint16 *mid)        //Ӧ��˼·���ԣ��˺�����������˸ʶ��
{


}

//-------------------------------------------------------------------------------------------------------------------
//  @brief  �������
//  @param  beacon_areaĿ��������
//  @return     ����cm
//  @note
//����ͷ��ʾ��Ŀ���Ⱥ;���ɷ���
//ʵ�����ݣ�beacon_len/beacon_dis cm
//74/30
//47-56/60
//40/80
//19/140
//14/150
//9/180
//8/210
//6/240
//5/270

//-------------------------------------------------------------------------------------------------------------------

uint16 estimate_distance(uint8 beacon_len)
{
    static uint16 last_dis = 0;
    uint16 dis,sqrtlen;
    sqrtlen = sqrt(beacon_len);
    dis = 1443/sqrt(beacon_len)/sqrt(sqrt(beacon_len));//cm
    dis = 0.6*dis + 0.4*last_dis;//��ͨ�˲�
    last_dis = dis;
    return dis;
}
//�������ܣ� �� mt9v03x_bin_image ��ֵ�˲�
//����ֵ��  void
void MedianFilter()
{
    for (int row = 1; row < MT9V03X_H - 1; row++)
    {
        for (int col = 1; col< MT9V03X_W; col++)
        {
            unsigned char mid, buf[9];
            int k = 0;
            for (int i = -2; i < 3; i++)
                for (int j = 0; j < 1; j++)
                {
                    buf[k]= mt9v03x_bin_image[row + i][col + j];
                    k++;
                }
                //ð������
            unsigned char temp;
            int i = 0, j = 0;
            for (i = 0; i < 5 - 1; i++)
            for (j = 0; j < 5 - 1 - i; j++)
            {
                if (buf[j] > buf[j + 1])
                    {
                        temp = buf[j];
                        buf[j] = buf[j + 1];
                        buf[j + 1] = temp;
                    }
            }

            mid = buf[2];
            mt9v03x_image[row][col] = mid;
        }
    }
    for (int row = 0; row < MT9V03X_H; row++)
        for (int col = 0; col< MT9V03X_W; col++)
            mt9v03x_bin_image[row][col] = mt9v03x_image[row][col];
}
