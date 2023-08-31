/*******************************************************
 ������ƺ����ļ�
 ver1.0-2021.4.8
 ver2.0-2021.5.25 HT-S-3505
 ver3.0-2021.6.15 HT-S-3505
 *******************************************************/

/******************************************************************************/
/*-------------------------- ע�⣡����������--------------------------------------------------*/
//�������ͬһ���������ָ��֮�䲻����ʱ������ᶪ֡����������Ӧ�ڶ���ָ��21.4.8


#include "MotorCtrl.h"
#include "zf_gpio.h"
#include "zf_stm_systick.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief  �������Ϳ����֣����ֹͣ����
//  @return     void
//  @note
//-------------------------------------------------------------------------------------------------------------------
void MotorStop()
{
    //�����֣���ͷ3E������ţ�ID�������룬���ݳ��ȣ�0����У��λ
    //����IDΪ01������ID02
    uint8 ctrl_code_l[7] = {0x3E, 0x01, 0x01, 0x50, 0x00, 0x00, 0x00};
    uint8 ctrl_code_r[7] = {0x3E, 0x02, 0x02, 0x50, 0x00, 0x00, 0x00};
    N_CRC16(ctrl_code_l,5);
    N_CRC16(ctrl_code_r,5);
    uart_putbuff(uart_motor, ctrl_code_l, 7);
    systick_delay_us(STM0, 1000);
    uart_putbuff(uart_motor, ctrl_code_r, 7);
    systick_delay_us(STM0, 1000);
}

//-----------------------------------------------------------------------------------------------------------------
//  @brief      ���ٶȻ������ת��PD����
//  @param      n_c            //����С�����ĵ��ٶȣ�δ֪����ת�٣�ȡֵ-1,000,000~+1,000,000�����Խ���ȡֵ300,000������������ƽ��ֵ����ֵ��ʾheadflag = 0����ֵheadflag = 1
//  @param      bias           //����ͷͼ��õ����ű��������λ�õ�ƫ����Ϊ��������ֵ��ù�һ��
//  @return     void
//  @note       �õ���n_l,n_r��ֵ��ʾ��ǰ����ֵ��ʾ���ת��ע�����ֶԳƷ���������ͬһ����������ת�����෴
//               n_l��ʾ01�ŵ��ת�٣�n_r��ʾ02�ŵ��ת��
//-------------------------------------------------------------------------------------------------------------------
extern sint16 omiga_c = 0;

void AngularVelocityLoop(sint16 n_c, sint16 bias)
{
    static sint16 last_bias;//���ٶȻ������ﴢ����һ��ƫ��
    sint32 KP,KD;

    if(n_c > 8100)
    {
        KP = 30;
        KD = 30;
    }
    else if(n_c > 3000)
    {
        KP = 20;
        KD = 20;
    }
    else
    {
        KP = 10;
        KD = 10;
    }
    omiga_c = ((sint16)KP*bias + (sint16)KD*(bias - last_bias)) ;
    //�޷�        Ӧ�ö�omiga_c �޷�           ����������
    if(omiga_c > max)omiga_c = max;
    else if(omiga_c < min)omiga_c = min;      //�����ɸ�

    last_bias = bias;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief  �������Ϳ����֣�����������ת�٣������������ٶȻ������Ե���
//  @param  n_l,n_r ������ת��
//  @param
//  @return     void
//  @note   ת������0.1rpm
//-------------------------------------------------------------------------------------------------------------------
void SendSpeedCtrlCode(sint16 n_l,sint16 n_r)
{
    //�����֣���ͷ3E������ţ��ֱ�ӻ�Ӧ�𣩣��豸��ַ�룬�����룬���ݳ��ȣ����ݣ�С�ˣ���->�ߣ���У��λ����λ��С�ˣ�
    //����IDΪ01������ID02
    uint8 ctrl_code_l[9] = {0x3E, 0x01, 0x01, 0x54, 0x02, 0x00, 0x00, 0x00, 0x00};
    uint8 ctrl_code_r[9] = {0x3E, 0x02, 0x02, 0x54, 0x02, 0x00, 0x00, 0x00, 0x00};

    n_l =-n_l;//����������෴,l�����headflag = 0 Ĭ�����ת����Ҫȡ��ʹ����ǰת����r���һ��
    ctrl_code_l[5] = (uint8)(n_l & 0x00FF);
    ctrl_code_l[6] = (uint8)((n_l & 0xFF00)>>8);

    ctrl_code_r[5] = (uint8)(n_r & 0x00FF);
    ctrl_code_r[6] = (uint8)((n_r & 0xFF00)>>8);

    N_CRC16(ctrl_code_l,7);
    N_CRC16(ctrl_code_r,7);

    uart_putbuff(uart_motor, ctrl_code_l, 9);
    systick_delay_us(STM0, 1000);
    uart_putbuff(uart_motor, ctrl_code_r, 9);
    systick_delay_us(STM0, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief  ԭ����ת
//  @param  n ������ת�� ��ֵ��ʱ�룬��ֵ˳ʱ��
//  @return     void
//  @note   ת������δ֪
//-------------------------------------------------------------------------------------------------------------------
void Circling(sint16 n)
{

    SendSpeedCtrlCode(n,-n);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief  ����ʽ���ת�ǿ���
//  @param  angle_inc�Ƕ�������9000��ʾ90�㣬36000��ʾ360�㣬�Դ����ƣ�������.h������һЩ�꣬�Ա���ԣ�max_spd����ٶȣ�ע��Ϊ�޷���32λ
//  @return     void
//  @note
//����������
//IncrementAngleCtrl(deg_5,deg_30);
//gpio_toggle(P20_8);
//systick_delay_ms(STM0, 50);
//-------------------------------------------------------------------------------------------------------------------
void IncrementAngleCtrl(sint16 angle_inc_l, sint16 angle_inc_r )
{

    uint8 code_l[9] = { 0x3E, 0x01, 0x01, 0x56, 0x02, 0x00, 0x00, 0x00, 0x00 } ;
    uint8 code_r[9] = { 0x3E, 0x02, 0x02, 0x56, 0x02, 0x00, 0x00, 0x00, 0x00 } ;

    angle_inc_l = -angle_inc_l;
    code_l[5] = (uint8)(angle_inc_l & 0x00FF);
    code_l[6] = (uint8)((angle_inc_l & 0xFF00)>>8);

    code_r[5] = (uint8)(angle_inc_r & 0x000000FF);
    code_r[6] = (uint8)((angle_inc_r & 0x0000FF00)>>8);

    N_CRC16(code_l,7);
    N_CRC16(code_r,7);

    uart_putbuff(uart_motor, code_l, 9);
    systick_delay_us(STM0, 1000);
    uart_putbuff(uart_motor, code_r, 9);
    systick_delay_us(STM0, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief  д������������RAM
//  @param
//          [5]�豸��ַ ID = 0x01�� 0x02
//          [6]������ֵ I_threshold = ��ֵ��A��/0.03
//          [7]��ѹ��ֵ U_threshold = ��ֵ��V��/0.2
//          [8]RS485������  Boderate485 = 0����ʾ115200��
//          [9:12]����ʽ�ǶȻ�angle_Kp float
//          [13:16]����ʽ�ǶȻ��ٶ�angle_speed float
//          [17:20]�ٶȻ�������speed_Kp float
//          [21:24]�ٶȻ�������speed_Ki float
//          [29]�ٶ��˲�ϵ�� Kf = val*100
//          [30]������ʰٷֱ�P = 1~100Ĭ��92
//  @return     void
//  @note
//-------------------------------------------------------------------------------------------------------------------
void SetMotorPID(uint8 I_threshold,uint8 U_threshold,
        float32 angle_Kp,float32 angle_speed,
        float32 speed_Kp,float32 speed_Ki)
{
    //������������0x0D��Ϊ 0x0E Ϊд��ROM���粻��ʧ
    uint8 code_l[33] = { 0x3E, 0x01, 0x01, 0x0D, 0x1A, 0x01 } ;
    uint8 code_r[33] = { 0x3E, 0x01, 0x02, 0x0D, 0x1A, 0x02 } ;
    uint8 Boderate485 = 0;
    uint8 Kf = 0;
    uint8 P = 92;
    code_l[6] = I_threshold;
    code_r[6] = I_threshold;
    code_l[7] = U_threshold;
    code_r[7] = U_threshold;
    code_l[8] = Boderate485;
    code_r[8] = Boderate485;
    float2uint8(&code_l[9],angle_Kp);
    float2uint8(&code_r[9],angle_Kp);
    float2uint8(&code_l[13],angle_speed);
    float2uint8(&code_r[13],angle_speed);
    float2uint8(&code_l[17],speed_Kp);
    float2uint8(&code_r[17],speed_Kp);
    float2uint8(&code_l[21],speed_Ki);
    float2uint8(&code_r[21],speed_Ki);
    code_l[29] = Kf;
    code_r[29] = Kf;
    code_l[30] = P;
    code_r[30] = P;
    N_CRC16(code_l,31);
    N_CRC16(code_r,31);
    uart_putbuff(uart_motor, code_l, 33);
    systick_delay_us(STM0, 1000);
    uart_putbuff(uart_motor, code_r, 33);
    systick_delay_us(STM0, 1000);
}
void float2uint8(uint8 *buf,float32 data)
{
    uint8 *p = (uint8*)&data + 3;
    buf[0] = *(p-3);
    buf[1] = *(p-2);
    buf[2] = *(p-1);
    buf[3] = *p;
}


//CRC16У����
uint8 auchCRCHi[]=
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

uint8 auchCRCLo[] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};
//У�麯��
void N_CRC16(uint8 *updata,uint8 len)
{
  uint8 uchCRCHi=0xff;
  uint8 uchCRCLo=0xff;
  uint16  uindex;
  uint8 l = len;
  uint8 *p = updata;
  while(l--)
  {
  uindex=uchCRCHi^(*p++);
  uchCRCHi=uchCRCLo^auchCRCHi[uindex];
  uchCRCLo=auchCRCLo[uindex];
  }
  updata[len] = uchCRCHi;
  updata[len+1] = uchCRCLo;
}

//��������
//����Ϊ����ٶ�
uint8 len_threshold = 0;
sint16 ramp_v = 0;
extern boolean slowdown = 0;
extern uint8 mode_dis = 0;
extern uint8 judgecounter = 0;
void Guiding(sint16 target_v)
{
    sint32 dis_v = 40*(beacon_dis+len_threshold);//�ٶ��Ǿ���ĺ���

    if(camera_change==1)
    {
        target_v = -target_v;
        dis_v = -dis_v;
    }

    if(mode_dis == 1)
    {
        ramp_v = MovingAverageFilter(0);//����Ϊ�˸����˲��������鷽���������
        ramp_v = (sint16)dis_v;
        if(beacon_dis < 50 )
            judgecounter++;
        if (judgecounter > 5)
            mode_dis = 2;


    }
    else if(mode_dis == 2)
        ramp_v = target_v/8;
    else
    {
        ramp_v = MovingAverageFilter(target_v);
        judgecounter=0;
    }

    //�����ٶ�ģʽת�������ж�
    if(mode_dis == 0 &&
            ((dis_v - ramp_v < 0 && camera_change==0) ||
            (dis_v - ramp_v > 0 && camera_change==1)))
        mode_dis = 1;

    AngularVelocityLoop(ramp_v,beacon_bias);//�ý��ٶ�������
}


/////////����ƽ���˲�///////////////
#define FILTER_N_Motor 20//30
sint16 MovingAverageFilter(sint16 target_v)
{
    uint8 i;
    static sint32 maFilter_sum = 0;
    static sint16 maFilter_buff[FILTER_N_Motor] = {0};
    static sint16 last_target_v;

    if(last_target_v*target_v<0 || mode_dis==1)
    {
        for(i = 0; i < FILTER_N_Motor ; i++)
            maFilter_buff[i] = 0; // ����������գ���ֹ������
        maFilter_sum = 0;
    }

    maFilter_sum -= maFilter_buff[0];
    for(i = 0; i < FILTER_N_Motor-1 ; i++)
        maFilter_buff[i] = maFilter_buff[i + 1]; // �����������ƣ���λ�ӵ�
    maFilter_buff[FILTER_N_Motor-1] = target_v;
    maFilter_sum += maFilter_buff[FILTER_N_Motor-1];


    last_target_v = target_v;
    return (sint16)(maFilter_sum/(sint16)FILTER_N_Motor);
}

/////////��Ȩ�����˲�///////////////
sint16 WeightedRecursionFilter(sint16 target_v)
{
    uint8 i;

    sint32 wrFilter_sum = 0;
    static sint16 wrFilter_buff[FILTER_N_Motor + 1] = {0};
    wrFilter_buff[FILTER_N_Motor] = target_v;
    for(i=0;i<FILTER_N_Motor;i++)
    {
        wrFilter_buff[i]=wrFilter_buff[i+1];
        wrFilter_sum += wrFilter_buff[i]*(i+1);
    }
    return (sint16)(wrFilter_sum / (sint16)(( FILTER_N_Motor*(FILTER_N_Motor+1) )>>1 ));
}

//�������ٶȷ��������·
void GyroscopeDampingLoop(sint16 omiga_c)
{

     get_icm20602_gyro_spi();

     sint16 bias = omiga_c + (sint16)icm_gyro_z;     //��������
     static sint16 last_bias = 0;

     sint16 omiga_loop = omiga_c;
     omiga_loop = omiga_c + bias ;

     //�޷�

     //ִ��
     sint16 n_l,n_r;
     n_l = ramp_v + omiga_loop;
     n_r = ramp_v - omiga_loop;


     SendSpeedCtrlCode(n_l,n_r);

     last_bias = bias;
 }




