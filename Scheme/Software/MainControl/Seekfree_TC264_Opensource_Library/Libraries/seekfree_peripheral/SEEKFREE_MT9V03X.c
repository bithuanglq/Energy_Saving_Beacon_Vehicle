
    /*********************************************************************************************************************
     * COPYRIGHT NOTICE
     * Copyright (c) 2020,逐飞科技
     * All rights reserved.
     * 技术讨论QQ群：三群：824575535
     *
     * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
     * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
     *
     * @file            总钻风
     * @company         成都逐飞科技有限公司
     * @author          逐飞科技(QQ3184284598)
     * @version         查看doc内version文件 版本说明
     * @Software        ADS v1.2.2
     * @Target core     TC264D
     * @Taobao          https://seekfree.taobao.com/
     * @date            2020-3-23
     * @note
                        接线定义：
                        ------------------------------------
                        模块管脚                        单片机管脚
                        SDA(51的RX)              查看SEEKFREE_MT9V03X.h文件中的MT9V03X_COF_UART_TX宏定义
                        SCL(51的TX)              查看SEEKFREE_MT9V03X.h文件中的MT9V03X_COF_UART_RX宏定义
                        场中断(VSY)                查看SEEKFREE_MT9V03X.h文件中的MT9V03X_VSYNC_PIN宏定义
                        行中断(HREF)               程序没有使用，因此不连接
                        像素中断(PCLK)              查看SEEKFREE_MT9V03X.h文件中的MT9V03X_PCLK_PIN宏定义
                        数据口(D0-D7)          查看SEEKFREE_MT9V03X.h文件中的MT9V03X_DATA_PIN宏定义
                        ------------------------------------

                        默认分辨率是                      188*120
                        默认FPS                 50帧
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

#define maxNum 1000                  //最大标签数预估数量
#define Pi  3.14159
//必须4字节对齐
IFX_ALIGN(4) uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8   mt9v03x_bin_image[MT9V03X_H][MT9V03X_W];

uint8   receive[3];
uint8   receive_num = 0;
vuint8  uart_receive_flag;

uint8   link_list_num;
uint8   link_list_num2;

boolean is_first_image = 1; //第一帧图像


uint16 Parent[maxNum]={0};                //父亲节点数组  并查集  第零位无效


uint16 label[MT9V03X_H][MT9V03X_W]={{0}};  //标签数组  以下数组正数为有效值      uint8 不够！！！
uint16 labelarea[maxNum]={0};
uint8 labelleft[maxNum]={0};
uint8 labelright[maxNum]={0};
uint8 labelup[maxNum]={0};
uint8 labeldown[maxNum]={0};

extern boolean slowdown;
extern uint8 mode_dis;
extern uint8 judgecounter;
//需要配置到摄像头的数据
int16 MT9V03X_CFG[CONFIG_FINISH][2]=
{
        {AUTO_EXP,          0},   //自动曝光设置      范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
        //一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
        {EXP_TIME,          450}, //曝光时间          摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
        {FPS,               50},  //图像帧率          摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
        {SET_COL,           MT9V03X_W}, //图像列数量        范围1-752     K60采集不允许超过188
        {SET_ROW,           MT9V03X_H}, //图像行数量        范围1-480
        {LR_OFFSET,         0},   //图像左右偏移量    正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
        {UD_OFFSET,         0},   //图像上下偏移量    正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
        {GAIN,              32},  //图像增益          范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度


        {INIT,              0}    //摄像头开始初始化
};

//---------------------------摄像头二
int16 MT9V03X2_CFG[CONFIG_FINISH][2]=
{
        {AUTO_EXP,          0},   //自动曝光设置      范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
        //一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
        {EXP_TIME,          450}, //曝光时间          摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
        {FPS,               50},  //图像帧率          摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
        {SET_COL,           MT9V03X_W}, //图像列数量        范围1-752     K60采集不允许超过188
        {SET_ROW,           MT9V03X_H}, //图像行数量        范围1-480
        {LR_OFFSET,         0},   //图像左右偏移量    正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
        {UD_OFFSET,         0},   //图像上下偏移量    正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
        {GAIN,              32},  //图像增益          范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度


        {INIT,              0}    //摄像头开始初始化
};

//从摄像头内部获取到的配置数据
int16 GET_CFG[CONFIG_FINISH-1][2]=
{
        {AUTO_EXP,          0},   //自动曝光设置
        {EXP_TIME,          0},   //曝光时间
        {FPS,               0},   //图像帧率
        {SET_COL,           0},   //图像列数量
        {SET_ROW,           0},   //图像行数量
        {LR_OFFSET,         0},   //图像左右偏移量
        {UD_OFFSET,         0},   //图像上下偏移量
        {GAIN,              0},   //图像增益
};

//摄像头二
int16 GET_CFG2[CONFIG_FINISH-1][2]=
{
        {AUTO_EXP,          0},   //自动曝光设置
        {EXP_TIME,          0},   //曝光时间
        {FPS,               0},   //图像帧率
        {SET_COL,           0},   //图像列数量
        {SET_ROW,           0},   //图像行数量
        {LR_OFFSET,         0},   //图像左右偏移量
        {UD_OFFSET,         0},   //图像上下偏移量
        {GAIN,              0},   //图像增益
};


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X摄像头串口中断函数
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       此函数在isr.c中 被串口中断函数调用
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
//  @brief      MT9V03X摄像头串口中断函数
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       此函数在isr.c中 被串口中断函数调用
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
//  @brief      配置摄像头内部配置信息
//  @param      uartn       选择使用的串口
//  @param      buff        发送配置信息的地址
//  @return     void
//  @since      v1.0
//  Sample usage:           调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
void set_config(UARTN_enum uartn, int16 buff[CONFIG_FINISH-1][2])
{
    uint16 temp, i;
    uint8  send_buffer[4];

    uart_receive_flag = 0;

    //设置参数  具体请参看问题锦集手册
    //开始配置摄像头并重新初始化
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
    //等待摄像头初始化成功
    while(!uart_receive_flag);
    uart_receive_flag = 0;
    while((0xff != receive[1]) || (0xff != receive[2]));
    //以上部分对摄像头配置的数据全部都会保存在摄像头上51单片机的eeprom中
    //利用set_exposure_time函数单独配置的曝光数据不存储在eeprom中
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取摄像头内部配置信息
//  @param      uartn       选择使用的串口
//  @param      buff        接收配置信息的地址
//  @return     void
//  @since      v1.0
//  Sample usage:           调用该函数前请先初始化串口
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
        
        //等待接受回传数据
        while(!uart_receive_flag);
        uart_receive_flag = 0;
        
        buff[i][1] = receive[1]<<8 | receive[2];
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取摄像头固件版本
//  @param      uartn       选择使用的串口
//  @return     void
//  @since      v1.0
//  Sample usage:           调用该函数前请先初始化串口
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

    //等待接受回传数据
    while(!uart_receive_flag);
    uart_receive_flag = 0;

    return ((uint16)(receive[1]<<8) | receive[2]);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      单独设置摄像头曝光时间
//  @param      uartn       选择使用的串口
//  @param      light       设置曝光时间越大图像越亮，摄像头收到后会根据分辨率及FPS计算最大曝光时间如果设置的数据过大，那么摄像头将会设置这个最大值
//  @return     uint16      当前曝光值，用于确认是否正确写入
//  @since      v1.0
//  Sample usage:           调用该函数前请先初始化串口
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

    //等待接受回传数据
    while(!uart_receive_flag);
    uart_receive_flag = 0;

    temp = receive[1]<<8 | receive[2];
    return temp;

}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      对摄像头内部寄存器进行写操作
//  @param      uartn       选择使用的串口
//  @param      addr        摄像头内部寄存器地址
//  @param      data        需要写入的数据
//  @return     uint16      寄存器当前数据，用于确认是否写入成功
//  @since      v1.0
//  Sample usage:           调用该函数前请先初始化串口
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

    //等待接受回传数据
    while(!uart_receive_flag);
    uart_receive_flag = 0;

    temp = receive[1]<<8 | receive[2];
    return temp;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X摄像头初始化
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:   使用FLEXIO接口采集摄像头
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_init(void)
{
    uint8 i;
    camera_type = CAMERA_GRAYSCALE;//设置连接摄像头类型
    camera_buffer_addr = mt9v03x_image[0];

    boolean interrupt_state = disableInterrupts();

    uart_init (MT9V03X_COF_UART, 9600, MT9V03X_COF_UART_TX, MT9V03X_COF_UART_RX);   //初始换串口 配置摄像头
    enableInterrupts();//开启中断

    //等待摄像头上电初始化成功 方式有两种：延时或者通过获取配置的方式 二选一
    //systick_delay_ms(STM0, 1000);//延时方式
    get_config(MT9V03X_COF_UART, GET_CFG);//获取配置的方式

    uart_receive_flag = 0;
    set_config(MT9V03X_COF_UART, MT9V03X_CFG);

    //获取配置便于查看配置是否正确
    get_config(MT9V03X_COF_UART, GET_CFG);

    disableInterrupts();

    //摄像头采集初始化
    //初始化 数据引脚
    for(i=0; i<8; i++)
    {
        gpio_init((PIN_enum)(MT9V03X_DATA_PIN+i), GPI, 0, PULLUP);
    }

    link_list_num = eru_dma_init(MT9V03X_DMA_CH, GET_PORT_IN_ADDR(MT9V03X_DATA_PIN), camera_buffer_addr, MT9V03X_PCLK_PIN, FALLING, MT9V03X_W*MT9V03X_H);//如果超频到300M 倒数第二个参数请设置为FALLING

    eru_init(MT9V03X_VSYNC_PIN, FALLING);   //初始化场中断，并设置为下降沿触发中断


    eru_disable_interrupt(MT9V03X_PCLK_PIN);
    eru_disable_interrupt(MT9V03X_VSYNC_PIN);
    restoreInterrupts(interrupt_state);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X2摄像头初始化
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:   使用FLEXIO接口采集摄像头
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_init(void)
{
    uint8 i;
    camera_type = CAMERA_GRAYSCALE;//设置连接摄像头类型
    camera_buffer_addr = mt9v03x_image[0];

    boolean interrupt_state = disableInterrupts();

    uart_init (MT9V03X2_COF_UART, 9600, MT9V03X2_COF_UART_TX, MT9V03X2_COF_UART_RX);   //初始换串口 配置摄像头
    enableInterrupts();//开启中断

    //等待摄像头上电初始化成功 方式有两种：延时或者通过获取配置的方式 二选一
    //systick_delay_ms(STM0, 1000);//延时方式
    get_config(MT9V03X2_COF_UART, GET_CFG2);//获取配置的方式

    uart_receive_flag = 0;
    set_config(MT9V03X2_COF_UART, MT9V03X2_CFG);

    //获取配置便于查看配置是否正确
    get_config(MT9V03X2_COF_UART, GET_CFG2);

    disableInterrupts();

    //摄像头采集初始化
    //初始化 数据引脚
    for(i=0; i<8; i++)
    {
        gpio_init((PIN_enum)(MT9V03X2_DATA_PIN+i), GPI, 0, PULLUP);
    }

    link_list_num2 = eru_dma2_init(MT9V03X2_DMA_CH, GET_PORT_IN_ADDR(MT9V03X2_DATA_PIN), camera_buffer_addr, MT9V03X2_PCLK_PIN, FALLING, MT9V03X_W*MT9V03X_H);//如果超频到300M 倒数第二个参数请设置为FALLING

    eru_init(MT9V03X2_VSYNC_PIN, FALLING);   //初始化场中断，并设置为下降沿触发中断

    eru_disable_interrupt(MT9V03X2_PCLK_PIN);
    eru_disable_interrupt(MT9V03X2_VSYNC_PIN);
    restoreInterrupts(interrupt_state);
}

uint8   mt9v03x_finish_flag = 0;    //一场图像采集完成标志位
uint8   mt9v03x_dma_int_num;    //当前DMA中断次数
//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X摄像头场中断
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               此函数在isr.c中被eru（GPIO中断）中断调用
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_vsync(void)
{
    CLEAR_GPIO_FLAG(MT9V03X_VSYNC_PIN);
    mt9v03x_dma_int_num = 0;
    if(!mt9v03x_finish_flag)//查看图像数组是否使用完毕，如果未使用完毕则不开始采集，避免出现访问冲突
    {
        if(1 == link_list_num)
        {
            //没有采用链接传输模式 重新设置目的地址
            DMA_SET_DESTINATION(MT9V03X_DMA_CH, camera_buffer_addr);
        }
        dma_start(MT9V03X_DMA_CH);
    }

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X2摄像头场中断
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               此函数在isr.c中被eru（GPIO中断）中断调用
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_vsync(void)
{
    CLEAR_GPIO_FLAG(MT9V03X2_VSYNC_PIN);
    mt9v03x_dma_int_num = 0;
    if(!mt9v03x_finish_flag)//查看图像数组是否使用完毕，如果未使用完毕则不开始采集，避免出现访问冲突
    {
        if(1 == link_list_num2)
        {
            //没有采用链接传输模式 重新设置目的地址
            DMA_SET_DESTINATION(MT9V03X2_DMA_CH, camera_buffer_addr);
        }
        dma_start(MT9V03X2_DMA_CH);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X摄像头DMA完成中断
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               此函数在isr.c中被dma中断调用
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_dma(void)
{
    static uint8 notfound_times = 0;

    CLEAR_DMA_FLAG(MT9V03X_DMA_CH);
    mt9v03x_dma_int_num++;


    if(mt9v03x_dma_int_num >= link_list_num)        //更新beacon_found    beacon_bias     beacon_area
    {
        //采集完成
        mt9v03x_dma_int_num = 0;
        mt9v03x_finish_flag = 1;//一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
        dma_stop(MT9V03X_DMA_CH);
        //关闭场中断
        eru_disable_interrupt(MT9V03X_VSYNC_PIN);
/*-----------------------------------------------二值化-----------------------------------------------*/
        uint32 time;
        systick_start(STM0);
        mean_filter(MT9V03X_W,MT9V03X_H);
//        GetOSTU(MT9V03X_W,MT9V03X_H);        //5ms

//        lcd_displayimage032(mt9v03x_bin_image[0],MT9V03X_W,MT9V03X_H);    //满屏打印一次54ms
        boolean not_found = 0;


/*-----------------------------------------------提取信标-----------------------------------------------*/
        not_found = extract_beacon(&beacon_area,&beacon_bias,&beacon_len,0);     //如果摄像头图像乱跳 多半是因为数组越界  检查labelnum是否超过了 maxNum-1  6ms
                                                                                 //还有可能是打印屏幕出错




/*-----------------------------------------------策略与调试-----------------------------------------------*/
        //未找到信标灯返回  并且不改变  beacon_area  beacon_bias   beacon_len

        if(!not_found)
        {
            notfound_times++;

            if(notfound_times>2)
            {
                beacon_found = 0;      //信标灯丢失
                mode_dis = 0;
                judgecounter = 0;
            }
        }
        else      //找到信标灯
        {
            notfound_times = 0;
            beacon_found = 1;
            slowdown = 0;
//        lcd_showuint16(0,0,beacon_area);        //以下两行不能同时打印，否则摄像头乱跳
//        lcd_showint16(0,0,beacon_bias);        //一定要在看到信标灯才能后打印
//        systick_delay_ms(STM0,10);             //打印int16需要延时 否则打印的数值不稳定
//        lcd_showuint8(0,2,beacon_len);          //打印函数特别容易出问题
        beacon_dis = estimate_distance(beacon_len);
//        lcd_showuint16(0,3,beacon_dis);
        }
//        lcd_showstr(0,1,"f");
        time = systick_getval(STM0);
//        lcd_showint32(0,TFT_Y_MAX/16-1,time,8);



/*-----------------------------------------------摄像头切换-----------------------------------------------*/

        //关闭摄像头1
        mt9v03x_finish_flag = 0;
#if 1
        if(beacon_found == 0)       //未找到信标灯则切换摄像头
        {
            camera_change = 1;
            eru_disable_interrupt(MT9V03X_PCLK_PIN);
            eru_enable_interrupt(MT9V03X2_VSYNC_PIN);
            eru_enable_interrupt(MT9V03X2_PCLK_PIN);
        }
        else                       //否则沿用当前摄像头
        {
            camera_change = 0;
            eru_enable_interrupt(MT9V03X_VSYNC_PIN);
        }
#else
        eru_enable_interrupt(MT9V03X_VSYNC_PIN);    //测试单摄像头时打开 并注释上边if else
#endif
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      MT9V03X2摄像头DMA完成中断
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               此函数在isr.c中被dma中断调用
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x2_dma(void)
{
    static uint8 notfound_times = 0;

    CLEAR_DMA_FLAG(MT9V03X2_DMA_CH);
    mt9v03x_dma_int_num++;


    if(mt9v03x_dma_int_num >= link_list_num2)        //更新beacon_found    beacon_bias     beacon_area
    {
        //采集完成
        mt9v03x_dma_int_num = 0;
        mt9v03x_finish_flag = 1;//一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
        dma_stop(MT9V03X2_DMA_CH);
        //关闭场中断
        eru_disable_interrupt(MT9V03X2_VSYNC_PIN);
/*-----------------------------------------------二值化-----------------------------------------------*/
        uint32 time;
        systick_start(STM0);
        mean_filter(MT9V03X_W,MT9V03X_H);
//        GetOSTU(MT9V03X_W,MT9V03X_H);        //5ms
//        lcd_displayimage032(mt9v03x_bin_image[0],MT9V03X_W,MT9V03X_H);    //满屏打印一次54ms
        boolean not_found = 0;



/*-----------------------------------------------提取信标-----------------------------------------------*/
        not_found = extract_beacon(&beacon_area,&beacon_bias,&beacon_len,0);     //如果摄像头图像乱跳 多半是因为数组越界  检查labelnum是否超过了 maxNum-1  6ms
                                                                                 //还有可能是打印屏幕出错


/*-----------------------------------------------策略与调试-----------------------------------------------*/
        //未找到信标灯返回  并且不改变  beacon_area  beacon_bias   beacon_len
        if(!not_found)
        {
            notfound_times++;
            if(notfound_times>2)
            {
                beacon_found = 0;      //信标灯丢
                mode_dis = 0;
                judgecounter = 0;
            }
        }
        else  //找到信标灯
        {
            notfound_times = 0;
            beacon_found = 1;
            slowdown = 0;

//        lcd_showuint16(0,0,beacon_area);        //以下两行不能同时打印，否则摄像头乱跳
//        lcd_showint16(0,0,beacon_bias);        //一定要在看到信标灯才能后打印
//        systick_delay_ms(STM0,10);             //打印int16需要延时 否则打印的数值不稳定
//        lcd_showuint8(0,0,beacon_len);          //打印函数特别容易出问题
          beacon_dis = estimate_distance(beacon_len);
//          lcd_showuint16(0,3,beacon_dis);
        }

//        lcd_showstr(0,1,"s");
        time = systick_getval(STM0);
//        lcd_showint32(0,TFT_Y_MAX/16-1,time,8);



/*-----------------------------------------------摄像头切换-----------------------------------------------*/
        //关闭摄像头2
        mt9v03x_finish_flag = 0;

#if 1
        if(beacon_found == 0)       //未找到信标灯则切换摄像头
        {
            camera_change = 0;
            eru_disable_interrupt(MT9V03X2_PCLK_PIN);
            eru_enable_interrupt(MT9V03X_VSYNC_PIN);
            eru_enable_interrupt(MT9V03X_PCLK_PIN);
        }
        else                        //否则沿用当前摄像头
        {
            camera_change = 1;
            eru_enable_interrupt(MT9V03X2_VSYNC_PIN);
        }
#else
        eru_enable_interrupt(MT9V03X2_VSYNC_PIN);    //测试单摄像头时打开 并注释上边if else
#endif
    }
}





/*******大津法
 *
 * 返回值： 二值化阈值：0~255
 *
 */
uint16 GetOSTU (uint16 width,uint16 height)    //二维数组作为参数是否可行？？？
{
    sint16 i, j;
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelshortegralBack = 0;
    uint32 Pixelshortegral = 0;
    sint32 PixelshortegralFore = 0;
    sint32 PixelFore = 0;
    float32 OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    sint16 MinValue, MaxValue;              //最大灰度和最小灰度
    uint16 Threshold = 0;
    uint8 HistoGram[256];               //灰度直方图

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
        {
            HistoGram[mt9v03x_image[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }

    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            if (mt9v03x_image[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
                mt9v03x_bin_image[i][j] = 255;          //255而不是1,TFT显示彩屏
            else
                mt9v03x_bin_image[i][j] = 0;
        }
    }

      return Threshold;                        //返回最佳阈值;
}


/***************平均值滤波
 *
 *
 * 地址传参：
 *
 *
 */
uint16 mean_filter(uint16 width,uint16 height)
{
    uint16 Threshold = 0;
    uint32 tv = 0;
    uint8 i,j;

    //累加
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            tv += mt9v03x_image[i][j];   //累加
        }
    }
    Threshold =(unsigned short)(tv / height / width);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
    Threshold = Threshold + 30;      //此处阈值设置，根据环境的光线来设定


    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            if (mt9v03x_image[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
                mt9v03x_bin_image[i][j] = 255;          //255而不是1,TFT显示彩屏
            else
                mt9v03x_bin_image[i][j] = 0;
        }
    }

      return Threshold;                        //返回最佳阈值;
}


/*!
 * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
 *
 * @param    imageIn    输入数组
 *           imageOut   输出数组      保存的二值化后的边沿信息
 *
 * @return
 *
 * @note
 *
 * @example
 */

void sobelAutoThreshold(uint8  imageIn[MT9V03X_H][MT9V03X_W], uint8  imageOut[MT9V03X_H][MT9V03X_W])
{
    /** 卷积核大小 */
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
            /* 计算不同方向梯度幅值  */
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

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
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



/*-------------------------------------------------最大连通域与并查集-----------------------------------------------*/




//-------------------------------------------------------------------------------------------------------------------
//  @brief      并查集找根
//  @return     标签号码
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
//  @brief      并查集合并
//
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline void merge(uint16 x,uint16 y)   //x y  所在并查集合并
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
//  @brief      并查集清零
//  @param      Parent[]前n + 1位置零
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline void union_find_clear(uint16 n)
{
    if(n==0xffff)n--;
    for(uint16 i=0;i<=n;i++)
        Parent[i]=0;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      并查集插入操作
//
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
inline void insert(uint16 left,uint16 up)      //up --> left
{
    if(left==up)return;         //同标签不操作
    if(Parent[left]!=0)
    {
        uint16 i=left;
        uint16 j=up;
        while(Parent[i]!=0)
            i=Parent[i];
        while(Parent[j]!=0)
            j=Parent[j];
        if(i==j)return ;        //同根则不操作

        Parent[j]=i;

    }
    else
        Parent[up]=left;        //不能Parent[left] = up
                                //可能因为会形成循环
}






/*********二值后图像提取信标面积和中心***********Two-Pass算法
 *
 *
   *  地址传参：信标面积     信标中心
   * 找到信标灯返回true
 *
 *
 *
 */
boolean  extract_beacon(uint16 *area,sint16 *bias,uint8 *len,boolean PRINT)
{
    uint16 labelnum=0;              //其取值范围 0x0000 ~ 0xfffe  其他无效  实际最大值应该是maxNum - 1
    uint8 i,j;        //i为列   j为行
    uint16 k;
    uint16 whitepixelnum = 0;

    //第一步   构建标签数组
    for(j=1;j<MT9V03X_H-5;j++)           //从上到下 从左到右 范围分别是  1~155  10~178
        for(i=10;i<MT9V03X_W-10;i++)
            if(mt9v03x_bin_image[j][i]==255)
            {
                whitepixelnum++;
                uint8 up = (label[j-1][i]==0)?0:1;      //上一个像素点 二值化
                uint16 up_label = label[j-1][i];
                uint8 left = (label[j][i-1]==0)?0:1;    //左一个像素点
                uint16 left_label = label[j][i-1];
                                                                    //根据左和上的标签有无 可以分成四种情况
                if(up==0&&left==0)
                {
                    labelnum++;
                    if(labelnum >= maxNum)return FALSE;             //标签过多则提前结束
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
                else                                //左上都有标记则取其小
                {
                    insert(left_label,up_label);    //更新 并查集 up-->left
                    if(up_label>left_label)
                    {
                        label[j][i]=left_label;
                        labelarea[left_label]++;
                        labelright[left_label]=i;
                    }
                    else                            //包括小于和等于两种情况
                    {
                        label[j][i]=up_label;
                        labelarea[up_label]++;
                        labeldown[up_label]=j;
                    }
                }
            }

    //像素过少则退出
    if(whitepixelnum < 5)return FALSE;

    //第二步   合并标签数组
    for(k=1;k<=labelnum&&k!=0xffff;k++)  //防止labelnum == 0xffff
    {
        if(Parent[k]!=0)        //不是根节点
        {
            uint16 p=0;
            p=find(k);          //寻找根节点
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



//    第三步   寻找面积最大连通域 或 形状匹配
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
//        if(labelarea[k]!=0&&(labelright[k]<(MT9V03X_W-10))&&(labelleft[k]>10)&&(labelup[k]>5))     //父亲节点 且 不在捕获区
//    if(labelarea[k]!=0)
//        {
//            validlabel = 1;
//            float temp_diff;
//            temp_diff = (float)(labelright[k] - labelleft[k])*(labeldown[k] - labelup[k])*Pi/4.0;
//            temp_diff = (float)labelarea[k]/temp_diff-1;
//            if(temp_diff<diff){         //形状面积匹配
//                diff = temp_diff;
//                Smax_label = k;
//            }
//        }
//    }
//    if(!validlabel) return FALSE;   //线性区无信标

    uint16 mid;
    mid = ((uint16)labelleft[Smax_label] + (uint16)labelright[Smax_label])>>1;
    *bias = (sint16)(mid - MT9V03X_W/2);
    *len = labelright[Smax_label] - labelleft[Smax_label];
    *area = labelarea[Smax_label];


    if(PRINT)
    {
        for(j=labelup[Smax_label];j<labeldown[Smax_label];j++)lcd_drawpoint(mid*TFT_X_MAX/MT9V03X_W,j,RED);         //可视化描线    竖直线
    }
    //并查集清零
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

    for( k=0;k<=labelnum;k++)    //注意uint16最大值
        Parent[k]=0;


    return TRUE;
}






/*********二值后图像提取信标面积和中心***中心扫描法
 *
 *
 *地址传参：信标面积     信标中心
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


    if(flag1 == 0)  //图像不在中间
    {
        for(i=MT9V03X_W/2 - 1;i>=0;i--)     //逐列扫描，从中间往左
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
                            right = (i>right?i:right);  //更新右界
                            break;  //下一列
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
                            left = (i<left)?i:left;     //更新左界
                            break;  //下一列
                        }
                    }

                }
            }


        if(!flag2)
        {
          for(i=MT9V03X_W/2 - 1;i<MT9V03X_W;i++)     //逐列扫描，从中间往右
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
                            left = (i<left)?i:left;     //更新左界
                            break;  //下一列
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
                            right = (i>right?i:right);  //更新右界
                            break;  //下一列
                            }
                    }
                }
            }
        }
        if(flag2)lcd_showchar(0,0,'L');
        else lcd_showchar(0,0,'R');
    }
    // 中间有图像
    else
    {
        for(i=MT9V03X_W/2;i>=0;i--)
        {
            uint8 flag3=0;
            for(j=0;j<MT9V03X_H/2;j++)
                if(mt9v03x_bin_image[j][i]==255){left=i;flag3=1;break;}
            if(!flag3)break;        //超出左边界
        }
        for(i=MT9V03X_W/2;i<MT9V03X_W;i++)
        {
            uint8 flag3=0;
            for(j=0;j<MT9V03X_H/2;j++)
                if(mt9v03x_bin_image[j][i]==255){right=i;flag3=1;break;}
            if(!flag3)break;        //超出右边界
        }
        lcd_showchar(0,0,'M');
    }


    //计算中心和面积
    if(left<right)*mid = ((uint16)left + (uint16)right)/2;
    else *mid = 0;                  //出错
    for(i=left;i<=right;i++)
        for(j=0;j<MT9V03X_H/2;j++)
            if(mt9v03x_bin_image[j][i]==255)S++;

    *area = S;
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief  前后图像异或寻找信标灯----------更新beacon_found    beacon_bias     beacon_area
//  @return     void
//  @note
//-------------------------------------------------------------------------------------------------------------------

void beacon_xor(uint16 width,uint16 height,uint16 *area,uint16 *mid)        //应用思路不对，此函数仅用于闪烁识别
{


}

//-------------------------------------------------------------------------------------------------------------------
//  @brief  估算距离
//  @param  beacon_area目标像素数
//  @return     距离cm
//  @note
//摄像头显示的目标宽度和距离成反比
//实测数据：beacon_len/beacon_dis cm
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
    dis = 0.6*dis + 0.4*last_dis;//低通滤波
    last_dis = dis;
    return dis;
}
//函数功能： 将 mt9v03x_bin_image 中值滤波
//返回值：  void
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
                //冒泡排序
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
