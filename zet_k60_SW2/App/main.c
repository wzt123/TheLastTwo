/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "handle.h"
uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 stopline_num = 0;                         ////第一次检测到起跑线不算，第二次才是停车
//uint8 img[CAMERA_W*CAMERA_H];

//函数声明
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void zet_motor(void);
void steer(void);
void zet_oled();
void zet_camera();
//void PIT0_IRQHandler(void);
void zf_oled(int16 val);
void PIT1_IRQHandler();
void wzt_bluetooth(void);
uint16 speed_rember_R[3] = {0};
uint16 speed_rember_L[3] = {0};
//char * int_to_char(int a);
//void img_extract(uint8 *dst, uint8 *src, uint32 srclen);

/*!
 *  @brief      main函数
 *  @since      v5.3
 *  @note       山外摄像头 LCD 测试实验
   
   */
   
  
void  main(void)
{
     //zet_bluetooth();
     uint16 send_data[3] = {0};
     int time1=0;
     char nrf_data=0;
      uint8 Edge_R[3]= {0};
     uint8 Edge_L[3]= {0};
     //uint32 time2 = 0;
     DisableInterrupts;
     NVIC_SetPriority(PORTA_IRQn,1);
     NVIC_SetPriority(DMA0_IRQn,2);
     
     Init_All();
     DELAY_MS(1000);
     
     EnableInterrupts;
     
     //set_vector_handler(PORTC_PORTD_VECTORn ,PORTC_PORTD_IRQHandler);
     
     set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);  
     set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);    
     //set_vector_handler(PIT1_VECTORn , PIT1_IRQHandler);    
    // enable_irq (PIT1_IRQn);
     
     //设置 PORTE 的中断服务函数为 PORTE_VECTORn
     //enable_irq(PORTC_PORTD_IRQn);
     int a=nrf_link_check();
   while(1)
    {
        pit_time_start(PIT1);
        camera_get_img();                                   //摄像头获取图像
        img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
        Search_Line();
        Find_Middle();
        
        Servo_control();
        
        
       speed_get_L = abs(ftm_quad_get(FTM1));          //获取FTM 正交解码 的脉冲数(负数表示反方向)
       if(Edge_L[0]!=0){
         if(abs(speed_get_L-speed_rember_L[2])<20)
         {
           speed_rember_L[0] = speed_rember_L[1];
           speed_rember_L[1] = speed_rember_L[2];
           speed_rember_L[2] = speed_get_L;
           
           Edge_L[0]=speed_rember_L[0];
           Edge_L[1]=speed_rember_L[1];
           Edge_L[2]=speed_rember_L[2];          
           speed_get_L=GetMedianNum(Edge_L,3);////左编码器滤波
         }
         else
         {
           speed_get_L = speed_rember_L[2];
         }
           
       }
       speed_get_R = lptmr_pulse_get();
       if(Edge_R[0]!=0)
       {
         if(abs(speed_get_R-speed_rember_R[2])<20)
         {
           speed_rember_R[0] = speed_rember_R[1];
           speed_rember_R[1] = speed_rember_R[2];
           speed_rember_R[2] = speed_get_R;
           
           Edge_R[0]=speed_rember_R[0];
           Edge_R[1]=speed_rember_R[1];
           Edge_R[2]=speed_rember_R[2];          
           speed_get_R=GetMedianNum(Edge_R,3);////右编码器滤波          
         }
         else
         {
           speed_get_R = speed_rember_R[2];
         }
           
       }
        ftm_quad_clean(FTM1);
        lptmr_pulse_clean();
        /*if(Cross_Flag==3&&stop_Flag !=1&&Ring_First_Row<=20&&Car==1)
        {
          stop_Car();
        
        }*/
        /*if(gpio_get(PTE10)&&gpio_get(PTE9)&&stop_Flag!=1)             //PTC8，PTC9触发中断
        {          
          if(stopline_num>0)
            stop_Car();
          else
            stopline_num++;
        }*/
        if(stop_Flag !=1)
        {  
          Motor_Out();
        }
        if(stop_Flag==1)
        {
          race[3]=1;///告诉后车遇到圆环且停好车了，准备超车，
        }
        ///蓝牙传送编码器的值
        send_data[0] = speed_get_L;
        send_data[1] = speed_get_R;
        //send_data[2] = Cross_Flag*500;
        vcan_sendware((uint8_t *)send_data, sizeof(send_data));
        
        nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里
        nrf_data = buff[0];
        ////////////////后车检测到超声波信号，发来一个1，表明超车成功
        /*if(buff[1]==1)
        {
          Car=1;
          stop_Flag=0;
        }*/
        
        Overtake_judge();
        dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 
        OLED_Print_Num1(88, 1, All_Black);
        OLED_Print_Num1(88, 2, error);
       // OLED_Print_Num1(88, 3, errorerror);
        OLED_Print_Num1(88, 3, Cross_Flag);
        OLED_Print_Num1(88, 4, speed_get_L);
        OLED_Print_Num1(88, 5, speed_get_R);
        //OLED_Print_Num1(88, 4, gpio_get(PTE10));
        //OLED_Print_Num1(88, 5, gpio_get(PTE9));
        //wzt_bluetooth(); 
        time1 = pit_time_get(PIT1)*100/(9*1024*1024);
        pit_close(PIT1);
        
        OLED_Print_Num1(88, 6, time1);
        
        //OLED_Print_Num1(88, 6, nrf_data);
    }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 25;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

