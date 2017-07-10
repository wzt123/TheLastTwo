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
  uint8 time1=0;
  sum_time = 0;
  char nrf_data=0;
  uint8 Edge_R[3]= {0};
  uint8 Edge_L[3]= {0};
  
  //uint32 time2 =  0;
  DisableInterrupts;
  NVIC_SetPriority(PORTA_IRQn,1);
  NVIC_SetPriority(DMA0_IRQn,2);
  
  Init_All();
  DELAY_MS(1000);
  
  EnableInterrupts;
  
  set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);  
  set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);    
  int a=nrf_link_check();
  
  //uint8 Chaoche_stop_time=0;
  uint8 Chaoche_start_time=0;
  while(a)
  {
    pit_time_start(PIT1);
    
    camera_get_img();                                   //摄像头获取图像
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //二值化图像
    Search_Line();
    if(Car==2)
    {
      nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里      
      
//      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==1)
//      {
//        Car=1;
//        ABDistance=0;
//        ABDistance_last=0;
//        gpio_set(PTE25,1);//后车开启超声波
//        gpio_set(PTE24,1);
//      }
      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==2)
        ChaoChe_Cross_temp=1;
      
      if(ChaoChe_Cross_temp==1)
      {
        Cross_Flag=1;
      }
      
    }
    
    if(Cross_Cnt==4)
    {
      if(Car==2&&ChaoChe_Cross_temp==1)
      {
        Car=1;
        ABDistance=0;
        ABDistance_last=0;
        gpio_set(PTE25,1);//开启超声波
        gpio_set(PTE24,1);
        NRF_SendData(20001);//告诉停着的车，超车成功，换标志位
        ChaoChe_Cross_temp=0;
      }
      
    }
    
    if(Car==1)
    {
      nrf_rx(buff,4);               //等待接收一个数据包，数据存储在buff里     
      if(buff[3]==0&&buff[2]==0&&buff[1]==0&&buff[0]==1)
      {
        Car=2;
        gpio_set(PTE25,0);//停着的车关闭超声波
        gpio_set(PTE24,0);
      }
    }
    
    Find_Middle();
    Road_Type();
    Servo_control();
    
    speed_get_L = abs(ftm_quad_get(FTM1));          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    speed_get_R = lptmr_pulse_get();
    ftm_quad_clean(FTM1);
    lptmr_pulse_clean();
        
    if(stop_Flag !=1&&Car_First_stop==0&&Car_Second_stop==0)//超车的时候电机不输出
    {  
      if(Car==1)
        Motor_Out();
      else if(Car==2)
      {
        if(ABDistance>1000||ABDistance<=10)
        {
          Motor_Out();
        }
        else if(Distance_stop_temp==0&&ABDistance<=1000&&ABDistance>10)
        {
          Distance_stop();
        }
      }
    }    
    
    if((Stop_Flag>1)&&Car==1&&Car_First_stop<2&&stop_Flag==0)
      stop_Car1();
    else if((Stop_Flag>1)&&Car==2&&stopLine_temp==0&&Car_Second_stop==0&&stop_Flag==0)
      stop_Car2();
    
    if(Cross_Flag==1&&(Left_stop>22||Right_stop>22)&&Car==1)
    {
      Chaoche_FrontCar();
    }
    
    
    ///蓝牙传送编码器的值
    send_data[0] = 0;
    send_data[1] = Cross_Flag*500;
    send_data[2] = 0;
    //vcan_sendware((uint16_t *)send_data, sizeof(send_data));
    if(speed_get_R<60&&speed_get_L<60)
    {
      dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 
      OLED_Print_Num1(88, 1, nrf_data);
      OLED_Print_Num1(88, 2, error);
      OLED_Print_Num1(88, 3, errorerror);
      OLED_Print_Num1(88, 4, Kp);
      OLED_Print_Num1(88, 5, Servo_value);
      time1 = pit_time_get(PIT1)*1000/(bus_clk_khz*1000);
      //wzt_bluetooth();     
      OLED_Print_Num1(88, 6, Servo_Value_Last);
    }
    if(Stop_Flag==1&&speed_get_R!=0&&speed_get_L!=0)
    {
      sum_time++; 
    }
    pit_close(PIT1);
    nrf_data = race[1];
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
/*
直道超车
*/
void zhidaochaoche(){
  /*if(Car==1)
    {
      if(sum_time>5&&sum_time<100&&stopLine_temp==0&&Chaoche_start_time==0)
      {
        if(Cross_Flag!=5&&Cross_Flag!=6&&speed_get_R>30&&speed_get_L>30)
        {
          Chaoche_stop();
          ChaoChe_stop_time++;
          //stop_Flag=0;
          //Chaoche_start();
        }
      }
    }
    else if(Car==2)
    {
      if(sum_time>15&&sum_time<100&&stopLine_temp==0&&Chaoche_start_time==0)
      {
        if(Cross_Flag!=5&&Cross_Flag!=6&&speed_get_R>30&&speed_get_L>30)
        {
          Chaoche_stop();
          ChaoChe_stop_time++;
          //stop_Flag=0;          
        }
      }
    }
    
    if(ChaoChe_stop_time>0&&ChaoChe_stop_time<50)
      ChaoChe_stop_time++;
    else if(ChaoChe_stop_time>=50)
    {
      ChaoChe_stop_time=0;
      stop_Flag=0;
      Chaoche_start();
      Chaoche_start_time++;
    }
    
    if(Chaoche_start_time>0&&Chaoche_start_time<20)
      Chaoche_start_time++;
    else if(Chaoche_start_time>=20)    {
      Chaoche_start_time=0;
    }
    if(sum_time>0&&sum_time<100)
    {
      ChaoChe_temp=1;
      if(Car==1)
      {
        Servomiddle=8770;
      }
      else if(Car==2)
        Servomiddle=8850;
    }
    else
    {
      ChaoChe_temp=0;
    }*/
}

/*!
*  @brief      DMA0中断服务函数
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
}

