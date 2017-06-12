/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,ɽ��Ƽ�
*     All rights reserved.
*     �������ۣ�ɽ����̳ http://www.vcan123.com
*
*     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
*     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
*
* @file       main.c
* @brief      ɽ��K60 ƽ̨������
* @author     ɽ��Ƽ�
* @version    v5.0
* @date       2013-08-28
*/

#include "common.h"
#include "include.h"
#include "handle.h"
uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
uint8 stopline_num = 0;                         ////��һ�μ�⵽�����߲��㣬�ڶ��β���ͣ��
//uint8 img[CAMERA_W*CAMERA_H];

//��������
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
*  @brief      main����
*  @since      v5.3
*  @note       ɽ������ͷ LCD ����ʵ��

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
  uint8 IR1_last = 0;
  uint8 IR2_last = 0;
  
  //uint32 time2 = 0;
  DisableInterrupts;
  NVIC_SetPriority(PORTA_IRQn,1);
  NVIC_SetPriority(DMA0_IRQn,2);
  
  Init_All();
  DELAY_MS(1000);
  
  EnableInterrupts;
  
  set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);  
  set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);    
  int a=nrf_link_check();
  uint8 IR1=0,IR2=0;
  while(a)
  {
    pit_time_start(PIT1);
    camera_get_img();                                   //����ͷ��ȡͼ��
    img_extract((uint8*)img,imgbuff,CAMERA_SIZE);           //��ֵ��ͼ��
    Search_Line();
    Find_Middle();
    
    Servo_control();
    IR1_last = IR1;
    IR1 = gpio_get(PTE10);///��һ�ߵĺ���Թ�
    
    speed_get_L = abs(ftm_quad_get(FTM1));          //��ȡFTM �������� ��������(������ʾ������)
    if(Edge_L[0]!=0){
      if(abs(speed_get_L-speed_rember_L[2])<20)
      {
        speed_rember_L[0] = speed_rember_L[1];
        speed_rember_L[1] = speed_rember_L[2];
        speed_rember_L[2] = speed_get_L;
        
        Edge_L[0]=speed_rember_L[0];
        Edge_L[1]=speed_rember_L[1];
        Edge_L[2]=speed_rember_L[2];          
        speed_get_L=GetMedianNum(Edge_L,3);////��������˲�
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
        speed_get_R=GetMedianNum(Edge_R,3);////�ұ������˲�          
      }
      else
      {
        speed_get_R = speed_rember_R[2];
      }
      
    }
    ftm_quad_clean(FTM1);
    lptmr_pulse_clean();
    IR2_last = IR2;
    IR2 = gpio_get(PTE9);///��һ���ٶ���һ�ߵĺ���Թ�
        
    if(stop_Flag !=1&&ChaoChe_stop!=1)//������ʱ���������
    {  
      Motor_Out();
    }    
    
    if(stop_Flag==1)
    {
      race[3]=1;///���ߺ�����Բ����ͣ�ó��ˣ�׼��������
    }
    
    if(Stop_Flag==2&&stop_Flag!=1)
    {      
      stop_Car();
    }
     
    /*if(Stop_Flag==1&&sum_time>2000)
    {
      if(Car==1&&Cross_Flag!=Cross_Flag_Last&&Cross_Flag_Last==3)
        ChaoChe_stop=1;
    }
      if(ChaoChe_stop==1)
      {
        ChaoChe_stop_time=0;
        Chaoche_stop();
      }
    */
    ///�������ͱ�������ֵ
    send_data[0] = speed_get_L;
    send_data[1] = speed_get_R;
    //send_data[2] = Cross_Flag*500;
    //vcan_sendware((uint8_t *)send_data, sizeof(send_data));
    
    nrf_rx(buff,4);               //�ȴ�����һ�����ݰ������ݴ洢��buff��
    nrf_data = buff[0];
    ////////////////�󳵼�⵽�������źţ�����һ��1�����������ɹ�
    /*if(buff[1]==1)
    {
    Car=1;
    stop_Flag=0;
  }*/
    
    Overtake_judge();
    dis_bmp(CAMERA_H,CAMERA_W,(uint8*)img,0x7F); 

    OLED_Print_Num1(88, 1, Cross_Cnt);
    OLED_Print_Num1(88, 2, error);
    OLED_Print_Num1(88, 3, errorerror);
    OLED_Print_Num1(88, 4, ABDistance);
    OLED_Print_Num1(88, 5, Ring_First_Row);

    //wzt_bluetooth(); 
    
    
    time1 = pit_time_get(PIT1)*100/(9*1024*1024);
    if(Stop_Flag==1)
    {
      sum_time +=time1; 
    }    
    pit_close(PIT1);
    
    OLED_Print_Num1(88, 6, Cross_Flag);
    
    //OLED_Print_Num1(88, 6, nrf_data);
  }
}

/*!
*  @brief      PORTA�жϷ�����
*  @since      v5.0
*/
void PORTA_IRQHandler()
{
  uint8  n;    //���ź�
  uint32 flag;
  
  while(!PORTA_ISFR);
  flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                   //���жϱ�־λ
  
  n = 25;                                             //���ж�
  if(flag & (1 << n))                                 //PTA29�����ж�
  {
    camera_vsync();
  }
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
  n = 28;
  if(flag & (1 << n))                                 //PTA28�����ж�
  {
    camera_href();
  }
#endif
  
  
}

/*!
*  @brief      DMA0�жϷ�����
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
}

