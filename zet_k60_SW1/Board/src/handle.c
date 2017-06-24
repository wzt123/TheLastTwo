#include "common.h"
#include "handle.h"
#include "include.h"
#include "function.h"

uint8 img[CAMERA_H][CAMERA_W];
int8 Road_Left[ROW_MAX]= {0}; //左线位置_列标
int8 Road_Right[ROW_MAX]= {0}; //右线位置_列标
int8 Road_Center[ROW_MAX]= {0}; //中心位置_列标
int8 Road_Width[ROW_MAX]= {0}; //每行赛道的宽度
uint16 Road_area;
uint8 Road_Change=0;
uint8 Road_type=0;

uint8 Left_Flag[ROW_MAX]= {0}; //本行左边线找到标志
uint8 Right_Flag[ROW_MAX]= {0}; //本行右边线找到标志

uint8 Left_Cnt=0;//左线计数 所有的
uint8 Right_Cnt=0;//右线计数 所有的
uint8 Right_cnt=0;//连续的左边线计数
uint8 Left_cnt=0;//连续的右边线计数

uint8 All_Black=0;//全黑行位置_行标
uint8 All_White=0;//全白行位置_行标
uint8 Black_Cnt=0;//全黑行计数
uint8 Black_cnt=0;//lianxu
uint8 White_cnt=0;//全白行计数_连续的
uint8 White_Cnt=0;//全白行计数_所有的
uint8 White_Ren=0;

uint8 Right_xian=0;
uint8 Left_xian=0;

uint16 Servo_value=8808;//舵机输出pwm值


uint8 Hinder_Start=0;
uint8 Hinder_Flag=0;

uint8 Cross_Flag=0;
uint8 Change_Flag;
uint8 CrossRow=0;

uint16 Servomiddle=8808;
uint32 Servo_max=8970;
uint32 Servo_min=8660;
float CenterLineSlope=0;

int16 error=0;
int32 error1=0;
int32 error2=0;
int16 errorerror=0;
uint8 Flag_L=0;
uint8 Flag_R=0;

uint8 Stop_Flag=0;
uint8 StopRow=0;
//uint8 a=0;
//uint8 l=0;
uint8 Lastline=0;

uint8 Left_sign=0;
uint8 Right_sign=0;
uint8 FirstBlackinCenter=0;
uint8 Overtake=0;
uint8 Car=0;
uint8 race[4]= {0}; //传输数据数组
uint8 buff[4]= {0}; //定义接收缓冲区buff[32]
uint8 Overtake_Flag=0;
int16 Servo_temp=0;

uint8 Left_addwidth[ROW_MAX]= {0}; //本行左边加宽区标志
uint8 Right_addwidth[ROW_MAX]= {0}; //本行右边加宽区标志
uint8 Left_c_in=0;
uint8 Left_c_out=0;
uint8 Right_c_in=0;
uint8 Right_c_out=0;
uint8 Left_r_in=0;
uint8 Left_r_out=0;
uint8 Right_r_in=0;
uint8 Right_r_out=0;
uint8 Overtake2=0;
uint16 L_Cnt=0;
uint8 C=0;
uint8 Turn_Left=0;

//////////////////////////
uint8 Kp=0;
uint8 Kd=0;
uint8 Row_Ptr=0;
uint8 Col_Ptr=0;
uint8 Bend_Right = 0;
uint8 Bend_Lift = 0;
float repair_slope_R = 0;/////右边线补线斜率
float repair_slope_L = 0;/////左边线补线斜率
uint8 Ring_First_Row = 0;
uint8 Ring_Second_Row = 0;
uint8 Ring_width = 0;
uint8 Ring_width_1 = 0;
uint8 Ring_width_2 = 0;
uint8 Cross_Flag_Last=0;
uint8 start_line_num[60] = {0};
uint8 stop_line_num = 0;
//uint8 Ring_Flag=0;
uint8 ring_time = 0;
uint8 cross_time = 0;
uint8 cross_Time = 0;
uint32 sum_time = 0;
uint8 Cross_Cnt=0;
uint8 Cross3_Cnt=0;
uint8 Out_Left=0;
uint8 Out_Right=0;

uint8 cross_num = 0;
uint8 white_Left_cnt = 0;
uint8 white_Right_cnt = 0;
uint8 weight_num_Cross [60]=
{
  10,10,10,10,10,
  10,10,10,10,10,
  10,10,10,10,10,
  100,100,100,100,100,
  100,100,100,100,100,
  100,100,100,100,100,
  
  100,100,100,100,100,
  100,100,100,100,100,
  100,100,100,100,100,
  100,100,100,100,100,
  100,100,100,100,100,
  100,100,100,100,100,
  
};
///////////////////////////

void Servo_PID();
//uint8 img[60][80];
void Cross_Search(void);
void filter_Middle(uint8 *Array);
//
void Ring();

////计算斜率
float Slope_Calculate(uint8 begin,uint8 end,uint8 *p)
{
  int xsum=0,ysum=0,xysum=0,x2sum=0;
  uint8 i=0;
  float result=0;
  static float resultlast;
  p=p+begin;
  for(i=begin; i<end; i++)
  {
    xsum+=i;
    ysum+=*p;
    xysum+=i*(*p);
    x2sum+=i*i;
    p=p+1;
  }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零
  {
    result=(float)((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
    result=resultlast;
  }
  return result;
  
}


//补线
void Calculate_Slope()
{
  uint8 i=0,j=0;
  uint8 Cross_Flag_Last=0;
  uint8 Left_stop=20;
  uint8 Left_start=59;
  uint8 Right_stop=20;  
  uint8 Right_start=59;
  float Left_Slope=0.0;
  float Right_Slope=0.0;
  for(Row_Ptr=59;Row_Ptr>All_White-2;Row_Ptr--)
  {
    if(Left_Flag[Row_Ptr]==1&&Left_Flag[Row_Ptr-1]==3&&Left_Flag[Row_Ptr-2]==3)
      Left_start=Row_Ptr+4;
    if(Right_Flag[Row_Ptr]==1&&Right_Flag[Row_Ptr-1]==3&&Right_Flag[Row_Ptr-2]==3)
      Right_start=Row_Ptr+4;
  }
  if(Left_start>59) Left_start=59;
  if(Right_start>59) Right_start=59;
  for(i=All_White-White_Cnt+2;i>10;i--)
  {
    for(j=Road_Left[Left_start]+40;j>Road_Left[Left_start];j--)
    {
      if(img[i][j]==0&&img[i][j+1]==0&&img[i][j+2]==255&&img[i][j+3]==255)
      {Road_Left[i]=j+1;
      
      break;}
    }
    for(j=Road_Right[Right_start]-40;j<Road_Right[Right_start];j++)
    {
      if(img[i][j-3]==255&&img[i][j-2]==255&&img[i][j-1]==0&&img[i][j]==0)
      {Road_Right[i]=j-1;
      
      break;}
    }
  }
  for(Row_Ptr=All_White-White_Cnt+2;Row_Ptr>=StopRow&&Row_Ptr>20;Row_Ptr--)
  {
    if(Left_Flag[Row_Ptr]==1&&Left_Flag[Row_Ptr+1]==3&&Left_Flag[Row_Ptr+2]==3)
      Left_stop=Row_Ptr-4;
    if(Right_Flag[Row_Ptr]==1&&Right_Flag[Row_Ptr+1]==3&&Right_Flag[Row_Ptr+2]==3)
      Right_stop=Row_Ptr-4;
  }
  if(Left_stop<20) Left_stop=20;
  if(Right_stop<20) Right_stop=20;
  Left_Slope=1.0*(Road_Left[Left_stop]-Road_Left[Left_start])/(Left_start-Left_stop);
  Right_Slope=1.0*(Road_Right[Right_start]-Road_Right[Right_stop])/(Right_start-Right_stop);
  j=0;
  for(i=Left_start-1;i>=Left_stop;i--)
  {
    j++;
    Road_Left[i]=(uint8)(Road_Left[Left_start]+Left_Slope*j+0.5);
  }
  for(i=Right_start-1,j=0;i>=Right_stop;i--)
  {
    j++;
    Road_Right[i]=(uint8)(Road_Right[Right_start]-Right_Slope*j+0.5);
  }
}
/*
舵机控制
*/
void Servo_control(void)
{
  if(Cross_Flag_Last==31&&Cross_Flag!=31)
  {
    ring_time++;
    cross_Time=0;
  }
  if(ring_time>0/*&&Cross_Flag!=3*/)
  {
    Cross_Flag=3;
    ring_time++;
  }
  
  if(Ring_First_Row==0&&ring_time>0)
  {
    ring_time=0;
  }
  
  if((Cross_Flag_Last==2||Cross_Flag_Last==4)&&(Cross_Flag!=2&&Cross_Flag!=4))
    cross_time++;
  if(cross_time!=0&&abs(error)<5)
   cross_time=0;
  
  if(Cross_Flag_Last==1&&ring_time==0)
    cross_Time=1;
  else if(cross_Time>0&&cross_Time<100)
  {
    cross_Time++;
  }
  else if(cross_Time>100)
    cross_Time=0;
  uint8 Row_Ptr=0;
  error=0;
  error1=0;
  error2=0;
  Kp=0;
  Kd=0;
  Servo_temp=0;
  uint8 l=0;
  // buff[0]=1;
  
  ///过障碍
  if(Cross_Flag==5)
  {
    Servomiddle=8850;
  }
  else if(Cross_Flag==6)
  {
    Servomiddle=8750;
  }
  else
  {
    Servomiddle=8808;
  }
  
  if(All_Black>2)
  {
    Lastline=All_Black;
  }
  else
  {
    Lastline=2;
  }
  
  l = 59-Lastline;

  for(Row_Ptr=59; Row_Ptr>Lastline; Row_Ptr--)
    {
      error+=(Road_Center[Row_Ptr]-40);
    }
    error = error/l;
    for(Row_Ptr=59; Row_Ptr>59-l/2; Row_Ptr--)
    {
      error1+=(Road_Center[Row_Ptr]-40);
    }
    for(Row_Ptr=59-l/2; Row_Ptr>Lastline; Row_Ptr--)
    {
      error2+=(Road_Center[Row_Ptr]-40);
    }
    
    error1 = error1*2/l;
    error2 = error2/(59-l/2-Lastline);
    errorerror = error2-error1;
    
    if(Cross_Flag==2||cross_time>0)
    {
      Kp =86;
      Servo_temp=Kp*error/10+90;
    }
    else if(Cross_Flag==4||cross_time>0)
    {
      Kp =86;
      Servo_temp=Kp*error/10-90;
    }
    //else if(Cross_Flag==3||Cross_Flag==31||ring_time>0
    else if(ring_time>0)
    {
        if(Car == 1)
        {
          Servo_temp=-Ring_First_Row*100/10-90;
        }
        else
        {
          //Servo_temp=Ring_First_Row*100/10+30;
          Servo_temp=-Ring_First_Row*100/10-90;
        }
      
    }
    else if(Cross_Flag==1)
    {
      Kp =66;
      Kd = 35;
      Servo_temp=Kp*error/10+Kd*errorerror/10;
    }
    else
    {      
      if(All_Black==0)
    {
      if(error<0)
      {
        Kp = 35;
        Kd = 10;
      }
      else
      {
        Kp = 35;
        Kd = 10;
      }
    }
    else if(All_Black<12)
    {
      if(error<0)
      {
        Kp = 25;
        Kd = 0;
      }
      else
      {
        Kp = 25;
        Kd = 0;
      }
    }
    /*else if(All_Black<16) ////直道入弯道或者270度时提前转角
    {
      if(error<0)
      {
        Kp = 30;
        Kd = 10;
      }
      else
      {
        Kp = 30;
        Kd=10;
      }
      
    }*/
    else if(All_Black<21) ////弯道入直道的时候
    {
      if(error<0)
      {
        Kp = 30;
        Kd = 7;
      }
      else
      {
        Kp = 25;
        Kd=4;
      }
      
    }
    else if(All_Black<23) ////弯道入直道的时候
    {
      if(error<0)
      {
        Kp = 33;
        Kd = 18;
      }
      else
      {
        Kp = 33;
        Kd=18;
      }
      
    }
    /*else if(All_Black<26)
    {
      if(error<0)
      {
        Kp=40;
        Kd=15;
      }
      else
      {
        Kp=40;
        Kd =15;
      }
    }*/
    
    else if(All_Black<27)
    {
      if(error<0)
      {
        Kp=33;
        Kd=18;
      }
      else
      {
        Kp=33;
        Kd =18;
      }
    }
    else if(All_Black<32)
    {
      if(error<0)
      {
        Kp=45;
        Kd=20;
      }
      else
      {
        Kp=45;
        Kd =20;
      }
    }
    else if(All_Black<36)
    {
      if(error<0)
      {
        Kp = 50;
        Kd = 28;
      }
      else
      {
        Kp = 50;
        Kd = 28;
      }
    }
    else if(All_Black<41)
    {
      if(error<0)
      {
        Kp = 55;
        Kd = 30;
      }
      else
      {
        Kp = 55;
        Kd = 30;
      }
    }
    else
    {
      if(error<0)
      {
        Kp=100;
        Kd=35;
      }
      else
      {
        Kp=100;
        Kd =35;
      }
    }

    Servo_temp=Kp*error/10+Kd*errorerror/10;
    }
    
    /*if(cross_num>15)
    {
      if(error<0)
        Servo_temp = Servo_temp-cross_num;
      else
        Servo_temp = Servo_temp+cross_num;
    }*/
    
    Servo_value=Servomiddle+Servo_temp;
    
  if(Servo_value<Servo_min)
    Servo_value = Servo_min;
  if(Servo_value>Servo_max)
    Servo_value = Servo_max;
  ftm_pwm_duty(FTM0,FTM_CH3,Servo_value);
}
//NRF
void Overtake_judge()
{
  if(Car==1)
  {
    nrf_tx(race,4);
    while(nrf_tx_state() == NRF_TXING);//等待发送完成 
    
    race[1]=0;
  }
  else if(Car==2)
  {
    nrf_tx(race,4);
    while(nrf_tx_state() == NRF_TXING);//等待发送完成
    race[0]=0;
    
  }
}

//滤波
uint8 GetMedianNum(uint8 *Array,uint8 FilterLen)
{
  uint8 i,j;// 循环变量
  uint8 Temp;
  for (j=0; j<FilterLen-1; j++) // 对数组进行排序
  {
    for (i =0; i<FilterLen-j-1; i++)
    {
      if (Array[i]>Array[i + 1])
      {
        Temp = Array[i];
        Array[i] = Array[i+1];
        Array[i+1] = Temp;
      }
    }
  }
  // 计算中值
  if ((FilterLen & 1) > 0)
  {
    // 数组有奇数个元素，返回中间一个元素
    Temp = Array[(FilterLen + 1) / 2];
  }
  else
  {
    // 数组有偶数个元素，返回中间两个元素平均值
    Temp = (Array[FilterLen/2] + Array[FilterLen/2 + 1])/2;
  }
  return Temp;
}
//边线滤波
void Edge_Filter()
{
  uint8 Row_Ptr=0;
  uint8 temp=0;
  uint8 temp2=0;
  if(StopRow>=10)
    temp=StopRow-2;
  else
    temp=2;
  if(StopRow>=50)
    temp2=50;
  else
    temp2=StopRow;
  for(Row_Ptr=temp2+2; Row_Ptr>=temp; Row_Ptr--)
  {
    
    if(Road_Left[Row_Ptr]<=1)
      Road_Right[Row_Ptr]=79;
    if(Road_Right[Row_Ptr]>=78)
      Road_Left[Row_Ptr]=0;
    else
    {
      Road_Left[Row_Ptr]=Road_Left[Row_Ptr];
      Road_Right[Row_Ptr]=Road_Right[Row_Ptr];
    }
  }
}
//寻中线
void Find_Middle()
{
  Row_Ptr=0;
  uint8 CutLen=8;
  uint8 Cut=0;
  uint8 Edge[3]= {0};
  uint8 CutPos=0;//中线断开位置
  uint8 Var=0;
  Road_area=0;
  cross_num =0;
  FirstBlackinCenter=0;
  Overtake=0;
  //
  Out_Right=0;
  Out_Left=0;
  /////
  uint8 repair_R[60]= {0}; /////右边丢失的求斜率的数组
  uint8 repair_L[60]= {0}; /////左边丢失的求斜率的数组
  repair_slope_R=0;
  repair_slope_L=0;
  /////
  //filter_Middle(Road_Center);
  
  //斜入十字判断
  if(Cross_Flag==1&&StopRow>All_Black)
  {
    Calculate_Slope();
    if(Cross_Cnt==0) Cross_Cnt=1;//一个十字路口
    else if(Cross_Cnt==1) Cross_Cnt=2;
    else if(Cross_Cnt>2&&Cross_Cnt<5) Cross_Cnt=1;
  }
  else if(Cross_Flag==0)
  {
    if(Cross_Cnt==2) Cross_Cnt=3;//第一个十字路口结束
    else if(Cross_Cnt==3) Cross_Cnt=4;
    else if(Cross_Cnt==4) Cross_Cnt=5;
  }
  if(Cross_Cnt==5&&error>10) Cross_Cnt=6;//右转
  else if(Cross_Cnt==5&&error<-10) Cross_Cnt=7;//左转
  if(Cross_Cnt==6)   //右转判断左线
  {
    for(Row_Ptr=55;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Road_Left[Row_Ptr]>Road_Left[Row_Ptr+1]&&
         Road_Left[Row_Ptr+1]>Road_Left[Row_Ptr+2]&&
           Road_Left[Row_Ptr+2]>Road_Left[Row_Ptr+3]&&
              Road_Left[Row_Ptr-1]<=Road_Left[Row_Ptr]&&
               Road_Left[Row_Ptr-2]<=Road_Left[Row_Ptr-1]&&
               Road_Left[Row_Ptr-3]<Road_Left[Row_Ptr-2])
      {
        Cross_Flag=2;
        cross_num = Row_Ptr;
        Flag_L++;
        break;
      }
    }
    if(Flag_L>0&&Cross_Flag!=2)
    {
      Flag_L=0;
      Cross_Cnt=0;
    }
  }
  else if(Cross_Cnt==7) //左转判断右线
  {
    for(Row_Ptr=55;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Road_Right[Row_Ptr]<Road_Right[Row_Ptr+1]&&
         Road_Right[Row_Ptr+1]<Road_Right[Row_Ptr+2]&&
           Road_Right[Row_Ptr+2]<Road_Right[Row_Ptr+3]&&
              Road_Right[Row_Ptr-1]>=Road_Right[Row_Ptr]&&
               Road_Right[Row_Ptr-2]>=Road_Right[Row_Ptr-1]&&
               Road_Right[Row_Ptr-3]>Road_Right[Row_Ptr-2])
      {
        Cross_Flag=4;        
        cross_num = Row_Ptr;
        Flag_R++;
        break;
      }
    }
    if(Flag_R>0&&Cross_Flag!=4)
    {
      Flag_R=0;
      Cross_Cnt=0;
    }
  }
  //************************//
  //出圆环判断
  /*if(Cross_Flag==3)
  {
    if(Cross3_Cnt==0) Cross3_Cnt=1;//遇到圆环
    else if(Cross3_Cnt>1) Cross3_Cnt=1;
  }
  else if(Cross_Flag!=3)
  {
    if(Cross3_Cnt==1) Cross3_Cnt=2;//第一个十字路口结束
    //else if(Cross3_Cnt==2) Cross3_Cnt=3;
  }
  if(Cross3_Cnt==2&&error>10) Cross3_Cnt=4;//右边过
  else if(Cross3_Cnt==2&&error<-10) Cross3_Cnt=5;//左边过
  if(Cross3_Cnt==4&&error<-10) Cross3_Cnt=6; //右边过
  else if(Cross3_Cnt==5&&error>10) Cross3_Cnt=7; //左边过
  if(Cross3_Cnt==7) //左边过找左线
  {
    for(Row_Ptr=52;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Road_Left[Row_Ptr]>Road_Left[Row_Ptr+1]&&
         Road_Left[Row_Ptr+1]>Road_Left[Row_Ptr+2]&&
           Road_Left[Row_Ptr+2]>Road_Left[Row_Ptr+3]&&
              Road_Left[Row_Ptr-1]<=Road_Left[Row_Ptr]&&
               Road_Left[Row_Ptr-2]<=Road_Left[Row_Ptr-1]&&
               Road_Left[Row_Ptr-3]<Road_Left[Row_Ptr-2])
      {
        Cross3_Cnt=8; //左转找到拐点
        break;
      }
    }
  }
  else if(Cross3_Cnt==6) //右边过判断右线
  {
    for(Row_Ptr=52;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Road_Right[Row_Ptr]<Road_Right[Row_Ptr+1]&&
         Road_Right[Row_Ptr+1]<Road_Right[Row_Ptr+2]&&
           Road_Right[Row_Ptr+2]<Road_Right[Row_Ptr+3]&&
              Road_Right[Row_Ptr-1]>=Road_Right[Row_Ptr]&&
               Road_Right[Row_Ptr-2]>=Road_Right[Row_Ptr-1]&&
               Road_Right[Row_Ptr-3]>Road_Right[Row_Ptr-2])
      {
        Cross3_Cnt=9; //右转找到拐点
        break;
      }
    }
  }
  if(Cross3_Cnt==8)
  {
    for(Row_Ptr=52;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Road_Left[Row_Ptr]>Road_Left[Row_Ptr+1]&&
         Road_Left[Row_Ptr+1]>Road_Left[Row_Ptr+2]&&
           Road_Left[Row_Ptr+2]>Road_Left[Row_Ptr+3]&&
              Road_Left[Row_Ptr-1]<=Road_Left[Row_Ptr]&&
               Road_Left[Row_Ptr-2]<=Road_Left[Row_Ptr-1]&&
               Road_Left[Row_Ptr-3]<Road_Left[Row_Ptr-2])
      {
                //左转找到拐点
        break;
      }
    }
    if(Row_Ptr==All_Black)
    {
      Out_Left=1; //左转出圆环标志
     // Cross3_Cnt=0;
    }
  }
  else if(Cross3_Cnt==9)
  {
    for(Row_Ptr=52;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Road_Left[Row_Ptr]>Road_Left[Row_Ptr+1]&&
         Road_Left[Row_Ptr+1]>Road_Left[Row_Ptr+2]&&
           Road_Left[Row_Ptr+2]>Road_Left[Row_Ptr+3]&&
              Road_Left[Row_Ptr-1]<=Road_Left[Row_Ptr]&&
               Road_Left[Row_Ptr-2]<=Road_Left[Row_Ptr-1]&&
               Road_Left[Row_Ptr-3]<Road_Left[Row_Ptr-2])
      {
                //左转找到拐点
        break;
      }
    }
    if(Row_Ptr==All_Black)
    {
      Out_Right=1; //右转出圆环标志
     // Cross3_Cnt=0;
    }
  }*/

  //*******************// 
    for(Row_Ptr=59; Row_Ptr>56; Row_Ptr--)
    {
      if(start_line_num[Row_Ptr]<7)
        Road_Center[Row_Ptr]=(Road_Right[Row_Ptr]+Road_Left[Row_Ptr])/2;
      else
        Road_Center[Row_Ptr] = 40;
    }
  
  for(Row_Ptr=56; Row_Ptr>All_Black; Row_Ptr--)
  { 
    if(Left_Flag[Row_Ptr]==1 && Right_Flag[Row_Ptr]==1)
    {
      Road_Center[Row_Ptr]=(Road_Right[Row_Ptr]+Road_Left[Row_Ptr])/2;
    }
    else if(Left_Flag[Row_Ptr]==1)//左边有线
    {
      Road_Center[Row_Ptr]=Road_Center[Row_Ptr+1]+(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+1]);
    }
    else if(Right_Flag[Row_Ptr]==1)//右边有线左转
    {
      Road_Center[Row_Ptr]=Road_Center[Row_Ptr+1]+(Road_Right[Row_Ptr]-Road_Right[Row_Ptr+1]);
    }
    
    /*else if(Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3 &&
    (Left_Flag[CrossRow+2]==1 || Right_Flag[CrossRow+2]==1)
    &&(Left_Flag[Row_Ptr-3]!=1 && Right_Flag[Row_Ptr-3]!=1 ))
    {
    Road_Center[Row_Ptr]=Road_Center[CrossRow+2];
  }*/
    else
    {
      Road_Center[Row_Ptr]=(Road_Right[Row_Ptr]+Road_Left[Row_Ptr])/2;
    }
   
    
    ////排除中线跳变
    if(Road_Center[Row_Ptr]-Road_Center[Row_Ptr+1]>30&&Cross_Flag==0&&error*errorerror<0)
    {
      Road_Center[Row_Ptr] = Road_Center[Row_Ptr+1];
    }
    
    /*if(Road_Center[Row_Ptr]>=77||Road_Center[Row_Ptr]<=3)
    {  
        All_Black=Row_Ptr;
    }*/
    if(Road_Center[Row_Ptr]<0) Road_Center[Row_Ptr]=0;
    if(Road_Center[Row_Ptr]>79) Road_Center[Row_Ptr]=79;
  }//结束for
  
  for(Row_Ptr=59; Row_Ptr>All_Black; Row_Ptr--)
  {
    Edge[0]=Road_Center[Row_Ptr];
    Edge[1]=Road_Center[Row_Ptr-1];
    Edge[2]=Road_Center[Row_Ptr-2];
    
    Road_Center[Row_Ptr]=GetMedianNum(Edge,3);
    int a=Road_Center[Row_Ptr];
    //if(Status==0)
    //{
    /*if(img[Row_Ptr][a]==255&&Left_sign==1&&Right_sign==1&&All_Black==0&&a<=60&&a>=20)
    {
    Var++;
    if(Var==1)
    FirstBlackinCenter=Row_Ptr;
    if(FirstBlackinCenter>=15)
    {
    Overtake=1;
    Overtake_Flag=1;
  }
  }*/
    //}
    //img[Row_Ptr][Road_Center[Row_Ptr]]=0;
    img[Row_Ptr][a]=0;
    a=Road_Left[Row_Ptr]+2;
    img[Row_Ptr][a]=0;
    a=Road_Right[Row_Ptr]-2;
    img[Row_Ptr][a]=0;
  }
  //filter_Middle(Road_Center);
  //结束for_滤中线
}

//寻边线
uint8 a=1;
uint8 ring_num;
void Search_Line(void)
{
  Cross_Flag_Last=Cross_Flag; 
  Row_Ptr=0;
  Col_Ptr=0;
  uint8 LFlag=0;
  uint8 RFlag=0;
  uint8 WFlag=0;
  uint8 SFlag=0;
  uint8 Cut_Width=10;
  int8 LEnd=0;
  int8 REnd=79;
  
  uint8 Left_left=0;
  uint8 Right_right=0;
  uint8 Left_J=0;
  uint8 Left_Y=0;
  uint8 Right_J=0;
  uint8 Right_Y=0;
  uint8 Cross_flag=0;
  
  a=1;
  uint8 b=79;
  uint8 i,j,k;
  uint8 a_f=0,b_f=0,c_f=0;
  
  Left_Cnt=0;
  Right_Cnt=0;
  Left_cnt=0;
  Right_cnt=0;
  Cross_Flag=0;
  Change_Flag=0;
  Black_Cnt=0;
  Black_cnt=0;
  White_cnt=0;
  White_Cnt=0;
  All_Black=0;
  All_White=0;
  Road_Change=0;
  CrossRow=59;
  StopRow=0;
 // Stop_Flag=0;
  //a=0;
  Left_sign=0;
  Right_sign=0;
  
  Left_c_in=0;
  Left_c_out=0;
  Right_c_in=0;
  Right_c_out=0;
  Left_r_in=0;
  Left_r_out=0;
  Right_r_in=0;
  Right_r_out=0;
  //////////////////////////
  stop_line_num = 0;
  Ring_width_1 = 50;
  Ring_width_2 = 30;
  Ring_width =0;
  Ring_First_Row=0;  
  Ring_Second_Row=0;
  Bend_Lift =0;
  Bend_Right = 0;
  uint8 ring_flag=0;
  ring_num=0;
  white_Left_cnt = 0;
  white_Right_cnt = 0;

  ///////////////////////
  //前三行搜线开始
  for(Row_Ptr=59; Row_Ptr>56; Row_Ptr--)
  {
    Road_Left[Row_Ptr]=0;
    Road_Right[Row_Ptr]=79;
    Left_Flag[Row_Ptr]=0;
    Right_Flag[Row_Ptr]=0;
    Road_Center[Row_Ptr]=0;
    
    start_line_num[Row_Ptr] = 0;
      for(Col_Ptr=0;Col_Ptr<75;Col_Ptr++)
      {      
        if(img[Row_Ptr][Col_Ptr]==0 &&img[Row_Ptr][Col_Ptr+1]==0 && img[Row_Ptr][Col_Ptr+2]==0&&
           img[Row_Ptr][Col_Ptr+3]==255&& img[Row_Ptr][Col_Ptr+4]==255&& img[Row_Ptr][Col_Ptr+5]==255)
        {
          start_line_num[Row_Ptr] ++;
        }      
      }
      if(start_line_num[Row_Ptr]>6)
      {
        stop_line_num++;
      }
      if(stop_line_num>=3&&stop_Flag!=1&&Stop_Flag!=0)
      {
        if(sum_time>8000)
        {
          Stop_Flag=2;
        }
      }
      else if(stop_line_num>=3&&Stop_Flag==0)
      {
        Stop_Flag=1;
      }
    
    //内层for开始 从中心向左边
    for(Col_Ptr=60; Col_Ptr>2; Col_Ptr--)
    {
      if(img[Row_Ptr][Col_Ptr-2]==0 && img[Row_Ptr][Col_Ptr-1]==0&&
         img[Row_Ptr][Col_Ptr]==255&& img[Row_Ptr][Col_Ptr+1]==255)
      {
        Road_Left[Row_Ptr]=Col_Ptr-1;
        Left_Flag[Row_Ptr]=1;
        Left_Cnt++;
        break;
      }
    }//结束内层for
    //内层for开始从中心向右
    for(Col_Ptr=20; Col_Ptr<77; Col_Ptr++)
    {
      if(img[Row_Ptr][Col_Ptr-1]==255 && img[Row_Ptr][Col_Ptr]==255&&
         img[Row_Ptr][Col_Ptr+1]==0&& img[Row_Ptr][Col_Ptr+2]==0)
      {
        Road_Right[Row_Ptr]=Col_Ptr+1;
        Right_Flag[Row_Ptr]=1;
        Right_Cnt++;
        break;
      }
    }//结束内层for
    if(Road_Left[Row_Ptr]>Road_Right[Row_Ptr])
    {
      Left_Flag[Row_Ptr]=0;
      Right_Flag[Row_Ptr]=0;
      Road_Left[Row_Ptr]=Road_Left[Row_Ptr+1];
      Road_Right[Row_Ptr]=Road_Right[Row_Ptr+1];
    }
  }//结束三行搜线 外层for
  Road_Width[59]=Road_Right[59]-Road_Left[59];
  Road_Width[58]=Road_Right[58]-Road_Left[58];
  Road_Width[57]=Road_Right[57]-Road_Left[57];
  //其余行搜索
  //if(Left_Cnt>0 || Right_Cnt>0)
  
  

  for(Row_Ptr=56; Row_Ptr>2&&Row_Ptr>All_Black; Row_Ptr--)
  {
    Left_addwidth[Row_Ptr]=0;
    Right_addwidth[Row_Ptr]=0;
    Road_Left[Row_Ptr]=0;
    Road_Right[Row_Ptr]=79;
    Left_Flag[Row_Ptr]=0;
    Right_Flag[Row_Ptr]=0;
    Road_Center[Row_Ptr]=0;
    //从左到右检测起跑线
    if(Row_Ptr>25)
    {
      
      start_line_num[Row_Ptr] = 0;
      for(Col_Ptr=0;Col_Ptr<75;Col_Ptr++)
      {      
        if(img[Row_Ptr][Col_Ptr]==0 &&img[Row_Ptr][Col_Ptr+1]==0 && img[Row_Ptr][Col_Ptr+2]==0&&
           img[Row_Ptr][Col_Ptr+3]==255&& img[Row_Ptr][Col_Ptr+4]==255&& img[Row_Ptr][Col_Ptr+5]==255)
        {
          start_line_num[Row_Ptr] ++;
        }      
      }
      if(start_line_num[Row_Ptr]>6)
      {
        stop_line_num++;
      }
      if(stop_line_num>=3&&stop_Flag!=1&&Stop_Flag!=0)
      {
        if(sum_time>8000)
        {
          Stop_Flag=2;
        }
      }
      else if(stop_line_num>=3&&Stop_Flag==0)
      {
        Stop_Flag=1;
      }
    }
    
    
    //确定左边下一行的搜范围
    if(Row_Ptr>30)
    {
      Col_Ptr=Road_Left[Row_Ptr+1]+6;
      LEnd=Road_Left[Row_Ptr+1]-6;
    }
    else if(Row_Ptr>14 && Row_Ptr<=30)
    {
      Col_Ptr=Road_Left[Row_Ptr+1]+10;
      LEnd=Road_Left[Row_Ptr+1]-10;
    }
    else
    {
      Col_Ptr=Road_Left[Row_Ptr+1]+20;
      LEnd=Road_Left[Row_Ptr+1]-20;
    }
    //              Col_Ptr=(Road_Left[Row_Ptr+1]+Road_Right[Row_Ptr+1])/2;
    //              if(Row_Ptr>30)                     {LEnd=Road_Left[Row_Ptr+1]-6;}
    //              else if(Row_Ptr>14 && Row_Ptr<=30) {LEnd=Road_Left[Row_Ptr+1]-10;}
    //              else                               {LEnd=Road_Left[Row_Ptr+1]-20;}
    if(Cross_Flag==1)
    {
      Col_Ptr=Road_Right[Row_Ptr+1]-20;
      LEnd=1;
    }
    
    if(Left_Flag[Row_Ptr+1]==3)
    {
      Col_Ptr=45;
      LEnd=1;
    }
    
    if(Col_Ptr>Road_Right[Row_Ptr+1]) Col_Ptr=Road_Right[Row_Ptr+1];//防止越界
    if(Col_Ptr>76) Col_Ptr=76;
    if(LEnd<3) LEnd=3;//for开始搜索左边
    
    
    for(; Col_Ptr>LEnd; Col_Ptr--)
    {
      if(img[Row_Ptr][Col_Ptr-2]==0 && img[Row_Ptr][Col_Ptr-1]==0&&
         img[Row_Ptr][Col_Ptr]==255&& img[Row_Ptr][Col_Ptr+1]==255)
      {
        Road_Left[Row_Ptr]=Col_Ptr;
        Left_Flag[Row_Ptr]=1;
        Left_Cnt++;
        if(LFlag==0 && Left_Flag[Row_Ptr+1]==1)
        {
          Left_cnt++;   //统计连续的黑线计数
        }
        else
        {
          LFlag=1;   //重找到线
        }
        break;
      }
    }//结束左边for循环
    if(Col_Ptr==LEnd && img[Row_Ptr][39]==0 && img[Row_Ptr][40]==0 && img[Row_Ptr][41]==0)
      Left_Flag[Row_Ptr]=2;//在搜线范围内没找到
    if(Col_Ptr==LEnd && img[Row_Ptr][39]==255 && img[Row_Ptr][40]==255 && img[Row_Ptr][41]==255)
    {
      Left_Flag[Row_Ptr]=3;//在搜线范围内没找到
    }
    if(Row_Ptr==30&&Col_Ptr==LEnd&&img[Row_Ptr][10]==255) Left_sign=1;
    //确定右边下一行的搜线位置
    //              Col_Ptr=(Road_Left[Row_Ptr+1]+Road_Right[Row_Ptr+1])/2;
    //              if(Row_Ptr>30)                     {REnd=Road_Right[Row_Ptr+1]+6;}
    //              else if(Row_Ptr>14 && Row_Ptr<=30) {REnd=Road_Right[Row_Ptr+1]+10;}
    //              else                               {REnd=Road_Right[Row_Ptr+1]+20;}
    if(Row_Ptr>30)
    {
      Col_Ptr=Road_Right[Row_Ptr+1]-6;
      REnd=Road_Right[Row_Ptr+1]+6;
    }
    else if(Row_Ptr>14 && Row_Ptr<=30)
    {
      Col_Ptr=Road_Right[Row_Ptr+1]-10;
      REnd=Road_Right[Row_Ptr+1]+10;
    }
    else
    {
      Col_Ptr=Road_Right[Row_Ptr+1]-20;
      REnd=Road_Right[Row_Ptr+1]+20;
    }
    if(Cross_Flag==1)
    {
      Col_Ptr=Road_Left[Row_Ptr+1]+20;
      REnd=78;
      //                      a=1;
    }
    
    
    if(Right_Flag[Row_Ptr+1]==3)
    {
      Col_Ptr=35;
      REnd=78;
    }
    
    
    if(Col_Ptr<Road_Left[Row_Ptr+1]) Col_Ptr=Road_Left[Row_Ptr+1];//防止越界
    if(Col_Ptr<3) Col_Ptr=3;
    if(REnd>76) REnd=76;//for开始_搜
    
    for(; Col_Ptr<REnd; Col_Ptr++)
    {
      if(img[Row_Ptr][Col_Ptr-1]==255 && img[Row_Ptr][Col_Ptr]==255&&
         img[Row_Ptr][Col_Ptr+1]==0&& img[Row_Ptr][Col_Ptr+2]==0)
      {
        Road_Right[Row_Ptr]=Col_Ptr;
        Right_Flag[Row_Ptr]=1;
        Right_Cnt++;
        if(RFlag==0 && Right_Flag[Row_Ptr+1]==1)
        {
          Right_cnt++;   //统计连续的黑线计数
        }
        else
        {
          RFlag=1;   //重找到线
        }
        break;
      }
    }//结束for_搜右边
    if(Cross_flag<2)
    {
      if(img[Row_Ptr][Road_Right[Row_Ptr+1]]==255&&Right_Flag[Row_Ptr+1]==3)
      {
        Road_Right[Row_Ptr]=Road_Right[Row_Ptr+1];
        Right_Flag[Row_Ptr]=3;
      }
      if(img[Row_Ptr][Road_Left[Row_Ptr+1]]==255&&Left_Flag[Row_Ptr+1]==3)
      {
        Road_Left[Row_Ptr]=Road_Left[Row_Ptr+1];
        Left_Flag[Row_Ptr]=3;
      }
    }
    if(Col_Ptr==REnd && img[Row_Ptr][39] ==0 && img[Row_Ptr][40]==0 && img[Row_Ptr][41]==0) Right_Flag[Row_Ptr]=2;//在搜线范围内没找到_全黑行
    if(Col_Ptr==REnd&& img[Row_Ptr][39]==255 && img[Row_Ptr][40]==255 && img[Row_Ptr][41]==255) Right_Flag[Row_Ptr]=3;//在搜线范围内没找到_全白行
    if(Row_Ptr==30&&Col_Ptr==REnd&&img[Row_Ptr][70]==255) Right_sign=1;
    Road_Width[Row_Ptr]=Road_Right[Row_Ptr]-Road_Left[Row_Ptr];
    if((Left_Flag[Row_Ptr]==2 && Right_Flag[Row_Ptr]==2) && (Left_Flag[Row_Ptr+1]==2 && Right_Flag[Row_Ptr+1]==2) &&(Left_Flag[Row_Ptr+2]==2 && Right_Flag[Row_Ptr+2]==2) )
    {
      Black_Cnt++;//统计所有的全黑行
      if(Black_Cnt==1)
      {
        All_Black=Row_Ptr+2;   //记录第一次找到全黑行的位置
      }
    }
    if(Road_Right[Row_Ptr]<=5||Road_Left[Row_Ptr]>=75||
       abs(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+1])>45||
         abs(Road_Right[Row_Ptr]-Road_Right[Row_Ptr+1])>45)
      All_Black=Row_Ptr;
    if(Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3)
    {
      White_Cnt++;////统计全白行个数_十字
      if(White_Cnt==1)
      {
        All_White=Row_Ptr;   //记录第一次找到全白行的位置
      }
    }
    /*if((Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3)&&
    (Left_Flag[Row_Ptr+1]==3 && Right_Flag[Row_Ptr+1]==3)&&
    (Left_Flag[Row_Ptr+2]==3&& Right_Flag[Row_Ptr+2]==3)&&(
    (Left_Flag[Row_Ptr+3]==1&& Right_Flag[Row_Ptr+3]==1)||
    (Left_Flag[Row_Ptr+4]==1&& Right_Flag[Row_Ptr+4]==1)||
    (Left_Flag[Row_Ptr+5]==1&& Right_Flag[Row_Ptr+5]==1)||
    (Left_Flag[Row_Ptr+6]==1&& Right_Flag[Row_Ptr+6]==1)||
    (Left_Flag[Row_Ptr+7]==1&& Right_Flag[Row_Ptr+7]==1)
    ))
    {
    Cross_Flag=1;
    if(Left_Flag[Row_Ptr+3]==1&& Right_Flag[Row_Ptr+3]==1)
    CrossRow=Row_Ptr+2;
    else if(Left_Flag[Row_Ptr+4]==1&& Right_Flag[Row_Ptr+4]==1)
    CrossRow=Row_Ptr+3;
    else if(Left_Flag[Row_Ptr+5]==1&& Right_Flag[Row_Ptr+5]==1)
    CrossRow=Row_Ptr+4;
    else if(Left_Flag[Row_Ptr+6]==1&& Right_Flag[Row_Ptr+6]==1)
    CrossRow=Row_Ptr+5;
    else if(Left_Flag[Row_Ptr+7]==1&& Right_Flag[Row_Ptr+7]==1)
    CrossRow=Row_Ptr+6;
    
  }//如果有连续三行丢线，判断为十字
    else if((Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3)&&
    (Left_Flag[Row_Ptr+1]==3 && Right_Flag[Row_Ptr+1]==3)&&
    (Left_Flag[Row_Ptr+2]==3&& Right_Flag[Row_Ptr+2]==3)&&Row_Ptr>57)
    {
    Cross_Flag=2;
    CrossRow = 59;
  }///如果在车头连续三行丢线，十字路口另外一种情况
    else*/
    
    
    if((Left_Flag[Row_Ptr+2]==3 && Right_Flag[Row_Ptr+2]==3)&&
       (Left_Flag[Row_Ptr+1]==3 && Right_Flag[Row_Ptr+1]==3)&&
         (Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3)&&(
                                                             (Left_Flag[Row_Ptr-1]==1 && Right_Flag[Row_Ptr-1]==1)||
                                                               (Left_Flag[Row_Ptr-2]==1 && Right_Flag[Row_Ptr-2]==1)||
                                                                 (Left_Flag[Row_Ptr-3]==1 && Right_Flag[Row_Ptr-3]==1)||
                                                                   (Left_Flag[Row_Ptr-4]==1 && Right_Flag[Row_Ptr-4]==1)||
                                                                     (Left_Flag[Row_Ptr-5]==1 && Right_Flag[Row_Ptr-5]==1)||
                                                                       (Left_Flag[Row_Ptr-6]==1 && Right_Flag[Row_Ptr-6]==1))&&Row_Ptr>All_Black)//如果四行后两行丢线前两行重新找到线，十字
    {
      Cross_Flag=1;
      Cross_flag++;
      // Cross_Flag_Last=Cross_Flag;
      if(Left_Flag[Row_Ptr-1]==1 && Right_Flag[Row_Ptr-1]==1)
        StopRow=Row_Ptr-1;
      else if(Left_Flag[Row_Ptr-2]==1 && Right_Flag[Row_Ptr-2]==1)
        StopRow=Row_Ptr-2;
      else if(Left_Flag[Row_Ptr-3]==1 && Right_Flag[Row_Ptr-3]==1)
        StopRow=Row_Ptr-3;
      else if(Left_Flag[Row_Ptr-4]==1 && Right_Flag[Row_Ptr-4]==1)
        StopRow=Row_Ptr-4;
      else if(Left_Flag[Row_Ptr-5]==1 && Right_Flag[Row_Ptr-5]==1)
        StopRow=Row_Ptr-5;
      else if(Left_Flag[Row_Ptr-6]==1 && Right_Flag[Row_Ptr-6]==1)
        StopRow=Row_Ptr-6;
    }
    else if(Left_Flag[Row_Ptr+4]==3 &&Right_Flag[Row_Ptr+4] ==1&&
            Left_Flag[Row_Ptr+3]==3 &&Right_Flag[Row_Ptr+3] ==1&&
              Left_Flag[Row_Ptr+2]==3 &&Right_Flag[Row_Ptr+2] ==1&&
                Left_Flag[Row_Ptr+1]==3 &&Right_Flag[Row_Ptr+1] ==1&&
                  Left_Flag[Row_Ptr]==3 &&Row_Ptr<48&&Row_Ptr>7)
    {
      Bend_Lift = 1;
    }
    else if(Right_Flag[Row_Ptr+4]==3&&Left_Flag[Row_Ptr+4]==1&&
            Right_Flag[Row_Ptr+3]==3&&Left_Flag[Row_Ptr+3]==1&&
              Right_Flag[Row_Ptr+2]==3&&Left_Flag[Row_Ptr+2]==1&&
                Right_Flag[Row_Ptr+1]==3&&Left_Flag[Row_Ptr+1]==1&&
                  Right_Flag[Row_Ptr]==3&&Row_Ptr<48&&Row_Ptr>7)
    {
      Bend_Right = 1;
    }
    
    if(Right_Flag[Row_Ptr]==3)
    {
      white_Right_cnt++;
    }
    if(Left_Flag[Row_Ptr]==3)
    {
      white_Left_cnt++;
    }
    //if(  (Row_Ptr>All_Black+6)&&(Cross_Flag!=3||Cross_Flag!=4))
    ring_flag=0;
    for(Col_Ptr=Road_Left[Row_Ptr]; Col_Ptr<Road_Right[Row_Ptr]-3; Col_Ptr++)
    {      
      if(ring_flag==0&&img[Row_Ptr][Col_Ptr]==255&&img[Row_Ptr][Col_Ptr+1]==255&&img[Row_Ptr][Col_Ptr+2]==0&&img[Row_Ptr][Col_Ptr+3]==0) 
      {
        ring_flag=1;///找到黑块左跳变点
        if(Col_Ptr+2<Ring_width_1)
          Ring_width_1 = Col_Ptr+2;///黑块左边最小的列
        a=Col_Ptr+2;///左边界

      }
      else if(ring_flag==1&&img[Row_Ptr][Col_Ptr]==0&&img[Row_Ptr][Col_Ptr+1]==0&&img[Row_Ptr][Col_Ptr+2]==255&&img[Row_Ptr][Col_Ptr+3]==255) 
      {
        b=Col_Ptr;//右边界
        ring_flag=2;//找到黑块右跳变点
        ring_num++;//黑块行数加1
        if(Col_Ptr+1>Ring_width_2)
          Ring_width_2 = Col_Ptr+1;//黑块右边最大的的列
        if(ring_num==1) Ring_First_Row=Row_Ptr;
        break;
      }
    }
    
      if(Ring_width_2-Ring_width_1>Ring_width)
      {
        Ring_width = Ring_width_2-Ring_width_1;
      }
    for(Col_Ptr=Ring_width_1;Col_Ptr<Ring_width_2; Col_Ptr++)
    {
      if(img[Row_Ptr][Col_Ptr]==0&&img[Row_Ptr][Col_Ptr+1]==0&&img[Row_Ptr][Col_Ptr+2]==0)
      {
        break;
      }
    }
    uint8 samll_Ring_temp=0;
    if(abs(Ring_width_2-Col_Ptr)<3)//障碍
    {     
      samll_Ring_temp=1;
      if(/*Ring_width>10&&Stop_Flag!=0&&sum_time>1000&&White_Cnt>3&&*/cross_Time==0&&Stop_Flag!=0&&sum_time>1000)///经过起跑线才识别圆环，排除起跑线误判，sum_time是经过起跑线才计时
      {
        Cross_Flag=3;/////标记为小圆环
      }
      else if(ring_num>5&&ring_time==0)
      {
        if(Road_Right[Row_Ptr]-Ring_width_2>Ring_width_1-Road_Left[Row_Ptr])
        {
          Cross_Flag=5;
        }
        else if(Road_Right[Row_Ptr]-Ring_width_2<Ring_width_1-Road_Left[Row_Ptr])
        {
          Cross_Flag=6;
        }
        
      }
    }
    //入圆环前两边拐点   
    Left_J=0;
    Left_Y=0;
    if(Left_left!=1&&Row_Ptr<53&&Row_Ptr>8) //左拐点确定
    {
      for(Col_Ptr=Row_Ptr;Col_Ptr<Row_Ptr+5;Col_Ptr++)
      {
        if(Road_Left[Col_Ptr]<Road_Left[Col_Ptr+1]||Road_Left[Col_Ptr]==0||Road_Left[Col_Ptr+1]==0)
        {
          Left_J=0;
          break;
        }
        else if(Road_Left[Col_Ptr]>Road_Left[Col_Ptr+1]) Left_J=1;
      }
      for(Col_Ptr=Row_Ptr;Col_Ptr>Row_Ptr-5;Col_Ptr--)
      {
        if(Road_Left[Col_Ptr-1]>Road_Left[Col_Ptr]||Road_Left[Col_Ptr]==0||Road_Left[Col_Ptr-1]==0)
        {

          Left_Y=0;
          break;
        }
        else if(Road_Left[Col_Ptr-1]<Road_Left[Col_Ptr]) 
        {
          Left_Y=1;
          break;
        }
      }
      if(Left_J==1&&Left_Y==1)
      {
        Left_left=1;
        Left_xian=Row_Ptr;
      }
    }
    Right_J=0;
    Right_Y=0;
    if(Right_right!=1&&Row_Ptr<53&&Row_Ptr>8) //右拐点确定
    {
      for(Col_Ptr=Row_Ptr;Col_Ptr<Row_Ptr+5;Col_Ptr++)
      {
        if(Road_Right[Col_Ptr]>Road_Right[Col_Ptr+1]||Road_Right[Col_Ptr]==79||Road_Right[Col_Ptr+1]==79)
        {
          Right_J=0;
          break;
        }
        else if(Road_Right[Col_Ptr]<Road_Right[Col_Ptr+1]) Right_J=1;
      }
      for(Col_Ptr=Row_Ptr;Col_Ptr>Row_Ptr-5;Col_Ptr--)
      {
        if(Road_Right[Col_Ptr-1]<Road_Right[Col_Ptr]||Road_Right[Col_Ptr]==79||Road_Right[Col_Ptr-1]==79)
        {
          Right_Y=0;
          break;
        }
        else if(Road_Right[Col_Ptr-1]>Road_Right[Col_Ptr]) 
        {
          Right_Y=1;
          break;
        }
      }
      if(Right_J==1&&Right_Y==1)
      {
        Right_right=1;
        Right_xian=Row_Ptr;
      }
    }
    
      
    /*if(abs(Ring_width_2-Col_Ptr)<3)////从黑块的左边往右找，如果Col_Ptr接近了黑块最右边，说明圆环上面有白的，判断为圆环
    {      
        if(ring_num>5)
        {
            if(Ring_width>10&&Stop_Flag!=0&&sum_time>1000)///经过起跑线才识别圆环，排除起跑线误判，sum_time是经过起跑线才计时
            {
              Cross_Flag=3;/////标记为小圆环
            }
        }
    }*/
    if(ring_num>0&&Right_right==1&&Left_left==1&&(abs(Right_xian-Left_xian))<10)
    {
      Cross_Flag=31;/////标记为大圆环
    }

    if(samll_Ring_temp==1&&cross_Time==0&&(Right_right==1||Left_left==1)&&Stop_Flag!=0&&sum_time>1000)
    {
      Cross_Flag=3;
      ring_time++;
    }
    if(Road_Left[Row_Ptr]>Road_Right[Row_Ptr])
    {
      Road_Left[Row_Ptr]=Road_Left[Row_Ptr+1];
      Road_Right[Row_Ptr]=Road_Right[Row_Ptr+1];
    }
  }//结束for 行循环
  // }//结束if判断
}
//寻线函数结束

//寻线函数结束
