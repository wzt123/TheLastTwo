
#include "ChaoShenBo.h"
#include "common.h"
#include "include.h"

uint32 SysTimer = 0;
uint32 chaoshengboTime=0;//读取到的超声波时间,单位微秒
uint32 ABDistance = 0;//换算后的发送接收模块的距离,单位毫米
uint32 ABDistance_last =0; 
uint8 ABDistance_num=12;
uint32 ABDistance_Rember[12] ={0};
uint32 ABDistance_A =0;
uint32 ABDistance_B =0;
uint32 ABDistance_C =0;
uint32 filter[3] = {0};
uint32 filter_ChaoShenBo(uint32 *Array,uint8 FilterLen);


uint32 setdelay(unsigned long t)
{
  SysTimer =  RTC_TSR*1000L ;
  SysTimer+=  RTC_TPR*1000L/32768L;
  return(SysTimer + t - 1);                             
}

uint16 checkdelay (unsigned long t)
{
  SysTimer =  RTC_TSR*1000L ;
  SysTimer+=  RTC_TPR*1000L/32768L;
  return(((t - SysTimer) & 0x80000000) >> 16);
}


/*
*超声波测距中断函数
A29
*/
void PORTE_IRQHandler(void)
{
  uint32 flag;
  flag = PORTE_ISFR;
  /*if((flag & (1 << 9)||flag & (1 << 10))&&Car==1)                                 //PTE9,PTE10触发中断
  {
    if(gpio_get(PTE10)&&gpio_get(PTE9)&&stop_Flag!=1)             //PTC8，PTC9触发中断
    {
      //stop_Car();
    }
  }*/
  PORTE_ISFR|= 0xFFFFFFFFu;   //清除标志位 
  if(gpio_get(PTE27))//检测到IO口是高电平，那么就是上升沿
  {//如果是上升沿中断

    PIT_TCTRL0  &= ~PIT_TCTRL_TEN_MASK;//停止定时器
    PIT_LDVAL0  = 0xFFFFFFFF;//32位定时器，装最大值即可
    PIT_TCTRL0  |= PIT_TCTRL_TEN_MASK;//启动定时器
  }
  else
  {
    
    chaoshengboTime = (0xFFFFFFFF - PIT_CVAL0)/90;//50M总线时钟，计算得到时间，单位是微秒
    ABDistance = chaoshengboTime * 340/1000;//一秒钟的声音速度假设为340米，由于chaoshengboTime单位是微秒，/1000后得到单位是mm
    ////
    if(ABDistance<300)
    {
      ABDistance = ABDistance_last;
    }
    else if(abs(ABDistance_last-ABDistance)<400)
    {
      ABDistance_A = ABDistance_B;
      ABDistance_B = ABDistance_C;
      ABDistance_C = ABDistance_last;
      ABDistance_last = ABDistance;
    }
    else
    {
      ABDistance = ABDistance_last;
    }
    
    filter[0] = ABDistance_A;
    filter[1] = ABDistance_B; 
    filter[2] = ABDistance_C;
    ABDistance = filter_ChaoShenBo(filter,3);
    ////////////如果第一车检测到的超声波不是反射的，说明超车成功
    /*if(Car==1&&ABDistance<500)
    {
      overTake_num++;////超车次数加1 
      Car=2;//车次变换为2
      stop_Flag=0;//重新启动车
      race[1]=1;///告诉前车超车成功
    }*/
    PIT_TCTRL0  &= ~PIT_TCTRL_TEN_MASK;//停止定时器
  }
}

///超声波专用滤波函数
uint32 filter_ChaoShenBo(uint32 *Array,uint8 FilterLen)
{
  uint8 i,j;// 循环变量
  uint32 Temp;
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


/*uint32 filter_ChaoShenBo(uint32 *Array,uint8 FilterLen)
{
  uint8 i,j;// 循环变量
  uint32 Temp,sum;
  for (j=0; j<FilterLen; j++) 
  {
    sum += Array[i];
  }
  Temp = sum/FilterLen;
  return Temp;
}*/





