
#include "ChaoShenBo.h"
#include "common.h"
#include "include.h"

uint32 SysTimer = 0;
uint32 chaoshengboTime=0;//��ȡ���ĳ�����ʱ��,��λ΢��
uint32 ABDistance = 0;//�����ķ��ͽ���ģ��ľ���,��λ����
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
*����������жϺ���
A29
*/
void PORTE_IRQHandler(void)
{
  uint32 flag;
  flag = PORTE_ISFR;
  /*if((flag & (1 << 9)||flag & (1 << 10))&&Car==1)                                 //PTE9,PTE10�����ж�
  {
    if(gpio_get(PTE10)&&gpio_get(PTE9)&&stop_Flag!=1)             //PTC8��PTC9�����ж�
    {
      //stop_Car();
    }
  }*/
  PORTE_ISFR|= 0xFFFFFFFFu;   //�����־λ 
  if(gpio_get(PTE27))//��⵽IO���Ǹߵ�ƽ����ô����������
  {//������������ж�

    PIT_TCTRL0  &= ~PIT_TCTRL_TEN_MASK;//ֹͣ��ʱ��
    PIT_LDVAL0  = 0xFFFFFFFF;//32λ��ʱ����װ���ֵ����
    PIT_TCTRL0  |= PIT_TCTRL_TEN_MASK;//������ʱ��
  }
  else
  {
    
    chaoshengboTime = (0xFFFFFFFF - PIT_CVAL0)/90;//50M����ʱ�ӣ�����õ�ʱ�䣬��λ��΢��
    ABDistance = chaoshengboTime * 340/1000;//һ���ӵ������ٶȼ���Ϊ340�ף�����chaoshengboTime��λ��΢�룬/1000��õ���λ��mm
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
    ////////////�����һ����⵽�ĳ��������Ƿ���ģ�˵�������ɹ�
    /*if(Car==1&&ABDistance<500)
    {
      overTake_num++;////����������1 
      Car=2;//���α任Ϊ2
      stop_Flag=0;//����������
      race[1]=1;///����ǰ�������ɹ�
    }*/
    PIT_TCTRL0  &= ~PIT_TCTRL_TEN_MASK;//ֹͣ��ʱ��
  }
}

///������ר���˲�����
uint32 filter_ChaoShenBo(uint32 *Array,uint8 FilterLen)
{
  uint8 i,j;// ѭ������
  uint32 Temp;
  for (j=0; j<FilterLen-1; j++) // �������������
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
  // ������ֵ
  if ((FilterLen & 1) > 0)
  {
    // ������������Ԫ�أ������м�һ��Ԫ��
    Temp = Array[(FilterLen + 1) / 2];
  }
  else
  {
    // ������ż����Ԫ�أ������м�����Ԫ��ƽ��ֵ
    Temp = (Array[FilterLen/2] + Array[FilterLen/2 + 1])/2;
  }
  return Temp;
}


/*uint32 filter_ChaoShenBo(uint32 *Array,uint8 FilterLen)
{
  uint8 i,j;// ѭ������
  uint32 Temp,sum;
  for (j=0; j<FilterLen; j++) 
  {
    sum += Array[i];
  }
  Temp = sum/FilterLen;
  return Temp;
}*/





