#include "common.h"
#include "handle.h"
#include "include.h"
#include "function.h"

uint8 img[CAMERA_H][CAMERA_W];
int8 Road_Left[ROW_MAX]= {0}; //����λ��_�б�
int8 Road_Right[ROW_MAX]= {0}; //����λ��_�б�
int8 Road_Center[ROW_MAX]= {0}; //����λ��_�б�
int8 Road_Width[ROW_MAX]= {0}; //ÿ�������Ŀ��

uint16 Road_area;
uint8 Road_Change=0;
uint8 Road_type=0;

uint8 Left_Flag[ROW_MAX]= {0}; //����������ҵ���־
uint8 Right_Flag[ROW_MAX]= {0}; //�����ұ����ҵ���־

int8 Road_Left_f[ROW_MAX]= {0}; //����λ��_�б긱
int8 Road_Right_f[ROW_MAX]= {0}; //����λ��_�б긱

uint8 Left_Cnt=0;//���߼��� ���е�
uint8 Right_Cnt=0;//���߼��� ���е�
uint8 Right_cnt=0;//����������߼���
uint8 Left_cnt=0;//�������ұ��߼���

uint8 All_Black=0;//ȫ����λ��_�б�
uint8 All_White=0;//ȫ����λ��_�б�
uint8 Black_Cnt=0;//ȫ���м���
uint8 Black_cnt=0;//lianxu
uint8 White_cnt=0;//ȫ���м���_������
uint8 White_Cnt=0;//ȫ���м���_���е�
uint8 White_Ren=0;

uint8 Right_xian=0;
uint8 Left_xian=0;

uint16 Servo_value=8808;//������pwmֵ
uint8 ring_num;

uint8 Hinder_Start=0;
uint8 Hinder_Flag=0;

uint8 Cross_Flag=0;
uint8 Change_Flag;
uint8 CrossRow=0;

uint16 Servomiddle=8775;
uint16 Servomiddle_rember=8770;
uint16 Servo_max=8938;
uint16 Servo_min=8612;
float CenterLineSlope=0;

int16 error=0;   //0~40����
int32 error1=0;  //
int32 error2=0;  //
int16 errorerror=0; //0~35����
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
uint8 race[4]= {0}; //������������
uint8 buff[4]= {0}; //������ջ�����buff[32]
uint8 Overtake_Flag=0;
int16 Servo_temp=0;
uint8 End_zuo=0;
uint8 End_you=0;
uint8 Left_addwidth[ROW_MAX]= {0}; //������߼ӿ�����־
uint8 Right_addwidth[ROW_MAX]= {0}; //�����ұ߼ӿ�����־
uint8 Left_c_in=0;
uint8 Left_c_out=0;
uint8 Right_c_in=0;
uint8 Right_c_out=0;
uint8 Left_r_in=0;
uint8 Left_r_out=0;
uint8 Right_r_in=0;
uint8 Right_r_out=0;
//uint8 Overtake2=0;
uint16 L_Cnt=0;
uint8 C=0;
uint8 Turn_Left=0;
uint8 Left_IPcnt=0;
//////////////////////////
uint8 Kp=0;
uint8 Kd=0;
uint8 Row_Ptr=0;
uint8 Col_Ptr=0;
uint8 Bend_Right = 0;
uint8 Bend_Lift = 0;
float repair_slope_R = 0;/////�ұ��߲���б��
float repair_slope_L = 0;/////����߲���б��
uint8 Ring_First_Row = 0;
uint8 Ring_First_zuo = 0;
uint8 Ring_First_you = 0;
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
uint8 stopLine_temp=0;
uint8 cross_num = 0;
uint8 white_Left_cnt = 0;
uint8 white_Right_cnt = 0;
uint8 Cross_flag =0;
uint8 Cross_Flag_3=0;
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

////����б��
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
  if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
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



//ʮ�ֲ���
uint8 Left_stop=0;
uint8 Right_stop=0;

void Calculate_Slope()
{
  uint8 i=0,j=0;
  uint8 k=0,l=0;
  uint8 Cross_Flag_Last=0;
  Left_stop=10;
  uint8 Left_start=58;  
  Right_stop=10;  
  uint8 Right_start=58;
  float Left_Slope=0.0;
  float Right_Slope=0.0;
  for(Row_Ptr=57;Row_Ptr>6&&Row_Ptr>StopRow;Row_Ptr--)
  {
    if(Road_Left[Row_Ptr-1]<Road_Left[Row_Ptr]&&
       Road_Left[Row_Ptr-2]<=Road_Left[Row_Ptr-1]&&
         Road_Left[Row_Ptr-3]<=Road_Left[Row_Ptr-2])
    {
      Left_start=Row_Ptr;
      break;
    }
  }
  for(Row_Ptr=57;Row_Ptr>6&&Row_Ptr>StopRow;Row_Ptr--)
  {
    if(Road_Right[Row_Ptr-1]>Road_Right[Row_Ptr]&&
       Road_Right[Row_Ptr-2]>=Road_Right[Row_Ptr-1]&&
         Road_Right[Row_Ptr-3]>=Road_Right[Row_Ptr-2])
    {
      Right_start=Row_Ptr;
      break;
    }
  }
  k=Road_Left[Left_start];
  Left_stop=Left_start-15;
  if(Left_start==58)
  {
    for(Row_Ptr=Left_start-4;Row_Ptr>3&&Row_Ptr>All_Black;Row_Ptr--)
    {
      if(abs(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+1])<4&&abs(Road_Left[Row_Ptr+1]-Road_Left[Row_Ptr+2])<4&&Left_Flag[Row_Ptr]==1)
      {
        Left_stop=Row_Ptr;
        k=Road_Left[Row_Ptr];
        break;
      }
    }
  }
  else
  {
    for(Row_Ptr=Left_start-4;Row_Ptr>3&&Row_Ptr>All_Black;Row_Ptr--)
    {    
      
      if(img[Row_Ptr][Road_Left[Left_start]]==0&&img[Row_Ptr-1][Road_Left[Left_start]]==0)
      {
        Left_stop=Row_Ptr-3;
        for(i=Road_Left[Left_start];i<Road_Right[Right_start];i++)
        {
          if(img[Row_Ptr-3][i]==0&&img[Row_Ptr-3][i+1]==0&&img[Row_Ptr-3][i+2]==255&&img[Row_Ptr-3][i+3]==255)
          {
            k=i+2;
            Left_stop=Row_Ptr-3;
            break;
          }
        }
        if(i!=Road_Right[Right_start]) break;
      }
    }
  }
  l=Road_Right[Right_start];
  Right_stop=Right_start-15;
  if(Right_start==58)
  {
    for(Row_Ptr=Left_start-4;Row_Ptr>3&&Row_Ptr>All_Black;Row_Ptr--)
    {
      if(abs(Road_Right[Row_Ptr]-Road_Right[Row_Ptr]+1)<4&&abs(Road_Right[Row_Ptr+1]-Road_Right[Row_Ptr+2])<4&&Right_Flag[Row_Ptr]==1)
      {
        Right_stop=Row_Ptr;
        l=Road_Right[Row_Ptr];
        break;
      }
    }
  }
  else
  {
    
    for(Row_Ptr=Right_start-4;Row_Ptr>3&&Row_Ptr>All_Black;Row_Ptr--)
    {
      
      if(img[Row_Ptr][Road_Right[Right_start]]==0&&img[Row_Ptr-1][Road_Right[Right_start]]==0)
      {
        
        Right_stop=Row_Ptr-3;
        for(i=Road_Right[Right_start];i>Road_Left[Left_start];i--)
        {
          if(img[Row_Ptr-3][i]==0&&img[Row_Ptr-3][i-1]==0&&img[Row_Ptr-3][i-2]==255&&img[Row_Ptr-3][i-3]==255)
          {
            l=i-2;
            Right_stop=Row_Ptr-3;
            break;
          }
        }
        if(i!=Road_Left[Left_start]) break;
      }
      
    }
  }

  Left_Slope=1.0*(k-Road_Left[Left_start])/(Left_start-Left_stop);
  Right_Slope=1.0*(Road_Right[Right_start]-l)/(Right_start-Right_stop);
  j=0;
  for(i=Left_start-1;i>Left_stop-1;i--)
  {
    j++;
    Road_Left[i]=(uint8)(Road_Left[Left_start]+Left_Slope*j+0.5);
  }
  for(i=Right_start-1,j=0;i>Right_stop-1;i--)
  {
    j++;
    Road_Right[i]=(uint8)(Road_Right[Right_start]-Right_Slope*j+0.5);
  }
  j=(Road_Left[Left_start]+Road_Right[Right_start])/2;
  i=(End_zuo+End_you)/2;
  if(Cross_Flag_3==1&&((abs(i-j)>10)||ring_num==0)) Cross_Flag_3=2;

}
/*
�������
*/
void Servo_control(void)
{
  if(Cross_Flag_Last==31&&Cross_Flag!=31)
  {
    ring_time++;
    cross_Time=0;
  }
  if(ring_time>0)
  {
    Cross_Flag=31;
    ring_time++;
  }
  
  if(Ring_First_Row==0&&ring_time>0)
  {
    ring_time=0;
    Cross_Flag=0;
  }
  
  if((Cross_Flag_Last==2||Cross_Flag_Last==4)&&(Cross_Flag!=2&&Cross_Flag!=4))
    cross_time++;
  if(cross_time!=0&&abs(error)<5)
   cross_time=0;
  
  if(Cross_flag>1)
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
  
  ///���ϰ�
  if(Cross_Flag==5)
  {
    Servomiddle=8853;
  }
  else if(Cross_Flag==6)
  {
    Servomiddle=8690;
  }
  else if(stopLine_temp==0)
  {
    Servomiddle=Servomiddle_rember;
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
    if(errorerror*error<0/*&&abs(error-errorerror)>15*/&&Cross_Flag==0)
    {
//      errorerror= - errorerror*19/2;
//      error = error;
    }
    
//    if(Cross_Flag==2||cross_time>0)
//    {
//      Kp =88;
//      Servo_temp=Kp*error/10+80;
//    }
//    else if(Cross_Flag==4||cross_time>0)
//    {
//      Kp =88;
//      Servo_temp=Kp*error/10-110;
//    }
    //else if(Cross_Flag==3||Cross_Flag==31||ring_time>0
    /*else if(Cross_Flag==31&&Ring_First_Row>10)//ԽСת��Խ��
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
      
    }*/
    else if(Cross_Flag==1)
    {
      Kp =70;
      Kd = 35;
      Servo_temp = Kp*error+Kd*errorerror;
      Servo_temp = Servo_temp/10;
    }
    else if(Stop_Flag==2||Stop_Flag==21)
    {
      Kp =40;
      Kd = 40;
      Servo_temp = Kp*error/10+Kd*errorerror/10;
      
    }
    else
    {      
      if(All_Black==0)
    {
      if(error<0)               //��ת
      {
        Kp = 40;
        Kd = 12;
      }
      else
      {
        Kp = 39;
        Kd = 12;
      }
    }
    else if(All_Black<12)       //��ֱ�������
    {
      if(error<0)
      {
        Kp = 39;
        Kd = 8;
      }
      else
      {
        Kp = 39;
        Kd = 8;
      }
    }
    else if(All_Black<16)       //ֱ�����������270��ʱ��ǰת��
    {
      if(error<0)               //��ת
      {
        Kp = 38;
        Kd = 14;
      }
      else
      {
        Kp = 38;
        Kd = 12;
      }
      
    }
    else if(All_Black<21)       //�����ֱ����ʱ��
    {
      if(error<0)
      {
        Kp = 33;
        Kd = 12;
      }
      else   //��ת
      {
        Kp = 33;
        Kd = 12;
      }
      
    }
    else if(All_Black<23)       //�����ֱ����ʱ��
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
    
    else if(All_Black<25)       //����ڲ�
    {
      if(error<0)
      {
        Kp=44;
        Kd=20;
      }
      else
      {
        Kp=44;
        Kd =20;
      }
    }
    else if(All_Black<32)       //��������
    {
      if(error<0)
      {
        Kp=48;
        Kd=20;
      }
      else
      {
        Kp=48;
        Kd =20;
      }
    }
    else if(All_Black<36)       //��������
    {
      if(error<0)
      {
        Kp = 53;
        Kd = 29;
      }
      else
      {
        Kp = 53;
        Kd = 29;
      }
    }
    else if(All_Black<41)
    {
      if(error<0)
      {
        Kp = 58;
        Kd = 30;
      }
      else
      {
        Kp = 58;
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

//�˲�
uint8 GetMedianNum(uint8 *Array,uint8 FilterLen)
{
  uint8 i,j;// ѭ������
  uint8 Temp;
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
//�����˲�
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
//Ѱ����
void Find_Middle()
{
  Row_Ptr=0;
  uint8 Left_J=0;
  uint8 Left_Y=0;
  uint8 Left_left=0;
  uint8 CutLen=8;
  uint8 Cut=0;
  uint8 Edge[3]= {0};
  uint8 CutPos=0;//���߶Ͽ�λ��
  uint8 Var=0;
  Road_area=0;
  cross_num =0;
  FirstBlackinCenter=0;
  //Overtake=0;  
  int8 k1=0;
  int8 k2=0;
  int8 k3=0;
  int8 k4=0;
  uint8 i=0,j=0;
  float Right_Slope=0.0;
  //
  Out_Right=0;
  //Out_Left=0;
  /////
  uint8 repair_R[60]= {0}; /////�ұ߶�ʧ����б�ʵ�����
  uint8 repair_L[60]= {0}; /////��߶�ʧ����б�ʵ�����
  repair_slope_R=0;
  repair_slope_L=0;
  /////
  //filter_Middle(Road_Center);
  
  //б��ʮ���ж�
  if(Cross_Flag==1&&StopRow>All_Black&&Cross_Flag_Last!=3)
  {
    Calculate_Slope();
    if(Cross_Cnt==0) Cross_Cnt=1;//һ��ʮ��·��
    else if(Cross_Cnt==1) Cross_Cnt=2;
    else if(Cross_Cnt>2&&Cross_Cnt<5) Cross_Cnt=1;
  }
  else if(Cross_Flag==0)
  {
    if(Cross_Cnt==2) Cross_Cnt=3;//��һ��ʮ��·�ڽ���
    else if(Cross_Cnt==3) Cross_Cnt=4;
    //else if(Cross_Cnt==4) Cross_Cnt=5;
  }
  //if(Cross_Cnt==5&&error>10) Cross_Cnt=6;//��ת
  //else if(Cross_Cnt==5&&error<-10) Cross_Cnt=7;//��ת
  //if(Cross_Cnt==4)  
  //{
  if(Cross_Flag_Last!=31)
  {
    for(Row_Ptr=55;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Left_Flag[Row_Ptr]==1&&Left_Flag[Row_Ptr-6]==1&&Left_Flag[Row_Ptr-12]==1)
      {
        k1 = Slope_Calculate(Row_Ptr-6,Row_Ptr,(uint8*)Road_Left);
        k2 = Slope_Calculate(Row_Ptr-12,Row_Ptr-6,(uint8*)Road_Left);
        
        if(k1*k2<0)
        {
          
          cross_num = Row_Ptr;
          All_Black=Row_Ptr;
          
          break;
        }
      }
    }
    
    for(Row_Ptr=55;Row_Ptr>All_Black;Row_Ptr--)
    {
      if(Right_Flag[Row_Ptr]==1&&Right_Flag[Row_Ptr-6]==1&&Right_Flag[Row_Ptr-12]==1)
      {
        k3 = Slope_Calculate(Row_Ptr-6,Row_Ptr,(uint8*)Road_Right);
        k4 = Slope_Calculate(Row_Ptr-12,Row_Ptr-6,(uint8*)Road_Right);
        if(k3*k4<0)
        {
          
          cross_num = Row_Ptr;
          All_Black=Row_Ptr;
          
          break;
        }
      }
    }
    if(Cross_Cnt>0&&All_Black<3)
    {      
      Cross_Cnt=0;
    }
  }
  /////////////////////////////////////////////////////////////
  if(Cross_Flag_Last==31) //Բ�������ߣ���ת
  {
    if(Right_xian==0) Right_xian=57;
    Right_Slope=1.0*(Road_Right[Right_xian]-End_zuo)/(Right_xian-Ring_First_zuo);
    for(i=Right_xian-1,j=0;i>Ring_First_Row;i--)
    {
      j++;
      Road_Right[i]=(uint8)(Road_Right[Right_xian]-Right_Slope*j+0.5);
    }
  }
  //}
  /*if(Cross_Flag_Last==32) //Բ�������ߣ���ת
  {
    if(Left_xian==0) Left_xian=57;
    Right_Slope=1.0*(Road_Left[Left_xian]-End_you)/(Left_xian-Ring_First_you);
    for(i=Left_xian-1,j=0;i>Ring_First_Row;i--)
    {
      j++;
      Road_Left[i]=(uint8)(Road_Left[Left_xian]-Right_Slope*j+0.5);
    }
  }*/
  //************************//
  //��Բ���ж�
  if(Cross_Flag==31)
  {
    if(Cross3_Cnt==0) Cross3_Cnt=1;//����Բ��
  }
  else if(Cross_Flag_Last!=31)
  {
    if(Cross3_Cnt==1) Cross3_Cnt=2;//����Բ��
  }
  if(Cross3_Cnt==2&&error>10) Cross3_Cnt=7; //��߹�
  if(Cross3_Cnt==7) //��߹�������
  {
    for(Row_Ptr=48;Row_Ptr>3;Row_Ptr--)
    {
      Left_J=0;
      Left_Y=0;
      //��յ�ȷ��
      
      for(Col_Ptr=Row_Ptr+5;Col_Ptr<Row_Ptr+10;Col_Ptr++)
      {
        if(Road_Left[Col_Ptr]<Road_Left[Col_Ptr+1]||Road_Left[Col_Ptr]==0||Road_Left[Col_Ptr+1]==0)
        {
          Left_J=0;
          break;
        }
        else if(Road_Left[Col_Ptr]>Road_Left[Col_Ptr+1]) Left_J=1;
      }
      for(Col_Ptr=Row_Ptr+5;Col_Ptr>Row_Ptr;Col_Ptr--)
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
        
        Cross3_Cnt=8;
        break;
      }
    }
  }
  else if(Cross3_Cnt==8)
  {
    Left_left=0;
    for(Row_Ptr=48;Row_Ptr>3;Row_Ptr--)
    {
      Left_J=0;
      Left_Y=0;
      //��յ�ȷ��
      
      for(Col_Ptr=Row_Ptr+5;Col_Ptr<Row_Ptr+10;Col_Ptr++)
      {
        if(Road_Left[Col_Ptr]<Road_Left[Col_Ptr+1]||Road_Left[Col_Ptr]==0||Road_Left[Col_Ptr+1]==0)
        {
          Left_J=0;
          break;
        }
        else if(Road_Left[Col_Ptr]>Road_Left[Col_Ptr+1]) Left_J=1;
      }
      for(Col_Ptr=Row_Ptr+5;Col_Ptr>Row_Ptr;Col_Ptr--)
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
        break;
      }
    }
    if(Left_left==0) 
    {
      Left_IPcnt++;
    }
    if(Left_IPcnt>2)
    {
      Cross3_Cnt=0;
      Out_Left=1;//��Բ����־
      Left_IPcnt=0;
    }
  }
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
      else if(Left_Flag[Row_Ptr]==1)//�������
      {
        Road_Center[Row_Ptr]=Road_Center[Row_Ptr+1]+(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+1]);
      }
      else if(Right_Flag[Row_Ptr]==1)//�ұ�������ת
      {
        Road_Center[Row_Ptr]=Road_Center[Row_Ptr+1]+(Road_Right[Row_Ptr]-Road_Right[Row_Ptr+1]);
      }
      else
      {
        Road_Center[Row_Ptr]=(Road_Right[Row_Ptr]+Road_Left[Row_Ptr])/2;
      }
    
    ////�ų���������
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
  }//����for
  
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
    img[Row_Ptr][40]=0;
    img[Row_Ptr][40+error]=0;
  }
  //filter_Middle(Road_Center);
  //����for_������
}

//Ѱ����
uint8 a=1,i,j;

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
  uint8 End_zui=0;
  uint8 Left_left=0;
  uint8 Right_right=0;
  uint8 Left_J=0;
  uint8 Left_Y=0;
  uint8 Left_J0=0;
  uint8 Left_J1=0;
  uint8 Left_Y0=0;
  uint8 Left_Y1=0;
  uint8 Right_J=0;
  uint8 Right_Y=0;
  uint8 Right_J0=0;
  uint8 Right_J1=0;
  uint8 Right_Y0=0;
  uint8 Right_Y1=0;
  Cross_flag=0;
  Right_xian=0;
  Left_xian=0;
  a=1;
  uint8 b=79;
  uint8 k;
  uint8 a_f=0,b_f=0,c_f=0;
  uint8 Left_diu=0;
  uint8 Right_diu=0;
  
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
  Cross_Flag_3=0;
  ring_num=0;
  white_Left_cnt = 0;
  white_Right_cnt = 0;
  stopLine_temp=0;
  ///////////////////////
  //ǰ�������߿�ʼ
  for(Row_Ptr=59; Row_Ptr>56; Row_Ptr--)
  {
    Road_Left[Row_Ptr]=0;
    Road_Right[Row_Ptr]=79;
    Left_Flag[Row_Ptr]=0;
    Right_Flag[Row_Ptr]=0;
    Road_Center[Row_Ptr]=0;
    
    Road_Left_f[Row_Ptr]=Road_Left[Row_Ptr];
    Road_Right_f[Row_Ptr]=Road_Right[Row_Ptr];
    
    start_line_num[Row_Ptr] = 0;
      for(Col_Ptr=0;Col_Ptr<75;Col_Ptr++)
      {      
        if(img[Row_Ptr][Col_Ptr]==0 &&img[Row_Ptr][Col_Ptr+1]==0 && img[Row_Ptr][Col_Ptr+2]==0&&
           img[Row_Ptr][Col_Ptr+3]==255&& img[Row_Ptr][Col_Ptr+4]==255&& img[Row_Ptr][Col_Ptr+5]==255)
        {
          start_line_num[Row_Ptr] ++;
        }      
      }
      if(start_line_num[Row_Ptr]>4)
      {
        stop_line_num++;
      }
      if(stop_line_num>=3&&stop_Flag!=1&&Stop_Flag!=0)
      {
        stopLine_temp=1;
        if(sum_time>80)
        {
          Stop_Flag=2;
        }
      }
      else if(stop_line_num>=3&&Stop_Flag==0)
      {
        stopLine_temp=1;
        Stop_Flag=1;
      }
    
    //�ڲ�for��ʼ �����������
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
    }//�����ڲ�for
    //�ڲ�for��ʼ����������
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
    }//�����ڲ�for
    if(Road_Left[Row_Ptr]>Road_Right[Row_Ptr])
    {
      Left_Flag[Row_Ptr]=0;
      Right_Flag[Row_Ptr]=0;
      Road_Left[Row_Ptr]=Road_Left[Row_Ptr+1];
      Road_Right[Row_Ptr]=Road_Right[Row_Ptr+1];
    }
    Road_Left_f[Row_Ptr]=Road_Left[Row_Ptr];
    Road_Right_f[Row_Ptr]=Road_Right[Row_Ptr];
  }//������������ ���for
  Road_Width[59]=Road_Right[59]-Road_Left[59];
  Road_Width[58]=Road_Right[58]-Road_Left[58];
  Road_Width[57]=Road_Right[57]-Road_Left[57];
  //����������
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
    
    Road_Left_f[Row_Ptr]=Road_Left[Row_Ptr];
    Road_Right_f[Row_Ptr]=Road_Right[Row_Ptr];
    //�����Ҽ��������
    
    if(Row_Ptr>6)
    {
      
      start_line_num[Row_Ptr] = 0;
      for(Col_Ptr=0;Col_Ptr<75;Col_Ptr++)
      {      
        if(img[Row_Ptr][Col_Ptr]==0 &&img[Row_Ptr][Col_Ptr+1]==0 &&img[Row_Ptr][Col_Ptr+2]==255 &&img[Row_Ptr][Col_Ptr+3]==255 )
        {
          start_line_num[Row_Ptr] ++;
        }
      }
      if(start_line_num[Row_Ptr]>4)
      {
        stop_line_num++;
      }
      
      if(Car==1)
      {
        if(stop_line_num>=3&&stop_Flag!=1&&Stop_Flag!=0)
        {
          stopLine_temp=1;
          
          if(Stop_Flag==2)
          {
            Stop_Flag=21;
          }
          
          if(sum_time>100&&Stop_Flag!=21)
          {
            Stop_Flag=2;
          }
          
        }
        else if(stop_line_num>=3&&Stop_Flag==0)
        {
          stopLine_temp=1;
          if(Row_Ptr>15)
          {
            Stop_Flag=1;
          }
        }
      }
      else
      {
        if(stop_line_num>=3&&stop_Flag!=1&&Stop_Flag!=0)
        {
          stopLine_temp=1;
          if(sum_time>100&&Row_Ptr>20)
          {
            Stop_Flag=2;
          }
        }
        else if(stop_line_num>=3&&Stop_Flag==0)
        {
          stopLine_temp=1;
          if(Row_Ptr>15)
          {
            Stop_Flag=1;
          }
        }
      }
    }
    
    
    //ȷ�������һ�е��ѷ�Χ
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
    
    if(Col_Ptr>Road_Right[Row_Ptr+1]) Col_Ptr=Road_Right[Row_Ptr+1];//��ֹԽ��
    if(Col_Ptr>76) Col_Ptr=76;
    if(LEnd<3) LEnd=3;//for��ʼ�������
    
    
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
          Left_cnt++;   //ͳ�������ĺ��߼���
        }
        else
        {
          LFlag=1;   //���ҵ���
        }
        break;
      }
    }//�������forѭ��
    if(Col_Ptr==LEnd && img[Row_Ptr][39]==0 && img[Row_Ptr][40]==0 && img[Row_Ptr][41]==0)
      Left_Flag[Row_Ptr]=2;//�����߷�Χ��û�ҵ�
    if(Col_Ptr==LEnd && img[Row_Ptr][39]==255 && img[Row_Ptr][40]==255 && img[Row_Ptr][41]==255)
    {
      Left_Flag[Row_Ptr]=3;//�����߷�Χ��û�ҵ�
    }
    if(Row_Ptr==30&&Col_Ptr==LEnd&&img[Row_Ptr][10]==255) Left_sign=1;
    //ȷ���ұ���һ�е�����λ��
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
    
    
    if(Col_Ptr<Road_Left[Row_Ptr+1]) Col_Ptr=Road_Left[Row_Ptr+1];//��ֹԽ��
    if(Col_Ptr<3) Col_Ptr=3;
    if(REnd>76) REnd=76;//for��ʼ_��
    
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
          Right_cnt++;   //ͳ�������ĺ��߼���
        }
        else
        {
          RFlag=1;   //���ҵ���
        }
        break;
      }
    }//����for_���ұ�
    Road_Left_f[Row_Ptr]=Road_Left[Row_Ptr];
    Road_Right_f[Row_Ptr]=Road_Right[Row_Ptr];
    if(Cross_flag<2)
    {
      if(img[Row_Ptr][Road_Right_f[Row_Ptr+1]]==255&&Road_Right_f[Row_Ptr+1]==79)
      {
        Road_Right_f[Row_Ptr]=Road_Right_f[Row_Ptr+1];
        //Right_Flag[Row_Ptr]=3;
      }
      if(img[Row_Ptr][Road_Left_f[Row_Ptr+1]]==255&&Road_Left_f[Row_Ptr+1]==0)
      {
        Road_Left_f[Row_Ptr]=Road_Left_f[Row_Ptr+1];
        //Left_Flag[Row_Ptr]=3;
      }
    }
    if(Col_Ptr==REnd && img[Row_Ptr][39] ==0 && img[Row_Ptr][40]==0 && img[Row_Ptr][41]==0) Right_Flag[Row_Ptr]=2;//�����߷�Χ��û�ҵ�_ȫ����
    if(Col_Ptr==REnd&& img[Row_Ptr][39]==255 && img[Row_Ptr][40]==255 && img[Row_Ptr][41]==255) Right_Flag[Row_Ptr]=3;//�����߷�Χ��û�ҵ�_ȫ����
    if(Row_Ptr==30&&Col_Ptr==REnd&&img[Row_Ptr][70]==255) Right_sign=1;
    Road_Width[Row_Ptr]=Road_Right[Row_Ptr]-Road_Left[Row_Ptr];
    if((Left_Flag[Row_Ptr]==2 && Right_Flag[Row_Ptr]==2) && (Left_Flag[Row_Ptr+1]==2 && Right_Flag[Row_Ptr+1]==2) &&(Left_Flag[Row_Ptr+2]==2 && Right_Flag[Row_Ptr+2]==2) )
    {
      Black_Cnt++;//ͳ�����е�ȫ����
      if(Black_Cnt==1)
      {
        All_Black=Row_Ptr+2;   //��¼��һ���ҵ�ȫ���е�λ��
      }
    }
    if(Road_Right[Row_Ptr]<=5||Road_Left[Row_Ptr]>=75||
       abs(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+1])>45||
         abs(Road_Right[Row_Ptr]-Road_Right[Row_Ptr+1])>45)
      All_Black=Row_Ptr;
    if(Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3)
    {
      White_Cnt++;////ͳ��ȫ���и���_ʮ��
      if(White_Cnt==1)
      {
        All_White=Row_Ptr;   //��¼��һ���ҵ�ȫ���е�λ��
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
    
  }//������������ж��ߣ��ж�Ϊʮ��
    else if((Left_Flag[Row_Ptr]==3 && Right_Flag[Row_Ptr]==3)&&
    (Left_Flag[Row_Ptr+1]==3 && Right_Flag[Row_Ptr+1]==3)&&
    (Left_Flag[Row_Ptr+2]==3&& Right_Flag[Row_Ptr+2]==3)&&Row_Ptr>57)
    {
    Cross_Flag=2;
    CrossRow = 59;
  }///����ڳ�ͷ�������ж��ߣ�ʮ��·������һ�����
    else*/
    
    if(Row_Ptr<54&&(Left_Flag[Row_Ptr+2]==3 && Right_Flag[Row_Ptr+2]==3)&&
       (Left_Flag[Row_Ptr+3]==3 && Right_Flag[Row_Ptr+3]==3)&&
           (Left_Flag[Row_Ptr+1]==1 || Right_Flag[Row_Ptr+1]==1)&&
             (Left_Flag[Row_Ptr]==1 || Right_Flag[Row_Ptr]==1)&&Row_Ptr>All_Black)
    {
      Cross_Flag_3=1;
    }
    else if(Row_Ptr<50&&(Left_Flag[Row_Ptr+8]==3 && Right_Flag[Row_Ptr+8]==3)&&
       (Left_Flag[Row_Ptr+7]==3 && Right_Flag[Row_Ptr+7]==3)&&
         (Left_Flag[Row_Ptr+6]==3 && Right_Flag[Row_Ptr+6]==3)&&(
                                                             (Left_Flag[Row_Ptr+5]==1 && Right_Flag[Row_Ptr+5]==1)||
                                                               (Left_Flag[Row_Ptr+4]==1 && Right_Flag[Row_Ptr+4]==1)||
                                                                 (Left_Flag[Row_Ptr+3]==1 && Right_Flag[Row_Ptr+3]==1)||
                                                                   (Left_Flag[Row_Ptr+2]==1 && Right_Flag[Row_Ptr+2]==1)||
                                                                     (Left_Flag[Row_Ptr+1]==1 && Right_Flag[Row_Ptr+1]==1)||
                                                                       (Left_Flag[Row_Ptr]==1 && Right_Flag[Row_Ptr]==1))&&Row_Ptr>All_Black)//������к����ж���ǰ���������ҵ��ߣ�ʮ��
    {
      Cross_Flag=1;
      Cross_flag++;
      // Cross_Flag_Last=Cross_Flag;
      if(Left_Flag[Row_Ptr+5]==1 && Right_Flag[Row_Ptr+5]==1)
        StopRow=Row_Ptr+5;
      else if(Left_Flag[Row_Ptr+4]==1 && Right_Flag[Row_Ptr+4]==1)
        StopRow=Row_Ptr+4;
      else if(Left_Flag[Row_Ptr+3]==1 && Right_Flag[Row_Ptr+3]==1)
        StopRow=Row_Ptr+3;
      else if(Left_Flag[Row_Ptr+2]==1 && Right_Flag[Row_Ptr+2]==1)
        StopRow=Row_Ptr+2;
      else if(Left_Flag[Row_Ptr+1]==1 && Right_Flag[Row_Ptr+1]==1)
        StopRow=Row_Ptr+1;
      else if(Left_Flag[Row_Ptr]==1 && Right_Flag[Row_Ptr]==1)
        StopRow=Row_Ptr;
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
    for(Col_Ptr=Road_Left_f[Row_Ptr]; Col_Ptr<Road_Right_f[Row_Ptr]-3; Col_Ptr++)
    {      
      if(ring_flag==0&&img[Row_Ptr][Col_Ptr]==255&&img[Row_Ptr][Col_Ptr+1]==255&&img[Row_Ptr][Col_Ptr+2]==0&&img[Row_Ptr][Col_Ptr+3]==0) 
      {
        ring_flag=1;///�ҵ��ڿ��������
        if(Col_Ptr+2<Ring_width_1)
          Ring_width_1 = Col_Ptr+2;///�ڿ������С����
        a=Col_Ptr+1;///��߽�
      }
      else if(ring_flag==1&&img[Row_Ptr][Col_Ptr]==0&&img[Row_Ptr][Col_Ptr+1]==0&&img[Row_Ptr][Col_Ptr+2]==255&&img[Row_Ptr][Col_Ptr+3]==255) 
      {
        b=Col_Ptr+2;//�ұ߽�
        ring_flag=2;//�ҵ��ڿ��������
        ring_num++;//�ڿ�������1
        if(Col_Ptr+1>Ring_width_2)
          Ring_width_2 = Col_Ptr+1;//�ڿ��ұ����ĵ���
        if(ring_num==1) 
        {
          End_zuo=a;
          End_you=b;
          Ring_First_Row=Row_Ptr;
          Ring_First_zuo=Row_Ptr;
          Ring_First_you=Row_Ptr;          
        }
        else if(ring_num>1&&End_zui==0)
        {
          if(End_zuo>a) 
          {
            End_zuo=a;
            Ring_First_zuo=Row_Ptr;
          }
          else if(End_zuo<a) End_zui=1;
          if(End_you<b) 
          {
            End_you=b;
            Ring_First_you=Row_Ptr;
          }
          else if(End_you>b) End_zui=1;
        }
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
    if(abs(Ring_width_2-Col_Ptr)<3)//�ϰ�
    {     
      //samll_Ring_temp=1;
      //if(/*Ring_width>10&&Stop_Flag!=0&&sum_time>1000&&White_Cnt>3&&*/cross_Time==0&&Stop_Flag!=0&&White_Cnt>3&&sum_time>1000)///���������߲�ʶ��Բ�����ų����������У�sum_time�Ǿ��������߲ż�ʱ
      //{
        //Cross_Flag=3;/////���ΪСԲ��
      //}
      //else 
        if(ring_num>5&&ring_time==0&&Road_type==1&&stopLine_temp==0)
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
    //��Բ��ǰ���߹յ�   
    Left_J=0;
    Left_Y=0;
    Left_J0=0;
    Left_J1=0;
    Left_Y0=0;
    Left_Y1=0;
    if((abs(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+1]))>10&&(abs(Road_Left[Row_Ptr]-Road_Left[Row_Ptr+2]))>10&&Left_diu==0) Left_diu=Row_Ptr;
    if((abs(Road_Right[Row_Ptr+2]-Road_Right[Row_Ptr]))>10&&(abs(Road_Right[Row_Ptr+1]-Road_Right[Row_Ptr]))>10&&Right_diu==0) Right_diu=Row_Ptr;
    if(Left_left!=1&&Row_Ptr<48&&Row_Ptr>(Right_xian-10)&&Row_Ptr>Left_diu) //��յ�ȷ��
    {
      for(Col_Ptr=Row_Ptr+5;Col_Ptr<Row_Ptr+10;Col_Ptr++)
      {
        if(Road_Left[Col_Ptr]<Road_Left[Col_Ptr+1]||Road_Left[Col_Ptr]==0||Road_Left[Col_Ptr+1]==0)
        {
          Left_J0++;
          if(Left_J0>1)
          {
            Left_J=0;
            break;
          }
        }
        else if(Road_Left[Col_Ptr]>Road_Left[Col_Ptr+1])
        {
          Left_J1++;
          if(Left_J1>1)
              Left_J=1;
        }
      }
      for(Col_Ptr=Row_Ptr+5;Col_Ptr>Row_Ptr;Col_Ptr--)
      {
        if(Road_Left[Col_Ptr-1]>Road_Left[Col_Ptr]||Road_Left[Col_Ptr]==0||Road_Left[Col_Ptr-1]==0)
        {
          Left_Y0++;
          if(Left_Y0>1)
          {
             Left_Y=0;
             break;
          }
        }
        else if(Road_Left[Col_Ptr-1]<Road_Left[Col_Ptr]) 
        {
          Left_Y1++;
          if(Left_Y1>1)
          {
            Left_Y=1;
            break;
          }  
        }
      }
      if(Left_J==1&&Left_Y==1&&(abs(Road_Left[Row_Ptr+5]-Road_Left[Row_Ptr+4]))<11&&(abs(Road_Left[Row_Ptr+5]-Road_Left[Row_Ptr+6]))<11)
      {
        Left_left=1;
        Left_xian=Row_Ptr+5;
      }
    }
    Right_J=0;
    Right_Y=0;
    Right_J0=0;
    Right_J1=0;
    Right_Y0=0;
    Right_Y1=0;
    if(Right_right!=1&&Row_Ptr<48&&Row_Ptr>(Left_xian-10)&&Row_Ptr>Right_diu) //�ҹյ�ȷ��
    {
      for(Col_Ptr=Row_Ptr+5;Col_Ptr<Row_Ptr+10;Col_Ptr++)
      {
        if(Road_Right[Col_Ptr]>Road_Right[Col_Ptr+1]||Road_Right[Col_Ptr]==79||Road_Right[Col_Ptr+1]==79)
        {
          Right_J0++;
          if(Right_J0>1)
          {
            Right_J=0;
            break;
          }
          
        }
        else if(Road_Right[Col_Ptr]<Road_Right[Col_Ptr+1]) 
        {
          Right_J1++;
          if(Right_J1>1)
          {
            Right_J=1;
          }         
        }
      }
      for(Col_Ptr=Row_Ptr+5;Col_Ptr>Row_Ptr;Col_Ptr--)
      {
        if(Road_Right[Col_Ptr-1]<Road_Right[Col_Ptr]||Road_Right[Col_Ptr]==79||Road_Right[Col_Ptr-1]==79)
        {
          Right_Y0++;
          if(Right_Y0>1)
          {
            Right_Y=0;
            break;
          }
        }
        else if(Road_Right[Col_Ptr-1]>Road_Right[Col_Ptr]) 
        {
          Right_Y1++;
          if(Right_Y1>1)
          {
            Right_Y=1;
            break;
          }
        }
      }
      if(Right_J==1&&Right_Y==1&&(abs(Road_Right[Row_Ptr+4]-Road_Right[Row_Ptr+5]))<11&&(abs(Road_Right[Row_Ptr+6]-Road_Right[Row_Ptr+5]))<11)
      {
        Right_right=1;
        Right_xian=Row_Ptr+5;
      }
    }
    
      
    /*if(abs(Ring_width_2-Col_Ptr)<3)////�Ӻڿ����������ң����Col_Ptr�ӽ��˺ڿ����ұߣ�˵��Բ�������а׵ģ��ж�ΪԲ��
    {      
        if(ring_num>5)
        {
            if(Ring_width>10&&Stop_Flag!=0&&sum_time>1000)///���������߲�ʶ��Բ�����ų����������У�sum_time�Ǿ��������߲ż�ʱ
            {
              Cross_Flag=3;/////���ΪСԲ��
            }
        }
    }*/
    if(ring_num>1&&Right_right==1&&Left_left==1&&(abs(Right_xian-Left_xian))<10&&Right_xian>Ring_First_Row&&Left_xian>Ring_First_Row&&Ring_First_Row>4)
    {
      for(i=Left_xian;i>Ring_First_Row;i--)
      {
        if(img[i][Road_Left[Left_xian]]==0&&img[i+1][Road_Left[Left_xian]]==0) break;
      }
      for(j=Right_xian;j>Ring_First_Row;j--)
      {
        if(img[j][Road_Right[Right_xian]]==0&&img[j+1][Road_Right[Right_xian]]==0) break;
      }
      if(i==Ring_First_Row&&j==Ring_First_Row)
        Cross_Flag=31;/////���Ϊ��Բ��
    }

    /*if(samll_Ring_temp==1&&cross_Time==0&&(Right_right==1||Left_left==1)&&Stop_Flag!=0&&sum_time>1000)
    {
      Cross_Flag=3;
      ring_time++;
    }*/
    if(Road_Left[Row_Ptr]>Road_Right[Row_Ptr])
    {
      Road_Left[Row_Ptr]=Road_Left[Row_Ptr+1];
      Road_Right[Row_Ptr]=Road_Right[Row_Ptr+1];
    }
  }//����for ��ѭ��
  if(ring_num>1&&Right_right==1&&Left_left==0&&Right_xian>Ring_First_Row) //youб��Բ��
  {
    //max_xian=Right_xian;
    
    for(i=Right_xian-6;i>4&&i>All_Black;i--)
    {
      if(Right_Flag[i+4]!=1&&Right_Flag[i+3]!=1) break;
      if((abs(Road_Right[i+2]-Road_Right[i+1]))>10) break;
      if(Road_Right[i]<=Road_Right[i+1]&&Road_Right[i+2]<=Road_Right[i+3]&&Road_Right[i+3]<Road_Right[i+4]&&Right_Flag[i+4]==1)
      {
        for(j=Right_xian;j>Ring_First_Row;j--)
        {
          if(img[j][Road_Right[Right_xian]]==0&&img[j+1][Road_Right[Right_xian]]==0) break;
        }
        if(j==Ring_First_Row)
        {
          Cross_Flag=31;
          break;
        }
      }
    }
  }
  if(ring_num>1&&Right_right==0&&Left_left==1&&Left_xian>Ring_First_Row) //zuoб��Բ��
  {
    //max_xian=Right_xian;
    
    for(i=Left_xian-6;i>4&&i>All_Black;i--)
    {
      if(Left_Flag[i+4]!=1&&Left_Flag[i+3]!=1) break;
      if(abs(Road_Left[i+1]-Road_Left[i+2])>10) break;
      if(Road_Left[i]>=Road_Left[i+1]&&Road_Left[i+2]>=Road_Left[i+3]&&Road_Left[i+3]>Road_Left[i+4]&&Left_Flag[i+4]==1)
      {
        for(j=Left_xian;j>Ring_First_Row;j--)
        {
          if(img[j][Road_Left[Left_xian]]==0&&img[j+1][Road_Left[Left_xian]]==0) break;
        }
        if(j==Ring_First_Row)
        {
          Cross_Flag=31;
          break;
        }
      }
    }
  }
  // }//����if�ж�
}
//Ѱ�ߺ�������
//���������ж�
uint32 Road_Type_sum_error;
uint8  Road_Type_error;
void Road_Type(void)
{
  Road_type=0;
  Road_Type_sum_error=0;
  uint8 l=(59-All_Black)/4;
  uint8 last_i=59;
  Road_Type_error=0;
  for(int i=55;i>=All_Black;i=i-4)
  {
    Road_Type_sum_error=Road_Type_sum_error+abs(Road_Center[last_i]-Road_Center[i]);
    last_i = i;
  }
  Road_Type_error=Road_Type_sum_error/l;
  if(All_Black<2&&Road_Type_error<2)
    Road_type=1;//ֱ��
  else
    Road_type=2;//��������
}