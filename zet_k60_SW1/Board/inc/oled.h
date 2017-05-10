#ifndef __OLED_H__
#define __OLED_H__

//#define LCD_Port     PTB


#define OLED_SCLK_Pin    PTC10    //D0
#define OLED_SDIN_Pin    PTC11      //D1
#define OLED_RST_Pin     PTC12     //RES
#define OLED_DC_Pin      PTC13      //DC
//#define OLED_CS_Pin      PTA6      //CS

#define OLED_SCLK_Clr()   PTXn_T(OLED_SCLK_Pin,OUT)=0//D0
#define OLED_SCLK_Set()   PTXn_T(OLED_SCLK_Pin,OUT)=1

#define OLED_SDIN_Clr()   PTXn_T(OLED_SDIN_Pin,OUT)=0//D1
#define OLED_SDIN_Set()   PTXn_T(OLED_SDIN_Pin,OUT)=1

#define OLED_RST_Clr()    PTXn_T(OLED_RST_Pin,OUT)=0//RES
#define OLED_RST_Set()    PTXn_T(OLED_RST_Pin,OUT)=1

#define OLED_DC_Clr()     PTXn_T(OLED_DC_Pin,OUT)=0//DC
#define OLED_DC_Set()     PTXn_T(OLED_DC_Pin,OUT)=1
 		     
//#define OLED_CS_Clr()     PTXn_T(OLED_CS_Pin,OUT)=0//CS
//#define OLED_CS_Set()     PTXn_T(OLED_CS_Pin,OUT)=1

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
#define SIZE            16

/******常用颜色*****/
#define RED     0XF800    //红色
#define GREEN   0X07E0    //绿色
#define BLUE    0X001F    //蓝色
#define BRED    0XF81F
#define GRED    0XFFE0    //灰色
#define GBLUE   0X07FF    //
#define BLACK   0X0000    //黑色
#define WHITE   0XFFFF    //白色
#define YELLOW  0xFFE0    //黄色

extern void oled_display_on(void);
extern void oled_display_off(void);	   							   		    
extern void oled_init(void);
extern void oled_clear(void);
extern void oled_show_char(uint8 x,uint8 y,uint8 chr,uint8 size,uint8 mode);
extern void oled_show_num(uint8 x,uint8 y,int32 num,uint8 size,uint8 mode);
extern void oled_show_string(uint8 x,uint8 y, uint8 *p,uint8 size,uint8 mode);	
extern void oled_show_logo(void);
extern void oled_show_picture(void);

#endif