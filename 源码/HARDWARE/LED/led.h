#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define LED0 PAout(8)	// PA8
#define LED1 PDout(2)	// PD2

#define PIC PAout(4)	// PA4
#define FOC PAout(5)	// PA5
#define REC PAout(6)	// PA6
#define POWER PAout(7)	// PA7

#define ZOOM_FL PBout(14)	// PB12
#define ZOOM_FF PBout(15)	// PB13
#define ZOOM_NL PBout(12)	// PB14
#define ZOOM_NF PBout(13)	// PAB15

void LED_Init(void);//��ʼ��

		 				    
#endif
