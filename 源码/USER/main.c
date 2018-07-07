#include "led.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
//#include "usart.h"
#include "FutabaSBUS.h"
//ALIENTEK Mini STM32�����巶������9
//���벶��ʵ��   
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾


extern u8  TIM2CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM2CH1_CAPTURE_VAL;	//���벶��ֵ
extern int16_t channels[];
extern uint8_t  failsafe_status;
extern uint8_t sbus_data[25];
extern int16_t servos[18];
extern uint8_t sbus_pointer;
extern u8 sbus_passthrough;


 int main(void)
 {	
	 
//	u32 temp=0; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	
	//uart_init(115200);	 			//115200	 
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	FutabaSBUS();
  while(1)
	{
 		delay_ms(10);
		if((channels[4]>=147)&&(channels[4]<=1902))
		{
			if(channels[4]<=419)
			{
				PIC=1;
				FOC=1;
			}
			if((channels[4]>912)&&(channels[4]<1134))
			{
				PIC=1;
				FOC=0;
			}
			if(channels[4]>=1560)
			{
				PIC=0;
				FOC=0;
			}
		}
		if((channels[5]>=147)&&(channels[5]<=1902))
		{
			if(channels[5]<=912)
			{
				REC=1;
			}
			else
			{
				REC=0;
			}
		}
		if((channels[7]>=147)&&(channels[7]<=1902))
		{
			if(channels[7]<=912)
			{
				POWER=1;
			}
			else
			{
				POWER=0;
			}
		}
		if((channels[3]>=147)&&(channels[3]<=1902))
		{
			if(channels[3]<=419)
			{
				ZOOM_FL=1;
				ZOOM_FF=1;
				ZOOM_NL=0;
				ZOOM_NF=0;
			}
			if((channels[3]>419)&&(channels[3]<=912))
			{
				ZOOM_FL=1;
				ZOOM_FF=1;
				ZOOM_NL=0;
				ZOOM_NF=1;
			}
			if((channels[3]>912)&&(channels[3]<1134))
			{
				ZOOM_FL=1;
				ZOOM_FF=1;
				ZOOM_NL=1;
				ZOOM_NF=1;
			}
			if((channels[3]>=1134)&&(channels[3]<1560))
			{
				ZOOM_FL=0;
				ZOOM_FF=1;
				ZOOM_NL=1;
				ZOOM_NF=1;
			}
			if(channels[3]>=1560)
			{
				ZOOM_FL=0;
				ZOOM_FF=0;
				ZOOM_NL=1;
				ZOOM_NF=1;
			}
		}
		
	  //update_servos();
	}
}
