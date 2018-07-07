#include "FutabaSBUS.h"

uint8_t sbus_data[25] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
int16_t channels[18]  = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
int16_t servos[18]    = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t  failsafe_status = SBUS_SIGNAL_FAILSAFE;
uint8_t sbus_pointer = 0;
u8 sbus_passthrough = 0;
uint8_t sbus_data2[25] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
void FutabaSBUS(void){
	
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = 100000;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1       
}

void USART1_IRQHandler(void)
{
	// Read all received data and calculate channel data
    uint8_t i;
    u8 data;
		uint8_t byte_in_sbus = 1;
		uint8_t bit_in_sbus = 0;
		uint8_t ch = 0;
		uint8_t bit_in_channel = 0;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
		{
				data =USART_ReceiveData(USART1);	//读取接收到的数据
        switch (sbus_pointer) {
            case 0: // Byte 1
                if (data==0x0f) {
                    sbus_data[sbus_pointer] = data;
                    sbus_pointer++;
                }
                break;

            case 24:    // Byte 25 >> if last byte == 0x00 >> convert data
                if (data==0x00) {
                    sbus_data[sbus_pointer] = data;
										
                    // clear channels[]
                    for (i=0; i<16; i++) {channels[i] = 0;}

                    // reset counters
                    byte_in_sbus = 1;
                    bit_in_sbus = 0;
                    ch = 0;
                    bit_in_channel = 0;

                    // process actual sbus data
                    for (i=0; i<176; i++) {
                        if (sbus_data[byte_in_sbus] & (1<<bit_in_sbus)) {
                            channels[ch] |= (1<<bit_in_channel);
                        }
                        bit_in_sbus++;
                        bit_in_channel++;

                        if (bit_in_sbus == 8) {
                            bit_in_sbus =0;
                            byte_in_sbus++;
                        }
                        if (bit_in_channel == 11) {
                            bit_in_channel =0;
                            ch++;
                        }
                    }
                    // DigiChannel 1
                    if (sbus_data[23] & (1<<0)) {
                        channels[16] = 1;
                    }else{
                        channels[16] = 0;
                    }
                    // DigiChannel 2
                    if (sbus_data[23] & (1<<1)) {
                        channels[17] = 1;
                    }else{
                        channels[17] = 0;
                    }
                    // Failsafe
                    failsafe_status = SBUS_SIGNAL_OK;
                    if (sbus_data[23] & (1<<2)) {
                        failsafe_status = SBUS_SIGNAL_LOST;
                    }
                    if (sbus_data[23] & (1<<3)) {
                        failsafe_status = SBUS_SIGNAL_FAILSAFE;
                    }
                }
								else
								{
									delay_us(1500);
								}
								sbus_pointer = 0;
                break;

            default:  // collect Channel data (11bit) / Failsafe information
                sbus_data[sbus_pointer] = data;
                sbus_pointer++;
        }
    }
}
void update_servos(void) {
    // Send data to servos
    // Passtrough mode = false >> send own servo data
    // Passtrough mode = true >> send received channel data
    uint8_t i;
	  // reset counters
		uint8_t ch = 0;
		uint8_t bit_in_servo = 0;
		uint8_t byte_in_sbus = 1;
		uint8_t bit_in_sbus = 0;
    if (!sbus_passthrough) {
        // clear received channel data
        for (i=1; i<24; i++) {
            sbus_data2[i] = 0;
        }
    
        
        
        // store servo data
        for (i=0; i<176; i++) {
            if (channels[ch] & (1<<bit_in_servo)) {
                sbus_data2[byte_in_sbus] |= (1<<bit_in_sbus);
            }
            bit_in_sbus++;
            bit_in_servo++;

            if (bit_in_sbus == 8) {
                bit_in_sbus =0;
                byte_in_sbus++;
            }
            if (bit_in_servo == 11) {
                bit_in_servo =0;
                ch++;
            }
        }
    
        // DigiChannel 1
        if (channels[16] == 1) {
            sbus_data2[23] |= (1<<0);
        }
        // DigiChannel 2
        if (channels[17] == 1) {
            sbus_data2[23] |= (1<<1);
        }        
        
        // Failsafe
        if (failsafe_status == SBUS_SIGNAL_LOST) {
            sbus_data2[23] |= (1<<2);
        }
        
        if (failsafe_status == SBUS_SIGNAL_FAILSAFE) {
            sbus_data2[23] |= (1<<2);
            sbus_data2[23] |= (1<<3);
        }
    }
    // send data out
    for (i=0;i<25;i++) {
//			while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
//			USART2->DR =sbus_data[i];
				while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
				USART1->DR =sbus_data2[i];
    }
}
int16_t FutabaSBUSchannel(uint8_t ch) {
    // Read channel data
    if ((ch>0)&&(ch<=16)){
        return channels[ch-1];
    }else{
        return 1023;
    }
}

uint8_t FutabaSBUSdigichannel(uint8_t ch) {
    // Read digital channel data
    if ((ch>0) && (ch<=2)) {
        return channels[15+ch];
    }else{
        return 0;
    }
}

void FutabaSBUSservo(uint8_t ch, int16_t position) {
    // Set servo position
    if ((ch>0)&&(ch<=16)) {
        if (position>2048) {position=2048;}
        servos[ch-1] = position;
    }
}

void FutabaSBUSdigiservo(uint8_t ch, uint8_t position) {
    // Set digital servo position
    if ((ch>0) && (ch<=2)) {
        if (position>1) {position=1;}
        servos[15+ch] = position;
    }
}

uint8_t FutabaSBUSfailsafe(void) {return failsafe_status;}

void FutabaSBUS_w_passthrough(u8 mode) {
    // Set passtrough mode, if true, received channel data is send to servos
    sbus_passthrough = mode;
}

u8 FutabaSBUS_r_passthrough(void) {
    // Return current passthrough mode
    return sbus_passthrough;
}
