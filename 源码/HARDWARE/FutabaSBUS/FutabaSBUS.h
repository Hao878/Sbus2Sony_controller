#ifndef MBED_FUTABA_SBUS_H
#define MBED_FUTABA_SBUS_H

#include "sys.h"
#include "usart.h"
#include "delay.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

/** create a FutabaSBUS object connected to the specified serial pins
    *
    * &param pin serial tx,rx to connect to
    */
    void FutabaSBUS(void);
    
    /** Read channel(1..16), digital raw data
    *
    * &param raw data from receiver range from 0 to 4096, normal from 352 1696
    */
    int16_t channel(uint8_t ch);
    
    /** Read digital channel(1..2), range 0..1
    *
    * &param range 0..1
    */
    uint8_t digichannel(uint8_t ch);

    /** Set servo position, raw data, range 200..2000?
    *
    * &param raw data 0..2048
    */
    void servo(uint8_t ch, int16_t position);

    /** Set digital channel, 0..1
    *
    * &param range 0..1
    */
    void digiservo(uint8_t ch, uint8_t position);

    /** Read failsafe condition
    *
    * &param 0=no failsafe 1=lost signal 3=failsafe
    */
    uint8_t failsafe(void);

    /** Set logical data passtrough - servo values are ignored, using received data
    *
    * &param bool
    */
    void FutabaSBUS_w_passthrough(u8 mode);

    /** Read logical data passtrough
    *
    * &param bool
    */
    u8 FutabaSBUS_r_passthrough(void);
		
		void update_servos(void);

#endif
