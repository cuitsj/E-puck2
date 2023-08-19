#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "serial_comm.h"

///////////////////////////////////////////////
#include "motors.h"
#include "sensors/proximity.h"
#include "audio/play_melody.h"

//////////////////////////////////////////////

///////////////////自己添加///////////////////////
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


///////////////////////////////////////////


static THD_WORKING_AREA(comm_thd_wa, 1024);

static THD_FUNCTION(comm_thd, arg) {
    (void) arg;
    chRegSetThreadName(__FUNCTION__);
    uint8_t recvBuff[4] = {0};
    uint8_t startBuff[4] = {0};
    uint8_t txBuff[12] = {0};
    uint8_t txBuff1[2] = {0};
    uint8_t BT_RX = 0;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    systime_t time;
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	int16_t prox_values_temp[8];
	int16_t leftSpeed = 0, rightSpeed = 0;
    uint8_t rgb_state = 0, rgb_counter = 0;
    uint16_t melody_state = 0, melody_counter = 0;
    uint8_t  start_flag = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(1) {
    	time = chVTGetSystemTime();
    	switch(get_selector())
    	{
    		case 0:
    			/***********************************************************小车自己避障********************************************************************/
    			messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    				prox_values_temp[0] = get_calibrated_prox(0);
    				prox_values_temp[1] = get_calibrated_prox(1);
    				prox_values_temp[2] = get_calibrated_prox(2);
    				prox_values_temp[5] = get_calibrated_prox(5);
    				prox_values_temp[6] = get_calibrated_prox(6);
    				prox_values_temp[7] = get_calibrated_prox(7);

    				txBuff1[0] = prox_values_temp[0]&0xff;
					txBuff1[1] = prox_values_temp[0]>>8;

					//发送红外传感器数据给FPGA
					chSequentialStreamWrite(&SD3, txBuff1, 2); // Blocking write

    				leftSpeed = MOTOR_SPEED_LIMIT/2 - prox_values_temp[0]*8 - prox_values_temp[1]*4 - prox_values_temp[2]*2 + prox_values_temp[7]*8 + prox_values_temp[6]*4 + prox_values_temp[5]*2;
    				rightSpeed = MOTOR_SPEED_LIMIT/2 - prox_values_temp[7]*8 - prox_values_temp[6]*4 - prox_values_temp[5]*2 + prox_values_temp[0]*8 + prox_values_temp[1]*4 + prox_values_temp[2]*2;
    				right_motor_set_speed(rightSpeed);
    				left_motor_set_speed(leftSpeed);

    				switch(rgb_state) {
    					case 0: // Red.
    						set_rgb_led(0, 10, 0, 0);
    						set_rgb_led(1, 10, 0, 0);
    						set_rgb_led(2, 10, 0, 0);
    						set_rgb_led(3, 10, 0, 0);
    						break;
    					case 1: // Green.
    						set_rgb_led(0, 0, 10, 0);
    						set_rgb_led(1, 0, 10, 0);
    						set_rgb_led(2, 0, 10, 0);
    						set_rgb_led(3, 0, 10, 0);
    						break;
    					case 2: // Blue.
    						set_rgb_led(0, 0, 0, 10);
    						set_rgb_led(1, 0, 0, 10);
    						set_rgb_led(2, 0, 0, 10);
    						set_rgb_led(3, 0, 0, 10);
    						break;
    				}
    				rgb_counter++;
    				if(rgb_counter == 100) {
    					rgb_counter = 0;
    					rgb_state = (rgb_state+1)%3;
    					set_body_led(2);
    					set_front_led(2);
    				}

    				melody_counter++;
    				if(melody_counter == 2000) {
    					melody_counter = 0;
    					melody_state = (melody_state+1)%NB_SONGS;
    					playMelody(melody_state, ML_SIMPLE_PLAY, NULL);
    				}

    				chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    			/***********************************************************小车自己避障********************************************************************/
    		break;
    		case 8:
				/***********************************************************小车通过FPGA控制避障********************************************************************/
    			messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    			if (start_flag == 0)
    			{
    				chSequentialStreamRead(&SD3, startBuff, 4); // Blocking read
    				start_flag=1;
    			}
    			//ADC读红外数据
				prox_values_temp[0] = get_calibrated_prox(0);
				prox_values_temp[1] = get_calibrated_prox(1);
				prox_values_temp[2] = get_calibrated_prox(2);
				prox_values_temp[5] = get_calibrated_prox(5);
				prox_values_temp[6] = get_calibrated_prox(6);
				prox_values_temp[7] = get_calibrated_prox(7);

				txBuff[0] = prox_values_temp[0]&0xff;
				txBuff[1] = prox_values_temp[0]>>8;

				txBuff[2] = prox_values_temp[1]&0xff;
				txBuff[3] = prox_values_temp[1]>>8;

				txBuff[4] = prox_values_temp[2]&0xff;
				txBuff[5] = prox_values_temp[2]>>8;

				txBuff[6] = prox_values_temp[5]&0xff;
				txBuff[7] = prox_values_temp[5]>>8;

				txBuff[8] = prox_values_temp[6]&0xff;
				txBuff[9] = prox_values_temp[6]>>8;

				txBuff[10] = prox_values_temp[7]&0xff;
				txBuff[11] = prox_values_temp[7]>>8;

				//发送红外传感器数据给FPGA
				chSequentialStreamWrite(&SD3, txBuff, 12); // Blocking write

    			//等待接收FPGA的数据
    			chSequentialStreamRead(&SD3, recvBuff, 4); // Blocking read
    			leftSpeed = (recvBuff[1]<<8)|recvBuff[0];
    			rightSpeed = (recvBuff[3]<<8)|recvBuff[2];
//				leftSpeed = MOTOR_SPEED_LIMIT/2 - prox_values_temp[0]*8 - prox_values_temp[1]*4 - prox_values_temp[2]*2;
//				rightSpeed = MOTOR_SPEED_LIMIT/2 - prox_values_temp[7]*8 - prox_values_temp[6]*4 - prox_values_temp[5]*2;
    			if (leftSpeed >= MOTOR_SPEED_LIMIT)
    			{
    				leftSpeed = MOTOR_SPEED_LIMIT;
    			}
    			else if (leftSpeed <= -MOTOR_SPEED_LIMIT)
    			{
    				leftSpeed = -MOTOR_SPEED_LIMIT;
    			}
    			if (rightSpeed >= MOTOR_SPEED_LIMIT)
				{
    				rightSpeed = MOTOR_SPEED_LIMIT;
				}
				else if (rightSpeed <= -MOTOR_SPEED_LIMIT)
				{
					rightSpeed = -MOTOR_SPEED_LIMIT;
				}
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);




				//led动画
				switch(rgb_state) {
					case 0: // Red.
						set_rgb_led(0, 10, 0, 0);
						set_rgb_led(1, 10, 0, 0);
						set_rgb_led(2, 10, 0, 0);
						set_rgb_led(3, 10, 0, 0);
						break;
					case 1: // Green.
						set_rgb_led(0, 0, 10, 0);
						set_rgb_led(1, 0, 10, 0);
						set_rgb_led(2, 0, 10, 0);
						set_rgb_led(3, 0, 10, 0);
						break;
					case 2: // Blue.
						set_rgb_led(0, 0, 0, 10);
						set_rgb_led(1, 0, 0, 10);
						set_rgb_led(2, 0, 0, 10);
						set_rgb_led(3, 0, 0, 10);
						break;
				}
				rgb_counter++;
				if(rgb_counter == 100) {
					rgb_counter = 0;
					rgb_state = (rgb_state+1)%3;
					set_body_led(2);
					set_front_led(2);
				}

				melody_counter++;
				if(melody_counter == 2000) {
					melody_counter = 0;
					melody_state = (melody_state+1)%NB_SONGS;
					playMelody(melody_state, ML_SIMPLE_PLAY, NULL);
				}

				//等待10ms
				chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
				//		chThdSleepUntilWindowed(time, time + MS2ST(20)); // Refresh @ 50 Hz.
				//		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
				/***********************************************************小车通过FPGA控制避障********************************************************************/
			break;
    		default:
    			/***********************************************************蓝牙串口回环实验********************************************************************/
    			chSequentialStreamRead(&SD3, &BT_RX, 1); // Blocking read
				//chnReadTimeout(&SD3, &recvBuff, 1, MS2ST(10)); // Non-blocking read (timeout 10 ms)
				set_body_led(2);
				chSequentialStreamWrite(&SD3, &BT_RX, 1); // Blocking write
				//chnWriteTimeout(&SD3, &recvBuff, 1, MS2ST(10)); // Non-blocking write (timeout 10 ms)
    			/***********************************************************蓝牙串口回环实验********************************************************************/
    		break;
    	}
    }
}





int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    motors_init();
    proximity_start();
    serial_start();

    chThdCreateStatic(comm_thd_wa, sizeof(comm_thd_wa), NORMALPRIO, comm_thd, NULL);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
