#include <stdbool.h>
#include <stdint.h>

#include "battery.h"
#include "blackbox.h"
#include "config.h"
#include "control.h"
#include "drv_adc.h"
#include "drv_dshot.h"
#include "drv_led.h"
#include "drv_time.h"
#include "flash.h"
#include "gestures.h"
#include "imu.h"
#include "led.h"
#include "rx.h"
#include "sixaxis.h"

#include "debug.h"

float looptime; // in seconds
uint32_t lastlooptime;
uint32_t used_loop_time;
uint32_t max_used_loop_time;

extern char aux[ AUXNUMBER ];
extern int onground; // control.c

void failloop( int val );

void usermain()
{
	ledoff();
	delay( 1000 );

	time_init();
	pwm_init(); // For legacy reasons it's called pwm even in the Dshot driver.
	sixaxis_init();
	adc_init(); // DMA takes about 1 us once installed. Must be done early, adc is used in battery_init().
	flash_calculate_pid_c_identifier(); // Must be called before flash_load().
	flash_load(); // Must be called before rx_init() for autobind to work.
	rx_init();
	battery_init(); // Must be called before gyro_cal() to send initial battery voltage there.
	gyro_cal();
	imu_init();
	blackbox_init();

	lastlooptime = gettime() - LOOPTIME;

	while ( true ) {
		// Time measurements with ARMCLANG -O3:
		// sixaxis_read(): 11 +2 per gyro filter (contains 10 SPI bitbang time)
		// control(): 23 acro (+3 angle)
		// checkrx(): 17 us worst case for LOOPTIME < 1000; 39 us otherwise [+1 in case of nrf24 scrambling]

		const uint32_t loop_start_time = gettime();
		looptime = ( loop_start_time - lastlooptime ) * 1e-6f;
		lastlooptime = loop_start_time;

		if ( aux[ LEVELMODE ] ) {
			sixaxis_read(); // read gyro and accelerometer data
			imu(); // attitude calculations for level mode
		} else {
			// gyro_read(); // read just gyro data
			sixaxis_read(); // read gyro and accelerometer data for blackbox logging
		}
		control(); // all flight calculations and motors
		blackbox_log();
		battery();
		if ( onground ) {
			gestures(); // check gestures
		}
		process_led_command();
		checkrx(); // receiver function

		// for debug
		used_loop_time = gettime() - loop_start_time;
		if ( used_loop_time > max_used_loop_time ) {
			max_used_loop_time = used_loop_time;
		}

		while ( ( gettime() - loop_start_time ) < LOOPTIME );
	}
}

// 2 - low battery at powerup - if enabled by config
// 3 - radio chip not detected
// 4 - Gyro not found
// 5 - clock, interrupts, systick, bad code
// 6 - flash write error
void failloop( int val )
{
	for ( int i = 0; i <= 3; ++i ) {
		pwm_set( i, 0 );
	}
	while ( true ) {
		for ( int i = 0; i < val; ++i ) {
			ledon();
			delay( 200000 );
			ledoff();
			delay( 200000 );
		}
		delay( 800000 );
	}
}

void HardFault_Handler( void )
{
	failloop( 5 );
}

void MemManage_Handler( void )
{
	failloop( 5 );
}

void BusFault_Handler( void )
{
	failloop( 5 );
}

void UsageFault_Handler( void )
{
	failloop( 5 );
}
