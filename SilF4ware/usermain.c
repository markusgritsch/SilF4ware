#include <stdbool.h>
#include <stdint.h>

#include "battery.h"
#include "blackbox.h"
#include "config.h"
#include "control.h"
#include "drv_adc.h"
#include "drv_dshot.h"
#include "drv_led.h"
#include "drv_reset.h"
#include "drv_time.h"
#include "flash.h"
#include "gestures.h"
#include "imu.h"
#include "led.h"
#include "osd.h"
#include "rx.h"
#include "sixaxis.h"

#include "debug.h"

float looptime; // in seconds
uint32_t lastlooptime;
uint32_t fly_time;
uint32_t used_loop_time;
uint32_t max_used_loop_time;
uint32_t avg_used_loop_time;
uint32_t most_frequently_used_loop_time;
bool telemetry_transmitted;
bool packet_received;
static uint32_t loops_since_last_packet = 1000;

extern char aux[ AUXNUMBER ];
extern int onground; // control.c
extern float vbattfilt; // battery.c

void usermain()
{
	ledoff();

	time_init();
	pwm_init(); // For legacy reasons it's called pwm even in the Dshot driver.
	sixaxis_init();
	adc_init(); // DMA takes about 1 us once installed. Must be done early, adc is used in battery_init().
	flash_calculate_pid_c_identifier(); // Must be called before flash_load().
	flash_load(); // Must be called before rx_init() for autobind to work.
	rx_init();
	battery_init(); // Must be called before gyro_cal() to send initial battery voltage there.
	gyro_cal_check_flash();
	// imu_init(); Not really necessary since the gravity vector in brought in sync with accel values in imu() all the time.
	blackbox_init();
#ifdef OSD_ENABLE
	osd_init();
#endif // OSD_ENABLE

#ifdef AUTO_BOOTLOADER
	if ( vbattfilt < 1.0f ) {
		jump_to_bootloader();
	}
#endif // AUTO_BOOTLOADER

	lastlooptime = gettime() - LOOPTIME;
	while ( true ) { // main loop
		// Time measurements with ARMCLANG -O3:
		// sixaxis_read(): 11 +2 per gyro filter (contains 10 SPI bitbang time)
		// control(): 23 acro (+3 angle)
		// checkrx(): 17 us worst case for LOOPTIME < 1000; 39 us otherwise [+1 in case of nrf24 scrambling]

		const uint32_t loop_start_time = gettime();
		looptime = ( loop_start_time - lastlooptime ) * 1e-6f;
		lastlooptime = loop_start_time;
		++loops_since_last_packet;

		blackbox_log(); // First thing in the loop to get equal spacing between the calls.

		if ( loops_since_last_packet == 2 ) { // In this loops_since_last_packet we have enough time to waste, since telemetry
			// gets transmitted and therefore sending motor values is omitted. This reads the accel data only once every 5 ms.
			sixaxis_read(); // read gyro (and accelerometer data for blackbox logging)
		} else {
			gyro_read(); // read just gyro data
		}
		// Gyro filtering is done in sixaxis.c, i.e., at the end of gyro_read() and sixaxis_read().

#ifdef LEVELMODE
		imu(); // attitude calculations for level mode
#endif // LEVELMODE

		telemetry_transmitted = false;
		packet_received = checkrx(); // receiver function (This sets telemetry_transmitted = true in case telemetry was transmitted)
		if ( packet_received ) {
			loops_since_last_packet = 0;
		}

		const bool send_motor_values = ! telemetry_transmitted; // Skip to not interfere with sending telemetry.
		control( send_motor_values ); // all flight calculations, pid and motors

		battery();

		if ( onground ) {
			gestures(); // check gestures
		}

		process_led_command();

#ifdef OSD_ENABLE
		osd();
#endif // OSD_ENABLE

		if ( ! onground ) {
			fly_time += LOOPTIME;
		}

		// max_used_loop_time (for debug)
		used_loop_time = gettime() - loop_start_time;
		if ( used_loop_time > max_used_loop_time ) {
			max_used_loop_time = used_loop_time;
		}

		// avg_used_loop_time (for debug)
		static uint32_t cumulated_used_loop_time, cumulated_used_loop_time_counter;
		cumulated_used_loop_time += used_loop_time;
		++cumulated_used_loop_time_counter;
		if ( cumulated_used_loop_time_counter > 1024 ) { // 0.256 second at 4k loop frequency
			cumulated_used_loop_time /= 1024;
			if ( cumulated_used_loop_time > avg_used_loop_time ) {
				avg_used_loop_time = cumulated_used_loop_time;
			}
			cumulated_used_loop_time = 0;
			cumulated_used_loop_time_counter = 0;
		}

//#define LOOP_TIME_STATS
#ifdef LOOP_TIME_STATS
		// determines most_frequently_used_loop_time (for debug)
		static uint32_t loop_count;
		static uint32_t loop_time_stats[ 256 ];
		if ( used_loop_time > 255 ) {
			used_loop_time = 255;
		}
		++loop_time_stats[ used_loop_time ];
		++loop_count;
		if ( loop_count == 100 ) {
			loop_count = 0;
			uint32_t max_value = 0;
			for ( int i = 0; i < 256; ++i ) {
				if ( loop_time_stats[ i ] > max_value ) {
					max_value = loop_time_stats[ i ];
					most_frequently_used_loop_time = i;
				}
				loop_time_stats[ i ] = 0;
			}
		}
#endif // LOOP_TIME_STATS

		static uint32_t next_loop_start = 0;
		if ( next_loop_start == 0 ) {
			next_loop_start = loop_start_time;
		}
		static float correction = 0;
		correction += LOOPTIME / (float)WALLTIME_CORRECTION_FACTOR;
		next_loop_start += (uint32_t)correction;
		correction -= (uint32_t)correction;
		while ( gettime() < next_loop_start );
	}
}

// 2 - low battery at powerup - if enabled by config
// 3 - radio chip not detected
// 4 - Gyro not found
// 5 - clock, interrupts, systick, bad code
// 6 - flash write error
// 7 - ESC pins on more than two distinct GPIO ports
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
