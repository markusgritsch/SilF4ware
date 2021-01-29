#include <math.h> // fabsf
#include <stdbool.h>
#include <stdlib.h> // abs

#include "config.h"
#include "drv_reset.h"
#include "drv_time.h"
#include "fft.h"
#include "flash.h"
#include "gesture_detect.h"
#include "gestures.h"
#include "pid.h"
#include "rx.h"
#include "sixaxis.h"
#include "util.h"

extern int onground; // control.c
extern bool beep_motors_once; // control.c
extern bool ledcommand; // led.c
extern int ledblink; // led.c
extern char aux[ AUXNUMBER ];

int rx_bind_enable;
static int skip_accel_cal_on_save = 0;

void gestures( void )
{
	if ( ! onground ) {
		return;
	}

	const int command = gesture_detect();

	if ( command != GESTURE_NONE ) {

		if ( command == GESTURE_DDD ) {
			//skip accel calibration if pid gestures used
			if ( ! skip_accel_cal_on_save ) {
				gyro_cal(); // for flashing lights
				acc_cal();
			} else {
				ledcommand = true;
				skip_accel_cal_on_save = 0;
			}
			flash_save();
#if 1
			perform_system_reset();
#else
			flash_load();
#ifdef PID_GESTURE_TUNING
			// reset flash numbers
			extern int number_of_increments[ 3 ][ 3 ];
			for ( int i = 0; i < 3; ++i ) {
				for ( int j = 0; j < 3; ++j ) {
					number_of_increments[ i ][ j ] = 0;
				}
			}
#endif
			// reset loop time
			extern uint32_t lastlooptime;
			lastlooptime = gettime();
#endif
		}

#if ( defined RX_XN297_BAYANG_TELEMETRY || defined RX_NRF24_BAYANG_TELEMETRY )
		if ( command == GESTURE_UUU ) {
			rx_bind_enable = ! rx_bind_enable;
			if ( rx_bind_enable ) {
				ledblink = 1; // blink one time if enabled
			} else {
				ledcommand = true; // flash a few times if disabled
			}
			skip_accel_cal_on_save = 1;
		}
#endif

		if ( command == GESTURE_LLU ) {
			aux[ CH_AUX1 ] = 1;
			ledblink = 1;
			beep_motors_once = true;
		}

		if ( command == GESTURE_LLD ) {
			aux[ CH_AUX1 ] = 0;
			ledcommand = true;
			beep_motors_once = true;
		}

		if ( command == GESTURE_RRU ) {
			aux[ CH_AUX2 ] = 1;
			ledblink = 1;
			beep_motors_once = true;
		}

		if ( command == GESTURE_RRD ) {
			aux[ CH_AUX2 ] = 0;
			ledcommand = true;
			beep_motors_once = true;
		}

#ifdef PID_GESTURE_TUNING
		if ( command == GESTURE_UDU ) {
			ledblink = next_pid_axis(); // Cycle to next axis (Roll, Pitch, Yaw)
		}

		if ( command == GESTURE_UDD ) {
			ledblink = next_pid_term(); // Cycle to next pid term (P, I, D)
		}

		if ( command == GESTURE_UDR ) {
			ledblink = increase_pid(); // Increase by 10%
			skip_accel_cal_on_save = 1;
		}

		if ( command == GESTURE_UDL ) {
			ledblink = decrease_pid(); // Descrease by 10%
			skip_accel_cal_on_save = 1;
		}

		// flash long on zero
		if ( skip_accel_cal_on_save && ledblink == 0 ) {
			ledcommand = true;
		}
#endif // PID_GESTURE_TUNING

		if ( command == GESTURE_LRU ) {
			perform_system_reset();
		}

		if ( command == GESTURE_LRD ) {
			jump_to_bootloader();
		}

#ifdef BIQUAD_AUTO_NOTCH
		if ( command == GESTURE_RRR ) {
			extern float auto_notch_Hz;
			extern float gyro_array[ 2 ][ FFT_SIZE ];
			extern uint32_t gyro_array_index;
			auto_notch_Hz = 0.0f;
			const int bin_index_roll = get_max_amplitude_index( gyro_array[ 0 ], gyro_array_index );
			const int bin_index_pitch = get_max_amplitude_index( gyro_array[ 1 ], gyro_array_index );
			if ( abs( bin_index_roll - bin_index_pitch ) <= 1 ) { // Only if in the same or neighboring bin.
				const float fftHz_roll = bin_index_roll / ( LOOPTIME * 1e-6f ) / (float)FFT_SIZE;
				const float fftHz_pitch = bin_index_pitch / ( LOOPTIME * 1e-6f ) / (float)FFT_SIZE;
				const float fftHz_average = ( fftHz_roll + fftHz_pitch ) / 2.0f;
				if ( fftHz_average > 100.0f && fftHz_average < 1000.0f ) {
					auto_notch_Hz = fftHz_average;
					ledblink = 1;
				}
			}
			if ( auto_notch_Hz == 0.0f ) {
				ledcommand = true;
			}
			skip_accel_cal_on_save = 1;
			notify_telemetry_value( auto_notch_Hz );
		}
#endif // BIQUAD_AUTO_NOTCH
	}

#ifdef PID_STICK_TUNING
	static unsigned long next_update_time = 0;
	const unsigned long time = gettime();
	if ( time > next_update_time ) {
		bool is_stick_tuning_active = false;
		extern float rx[ 4 ];
		#define STICK_DEAD_ZONE 0.75f
		if ( rx[ 0 ] < -STICK_DEAD_ZONE && rx[ 1 ] > STICK_DEAD_ZONE ) { // left + front
			set_current_pid_term( 0 ); // P
			is_stick_tuning_active = true;
		} else if ( rx[ 0 ] > STICK_DEAD_ZONE && rx[ 1 ] > STICK_DEAD_ZONE ) { // right + front
			set_current_pid_term( 1 ); // I
			is_stick_tuning_active = true;
		} else if ( rx[ 0 ] > STICK_DEAD_ZONE && rx[ 1 ] < -STICK_DEAD_ZONE ) { // right + back
			set_current_pid_term( 2 ); // D
			is_stick_tuning_active = true;
		} else if ( rx[ 0 ] < -STICK_DEAD_ZONE && rx[ 1 ] < -STICK_DEAD_ZONE ) { // left + back
			next_pid_axis();
			next_update_time = time + 500000; // 0.5 seconds
		}
		if ( is_stick_tuning_active ) {
			const float tuning_dead_band = 0.1f; // 10% deadband
			if ( fabsf( rx[ 2 ] ) > tuning_dead_band ) {
				const float multiplier = 1.01f; // 1% change per update
				multiply_current_pid_value( rx[ 2 ] > 0.0f ? multiplier : 1.0f / multiplier );
				skip_accel_cal_on_save = 1;
				// update 4 .. 50 times per second:
				next_update_time = time + mapf( fabsf( rx[ 2 ] ), tuning_dead_band, 1.0f, 1e6f / 4.0f, 1e6f / 50.0f );
			}
		}
	}
#endif // PID_STICK_TUNING
}
