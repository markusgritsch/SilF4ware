#include <stdbool.h>

#include "config.h"
#include "drv_time.h"
#include "flash.h"
#include "gesture_detect.h"
#include "gestures.h"
#include "pid.h"
#include "sixaxis.h"

extern bool ledcommand;
extern int ledblink;
extern char aux[ AUXNUMBER ];

int rx_bind_enable;
static int skip_accel_cal_on_save = 0;

void gestures( void )
{
	const int command = gesture_detect();

	if ( command != GESTURE_NONE ) {

		if ( command == GESTURE_DDD ) {
			//skip accel calibration if pid gestures used
			if ( ! skip_accel_cal_on_save ) {
				gyro_cal(); // for flashing lights
				acc_cal();
			} else {
				ledcommand = 1;
				skip_accel_cal_on_save = 0;
			}
			flash_save();
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
		}

#if ( defined RX_BAYANG_PROTOCOL_TELEMETRY || defined RX_NRF24_BAYANG_TELEMETRY )
		if ( command == GESTURE_UUU ) {
			rx_bind_enable = ! rx_bind_enable;
			ledblink = 2 - rx_bind_enable;
			skip_accel_cal_on_save = 1;
		}
#endif

		if ( command == GESTURE_LLU ) {
			aux[ CH_AUX1 ] = 1;
			ledcommand = 1;
		}

		if ( command == GESTURE_LLD ) {
			aux[ CH_AUX1 ] = 0;
			ledcommand = 1;
		}

		if ( command == GESTURE_RRU ) {
			aux[ CH_AUX2 ] = 1;
			ledcommand = 1;
		}

		if ( command == GESTURE_RRD ) {
			aux[ CH_AUX2 ] = 0;
			ledcommand = 1;
		}

#ifdef PID_GESTURE_TUNING
		if ( command == GESTURE_UDU ) {
			ledblink = next_pid_term(); // Cycle to next pid term (P, I, D)
		}

		if ( command == GESTURE_UDD ) {
			ledblink = next_pid_axis(); // Cycle to next axis (Roll, Pitch, Yaw)
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
			ledcommand = 1;
		}
#endif // PID_GESTURE_TUNING

	}
}
