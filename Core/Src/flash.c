#include <stdint.h>

#include "config.h"
#include "drv_fmc.h"

extern float accelcal[]; // sixaxis.c
extern float * pids_array[ 3 ]; // pid.c

extern char rfchannel[ 4 ]; // rx.c
extern char rxaddress[ 5 ];
extern int telemetry_enabled;
extern int rx_bind_load;
extern int rx_bind_enable; // gestures.c

#define FMC_HEADER 0x12AA0001

static float pid_c_identifier = -5.0f;

void flash_calculate_pid_c_identifier( void )
{
	float result = 0;
	for ( int i = 0; i < 3; ++i ) {
		for ( int j = 0; j < 3; ++j ) {
			result += pids_array[ i ][ j ] * ( i + 1 ) * ( j + 1 ) * 0.932f;
		}
	}
	pid_c_identifier = result;
}

void flash_save( void )
{
	fmc_unlock();

	// Saving to flash immediately after flashing the firmware (which happens e.g. after changing the PIDs
	// at the end of flash_load()) does not work and some fmc_write() command triggers failloop( 6 ). This
	// dummy write seems to fix it, or at least we trigger failloop( 6 ) before fmc_erase() wipes all data.
	fmc_write( 0, 0 );

	fmc_erase();

	uint32_t addresscount = 0;

	fmc_write( addresscount++, FMC_HEADER );

	fmc_write_float( addresscount++, pid_c_identifier );

	for ( int i = 0; i < 3; ++i ) {
		for ( int j = 0; j < 3; ++j ) {
			fmc_write_float( addresscount++, pids_array[ i ][ j ] );
		}
	}

	fmc_write_float( addresscount++, accelcal[ 0 ] );
	fmc_write_float( addresscount++, accelcal[ 1 ] );
	fmc_write_float( addresscount++, accelcal[ 2 ] );

#if ( defined RX_BAYANG_PROTOCOL_TELEMETRY || defined RX_NRF24_BAYANG_TELEMETRY )

	if ( rx_bind_enable ) {
		fmc_write( 50, rxaddress[ 4 ] | ( telemetry_enabled << 8 ) );
		fmc_write( 51, rxaddress[ 0 ] | ( rxaddress[ 1 ] << 8 ) | ( rxaddress[ 2 ] << 16 ) | ( rxaddress[ 3 ] << 24 ) );
		fmc_write( 52, rfchannel[ 0 ] | ( rfchannel[ 1 ] << 8 ) | ( rfchannel[ 2 ] << 16 ) | ( rfchannel[ 3 ] << 24 ) );
	} else {
		// this will leave 0xFF's so it will be picked up as disabled
	}

#endif

	fmc_write( 4095, FMC_HEADER );

	fmc_lock();
}

void flash_load( void )
{
	uint32_t addresscount = 0;
	// check if saved data is present
	if ( fmc_read( addresscount++ ) == FMC_HEADER && fmc_read( 4095 ) == FMC_HEADER ) {

		const float saved_pid_identifier = fmc_read_float( addresscount++ );
		// load pids from flash if pid.c values are still the same
		if ( saved_pid_identifier == pid_c_identifier ) {
			for ( int i = 0; i < 3; ++i ) {
				for ( int j = 0; j < 3; ++j ) {
					pids_array[ i ][ j ] = fmc_read_float( addresscount++ );
				}
			}
		} else {
			addresscount += 9;
			// Coded PIDs in pid.c were changed, so make sure tuned PIDs in flash don't get loaded even
			// in case the coded PIDs are reverted to the ones for which the saved_pid_identifier matches.
			if ( saved_pid_identifier != -10.0f ) { // Only invalidate once.
				pid_c_identifier = -10.0f; // This triggers saving with -10 at the end of this function.
			}
		}

		accelcal[ 0 ] = fmc_read_float( addresscount++ );
		accelcal[ 1 ] = fmc_read_float( addresscount++ );
		accelcal[ 2 ] = fmc_read_float( addresscount++ );

#if ( defined RX_BAYANG_PROTOCOL_TELEMETRY || defined RX_NRF24_BAYANG_TELEMETRY )

		int temp = fmc_read( 52 );
		int error = 0;
		for ( int i = 0; i < 4; ++i ) {
			if ( ( ( temp >> ( i * 8 ) ) & 0xFF ) > 127 ) {
				error = 1;
			}
		}

		if ( ! error ) {
			rx_bind_load = rx_bind_enable = 1;

			rxaddress[ 4 ] = fmc_read( 50 );

			telemetry_enabled = fmc_read( 50 ) >> 8;
			temp = fmc_read( 51 );
			for ( int i = 0 ; i < 4; ++i ) {
				rxaddress[ i ] = ( temp >> ( i * 8 ) ) & 0xFF;
			}

			temp = fmc_read( 52 );
			for ( int i = 0 ; i < 4; ++i ) {
				rfchannel[ i ] = ( temp >> ( i * 8 ) ) & 0xFF;
			}
		}

#endif

		if ( pid_c_identifier == -10.0f ) {
			flash_save();
		}
	}
}
