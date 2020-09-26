// Based on https://github.com/silver13/smallosd

#include <math.h>
#include <stdbool.h>

#include "binary.h"
#include "config.h"
#include "drv_osd.h"
#include "drv_time.h"
#include "util.h"

// Status line position:
#define STATUS_POS_X 1
//#define STATUS_POS_Y 1 // top
#define STATUS_POS_Y 14 // bottom

// Artificial horizon center position:
#define HORIZON_POS_X 15
#define HORIZON_POS_Y 6

#define NTSC 0
#define PAL 1
#define NONE 2

#ifndef OSD_ENABLE
#define OSD_ENABLE NTSC
#endif

// osd video system (PAL/NTSC) at startup if no video input is present
// after input is present the last detected system will be used.
static uint8_t osdsystem = OSD_ENABLE;

static uint8_t lastvm0 = B01010101;
static uint8_t lastsystem = OSD_ENABLE;
static uint32_t timesystem;
static bool systemclear = false;

void osd_init( void )
{
	drv_osd_init();

	osd_set_baudrate();

	osd_writereg( VM0, B00000010 ); // soft reset
	delay( 100 );

	const uint8_t osdbl = osd_readreg( OSDBL ); // Mine is set to 0x1F (factory).
	osd_writereg( OSDBL, osdbl & B11101111 );

	if ( osdsystem == PAL ) {
		osd_writereg( VM0, B01001000 ); // Set PAL mode (NTSC by default) and enable display.
		lastvm0 = B01001000;
	} else {
		osd_writereg( VM0, B00001000);
		lastvm0 = B00001000;
	}

	osd_writereg( VM1, B00001110);

	osd_restore_baudrate();
}

// x: 0 .. 29, y: 0 .. 15 PAL (12 NTSC)
static void osd_print( const char data[], uint32_t size, uint8_t x, uint8_t y )
{
	if ( y > 12 && lastsystem == NTSC) {
		y = 12;
	}
	const uint32_t pos = x + y * 30;
	osd_print_display_memory( (uint8_t *)data, size, pos );
}

// x: 0 .. 29, y: 0 .. 15 PAL (12 NTSC)
static void osd_putc( const char character, uint8_t x, uint8_t y )
{
	if ( y > 12 && lastsystem == NTSC) {
		y = 12;
	}
	const uint32_t pos = x + y * 30;
	osd_putc_display_memory( (uint8_t)character, pos );
}

// Set the video output system PAL/NTSC
static void osd_setsystem( uint8_t system )
{
	const uint8_t vm0 = osd_readreg( VM0 );
	if ( system == PAL ) {
		lastvm0 = vm0 | B01000000;
	} else {
		lastvm0 = vm0 & B10111111;
	}
	osd_writereg( VM0, lastvm0 );
}

// Check detected video system
static void osd_checksystem()
{
	// NTSC/PAL position
	#define SYSTEM_XPOS 2
	#define SYSTEM_YPOS 3

	if ( ! systemclear && gettime() - timesystem > 3000000 ) {
		// Delete the PAL/NTSC/NONE message after a timeout
		osd_print( "    ", 4, SYSTEM_XPOS, SYSTEM_YPOS );
		systemclear = true;
	}

	const uint8_t stat = osd_readreg( STAT );
	if ( stat & B00000001 ) { // PAL
		if ( lastsystem != PAL ) {
			osd_print( "PAL ", 4, SYSTEM_XPOS, SYSTEM_YPOS );
			timesystem = gettime();
			systemclear = false;
			lastsystem = PAL;
			if ( osdsystem != PAL ) {
				// osd_clear();
				osd_setsystem( PAL );
			}
		}
	}
	if ( stat & B00000010) { // NTSC
		if ( lastsystem != NTSC ) {
			osd_print( "NTSC", 4, SYSTEM_XPOS, SYSTEM_YPOS );
			timesystem = gettime();
			systemclear = false;
			lastsystem = NTSC;
			if ( osdsystem != NTSC ) {
				// osd_clear();
				osd_setsystem( NTSC );
			}
		}
	}
	if ( ! ( stat | B00000011 ) ) { // No signal
		if ( lastsystem != NONE ) {
			osd_print( "NONE", 4, SYSTEM_XPOS, SYSTEM_YPOS );
			timesystem = gettime();
			systemclear = false;
			lastsystem = 2;
		}
	}
}

#define ORD_SPACE 0x00
#define ORD_RSSI 0x01
#define ORD_VOLT 0x06
#define ORD_ZERO 0x30
#define ORD_DIRECTION 0x60
#define ORD_CENTER 0x7E
#define ORD_HORIZON 0x80 // .. 0x88
#define ORD_BAR_LEFT 0x8A
#define ORD_BAR_FILLED 0x8B
#define ORD_BAR_HALF 0x8C
#define ORD_BAR_EMPTY 0x8D
#define ORD_BAR_LINE 0x8E
#define ORD_BAR_RIGHT 0x8F
#define ORD_BATT 0x90 // .. 0x96
#define ORD_TIME 0xBC
#define ORD_ZERO_POINT 0xC0
#define ORD_POINT_ZERO 0xD0
#define ORD_DOTS 0xE0

static char status[ 28 ];

static void fill_status( void )
{
	const bool blink = ( gettime() & 0x7FFFF ) < 200000; // roughly every second (524288 µs) for 0.2 s

#ifdef WARN_ON_LOW_BATTERY
	const float vbatt_min = WARN_ON_LOW_BATTERY;
#else
	const float vbatt_min = 3.0f;
#endif

	extern float vbatt_comp; // battery.c
	const float voltage = vbatt_comp + 0.005f;

	int8_t batt_level = 6.0f * ( voltage - vbatt_min ) / ( 4.2f - vbatt_min ) + 0.5f;
	if ( batt_level < 0 ) {
		batt_level = 0;
	} else if ( batt_level > 6 )  {
		batt_level = 6;
	}
	if ( vbatt_comp < vbatt_min + VOLTAGE_HYSTERESIS && blink ) {
		status[ 0 ] = ORD_SPACE;
		status[ 1 ] = ORD_SPACE;
		status[ 2 ] = ORD_SPACE;
		status[ 3 ] = ORD_SPACE;
		status[ 4 ] = ORD_SPACE;
	} else {
		status[ 0 ] = ORD_BATT + 6 - batt_level;
		status[ 1 ] = ORD_ZERO_POINT + (uint32_t)voltage % 10;
		status[ 2 ] = ORD_POINT_ZERO + (uint32_t)( voltage * 10.0f ) % 10;
		status[ 3 ] = ORD_ZERO + (uint32_t)( voltage * 100.0f ) % 10;
		status[ 4 ] = ORD_VOLT;
	}

	extern uint32_t fly_time;
	const uint32_t total_secs = fly_time / 1000000;
	const uint32_t minutes = total_secs / 60;
	const uint32_t seconds = total_secs % 60;
	status[ 5 ] = ORD_SPACE;
	status[ 6 ] = ORD_TIME;
	status[ 7 ] = ORD_ZERO + minutes / 10;
	status[ 8 ] = ORD_ZERO + minutes % 10;
	status[ 9 ] = ':';
	status[ 10 ] = ORD_ZERO + seconds / 10;
	status[ 11 ] = ORD_ZERO + seconds % 10;
	status[ 12 ] = ORD_SPACE;

	extern int packetpersecond; // rx.c
	if ( packetpersecond < 150 && blink ) {
		status[ 13 ] = ORD_SPACE;
		status[ 14 ] = ORD_SPACE;
		status[ 15 ] = ORD_SPACE;
		status[ 16 ] = ORD_SPACE;
	} else {
		status[ 13 ] = ORD_RSSI;
		status[ 14 ] = ORD_ZERO + packetpersecond / 100;
		status[ 15 ] = ORD_ZERO + packetpersecond / 10 % 10;
		status[ 16 ] = ORD_ZERO + packetpersecond % 10;
	}

	#define BAR_LEFT_POS 17
	#define BAR_RIGHT_POS 27
	#define BAR_LEFT_VALUE 0
	#define BAR_RIGHT_VALUE 200
	status[ BAR_LEFT_POS ] = ORD_BAR_LEFT;
	status[ BAR_RIGHT_POS ] = ORD_BAR_RIGHT;
	const float packets_per_step = ( BAR_RIGHT_VALUE - BAR_LEFT_VALUE ) / (float)( 2 * ( BAR_RIGHT_POS - BAR_LEFT_POS - 1 ) );
	const int steps = ( packetpersecond - BAR_LEFT_VALUE ) / packets_per_step + 0.5f;
	uint8_t i = BAR_LEFT_POS + 1;
	while ( i < BAR_LEFT_POS + 1 + steps / 2 && i < BAR_RIGHT_POS ) {
		status[ i ] = ORD_BAR_FILLED;
		++i;
	}
	if ( steps > 0 && i < BAR_RIGHT_POS ) {
		status[ i ] = steps % 2 == 0 ? ORD_BAR_LINE : ORD_BAR_HALF;
		++i;
	}
	while ( i < BAR_RIGHT_POS ) {
		status[ i ] = ORD_BAR_EMPTY;
		++i;
	}
}

#ifdef ARTIFICIAL_HORIZON
	#define HORIZON_SIZE 11 // 9 dots + 1 direction + 1 center
	#define HORIZON_DOTS ( HORIZON_SIZE - 2 )
#else
	#define HORIZON_SIZE 0
	#define HORIZON_DOTS 0
#endif	 // ARTIFICIAL_HORIZON
static char horizon_ord[ HORIZON_SIZE ];
static uint8_t horizon_x[ HORIZON_SIZE ];
static uint8_t horizon_y[ HORIZON_SIZE ];
static uint8_t old_horizon_x[ HORIZON_SIZE ];
static uint8_t old_horizon_y[ HORIZON_SIZE ];

void calc_horizon()
{
	for ( int i = 0; i < HORIZON_SIZE; ++i ) {
		old_horizon_x[ i ] = horizon_x[ i ];
		old_horizon_y[ i ] = horizon_y[ i ];
	}

	extern float GEstG[ 3 ]; // imu.c

	float direction_x = GEstG[ 0 ];
	float direction_y = GEstG[ 2 ];
	const float direction_mag = sqrtf( direction_x * direction_x + direction_y * direction_y );
	if ( direction_mag != 0.0f ) {
		direction_x /= direction_mag;
		direction_y /= direction_mag;
	}
	float vect_x = -GEstG[ 1 ] * direction_x;
	float vect_y = -GEstG[ 1 ] * direction_y;
	float vect_mag = sqrtf( vect_x * vect_x + vect_y * vect_y );
	const float expo = vect_mag * (float)HORIZON_LENS_EXPO + 1.0f - (float)HORIZON_LENS_EXPO;
	vect_x *= expo;
	vect_y *= expo;
	vect_mag *= expo;
	const float base_x = vect_x * HORIZON_SENSITIVITY + HORIZON_POS_X * 4 + 1;
	const float base_y = vect_y * HORIZON_SENSITIVITY + HORIZON_POS_Y * 6 + 2;
	for ( int i = 0; i < HORIZON_DOTS; ++i ) {
		const int offset = i - HORIZON_DOTS / 2;
		const int8_t x = lroundf( base_x + direction_y * 7 * offset );
		const int8_t y = lroundf( base_y - direction_x * 7 * offset );

		uint8_t pos_x;
		uint8_t pos_y;
		char glyph;
		if ( offset == 0 || x < 0 || x >= 30 * 4 || y < 0 || y >= 12 * 6 ) {
			pos_x = 0;
			pos_y = 0;
			glyph = ORD_SPACE;
		} else {
			pos_x = x / 4;
			pos_y = y / 6;
			glyph = ORD_DOTS + 4 * ( y % 6 ) + x % 4;
		}

		horizon_ord[ i ] = glyph;
		horizon_x[ i ] = pos_x;
		horizon_y[ i ] = pos_y;
	}

	if ( vect_mag * HORIZON_SENSITIVITY > 35.0f ) {
		const int direction =  lroundf( atan2_approx( vect_y, vect_x ) / ( 2.0f * PI_F ) * 16.0f );
		const float scale = 5.5f / vect_mag;
		horizon_ord[ HORIZON_DOTS ] = ORD_DIRECTION + ( 4 + 16 + direction ) % 16;
		horizon_x[ HORIZON_DOTS ] = HORIZON_POS_X + lroundf( vect_x * scale * 1.5f ); // Due to the 2/3 character aspect ratio.
		horizon_y[ HORIZON_DOTS ] = HORIZON_POS_Y + lroundf( vect_y * scale );
	} else {
		horizon_ord[ HORIZON_DOTS ] = ORD_SPACE;
		horizon_x[ HORIZON_DOTS ] = 0;
		horizon_y[ HORIZON_DOTS ] = 0;
	}

	// const int up =  lroundf( atan2_approx( direction_y, direction_x ) / ( 2.0f * PI_F ) * 16.0f );
	// horizon_ord[ HORIZON_DOTS + 1 ] = ORD_DIRECTION + ( 12 + 16 + up ) % 16;
	// horizon_ord[ HORIZON_DOTS + 1 ] = GEstG[ 2 ] >= 0.0f ? ORD_DIRECTION : ORD_DIRECTION + 8;
	// horizon_ord[ HORIZON_DOTS + 1 ] = GEstG[ 2 ] >= 0.0f ? 0x05 : 0x24;
	// horizon_ord[ HORIZON_DOTS + 1 ] = ORD_CENTER;
	horizon_ord[ HORIZON_DOTS + 1 ] = ORD_HORIZON + 3;
	horizon_x[ HORIZON_DOTS + 1 ] = HORIZON_POS_X;
	horizon_y[ HORIZON_DOTS + 1 ] = HORIZON_POS_Y;
}

static int32_t osd_index = -1;

void osd( void )
{
	osd_set_baudrate();

	if ( osd_index == -1 ) {
		fill_status();
	} else if ( osd_index == -2 ) {
#ifdef ARTIFICIAL_HORIZON
		calc_horizon();
#endif	 // ARTIFICIAL_HORIZON
	} else if ( osd_index < sizeof( status ) ) {
		// osd_print( status + osd_index, 1, STATUS_POS_X + osd_index, STATUS_POS_Y );
		osd_putc( status[ osd_index ], STATUS_POS_X + osd_index, STATUS_POS_Y );
	} else if ( osd_index < sizeof( status ) + 2 * sizeof( horizon_ord ) ) {
		const uint32_t horizon_index = osd_index - sizeof( status );
		const uint32_t index = horizon_index / 2;
		if ( horizon_index % 2 == 0 ) {
			if ( horizon_x[ index ] != old_horizon_x[ index ] || horizon_y[ index ] != old_horizon_y[ index ] ) {
				osd_putc( ORD_SPACE, old_horizon_x[ index ], old_horizon_y[ index ] );
			}
		} else {
			osd_putc( horizon_ord[ index ], horizon_x[ index ], horizon_y[ index ] );
		}
	}
	++osd_index;
	if ( osd_index == sizeof( status ) + 2 * sizeof( horizon_ord ) ) {
		osd_index = -2;
	}

	// osd_print( status, sizeof( status ), STATUS_POS_X, STATUS_POS_Y );
	// osd_checksystem(); // PAL/NTSC/no input check

	osd_restore_baudrate();
}
