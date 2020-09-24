// Based on https://github.com/silver13/smallosd

#include <stdbool.h>

#include "binary.h"
#include "config.h"
#include "drv_osd.h"
#include "drv_time.h"

#define DATA_POS_X 2
#define DATA_POS_Y 1 // top
// #define DATA_POS_Y 15 // bottom

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

#define SPACE_SYMBOL 0x00
#define RSSI_SYMBOL 0x01
#define VOLT_SYMBOL 0x06
#define ZERO_SYMBOL 0x30
#define HORIZON_SYMBOL 0x80 // .. 0x88
#define BAR_LEFT_SYMBOL 0x8A
#define BAR_FILLED_SYMBOL 0x8B
#define BAR_HALF_SYMBOL 0x8C
#define BAR_EMPTY_SYMBOL 0x8D
#define BAR_LINE_SYMBOL 0x8E
#define BAR_RIGHT_SYMBOL 0x8F
#define BATT_SYMBOL 0x97

static char data[ 26 ];

static void fill_data( void )
{
	// data[ 0 ] = BATT_SYMBOL;

	extern float vbatt_comp; // battery.c
	const float voltage = vbatt_comp + 0.005f;
	data[ 0 ] = ZERO_SYMBOL + (uint32_t)voltage % 10;
	data[ 1 ] = '.';
	data[ 2 ] = ZERO_SYMBOL + (uint32_t)( voltage * 10.0f ) % 10;
	data[ 3 ] = ZERO_SYMBOL + (uint32_t)( voltage * 100.0f ) % 10;
	data[ 4 ] = VOLT_SYMBOL;

#ifdef WARN_ON_LOW_BATTERY
	const float vbatt_min = WARN_ON_LOW_BATTERY;
#else
	const float vbatt_min = 3.0f;
#endif
	#define BAR_LEFT_SYMBOL_POS 5
	#define BAR_RIGHT_SYMBOL_POS 20
	const bool blink = ( gettime() & 0x7FFFF ) < 200000; // roughly every second (524288 µs) for 0.2 s
	if ( vbatt_comp < vbatt_min + VOLTAGE_HYSTERESIS && blink ) {
		for ( uint8_t i = BAR_LEFT_SYMBOL_POS; i <= BAR_RIGHT_SYMBOL_POS; ++i ) {
			data[ i ] = SPACE_SYMBOL;
		}
	} else {
		data[ BAR_LEFT_SYMBOL_POS ] = BAR_LEFT_SYMBOL;
		data[ BAR_RIGHT_SYMBOL_POS ] = BAR_RIGHT_SYMBOL;
		const float volt_per_step = ( 4.2f - vbatt_min ) / ( 2 * ( BAR_RIGHT_SYMBOL_POS - BAR_LEFT_SYMBOL_POS - 1 ) );
		const int steps = ( voltage - vbatt_min ) / volt_per_step + 0.5f;
		uint8_t i = BAR_LEFT_SYMBOL_POS + 1;
		while ( i < BAR_LEFT_SYMBOL_POS + 1 + steps / 2 && i < BAR_RIGHT_SYMBOL_POS ) {
			data[ i ] = BAR_FILLED_SYMBOL;
			++i;
		}
		if ( steps > 0 && i < BAR_RIGHT_SYMBOL_POS ) {
			data[ i ] = steps % 2 == 0 ? BAR_LINE_SYMBOL : BAR_HALF_SYMBOL;
			++i;
		}
		while ( i < BAR_RIGHT_SYMBOL_POS ) {
			data[ i ] = BAR_EMPTY_SYMBOL;
			++i;
		}
	}

	#define MIN_PACKETS 150
	#define MAX_PACKETS 200
	extern int packetpersecond; // rx.c
	int8_t level = 8 * ( packetpersecond - MIN_PACKETS ) / ( MAX_PACKETS - MIN_PACKETS );
	if ( level < 0 ) {
		level = 0;
	}
	data[ 21 ] = HORIZON_SYMBOL + 8 - level;
	if ( packetpersecond < MIN_PACKETS && blink ) {
		data[ 22 ] = SPACE_SYMBOL;
		data[ 23 ] = SPACE_SYMBOL;
		data[ 24 ] = SPACE_SYMBOL;
		data[ 25 ] = SPACE_SYMBOL;
	} else {
		data[ 22 ] = RSSI_SYMBOL;
		data[ 23 ] = ZERO_SYMBOL + packetpersecond / 100;
		data[ 24 ] = ZERO_SYMBOL + packetpersecond / 10 % 10;
		data[ 25 ] = ZERO_SYMBOL + packetpersecond % 10;
	}
}

static int32_t data_index = -1;

void osd( void )
{
	osd_set_baudrate();

	if ( data_index == -1 ) {
		fill_data();
	} else {
		// osd_print( data + data_index, 1, DATA_POS_X + data_index, DATA_POS_Y );
		osd_putc( data[ data_index ], DATA_POS_X + data_index, DATA_POS_Y );
	}
	++data_index;
	if ( data_index == sizeof( data ) ) {
		data_index = -1;
	}

	// osd_print( data, sizeof( data ), DATA_POS_X, DATA_POS_Y );
	// osd_checksystem(); // PAL/NTSC/no input check

	osd_restore_baudrate();
}
