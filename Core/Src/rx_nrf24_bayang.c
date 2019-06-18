#include <math.h> // fabsf
#include <stdbool.h>
#include <stdint.h>

#include "binary.h"
#include "config.h"
#include "drv_time.h"
#include "drv_xn297.h"
#include "rx.h"


// radio settings

// packet period in uS
#define PACKET_PERIOD 2000
#define PACKET_PERIOD_TELEMETRY 5000

// was 250 ( uS )
#define PACKET_OFFSET 0

#ifdef USE_STOCK_TX
#undef PACKET_PERIOD
#define PACKET_PERIOD 2000
#undef PACKET_OFFSET
#define PACKET_OFFSET 0
#endif

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4

//#define RX_DATARATE_250K

#define XN_TO_TX B00000010
#define XN_TO_RX B00000011

#define RX_MODE_NORMAL RXMODE_NORMAL
#define RX_MODE_BIND RXMODE_BIND


#ifdef RX_NRF24_BAYANG_TELEMETRY

// crc enable - rx side
#define crc_en 0 // zero or one only
// CRC calculation takes 11.25 us at 168 MHz. We turn it off since it is not used.

// xn297 to nrf24 emulation based on code from nrf24multipro by goebish
// DeviationTx by various contributors

const uint8_t xn297_scramble[] = {
	0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
	0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
	0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc
};

// from https://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
// reverse the bit order in a single byte
uint8_t swapbits( uint8_t a )
{
	unsigned int b = a;
	b = ( ( b * 0x0802LU & 0x22110LU ) | ( b * 0x8020LU & 0x88440LU ) ) * 0x10101LU >> 16;
	return b;
}

uint16_t crc16_update( uint16_t crc, uint8_t in )
{
	crc ^= in << 8;
	for ( uint8_t i = 0; i < 8; ++i ) {
		if ( crc & 0x8000 ) {
			crc = ( crc << 1 ) ^ 0x1021;
		} else {
			crc = crc << 1;
		}
	}
	return crc;
}

// crc calculated over address field (constant)
uint16_t crc_addr = 0;

// set both rx and tx address to a xn297 address
// the tx address can only send to another nrf24
// because it lacks the xn297 preamble
void nrf24_set_xn297_address( uint8_t * addr )
{
	uint8_t rxaddr[ 6 ] = { 0x2a };
	crc_addr = 0xb5d2;
	for ( int i = 5; i > 0; --i ) {
		rxaddr[ i ] = addr[ i - 1 ] ^ xn297_scramble[ 5 - i ];
		if ( crc_en ) {
			crc_addr = crc16_update( crc_addr, rxaddr[ i ] );
		}
	}

	// write rx address
	xn_writeregs( rxaddr, sizeof( rxaddr ) );
	// write tx address
	rxaddr[ 0 ] = 0x30;
	xn_writeregs( rxaddr, sizeof( rxaddr ) );
}

int crc_error = 0;

int nrf24_read_xn297_payload( int * rxdata, int size )
{
	xn_readpayload( rxdata, size );

	if ( crc_en ) {
		uint16_t crcx;
		crcx = crc_addr;
		for ( uint8_t i = 0; i < size - 2; ++i ) {
			crcx = crc16_update( crcx, rxdata[ i ] );
		}
		uint16_t crcrx = rxdata[ size - 2 ] << 8;
		crcrx |= rxdata[ size - 1 ] & 0xFF;

		// hardcoded for len 15
		if ( ( crcx ^ crcrx ) != 0x9BA7 ) {
			++crc_error;
			return 0;
		}
	}
	for ( int i = 0; i < size - crc_en * 2; ++i ) {
		rxdata[ i ] = swapbits( rxdata[ i ] ^ xn297_scramble[ i + 5 ] );
	}

	// crc correct or not used
	return 1;
}

void nrf24_write_xn297_payload( int * txdata, int size )
{
	for ( int i = 0; i < size; ++i ) {
		txdata[ i ] = swapbits( txdata[ i ] ) ^ xn297_scramble[ i + 5 ];
	}

	xn_writepayload( txdata, size );
}

bool failsafe = true;

float rx[ 4 ];
char aux[ AUXNUMBER ];
float aux_analog[ 2 ] = { 1.0, 1.0 };
#define CH_ANA_AUX1 0
#define CH_ANA_AUX2 1

char lasttrim[ 4 ];

char rfchannel[ 4 ];
uint8_t rxaddress[ 5 ];
int telemetry_enabled = 0;
int rx_bind_load = 0;

int rxmode = 0;
int rf_chan = 0;
int rxdata[ 17 + 2 * crc_en ];

int autobind_inhibit = 0;
int packet_period = PACKET_PERIOD;

void rx_init()
{
	spi_xn_csoff();
	delay( 1 );

	// always on (CH_ON) channel set 1
	aux[ AUXNUMBER - 2 ] = 1;
	// always off (CH_OFF) channel set 0
	aux[ AUXNUMBER - 1 ] = 0;
#ifdef AUX1_START_ON
	aux[ CH_AUX1 ] = 1;
#endif
#ifdef AUX2_START_ON
	aux[ CH_AUX2 ] = 1;
#endif

	delay( 100 );

	static uint8_t rxaddr[ 5 ] = { 0, 0, 0, 0, 0 };
	nrf24_set_xn297_address( rxaddr );

	xn_writereg( EN_AA, 0 );      // aa disabled
	xn_writereg( EN_RXADDR, 1 );  // pipe 0 only
#ifdef RX_DATARATE_250K
	xn_writereg( RF_SETUP, B00100110 );     // power / data rate 250K
#else
	xn_writereg( RF_SETUP, B00000110 );    // power / data rate 1000K
#endif

	xn_writereg( RX_PW_P0, 15 + crc_en * 2 );  // payload size
	xn_writereg( SETUP_RETR, 0 ); // no retransmissions
	xn_writereg( SETUP_AW, 3 );   // address size (5 bytes)
	xn_writereg( RF_CH, 0 );      // bind on channel 0
	xn_command( FLUSH_RX );
	xn_writereg( 0, XN_TO_RX );   // power up, crc disabled, rx mode

#ifdef RADIO_CHECK
	int rxcheck = xn_readreg( 0x0f ); // rx address pipe 5
	// should be 0xc6
	extern void failloop( int );
	if ( rxcheck != 0xc6 ) {
		failloop( 3 );
	}
#endif

	if ( rx_bind_load ) {
		// write new rx and tx address
		nrf24_set_xn297_address( rxaddress );

		xn_writereg( 0x25, rfchannel[ rf_chan ] ); // Set channel frequency
		rxmode = RX_MODE_NORMAL;
		if ( telemetry_enabled ) {
			packet_period = PACKET_PERIOD_TELEMETRY;
		}
	} else {
		autobind_inhibit = 1;
	}
}

//#define RXDEBUG

#ifdef RXDEBUG
unsigned long packettime;
int channelcount[ 4 ];
int failcount;
int skipstats[ 12 ];
int afterskip[ 12 ];
#warning "RX debug enabled"
#endif

int packetrx;
int packetpersecond;

static void send_telemetry( void );
static void nextchannel( void );

int loopcounter = 0;
unsigned int send_time;
int telemetry_send = 0;
int send_telemetry_next_loop = 0;
int oldchan = 0;

#define TELEMETRY_TIMEOUT 10000

static void beacon_sequence()
{
	static int beacon_seq_state = 0;

	switch ( beacon_seq_state ) {
		case 0: // send data
			if ( send_telemetry_next_loop || LOOPTIME >= 1000 ) {
				telemetry_send = 1;
				send_telemetry();
				++beacon_seq_state;
			}
			break;
		case 1: // wait for data to finish transmitting
			if ( ( xn_readreg( 0x17 ) & B00010000 ) ) {
				xn_writereg( 0, XN_TO_RX );
				beacon_seq_state = 0;
				telemetry_send = 0;
				nextchannel();
			} else { // if it takes too long we get rid of it
				if ( gettime() - send_time > TELEMETRY_TIMEOUT ) {
					xn_command( FLUSH_TX );
					xn_writereg( 0, XN_TO_RX );
					beacon_seq_state = 0;
					telemetry_send = 0;
				}
			}
			break;
		default:
			beacon_seq_state = 0;
			break;
	}
}

extern bool lowbatt;
extern float vbattfilt;
extern float vbatt_comp;

static void send_telemetry()
{
	int txdata[ 15 ];
	for ( int i = 0; i < 15; ++i ) {
		txdata[ i ] = i;
	}
	txdata[ 0 ] = 133;
	txdata[ 1 ] = lowbatt;

	// battery volt filtered
	int vbatt = vbattfilt * 100 + 0.5f;

#ifndef MOTOR_BEEPS_CHANNEL
#define MOTOR_BEEPS_CHANNEL CH_OFF
#endif

#ifdef DISPLAY_MAX_G_INSTEAD_OF_VOLTAGE
	extern float accel[ 3 ];
	extern int calibration_done;
	static float maxg = 0.0f;
	if ( fabsf( accel[ 2 ] ) > maxg && calibration_done ) {
		maxg = fabsf( accel[ 2 ] );
	}
	if ( aux[ MOTOR_BEEPS_CHANNEL ] ) { // reset displayed maxg
		maxg = 0.0f;
	}
	vbatt = maxg * 100;
#endif // DISPLAY_MAX_G_INSTEAD_OF_VOLTAGE

	txdata[ 3 ] = ( vbatt >> 8 ) & 0xff;
	txdata[ 4 ] = vbatt & 0xff;

	// battery volt compensated
	vbatt = vbatt_comp * 100 + 0.5f;
	txdata[ 5 ] = ( vbatt >> 8 ) & 0xff;
	txdata[ 6 ] = vbatt & 0xff;

	int temp = packetpersecond / 2;

#ifdef DISPLAY_MAX_USED_LOOP_TIME_INSTEAD_OF_RX_PACKETS
	extern uint32_t max_used_loop_time;
	if ( aux[ MOTOR_BEEPS_CHANNEL ] ) { // reset displayed max_used_loop_time
		max_used_loop_time = 0;
	}
	temp = max_used_loop_time / 2;
#endif // DISPLAY_MAX_USED_LOOP_TIME_INSTEAD_OF_RX_PACKETS

	if ( temp > 255 ) {
		temp = 255;
	}
	txdata[ 7 ] = temp; // rx strength

	if ( lowbatt ) {
		txdata[ 3 ] |= 1 << 3;
	}

#ifdef DISPLAY_PID_VALUES
	extern float * pids_array[ 3 ];
	extern int current_pid_axis, current_pid_term;
	static uint32_t pid_term = 0;
	const bool blink = ( gettime() & 0xFFFFF ) < 200000; // roughly every second (1048575 µs) for 0.2 s
	int pid_value;
	if ( current_pid_term == pid_term && blink ) {
		pid_value = 0;
	} else {
		pid_value = pids_array[ pid_term ][ current_pid_axis ] * 1000 + 0.5f;
	}
	txdata[ 8 ] = ( pid_value >> 8 ) & 0x3F;
	txdata[ 9 ] = pid_value & 0xff;
	txdata[ 8 ] |= pid_term << 6;
	++pid_term;
	if ( pid_term == 3 ) {
		pid_term = 0;
	}
#endif // DISPLAY_PID_VALUES

	int sum = 0;
	for ( int i = 0; i < 14; ++i ) {
		sum += txdata[ i ];
	}

	txdata[ 14 ] = sum;

	// xn_command( FLUSH_TX );

	xn_writereg( 0, 0 );
	xn_writereg( 0, XN_TO_TX );

	nrf24_write_xn297_payload( txdata, 15 );

	send_time = gettime();
}

static char checkpacket()
{
	int status = xn_readreg( 7 );
#if 1
	if ( status & ( 1 << MASK_RX_DR ) ) { // RX packet received
		xn_writereg( STATUS, 1 << MASK_RX_DR ); // rx clear bit
		return 1;
	}
#else
	if ( ( status & B00001110 ) != B00001110 ) { // rx fifo not empty
		return 2;
	}
#endif
	return 0;
}

static float packettodata( int * data )
{
	return ( ( ( data[ 0 ] & 0x0003 ) * 256 + data[ 1 ] ) / 1023.0f * 2.0f ) - 1.0f;
}

static float bytetodata( int byte )
{
	// 0.0 .. 2.0
	return byte / 200.0f * 2.0f;
}

static int decodepacket( void )
{
	if ( rxdata[ 0 ] == 165 ) {
		int sum = 0;
		for ( int i = 0; i < 14; ++i ) {
			sum += rxdata[ i ];
		}
		if ( ( sum & 0xFF ) == rxdata[ 14 ] ) {
			// roll, pitch, yaw: -1 .. 1
			rx[ 0 ] = packettodata( &rxdata[ 4 ] );
			rx[ 1 ] = packettodata( &rxdata[ 6 ] );
			rx[ 2 ] = packettodata( &rxdata[ 10 ] );
			// throttle: 0 .. 1
			rx[ 3 ] = ( ( rxdata[ 8 ] & 0x0003 ) * 256 + rxdata[ 9 ] ) / 1023.0f;

#ifdef USE_STOCK_TX
			char trims[ 4 ];
			trims[ 0 ] = rxdata[ 6 ] >> 2;
			trims[ 1 ] = rxdata[ 4 ] >> 2;

			for ( int i = 0; i < 2; ++i ) {
				if ( trims[ i ] != lasttrim[ i ] ) {
					aux[ CH_PIT_TRIM + i ] = trims[ i ] > lasttrim[ i ];
					lasttrim[ i ] = trims[ i ];
				}
			}
#else
			aux[ CH_INV ] = ( rxdata[ 3 ] & 0x80 ) ? 1 : 0; // inverted flag
			aux[ CH_VID ] = ( rxdata[ 2 ] & 0x10 ) ? 1 : 0;
			aux[ CH_PIC ] = ( rxdata[ 2 ] & 0x20 ) ? 1 : 0;
#endif

			aux[ CH_TO ] = ( rxdata[ 3 ] & 0x20 ) ? 1 : 0; // take off flag
			aux[ CH_EMG ] = ( rxdata[ 3 ] & 0x04 ) ? 1 : 0; // emg stop flag
			aux[ CH_FLIP ] = ( rxdata[ 2 ] & 0x08 ) ? 1 : 0;
			aux[ CH_EXPERT ] = ( rxdata[ 1 ] == 0xfa ) ? 1 : 0;
			aux[ CH_HEADFREE ] = ( rxdata[ 2 ] & 0x02 ) ? 1 : 0;
			aux[ CH_RTH ] = ( rxdata[ 2 ] & 0x01 ) ? 1 : 0; // rth channel

			aux_analog[ CH_ANA_AUX1 ] = bytetodata( rxdata[ 1 ] );
			aux_analog[ CH_ANA_AUX2 ] = bytetodata( rxdata[ 13 ] );

			// if ( aux[ LEVELMODE ] ) { // level mode expo
			// 	if ( EXPO_XY > 0.01) {
			// 		rx[ 0 ] = rcexpo( rx[ 0 ], EXPO_XY );
			// 		rx[ 1 ] = rcexpo( rx[ 1 ], EXPO_XY );
			// 	}
			// 	if ( EXPO_YAW > 0.01 ){
			// 		rx[ 2 ] = rcexpo( rx[ 2 ], EXPO_YAW );
			// 	}
			// } else { // acro mode expo
			// 	if ( ACRO_EXPO_XY > 0.01 ) {
			// 		rx[ 0 ] = rcexpo( rx[ 0 ], ACRO_EXPO_XY );
			// 		rx[ 1 ] = rcexpo( rx[ 1 ], ACRO_EXPO_XY );
			// 	}
			// 	if ( ACRO_EXPO_YAW > 0.01 ) {
			// 		rx[ 2 ] = rcexpo( rx[ 2 ], ACRO_EXPO_YAW );
			// 	}
			// }

			return 1; // valid packet
		}
		return 0; // sum fail
	}
	return 0; // first byte different
}

static void nextchannel()
{
	++rf_chan;
	rf_chan &= 3; // same as %4
	xn_writereg( 0x25, rfchannel[ rf_chan ] );
}

unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;

unsigned int skipchannel = 0;
int lastrxchan;
int timingfail = 0;

void checkrx( void )
{
	if ( LOOPTIME < 500 ) { // Kludge. 500 works perfectly for telemetry on nrf24, so we skip some calls accordingly.
		static int count;
		if ( count == 0 ) {
			count = 500 / LOOPTIME - 1;
		} else {
			--count;
			return;
		}
	}

	if ( send_telemetry_next_loop ) {
		beacon_sequence();
		send_telemetry_next_loop = 0;
		return;
	}

	const int packetreceived = checkpacket();

	if ( packetreceived ) {
		if ( rxmode == RX_MODE_BIND ) { // rx startup, bind mode
			if ( nrf24_read_xn297_payload( rxdata, 15 + 2 * crc_en ) ) {
			} else {
				return;
			}

			if ( rxdata[ 0 ] == 0xa4 || rxdata[ 0 ] == 0xa3 || rxdata[ 0 ] == 0xa2 || rxdata[ 0 ] == 0xa1 ) { // bind packet
				if ( rxdata[ 0 ] == 0xa3 || rxdata[ 0 ] == 0xa1 ) {
					telemetry_enabled = 1;
					packet_period = PACKET_PERIOD_TELEMETRY;
				}

				rfchannel[ 0 ] = rxdata[ 6 ];
				rfchannel[ 1 ] = rxdata[ 7 ];
				rfchannel[ 2 ] = rxdata[ 8 ];
				rfchannel[ 3 ] = rxdata[ 9 ];

				for ( int i = 0; i < 5; ++i ) {
					rxaddress[ i ] = rxdata[ i + 1 ];
				}
				// write new rx and tx address
				nrf24_set_xn297_address( rxaddress );

				xn_writereg( 0x25, rfchannel[ rf_chan ] ); // Set channel frequency
				rxmode = RX_MODE_NORMAL;
			}
		} else { // normal mode
#ifdef RXDEBUG
			++channelcount[ rf_chan ];
			packettime = gettime() - lastrxtime;
			if ( skipchannel && ! timingfail ) {
				++afterskip[ skipchannel ];
			}
			if ( timingfail ) {
				++afterskip[ 0 ];
			}
#endif

			unsigned long temptime = gettime();

			int pass = nrf24_read_xn297_payload( rxdata, 15 + 2 * crc_en );
			if ( pass ) {
				pass = decodepacket();
			}

			if ( pass ) {
				++packetrx;
				if ( telemetry_enabled ) {
					if ( LOOPTIME >= 1000 ) {
						beacon_sequence();
					} else {
						send_telemetry_next_loop = 1;
					}
				}
				skipchannel = 0;
				timingfail = 0;
				lastrxchan = rf_chan;
				lastrxtime = temptime;
				failsafetime = temptime;
				failsafe = false;
				// if ( ! telemetry_send && ! send_telemetry_next_loop ) {
				// 	nextchannel();
				// }
			} else {
#ifdef RXDEBUG
				++failcount;
#endif
			}
		} // end normal rx mode
	} // end packet received

	// finish sending if already started
	if ( telemetry_send ) {
		beacon_sequence();
	}

	unsigned long time = gettime();

	if ( time - lastrxtime > ( HOPPING_NUMBER * packet_period + 1000 ) && rxmode != RX_MODE_BIND ) {
		// channel with no reception
		lastrxtime = time;
		// set channel to last with reception
		if ( ! timingfail ) {
			rf_chan = lastrxchan;
		}
		// advance to next channel
		nextchannel();
		// set flag to discard packet timing
		timingfail = 1;
	}

	if ( ! timingfail && ! telemetry_send && skipchannel < HOPPING_NUMBER + 1 && rxmode != RX_MODE_BIND ) {
		unsigned int temp = time - lastrxtime;

		if ( temp > 1000 && ( temp - ( PACKET_OFFSET ) ) / ( (int)packet_period ) >= ( skipchannel + 1 ) ) {
			nextchannel();
#ifdef RXDEBUG
			++skipstats[ skipchannel ];
#endif
			++skipchannel;
		}
	}

	if ( time - failsafetime > FAILSAFETIME ) { // failsafe
		failsafe = true;
		rx[ 0 ] = 0;
		rx[ 1 ] = 0;
		rx[ 2 ] = 0;
		rx[ 3 ] = 0;
	}

	if ( ! failsafe ) {
		autobind_inhibit = 1;
	} else if ( ! autobind_inhibit && time > 15000000 ) {
		autobind_inhibit = 1;
		rxmode = RX_MODE_BIND;
		static uint8_t rxaddr[ 5 ] = { 0, 0, 0, 0, 0 };
		nrf24_set_xn297_address( rxaddr );
		xn_writereg( RF_CH, 0 ); // bind on channel 0
	}

	if ( gettime() - secondtimer > 1000000 ) {
		packetpersecond = packetrx;
		packetrx = 0;
		secondtimer = gettime();
	}
}

#endif // RX_NRF24_BAYANG_TELEMETRY
