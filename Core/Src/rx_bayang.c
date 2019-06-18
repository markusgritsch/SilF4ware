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
#define PACKET_PERIOD 3000
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

#define RX_MODE_NORMAL RXMODE_NORMAL
#define RX_MODE_BIND RXMODE_BIND


#ifdef RX_BAYANG_PROTOCOL_TELEMETRY

bool failsafe = true;

float rx[ 4 ];
char aux[ AUXNUMBER ];
float aux_analog[ 2 ] = { 1.0, 1.0 };
#define CH_ANA_AUX1 0
#define CH_ANA_AUX2 1

char lasttrim[ 4 ];

char rfchannel[ 4 ];
char rxaddress[ 5 ];
int telemetry_enabled = 0;
int rx_bind_load = 0;

int rxmode = 0;
int rf_chan = 0;
int rxdata[ 15 ];

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

#ifdef RADIO_XN297L

#ifndef TX_POWER
#define TX_POWER 7
#endif

	// Gauss filter amplitude - lowest
	static uint8_t demodcal[ 2 ] = { 0x39, B00000001 };
	xn_writeregs( demodcal, sizeof( demodcal ) );

	// powerup defaults
	//static uint8_t rfcal2[ 7 ] = { 0x3a, 0x45, 0x21, 0xef, 0xac, 0x3a, 0x50 };
	//xn_writeregs( rfcal2, sizeof( rfcal2 ) );

	static uint8_t rfcal2[ 7 ] = { 0x3a, 0x45, 0x21, 0xef, 0x2c, 0x5a, 0x50 };
	xn_writeregs( rfcal2, sizeof( rfcal2 ) );

	static uint8_t regs_1f[ 6 ] = { 0x3f, 0x0a, 0x6d, 0x67, 0x9c, 0x46 };
	xn_writeregs( regs_1f, sizeof( regs_1f ) );

	static uint8_t regs_1e[ 4 ] = { 0x3e, 0xf6, 0x37, 0x5d };
	xn_writeregs( regs_1e, sizeof( regs_1e ) );

#define XN_TO_RX B10001111
#define XN_TO_TX B10000010
#define XN_POWER B00000001 | ( ( TX_POWER & 7 ) << 3 )

#endif // RADIO_XN297L


#ifdef RADIO_XN297

	static uint8_t bbcal[ 6 ] = { 0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20 };
	xn_writeregs( bbcal, sizeof( bbcal ) );
	// new values
	static uint8_t rfcal[ 8 ] = { 0x3e, 0xc9, 0x9a, 0xA0, 0x61, 0xbb, 0xab, 0x9c };
	xn_writeregs( rfcal, sizeof( rfcal ) );
	// 0xa7 0x03
	static uint8_t demodcal[6] = { 0x39, 0x0b, 0xdf, 0xc4, B00100111, B00000000 };
	xn_writeregs( demodcal, sizeof( demodcal ) );

#ifndef TX_POWER
#define TX_POWER 3
#endif

#define XN_TO_RX B00001111
#define XN_TO_TX B00000010
#define XN_POWER (B00000001 | ( ( TX_POWER & 3 ) << 1 ) )

#endif // RADIO_XN297

	delay( 100 );

	// write rx address " 0 0 0 0 0 "

	static uint8_t rxaddr[ 6 ] = { 0x2a, 0, 0, 0, 0, 0 };
	xn_writeregs( rxaddr, sizeof( rxaddr ) );

	xn_writereg( EN_AA, 0 );      // aa disabled
	xn_writereg( EN_RXADDR, 1 );  // pipe 0 only
	xn_writereg( RF_SETUP, XN_POWER );    // power / data rate / lna
	xn_writereg( RX_PW_P0, 15 );  // payload size
	xn_writereg( SETUP_RETR, 0 ); // no retransmissions (redundant?)
	xn_writereg( SETUP_AW, 3 );   // address size (5 bytes)
	xn_command( FLUSH_RX );
	xn_writereg( RF_CH, 0 );      // bind on channel 0

#ifdef RADIO_XN297L
	xn_writereg( 0x1d, B00111000 );   // 64 bit payload, software ce
	static uint8_t cehigh_regs[ 2 ] = { 0xFD, 0 }; // internal CE high command, 0 also required
	xn_writeregs( cehigh_regs, sizeof( cehigh_regs ) );
#endif

#ifdef RADIO_XN297
	xn_writereg( 0x1d, B00011000 );   // 64 bit payload, software ce
#endif

	xn_writereg( 0, XN_TO_RX );   // power up, crc enabled, rx mode

#ifdef RADIO_CHECK
	int rxcheck = xn_readreg( 0x0f ); // rx address pipe 5
	// should be 0xc6
	extern void failloop( int );
	if ( rxcheck != 0xc6 ) {
		failloop( 3 );
	}
#endif

	if ( rx_bind_load ) {
		uint8_t rxaddr_regs[ 6 ] = { 0x2a };
		for ( int i = 1; i < 6; ++i ) {
			rxaddr_regs[ i ] = rxaddress[ i - 1 ];
		}
		// write new rx address
		xn_writeregs( rxaddr_regs, sizeof( rxaddr_regs ) );
		rxaddr_regs[ 0 ] = 0x30; // tx register (write) number

		// write new tx address
		xn_writeregs( rxaddr_regs, sizeof(rxaddr_regs) );

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

	xn_command( FLUSH_TX );

	xn_writereg( 0, XN_TO_TX );

	xn_writepayload( txdata, 15 );

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
	if ( LOOPTIME < 500 ) { // Kludge. 500 seems to works better for telemetry, so we skip some calls accordingly.
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
			xn_readpayload( rxdata, 15 );

			if ( rxdata[ 0 ] == 0xa4 || rxdata[ 0 ] == 0xa3 || rxdata[ 0 ] == 0xa2 || rxdata[ 0 ] == 0xa1 ) { // bind packet
				if ( rxdata[ 0 ] == 0xa3 || rxdata[ 0 ] == 0xa1 ) {
					telemetry_enabled = 1;
					packet_period = PACKET_PERIOD_TELEMETRY;
				}

				rfchannel[ 0 ] = rxdata[ 6 ];
				rfchannel[ 1 ] = rxdata[ 7 ];
				rfchannel[ 2 ] = rxdata[ 8 ];
				rfchannel[ 3 ] = rxdata[ 9 ];

				uint8_t rxaddr_regs[ 6 ] = { 0x2a };

				for ( int i = 1 ; i < 6; ++i ) {
					rxaddr_regs[ i ] = rxdata[ i ];
					rxaddress[ i - 1 ] = rxdata[ i ];
				}
				// write new rx address
				xn_writeregs( rxaddr_regs, sizeof( rxaddr_regs ) );
				rxaddr_regs[ 0 ] = 0x30; // tx register ( write ) number

				// write new tx address
				xn_writeregs( rxaddr_regs, sizeof( rxaddr_regs ) );

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

			xn_readpayload( rxdata, 15 );
			int pass = decodepacket();

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
		static uint8_t rxaddr[ 6 ] = { 0x2a, 0, 0, 0, 0, 0 };
		xn_writeregs( rxaddr, sizeof( rxaddr ) );
		xn_writereg( RF_CH, 0 ); // bind on channel 0
	}

	if ( gettime() - secondtimer > 1000000 ) {
		packetpersecond = packetrx;
		packetrx = 0;
		secondtimer = gettime();
	}
}

#endif // RX_BAYANG_PROTOCOL_TELEMETRY
