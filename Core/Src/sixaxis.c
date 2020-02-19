#include <math.h> // fabsf
#include <stdint.h>

#include "binary.h"
#include "config.h"
#include "drv_led.h"
#include "drv_mpu.h"
#include "drv_time.h"
#include "filter.h"
#include "sixaxis.h"
#include "util.h"

// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER 5

// gyro ids for the gyro check
#define GYRO_ID_MPUx0x0 0x68 // MPU3050, MPU6000, MPU6050
#define GYRO_ID_ICM20689 0x98
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72
#define GYRO_ID_ICM20602 0x12

static void process_gyronew_to_gyro( float gyronew[] ); // To avoid code duplication.

void sixaxis_init( void )
{
	spi_mpu_csoff();
	delay( 1 );

	// gyro soft reset
	// mpu_writereg( 107, 128 );

	delay( 40000 ); // 30 ms gyro start up time, 10 ms PLL settling time

	// disable I2C
	mpu_writereg( 106, 0x10 );

	// set pll to 1, clear sleep bit old type gyro (mpu-6050)
	mpu_writereg( 107, 1 );

	const int id = mpu_readreg( 117 );
	const int newboard = id != 0x68;

#ifdef GYRO_CHECK
	extern void failloop( int );
	if ( id != GYRO_ID_MPUx0x0 && id != GYRO_ID_ICM20689 &&
		 id != GYRO_ID_3 && id != GYRO_ID_4 && id != GYRO_ID_ICM20602 )
	{
		failloop( 4 );
	}
#endif

	delay( 100 );

	mpu_writereg( 28, B00011000 ); // 16G scale
	mpu_writereg( 25, B00000000 ); // Sample Rate = Gyroscope Output Rate (default)

	// acc lpf for the new gyro type
	// 0-6 (same as gyro)
	if ( newboard ) {
		mpu_writereg( 29, ACC_LOW_PASS_FILTER );
	}

	// gyro scale 2000 deg (FS =3)
	mpu_writereg( 27, 24 );

	// Gyro DLPF low pass filter
	mpu_writereg( 26, GYRO_LOW_PASS_FILTER );
}

float bb_accel[ 3 ];
float accel[ 3 ];
float gyro[ 3 ];
float gyro_notch_filtered[ 3 ];
float gyro_unfiltered[ 3 ];

float accelcal[ 3 ];
float gyrocal[ 3 ];

int calibration_done;

void sixaxis_read( void )
{
	uint32_t data[ 14 ];
	mpu_readdata( 59, data, 14 );

	accel[ 0 ] = -(int16_t)( ( data[ 0 ] << 8 ) + data[ 1 ] );
	accel[ 1 ] = -(int16_t)( ( data[ 2 ] << 8 ) + data[ 3 ] );
	accel[ 2 ] = (int16_t)( ( data[ 4 ] << 8 ) + data[ 5 ] );

	// remove bias
	accel[ 0 ] = accel[ 0 ] - accelcal[ 0 ];
	accel[ 1 ] = accel[ 1 ] - accelcal[ 1 ];
	accel[ 2 ] = accel[ 2 ] - accelcal[ 2 ];

#ifdef SENSOR_ROTATE_90_CW
{
	float temp = accel[ 1 ];
	accel[ 1 ] = accel[ 0 ];
	accel[ 0 ] = -temp;
}
#endif

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

#ifdef SENSOR_ROTATE_45_CCW
{
	float temp = accel[ 0 ];
	accel[ 0 ] = (accel[ 0 ] * INVSQRT2 + accel[ 1 ] * INVSQRT2 );
	accel[ 1 ] = -( temp * INVSQRT2 - accel[ 1 ] * INVSQRT2 );
}
#endif

#ifdef SENSOR_ROTATE_45_CW
{
	float temp = accel[ 1 ];
	accel[ 1 ] = (accel[ 1 ] * INVSQRT2 + accel[ 0 ] * INVSQRT2 );
	accel[ 0 ] = -( temp * INVSQRT2 - accel[ 0 ] * INVSQRT2 );
}
#endif

#ifdef SENSOR_ROTATE_90_CCW
{
	float temp = accel[ 1 ];
	accel[ 1 ] = -accel[ 0 ];
	accel[ 0 ] = temp;
}
#endif

#ifdef SENSOR_ROTATE_180
{
	accel[ 1 ] = -accel[ 1 ];
	accel[ 0 ] = -accel[ 0 ];
}
#endif

#ifdef SENSOR_INVERT
{
	accel[ 2 ] = -accel[ 2 ];
	accel[ 0 ] = -accel[ 0 ];
}
#endif

	// Very strong accel filtering to get rid of noise:
	static float accel_filt1st[ 3 ], accel_filt2nd[ 3 ];
	for ( int axis = 0; axis < 3; ++axis ) {
		#define SIXAXIS_READ_LOOPTIME 1000 // This assumes that sixaxis_read() is called every 1 ms from usermain().
		#define ACCEL_FILTER_HZ 20.0f // 20 Hz
		lpf( &accel_filt1st[ axis ], accel[ axis ], ALPHACALC( SIXAXIS_READ_LOOPTIME, 1e6f / (float)ACCEL_FILTER_HZ ) );
		lpf( &accel_filt2nd[ axis ], accel_filt1st[ axis ], ALPHACALC( SIXAXIS_READ_LOOPTIME, 1e6f / (float)ACCEL_FILTER_HZ ) );
		accel[ axis ] = accel_filt2nd[ axis ];
	}

	bb_accel[ 0 ] = accel[ 0 ];
	bb_accel[ 1 ] = accel[ 1 ];
	bb_accel[ 2 ] = accel[ 2 ];

	// reduce to accel in G
	accel[ 0 ] /= 2048.0f;
	accel[ 1 ] /= 2048.0f;
	accel[ 2 ] /= 2048.0f;

	// Gyro:

	float gyronew[ 3 ];
	//order
	gyronew[ 1 ] = (int16_t)( ( data[ 8 ] << 8 ) + data[ 9 ] );
	gyronew[ 0 ] = (int16_t)( ( data[ 10 ] << 8 ) + data[ 11 ] );
	gyronew[ 2 ] = (int16_t)( ( data[ 12 ] << 8 ) + data[ 13 ] );

	process_gyronew_to_gyro( gyronew );
}

void gyro_read( void )
{
	uint32_t data[ 6 ];
	mpu_readdata( 67, data, 6 );

	float gyronew[ 3 ];
	// order
	gyronew[ 1 ] = (int16_t)( ( data[ 0 ] << 8 ) + data[ 1 ] );
	gyronew[ 0 ] = (int16_t)( ( data[ 2 ] << 8 ) + data[ 3 ] );
	gyronew[ 2 ] = (int16_t)( ( data[ 4 ] << 8 ) + data[ 5 ] );

	process_gyronew_to_gyro( gyronew );
}

static void process_gyronew_to_gyro( float gyronew[] )
{
	gyronew[ 0 ] = gyronew[ 0 ] - gyrocal[ 0 ];
	gyronew[ 1 ] = gyronew[ 1 ] - gyrocal[ 1 ];
	gyronew[ 2 ] = gyronew[ 2 ] - gyrocal[ 2 ];

#ifdef SENSOR_ROTATE_45_CCW
{
	float temp = gyronew[ 1 ];
	gyronew[ 1 ] = gyronew[ 0 ] * INVSQRT2 + gyronew[ 1 ] * INVSQRT2;
	gyronew[ 0 ] = gyronew[ 0 ] * INVSQRT2 - temp * INVSQRT2;
}
#endif

#ifdef SENSOR_ROTATE_45_CW
{
	float temp = gyronew[ 0 ];
	gyronew[ 0 ] = gyronew[ 1 ] * INVSQRT2 + gyronew[ 0 ] * INVSQRT2;
	gyronew[ 1 ] = gyronew[ 1 ] * INVSQRT2 - temp * INVSQRT2;
}
#endif

#ifdef SENSOR_ROTATE_90_CW
{
	float temp = gyronew[ 1 ];
	gyronew[ 1 ] = -gyronew[ 0 ];
	gyronew[ 0 ] = temp;
}
#endif

#ifdef SENSOR_ROTATE_90_CCW
{
	float temp = gyronew[ 1 ];
	gyronew[ 1 ] = gyronew[ 0 ];
	gyronew[ 0 ] = -temp;
}
#endif

#ifdef SENSOR_ROTATE_180
{
	gyronew[ 1 ] = -gyronew[ 1 ];
	gyronew[ 0 ] = -gyronew[ 0 ];
}
#endif

#ifdef SENSOR_INVERT
{
	gyronew[ 1 ] = -gyronew[ 1 ];
	gyronew[ 2 ] = -gyronew[ 2 ];
}
#endif

	//gyronew[ 0 ] = -gyronew[ 0 ];
	gyronew[ 1 ] = -gyronew[ 1 ];
	gyronew[ 2 ] = -gyronew[ 2 ];

	for ( int i = 0; i < 3; ++i ) {
		// 16 bit, +-2000°/s -> 4000°/s / 2**16 * reading = °/s
		gyronew[ i ] = gyronew[ i ] * 0.061035156f * DEGTORAD;

		gyro[ i ] = gyronew[ i ];

		gyro_unfiltered[ i ] = gyro[ i ];

#ifdef RPM_FILTER
		gyro[ i ] = rpm_filter( gyro[ i ], i );
#endif

#ifdef BIQUAD_NOTCH_A_HZ
		gyro[ i ] = notch_a_filter( gyro[ i ], i );
#endif
#ifdef BIQUAD_NOTCH_B_HZ
		gyro[ i ] = notch_b_filter( gyro[ i ], i );
#endif
#ifdef BIQUAD_NOTCH_C_HZ
		gyro[ i ] = notch_c_filter( gyro[ i ], i );
#endif

		gyro_notch_filtered[ i ] = gyro[ i ];

#ifdef GYRO_LPF_1ST_HZ_BASE
		gyro[ i ] = gyro_lpf_filter( gyro[ i ], i );
#endif
#ifdef GYRO_LPF_2ND_HZ_BASE
		gyro[ i ] = gyro_lpf2_filter( gyro[ i ], i );
#endif

#ifdef GYRO_LPF_1ST_HZ
		static float gyro1st[ 3 ];
		lpf( &gyro1st[ i ], gyro[ i ], ALPHACALC( LOOPTIME, 1e6f / (float)( GYRO_LPF_1ST_HZ ) ) );
		gyro[ i ] = gyro1st[ i ];
#endif

#ifdef GYRO_YAW_LPF_1ST_HZ
		extern float aux_analog[ 2 ];
		if ( i == 2 ) { // yaw
			static float yaw1st;
			lpf( &yaw1st, gyro[ 2 ], ALPHACALC( LOOPTIME, 1e6f / (float)( GYRO_YAW_LPF_1ST_HZ ) ) );
			gyro[ 2 ] = yaw1st;
		}
#endif
	}
}

#define CAL_TIME 2.5e6f

void gyro_cal( void )
{
	uint32_t data[ 6 ];
	float limit[ 3 ];
	uint32_t time = gettime();
	uint32_t timestart = time;
	uint32_t timemax = time;

	float gyro_data[ 3 ];
	float gyro_data_lpf[ 3 ];

	for ( int i = 0; i < 3; i++) {
		limit[ i ] = gyrocal[ i ];
	}

	// 2.5 and 15 seconds
	while ( time - timestart < CAL_TIME && time - timemax < 15e6f ) {
		mpu_readdata( 67, data, 6 );

		gyro_data[ 1 ] = (int16_t)( ( data[ 0 ] << 8 ) + data[ 1 ] );
		gyro_data[ 0 ] = (int16_t)( ( data[ 2 ] << 8 ) + data[ 3 ] );
		gyro_data[ 2 ] = (int16_t)( ( data[ 4 ] << 8 ) + data[ 5 ] );

		#define GLOW_TIME 62500
		static int brightness = 0;
		led_pwm( brightness );
		if ( ( brightness & 1 ) ^ ( ( time - timestart ) % GLOW_TIME > ( GLOW_TIME >> 1 ) ) ) {
			++brightness;
		}

		brightness &= 0xF;

		for ( int i = 0; i < 3; ++i ) {
			// Apply some basic filtering to eliminate excessive amounts of noise:
			lpf( &gyro_data_lpf[ i ], gyro_data[ i ], ALPHACALC( LOOPTIME, 1e6f / 100.0f ) ); // 100 Hz

			if ( gyro_data_lpf[ i ] > limit[ i ] ) { limit[ i ] += 0.1f; } // 100 gyro bias / second change
			if ( gyro_data_lpf[ i ] < limit[ i ] ) { limit[ i ] -= 0.1f; }

			limitf( &limit[ i ], 800 );

			#define ALLOWED_MOTION_LIMIT 100
			if ( fabsf( gyro_data_lpf[ i ] ) > ALLOWED_MOTION_LIMIT + fabsf( limit[ i ] ) ) {
				timestart = gettime();
				brightness = 1;
			} else {
				lpf( &gyrocal[ i ], gyro_data_lpf[ i ], ALPHACALC( LOOPTIME, 2 * PI_F * 0.5f * 1e6f ) );
			}
		}

		// receiver function
		void checkrx( void );
		checkrx();

		while ( gettime() - time < LOOPTIME );
		time = gettime();
	}

	if ( time - timestart < CAL_TIME ) {
		for ( int i = 0; i < 3; i++ ) {
			gyrocal[ i ] = 0;
		}
	}

	calibration_done = 1;
}

void acc_cal( void )
{
	accelcal[ 2 ] = 2048;
	for ( int i = 0; i < 100; ++i ) {
		uint32_t data[ 6 ];
		mpu_readdata( 59, data, 6 );

		accel[ 0 ] = -(int16_t)( ( data[ 0 ] << 8 ) + data[ 1 ] );
		accel[ 1 ] = -(int16_t)( ( data[ 2 ] << 8 ) + data[ 3 ] );
		accel[ 2 ] = (int16_t)( ( data[ 4 ] << 8 ) + data[ 5 ] );

		#define DELAYTIME 1000 // The accelerometer is updated only at 1 kHz.
		for ( int x = 0; x < 3; ++x ) {
			lpf( &accelcal[ x ], accel[ x ], ALPHACALC( DELAYTIME, 0.072e6f ) );
		}
		delay( DELAYTIME );
		gettime(); // if it takes too long time will overflow so we call it here
	}
	accelcal[ 2 ] -= 2048;

	for ( int x = 0; x < 3; ++x ) {
		limitf( &accelcal[ x ], 500 );
	}
}
