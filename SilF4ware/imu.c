#include <math.h> // sqrtf
#include <stdbool.h>

#include "config.h"
// #include "drv_led.h" // for testing
#include "drv_time.h"
#include "filter.h"
#include "sixaxis.h"


#define ACC_1G 1.0f

// disable drift correction (for testing)
#define DISABLE_ACC false

// time to correct gyro-based gravity vector estimation using the accelerometer
// 1 .. 4 seconds are generally good
#define FILTERTIME 2 // seconds

// accel magnitude limits for drift correction
#define ACC_MIN 0.8f
#define ACC_MAX 1.2f


float GEstG[ 3 ] = { 0, 0, ACC_1G };

float attitude[ 3 ];

extern float gyro[ 3 ];
extern float accel[ 3 ];
extern float looptime;

void imu_init( void )
{
	// init the gravity vector with accel values
	for ( int i = 0; i < 100; ++i ) {
		sixaxis_read();
		#define DELAYTIME 1000 // The accelerometer is updated only at 1 kHz.
		for ( int x = 0; x < 3; ++x ) {
			lpf( &GEstG[ x ], accel[ x ] / 2048.0f, ALPHACALC( DELAYTIME, 0.036e6f ) );
		}
		delay( DELAYTIME );
		gettime(); // if it takes too long time will overflow so we call it here
	}
}

static float calcmagnitude( float vector[ 3 ] )
{
	float accmag = 0;
	for ( int axis = 0; axis < 3; ++axis ) {
		accmag += vector[ axis ] * vector[ axis ];
	}
	accmag = sqrtf( accmag );
	return accmag;
}

void imu( void )
{
	float deltaGyroAngle[ 3 ];

	for ( int i = 0 ; i < 3 ; ++i ) {
		deltaGyroAngle[ i ] = gyro[ i ] * looptime;
	}

	// GEstG[ 2 ] = GEstG[ 2 ] - deltaGyroAngle[ 0 ] * GEstG[ 0 ];
	// GEstG[ 0 ] = deltaGyroAngle[ 0 ] * GEstG[ 2 ] + GEstG[ 0 ];

	// GEstG[ 1 ] = GEstG[ 1 ] + deltaGyroAngle[ 1 ] * GEstG[ 2 ];
	// GEstG[ 2 ] = -deltaGyroAngle[ 1 ] * GEstG[ 1 ] + GEstG[ 2 ];

	// GEstG[ 0 ] = GEstG[ 0 ] - deltaGyroAngle[ 2 ] * GEstG[ 1 ];
	// GEstG[ 1 ] = deltaGyroAngle[ 2 ] * GEstG[ 0 ] + GEstG[ 1 ];

	const float GEstG0 = GEstG[ 0 ] + deltaGyroAngle[ 0 ] * GEstG[ 2 ] - deltaGyroAngle[ 2 ] * GEstG[ 1 ];
	const float GEstG1 = GEstG[ 1 ] + deltaGyroAngle[ 1 ] * GEstG[ 2 ] + deltaGyroAngle[ 2 ] * GEstG[ 0 ];
	const float GEstG2 = GEstG[ 2 ] - deltaGyroAngle[ 0 ] * GEstG[ 0 ] - deltaGyroAngle[ 1 ] * GEstG[ 1 ];
	GEstG[ 0 ] = GEstG0;
	GEstG[ 1 ] = GEstG1;
	GEstG[ 2 ] = GEstG2;

	// Bring the gravity vector in sync with accel values:
	const float accmag = calcmagnitude( accel );
	if ( ( accmag > ACC_MIN * ACC_1G ) && ( accmag < ACC_MAX * ACC_1G ) && ! DISABLE_ACC ) {
		for ( int axis = 0; axis < 3; ++axis ) {
			lpf( &GEstG[ axis ], accel[ axis ], ALPHACALC( LOOPTIME, 2 * PI_F * (float)FILTERTIME * 1e6f ) );
		}
		// Normalize GEstG after nudging it with accel (This is NFEs fix for the yaw slow down bug)
		const float GEstGmag = calcmagnitude( GEstG );
		for ( int axis = 0; axis < 3; ++axis ) {
			GEstG[ axis ] = GEstG[ axis ] * ACC_1G / GEstGmag;
		}
	// 	ledon();
	// } else {
	// 	ledoff();
	}

#ifdef DEBUG
	// vectorcopy( &GEstG[ 0 ], &EstG[ 0 ] );
	float atan2approx( float y, float x );
	attitude[ 0 ] = atan2approx( EstG[ 0 ], EstG[ 2 ] );
	attitude[ 1 ] = atan2approx( EstG[ 1 ], EstG[ 2 ] );
#endif
}

void vectorcopy( float * vector1, float * vector2 )
{
	for ( int axis = 0; axis < 3; ++axis ) {
		vector1[ axis ] = vector2[ axis ];
	}
}

#define OCTANTIFY(_x, _y, _o) do {                              \
	float _t;                                                   \
	_o= 0;                                                      \
	if(_y<  0)  {            _x= -_x;   _y= -_y; _o += 4; }     \
	if(_x<= 0)  { _t= _x;    _x=  _y;   _y= -_t; _o += 2; }     \
	if(_x<=_y)  { _t= _y-_x; _x= _x+_y; _y=  _t; _o += 1; }     \
} while( 0 );

// +-0.09 deg error
float atan2approx( float y, float x )
{
	if ( x == 0 ) {
		x = 123e-15f;
	}
	float phi = 0;
	float dphi;
	float t;

	OCTANTIFY( x, y, phi );

	t = y / x;
	// atan function for 0 - 1 interval
	dphi = t * ( ( PI_F / 4 + 0.2447f ) + t * ( ( -0.2447f + 0.0663f ) + t * ( -0.0663f ) ) );
	phi *= PI_F / 4;
	dphi = phi + dphi;
	if ( dphi > PI_F ) {
		dphi -= 2 * PI_F;
	}

	return RADTODEG * dphi;
}
