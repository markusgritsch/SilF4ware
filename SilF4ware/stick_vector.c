#include <math.h> // fabsf

#include "config.h"
#include "util.h"

extern float GEstG[ 3 ];
extern char aux[];

// error vector between stick position and quad orientation
// this is the output of this function
float errorvect[ 3 ];
float stickvector[ 3 ] = { 0, 0, 1 };

void stick_vector( float rx_roll, float rx_pitch )
{
	float pitch, roll;

	// rotate down vector to match stick position
	roll = rx_roll * (float)LEVEL_MAX_ANGLE * DEGTORAD;
	pitch = rx_pitch * (float)LEVEL_MAX_ANGLE * DEGTORAD;

	stickvector[ 0 ] = sin_approx( roll );
	stickvector[ 1 ] = sin_approx( pitch );
	stickvector[ 2 ] = cos_approx( roll ) * cos_approx( pitch );

	float mag2 = stickvector[ 0 ] * stickvector[ 0 ] + stickvector[ 1 ] * stickvector[ 1 ];

	if ( mag2 > 0.001f ) {
		mag2 = 1.0f / sqrtf( mag2 / ( 1 - stickvector[ 2 ] * stickvector[ 2 ] ) );
	} else {
		mag2 = 0.707f;
	}

	stickvector[ 0 ] *= mag2;
	stickvector[ 1 ] *= mag2;

#ifdef LEVEL_MODE_INVERTED_ENABLE
	extern int pwmdir;

	if ( pwmdir == REVERSE ) {
		stickvector[ 0 ] = -stickvector[ 0 ];
		stickvector[ 1 ] = -stickvector[ 1 ];
		stickvector[ 2 ] = -stickvector[ 2 ];
	}
#endif // LEVEL_MODE_INVERTED_ENABLE

	// find error between stick vector and quad orientation
	// vector cross product
	errorvect[ 1 ]= -( GEstG[ 1 ] * stickvector[ 2 ] - GEstG[ 2 ] * stickvector[ 1 ] );
	errorvect[ 0 ]= GEstG[ 2 ] * stickvector[ 0 ] - GEstG[ 0 ] * stickvector[ 2 ];

	// some limits just in case
	limitf( &errorvect[ 0 ], 1.0 );
	limitf( &errorvect[ 1 ], 1.0 );

// fix to recover if triggered inverted
// the vector cross product results in zero for opposite vectors, so it's bad at 180 error
// without this the quad will not invert if angle difference = 180

#ifdef LEVEL_MODE_INVERTED_ENABLE

	static int flip_active_once = 0;
	static int flipaxis = 0;
	static int flipdir = 0;
	int flip_active = 0;

	#define rollrate 1.0f
	#define g_treshold 0.125f
	#define roll_bias 0.25f

	if ( aux[ FN_INVERTED ] && GEstG[ 2 ] > g_treshold ) {
		flip_active = 1;
		// rotate around axis with larger leaning angle
		if ( flipdir ) {
			errorvect[ flipaxis ] = rollrate;
		} else {
			errorvect[ flipaxis ] = -rollrate;
		}
	} else if ( ! aux[ FN_INVERTED ] && GEstG[ 2 ] < -g_treshold ) {
		flip_active = 1;
		if ( flipdir ) {
			errorvect[ flipaxis ] = -rollrate;
		} else {
			errorvect[ flipaxis ] = rollrate;
		}
	} else {
		flip_active_once = 0;
	}

	// set common things here to avoid duplication
	if ( flip_active ) {
		if ( ! flip_active_once ) {
			// check which axis is further from center, with a bias towards roll
			// because a roll flip does not leave the quad facing the wrong way
			if( fabsf( GEstG[ 0 ] ) + roll_bias > fabsf( GEstG[ 1 ] ) ) {
				flipaxis = 0; // flip in roll axis
			} else {
				flipaxis = 1;
			}

			if ( GEstG[ flipaxis ] > 0 ) {
				flipdir = 1;
			} else {
				flipdir = 0;
			}

			flip_active_once = 1;
		}

		// set the error in other axis to return to zero
		errorvect[! flipaxis] = GEstG[! flipaxis];
	}

#endif // LEVEL_MODE_INVERTED_ENABLE

}
