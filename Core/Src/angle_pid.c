#include "config.h"
#include "util.h"

#define  APIDNUMBER  2

// ANGLE PIDS - used in level mode
// acro pids are also used at the same time in level mode

// yaw is done by the rate yaw pid
// Kp                          ROLL + PITCH
float apidkp[ APIDNUMBER ] = { 20 };

// Kd
float apidkd[ APIDNUMBER ] = { 0.0 };

// rate limit
#define OUTLIMIT_FLOAT (float)LEVEL_MAX_RATE

extern float timefactor; // pid.c

float apidoutput[ APIDNUMBER ];
float angleerror[ APIDNUMBER ];
float lasterror[ APIDNUMBER ];

float angle_pid( int x )
{
	// P term
	apidoutput[ x ] = angleerror[ x ] * apidkp[ 0 ];

	// D term
	apidoutput[ x ] = apidoutput[ x ] - ( angleerror[ x ] - lasterror[ x ] ) * apidkd[ 0 ] * timefactor;
	lasterror[ x ] = angleerror[ x ];

	limitf( &apidoutput[ x ], OUTLIMIT_FLOAT );

	return apidoutput[ x ];
}
