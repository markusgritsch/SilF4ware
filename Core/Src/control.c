#include <math.h> // fabsf, sqrtf
#include <stdbool.h>

#include "angle_pid.h"
#include "config.h"
#include "drv_dshot.h"
#include "filter.h"
#include "pid.h"
#include "stick_vector.h"
#include "util.h"

#include "debug.h"

int onground = 1;

float thrsum;
float mixmax;
float throttle_reversing_kick;
float throttle_reversing_kick_decrement;

float error[ PIDNUMBER ];
float setpoint[ PIDNUMBER ];

extern float vbattfilt; // battery.c
extern float vbatt_comp;

extern float gyro[ 3 ]; // sixaxis.c

extern float pidoutput[ PIDNUMBER ]; // pid.c
extern int pwmdir; // pid.c
extern float ierror[ PIDNUMBER ]; // pid.c

extern bool failsafe; // rx.c
extern float rx[ 4 ]; // rx.c
extern char aux[ AUXNUMBER ]; // rx.c
extern float aux_analog[ 2 ];

extern bool ledcommand; // led.c

float rxcopy[ 4 ];

void control( void )
{
	// rates / expert mode
	float rate_multiplier = 1.0f;

	if ( aux[ RATES ] ) {
	} else {
		rate_multiplier = LOW_RATES_MULTI;
	}

#ifdef INVERTED_ENABLE
	if ( ( aux[ FN_INVERTED ] && pwmdir != REVERSE ) || ( ! aux[ FN_INVERTED ] && pwmdir != FORWARD ) ) {
		// On motor direction changed:
		ierror[ 0 ] = ierror[ 1 ] = ierror[ 2 ] = 0.0f;
		throttle_hpf_reset( 200 ); // ms
		dterm_filter_reset( 0 ); // ms
#ifdef THROTTLE_REVERSING_KICK
		throttle_reversing_kick = THROTTLE_REVERSING_KICK;
		throttle_reversing_kick_decrement = THROTTLE_REVERSING_KICK * (float)LOOPTIME / 100000.0f; // 100 ms
#endif
	}

	if ( aux[ FN_INVERTED ] ) {
		pwmdir = REVERSE;
	} else {
		pwmdir = FORWARD;
	}
#endif

	for ( int i = 0; i < 3; ++i ) {
		rxcopy[ i ] = rx[ i ] * rate_multiplier;

#ifdef STICKS_DEADBAND
		if ( fabsf( rxcopy[ i ] ) <= STICKS_DEADBAND ) {
			rxcopy[ i ] = 0.0f;
		} else {
			if ( rxcopy[ i ] >= 0 ) {
				rxcopy[ i ] = mapf( rxcopy[ i ], STICKS_DEADBAND, 1, 0, 1 );
			} else {
				rxcopy[ i ] = mapf( rxcopy[ i ], -STICKS_DEADBAND, -1, 0, -1 );
			}
		}
#endif
	}

	rxcopy[ 3 ] = rx[ 3 ]; // throttle

#ifdef RX_SMOOTHING
	static float rxsmooth[ 4 ];
	static float lastRXcopy[ 4 ];
	static float stepRX[ 4 ];
	static int countRX[ 4 ];
	for ( int i = 0; i < 4; ++i ) {
		if ( rxcopy[ i ] != lastRXcopy[ i ] ) {
			static int step_count = 5000 / LOOPTIME; // Spread it evenly over 5 ms (PACKET_PERIOD)
			stepRX[ i ] = ( rxcopy[ i ] - lastRXcopy[ i ] ) / step_count;
			countRX[ i ] = step_count;
			lastRXcopy[ i ] = rxcopy[ i ];
		}
		if ( countRX[ i ] > 0 ) {
			--countRX[ i ];
			rxsmooth[ i ] += stepRX[ i ];
			rxcopy[ i ] = rxsmooth[ i ];
		} else {
			rxsmooth[ i ] = rxcopy[ i ];
		}
	}
#endif

	pid_precalc();

	// flight control

	if ( aux[ LEVELMODE ] ) { // level mode
		extern float angleerror[];
		extern float errorvect[]; // level mode angle error calculated by stick_vector.c
		extern float GEstG[ 3 ]; // gravity vector for yaw feedforward
		float yawerror[ 3 ] = { 0 }; // yaw rotation vector

		// calculate roll / pitch error
		stick_vector( rxcopy, 0 );

		float yawrate = rxcopy[ 2 ] * (float)MAX_RATEYAW * DEGTORAD;
		// apply yaw from the top of the quad
		yawerror[ 0 ] = GEstG[ 1 ] * yawrate;
		yawerror[ 1 ] = -GEstG[ 0 ] * yawrate;
		yawerror[ 2 ] = GEstG[ 2 ] * yawrate;

		// pitch and roll
		for ( int i = 0; i <= 1; ++i ) {
			angleerror[ i ] = errorvect[ i ];
			error[ i ] = angle_pid( i ) + yawerror[ i ] - gyro[ i ];
		}
		// yaw
		error[ 2 ] = yawerror[ 2 ] - gyro[ 2 ];

		// Set ierror to zero, otherwise it builds up and causes bounce back.
		ierror[ 0 ] = 0.0f; ierror[ 1 ] = 0.0f;
	} else { // rate mode
		setpoint[ 0 ] = rxcopy[ 0 ] * (float)MAX_RATE * DEGTORAD;
		setpoint[ 1 ] = rxcopy[ 1 ] * (float)MAX_RATE * DEGTORAD;
		setpoint[ 2 ] = rxcopy[ 2 ] * (float)MAX_RATEYAW * DEGTORAD;

		for ( int i = 0; i < 3; ++i ) {
			error[ i ] = setpoint[ i ] - gyro[ i ];
		}
	}

#ifdef PID_ROTATE_ERRORS
	rotateErrors();
#endif

	pid( 0 );
	pid( 1 );
	pid( 2 );

	float throttle = rxcopy[ 3 ];

#ifdef THRUST_LINEARIZATION
	#define AA_motorCurve THRUST_LINEARIZATION // 0 .. linear, 1 .. quadratic
	const float aa = AA_motorCurve;
	throttle = throttle * ( throttle * aa + 1 - aa ); // invert the motor curve correction applied further below
#endif

	if ( failsafe || aux[ THROTTLE_KILL_SWITCH ] ) {
		static uint32_t counter;
		++counter;
		if ( counter % 8 != 0 ) { // Make sure, we send either pwm_set() or motorbeep().
			for ( int i = 0; i < 4; ++i ) {
				pwm_set( i, 0 );
			}
		} else {
#ifdef MOTOR_BEEPS
#ifndef MOTOR_BEEPS_CHANNEL
#define MOTOR_BEEPS_CHANNEL CH_OFF
#endif
			motorbeep( MOTOR_BEEPS_CHANNEL );
#endif
		}

		onground = 1;
		thrsum = 0;
	} else { // motors on - normal flight
		onground = 0;

#ifdef THROTTLE_TRANSIENT_COMPENSATION_FACTOR
		if ( aux[ RATES ] ) {
			throttle += (float)THROTTLE_TRANSIENT_COMPENSATION_FACTOR * throttle_hpf( throttle );
			if ( throttle < 0.0f ) {
				throttle = 0;
			} else if ( throttle > 1.0f ) {
				throttle = 1.0f;
			}
		}
#endif

#ifdef THROTTLE_REVERSING_KICK
		// throttle = 0;
		// pidoutput[ 0 ] = pidoutput[ 1 ] = pidoutput[ 2 ] = 0;
		if ( throttle_reversing_kick > 0 ) {
			if ( throttle < throttle_reversing_kick ) {
				throttle = throttle_reversing_kick;
			}
			throttle_reversing_kick -= throttle_reversing_kick_decrement;
		}
#endif

#ifdef LVC_LOWER_THROTTLE
		static float throttle_i = 0.0f;
		float throttle_p = 0.0f;

			if ( vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW ) {
				throttle_p = ( (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt ) *(float)LVC_LOWER_THROTTLE_KP;
			}
			if ( vbatt_comp < (float) LVC_LOWER_THROTTLE_VOLTAGE ) {
				throttle_p = ((float) LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) *(float) LVC_LOWER_THROTTLE_KP;
			}
			if ( throttle_p > 1.0f ) {
				throttle_p = 1.0f;
			}
			if ( throttle_p > 0 ) {
				throttle_i += throttle_p * 0.0001f; //ki
			} else {
				throttle_i -= 0.001f;// ki on release
			}

			if ( throttle_i > 0.5f ) {
				throttle_i = 0.5f;
			} else if ( throttle_i < 0.0f ) {
				throttle_i = 0.0f;
			}

		throttle -= throttle_p + throttle_i;
#endif

		// throttle *= 2.0f /3.0f; // Limit throttle when flying with 3S because the motors cannot take more than 2S.

#ifdef INVERT_YAW_PID
		pidoutput[ 2 ] = -pidoutput[ 2 ];
#endif

		float mix[ 4 ];

#ifdef INVERTED_ENABLE
		if ( pwmdir == REVERSE ) { // inverted flight
			mix[ MOTOR_FR - 1 ] = throttle + pidoutput[ ROLL ] + pidoutput[ PITCH ] - pidoutput[ YAW ]; // FR
			mix[ MOTOR_FL - 1 ] = throttle - pidoutput[ ROLL ] + pidoutput[ PITCH ] + pidoutput[ YAW ]; // FL
			mix[ MOTOR_BR - 1 ] = throttle + pidoutput[ ROLL ] - pidoutput[ PITCH ] + pidoutput[ YAW ]; // BR
			mix[ MOTOR_BL - 1 ] = throttle - pidoutput[ ROLL ] - pidoutput[ PITCH ] - pidoutput[ YAW ]; // BL
		} else
#endif
		{ // normal mixer
			mix[ MOTOR_FR - 1 ] = throttle - pidoutput[ ROLL ] - pidoutput[ PITCH ] + pidoutput[ YAW ]; // FR
			mix[ MOTOR_FL - 1 ] = throttle + pidoutput[ ROLL ] - pidoutput[ PITCH ] - pidoutput[ YAW ]; // FL
			mix[ MOTOR_BR - 1 ] = throttle - pidoutput[ ROLL ] + pidoutput[ PITCH ] - pidoutput[ YAW ]; // BR
			mix[ MOTOR_BL - 1 ] = throttle + pidoutput[ ROLL ] + pidoutput[ PITCH ] + pidoutput[ YAW ]; // BL
		}

#ifdef INVERT_YAW_PID
		// we invert again cause it's used by the pid internally (for limit)
		pidoutput[ 2 ] = -pidoutput[ 2 ];
#endif

#ifdef MIX_SCALING
		float minMix = 1000.0f;
		float maxMix = -1000.0f;
		for ( int i = 0; i < 4; ++i ) {
			if ( mix[ i ] < minMix ) {
				minMix = mix[ i ];
			}
			if ( mix[ i ] > maxMix ) {
				maxMix = mix[ i ];
			}
		}
		const float mixRange = maxMix - minMix;
		float reduceAmount = 0.0f;
		if ( mixRange > 1.0f ) {
			const float scale = 1.0f / mixRange;
			for ( int i = 0; i < 4; ++i ) {
				mix[ i ] *= scale;
			}
			minMix *= scale;
			reduceAmount = minMix;
		} else {
			if ( maxMix > 1.0f ) {
				reduceAmount = maxMix - 1.0f;
			}
#ifdef ALLOW_MIX_INCREASING
			else if ( minMix < 0.0f ) {
				reduceAmount = minMix;
			}
#endif // ALLOW_MIX_INCREASING
		}
		if ( reduceAmount != 0.0f ) {
			for ( int i = 0; i < 4; ++i ) {
				mix[ i ] -= reduceAmount;
			}
		}
#endif // MIX_SCALING

		thrsum = 0;
		mixmax = 0;

		for ( int i = 0; i < 4; ++i ) {
#if defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)

#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
			if ( aux[ MOTORS_TO_THROTTLE_MODE ] ) {
#endif
				mix[ i ] = throttle;
				if ( i == MOTOR_FL - 1 && ( rx[ ROLL ] > 0.5f || rx[ PITCH ] < -0.5f ) ) { mix[ i ] = 0; }
				if ( i == MOTOR_BL - 1 && ( rx[ ROLL ] > 0.5f || rx[ PITCH ] > 0.5f ) ) { mix[ i ] = 0; }
				if ( i == MOTOR_FR - 1 && ( rx[ ROLL ] < -0.5f || rx[ PITCH ] < -0.5f ) ) { mix[ i ] = 0; }
				if ( i == MOTOR_BR - 1 && ( rx[ ROLL ] < -0.5f || rx[ PITCH ] > 0.5f ) ) { mix[ i ] = 0; }
				ledcommand = 1;
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
			}
#endif

#endif

#ifdef THRUST_LINEARIZATION
			// Computationally quite expensive:
			static float a, a_reci, b, b_sq;
			if ( a != AA_motorCurve ) {
				a = AA_motorCurve;
				if ( a > 0.0f ) {
					a_reci = 1 / a;
					b = ( 1 - a ) / ( 2 * a );
					b_sq = b * b;
				}
			}
			float mix_i = mix[ i ];
			if ( mix_i > 0.0f && a > 0.0f ) {
				mix_i = sqrtf( mix[ i ] * a_reci + b_sq ) - b;
			}
			pwm_set( i, mix_i );
#else
			pwm_set( i, mix[ i ] );
#endif

			if ( mix[ i ] < 0 ) {
				mix[ i ] = 0;
			} else if ( mix[ i ] > 1 )	{
				mix[ i ] = 1;
			}
			thrsum += mix[ i ];
			if ( mixmax < mix[ i ] ) {
				mixmax = mix[ i ];
			}
		}
		thrsum = thrsum / 4;
	}// end motors on
}
