#include <math.h> // fabsf
#include <stdint.h>
#include <stdlib.h> // abs

#include "config.h"
#include "filter.h"
#include "pid.h"
#include "util.h"


// #define RECTANGULAR_RULE_INTEGRAL // a.k.a. Midpoint Rule Integral
// #define TRAPEZOIDAL_RULE_INTEGRAL
#define SIMPSON_RULE_INTEGRAL


// aux_analog[ 0 ] -- aux_analog[ 1 ]

#define AA_pdScaleYawStabilizer 1.2f // multiply pdScaleValue by this value at full yaw
static float pdScaleValue = 1.0f; // updated in pid_precalc()

#define AA_pidkp ( x < 2 ? pdScaleValue * aux_analog[ 0 ] : 1.0f ) // Scale Kp and Kd only for roll and pitch.
#define AA_pidki 1.0f
#define AA_pidkd ( x < 2 ? pdScaleValue * aux_analog[ 1 ] : 1.0f ) // Scale Kp and Kd only for roll and pitch.

#define AA_pidScaleInverted 1.2f // multiply by this value when flying inverted

// if ( aux[ DEVO_CHAN_11 ] ) { // 3S
// } else { // 4S
// }

// if ( aux[ DEVO_CHAN_8 ] ) { // PID2
// } else { // PID1
// }

// if ( aux[ LEDS_ON ] ) { // ON
// } else { // OFF
// }

// unscaled PID values tuned for 4S
//                         { roll, pitch, yaw }
float pidkp[ PIDNUMBER ] = { 0.03, 0.03, 0.02 };
float pidki[ PIDNUMBER ] = { 0.50, 0.50, 2.0 };
float pidkd[ PIDNUMBER ] = { 0.15, 0.15, 0.0 };

// "setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
//#define ENABLE_SETPOINT_WEIGHTING
float pidb[ 3 ] = { 1.0, 1.0, 1.0 };

// output limit
const float outlimit[ PIDNUMBER ] = { 0.5, 0.5, 0.25 };

// limit of integral term (abs)
const float integrallimit[ PIDNUMBER ] = { 0.1, 0.1, 0.25 };


// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f H101
#define PID_VC_FACTOR 1.33f


// non changable things below
float * pids_array[ 3 ] = { pidkp, pidki, pidkd };
int number_of_increments[ 3 ][ 3 ] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
int current_pid_axis = 0;
int current_pid_term = 0;
float * current_pid_term_pointer = pidkp;

float ierror[ PIDNUMBER ] = { 0, 0, 0 };
float pidoutput[ PIDNUMBER ];
extern float setpoint[ PIDNUMBER ];
extern float error[ PIDNUMBER ];
extern float gyro[ 3 ];
extern float gyro_notch_filtered[ 3 ];
extern int onground;
extern float vbattfilt;
extern float battery_scale_factor;
extern float rxcopy[ 4 ];
extern float aux_analog[ 2 ];
extern float mixmax;
extern char aux[ AUXNUMBER ];

static float lasterror[ PIDNUMBER ];
#ifdef SIMPSON_RULE_INTEGRAL
static float lasterror2[ PIDNUMBER ];
#endif

// 0.0032f is there for legacy purposes, should be 0.001f = looptime
#define TIMEFACTOR ( 0.0032f / ( LOOPTIME * 1E-6f ) )
static float v_compensation = 1.0f;

float bb_p[ 3 ];
float bb_i[ 3 ];
float bb_d[ 2 ];
float bb_ff[ 3 ];

// pid calculation for acro ( rate ) mode
// input: error[ x ] = setpoint - gyro
// output: pidoutput[ x ] = change required from motors
void pid( int x )
{
	const float out_limit = outlimit[ x ] * v_compensation * battery_scale_factor;

	if ( onground ) {
		ierror[ x ] = 0.0f;
	}

	int i_windup = 0;
	if ( ( pidoutput[ x ] >= out_limit ) && ( error[ x ] > 0.0f ) ) {
		i_windup = 1;
	}

	if ( ( pidoutput[ x ] <= -out_limit) && ( error[ x ] < 0.0f ) ) {
		i_windup = 1;
	}

#ifdef TRANSIENT_WINDUP_PROTECTION
	static float avgSetpoint[ 2 ];
	if ( x < 2 ) { // Only for roll and pitch.
		lpf( &avgSetpoint[ x ], setpoint[ x ], ALPHACALC( LOOPTIME, 1e6f / 20.0f ) ); // 20 Hz
		const float hpfSetpoint = setpoint[ x ] - avgSetpoint[ x ]; // HPF = input - average_input
		if ( fabsf( hpfSetpoint ) > 0.1f ) { // 5.7 °/s
			i_windup = 1;
		}
	}
#endif // TRANSIENT_WINDUP_PROTECTION

#ifdef YAW_WINDUP_RESET
	if ( x == 2 ) { // Only for yaw.
		static float lastGyro;
		if ( fabsf( ierror[ x ] ) > 0.05f * battery_scale_factor && ( gyro[ x ] < 0.0f != lastGyro < 0.0f ) ) { // gyro crossed zero
			ierror[ x ] *= 0.2f;
		}
		lastGyro = gyro[ x ];
	}
#endif // YAW_WINDUP_RESET

	if ( ! i_windup ) {
#ifdef RECTANGULAR_RULE_INTEGRAL
		ierror[ x ] = ierror[ x ] + error[ x ] * pidki[ x ] * LOOPTIME * 1E-6f * AA_pidki;
#endif

#ifdef TRAPEZOIDAL_RULE_INTEGRAL
		ierror[ x ] = ierror[ x ] + ( error[ x ] + lasterror[ x ] ) * 0.5f * pidki[ x ] * LOOPTIME * 1E-6f * AA_pidki;
#endif

#ifdef SIMPSON_RULE_INTEGRAL
		ierror[ x ] = ierror[ x ] + 0.166666f * ( lasterror2[ x ] + 4 * lasterror[ x ] + error[ x ] ) * pidki[ x ] * LOOPTIME * 1E-6f * AA_pidki;
#endif
	}
	lasterror2[ x ] = lasterror[ x ];
	lasterror[ x ] = error[ x ];

	limitf( &ierror[ x ], integrallimit[ x ] * battery_scale_factor );

	// P term
#ifdef ENABLE_SETPOINT_WEIGHTING
	pidoutput[ x ] = ( setpoint[ x ] * pidb[ x ] - gyro[ x ] ) * pidkp[ x ] * AA_pidkp;
#else // b disabled
	pidoutput[ x ] = error[ x ] * pidkp[ x ] * AA_pidkp;
#endif
	bb_p[ x ] = pidoutput[ x ];

	if ( x < 2 ) { // Only for roll and pitch.

// https://www.rcgroups.com/forums/showpost.php?p=39606684&postcount=13846
// https://www.rcgroups.com/forums/showpost.php?p=39667667&postcount=13956
#ifdef FEED_FORWARD_STRENGTH

#ifdef RX_SMOOTHING
		static float lastSetpoint[ 2 ];
		float ff = ( setpoint[ x ] - lastSetpoint[ x ] ) * TIMEFACTOR * FEED_FORWARD_STRENGTH * pidkd[ x ] * AA_pidkd;
		lastSetpoint[ x ] = setpoint[ x ];
#else
		static float lastSetpoint[ 2 ];
		static float setpointDiff[ 2 ];
		static int ffCount[ 2 ];
		if ( setpoint[ x ] != lastSetpoint[ x ] ) {
			static int step_count = 5000 / LOOPTIME; // Spread it evenly over 5 ms (PACKET_PERIOD)
			setpointDiff[ x ] = ( setpoint[ x ] - lastSetpoint[ x ] ) / step_count;
			ffCount[ x ] = 5;
			lastSetpoint[ x ] = setpoint[ x ];
		}

		float ff = 0.0f;
		if ( ffCount[ x ] > 0 ) {
			--ffCount[ x ];
			ff = setpointDiff[ x ] * TIMEFACTOR * FEED_FORWARD_STRENGTH * pidkd[ x ] * AA_pidkd;
		}
#endif // RX_SMOOTHING

		// Distribute ff over a few loops. Otherwise it gets clipped and leads to audible motor clicks:
#if 1
		static float avgFF[ 2 ];
		lpf( &avgFF[ x ], ff, ALPHACALC( LOOPTIME, 1e6f / 20.0f ) ); // 20 Hz
		ff = avgFF[ x ];
#else
		// Moving average filter:
		#define MA_SIZE ( 2000 / LOOPTIME ) // 16 points at 8k loop frequency
		static float ma_value[ 2 ];
		static float ma_array[ 2 ][ MA_SIZE ];
		static uint8_t ma_index[ 2 ];
		ma_value[ x ] -= ma_array[ x ][ ma_index[ x ] ];
		ma_value[ x ] += ff;
		ma_array[ x ][ ma_index[ x ] ] = ff;
		++ma_index[ x ];
		if ( ma_index[ x ] == MA_SIZE ) {
			ma_index[ x ] = 0;
		}
		ff = ma_value[ x ] / MA_SIZE;
#endif

		// Attenuate FF towards stick center position (aka FF Transition):
		const float absStickDeflection = fabsf( rxcopy[ x ] );
#if 0
		const float scalingValue = 0.0f; // Scale FF by that much at stick center.
		const float scalingBreakpoint = 0.5f * AA_ffScalingBreakpoint; // No scaling when the stick deflection is above that value.
		if ( absStickDeflection < scalingBreakpoint ) {
			ff *= 1.0f + ( 1.0f - absStickDeflection / scalingBreakpoint ) * ( scalingValue - 1.0f );
		}
#else
		ff *= absStickDeflection; // Scale linearly from center to max deflection.
#endif

#ifdef SMART_FF
		if ( ff < 0.0f == pidoutput[ x ] < 0.0f ) {
			if ( fabsf( ff ) > fabsf( pidoutput[ x ] ) ) {
				pidoutput[ x ] = ff; // Take the larger of P or FF as long as P and FF have the same sign.
			}
		} else {
			pidoutput[ x ] += ff; // Always add FF if the signs are opposite.
		}
#else
		pidoutput[ x ] += ff;
#endif // SMART_FF
		bb_ff[ x ] = ff;

#endif // FEED_FORWARD_STRENGTH

	} else { // x == 2: yaw

#ifdef FEED_FORWARD_YAW
	#ifndef RX_SMOOTHING
		#error "FEED_FORWARD_YAW only works with RX_SMOOTHING enabled."
	#endif
		static float lastSetpointYaw;
		float ff = ( setpoint[ x ] - lastSetpointYaw ) * TIMEFACTOR * FEED_FORWARD_YAW;
		lastSetpointYaw = setpoint[ x ];

		static float avgFFyaw;
		lpf( &avgFFyaw, ff, ALPHACALC( LOOPTIME, 1e6f / 20.0f ) ); // 20 Hz
		ff = avgFFyaw;

		pidoutput[ x ] += ff;
		bb_ff[ x ] = ff;
#endif // FEED_FORWARD_YAW

	}

	// I term
	pidoutput[ x ] += ierror[ x ];
	bb_i[ x ] = ierror[ x ];

	// D term
	if ( pidkd[ x ] > 0.0f ) { // skip yaw D term if not set
		float dterm;
		static float lastrate[ 3 ];
#ifdef CASCADE_GYRO_AND_DTERM_FILTER
		dterm = - ( gyro[ x ] - lastrate[ x ] ) * pidkd[ x ] * TIMEFACTOR * AA_pidkd;
		lastrate[ x ] = gyro[ x ];
#else
		dterm = - ( gyro_notch_filtered[ x ] - lastrate[ x ] ) * pidkd[ x ] * TIMEFACTOR * AA_pidkd;
		lastrate[ x ] = gyro_notch_filtered[ x ];
#endif // CASCADE_GYRO_AND_DTERM_FILTER
		dterm = dterm_filter( dterm, x );
#ifdef DTERM_LPF_1ST_HZ
		static float dterm1st[ 3 ];
		lpf( &dterm1st[ x ], dterm, ALPHACALC( LOOPTIME, 1e6f / (float)( DTERM_LPF_1ST_HZ ) ) );
		dterm = dterm1st[ x ];
#endif // DTERM_LPF_1ST_HZ
		pidoutput[ x ] += dterm;
		bb_d[ x ] = dterm;
	}

#ifdef PID_VOLTAGE_COMPENSATION
	pidoutput[ x ] *= v_compensation;
#endif

	pidoutput[ x ] *= battery_scale_factor;

#ifdef INVERTED_ENABLE
	if ( aux[ FN_INVERTED ] ) {
		pidoutput[ x ] *= AA_pidScaleInverted;
	}
#endif

	limitf( &pidoutput[ x ], out_limit );
}

void pid_precalc()
{
#ifdef PID_VOLTAGE_COMPENSATION
	v_compensation = mapf( vbattfilt, 3.0f, 4.0f, PID_VC_FACTOR, 1.0f );
	if ( v_compensation > PID_VC_FACTOR ) {
		v_compensation = PID_VC_FACTOR;
	}
	if ( v_compensation < 1.0f ) {
		v_compensation = 1.0f;
	}
#endif

	pdScaleValue = 1.0f; // constant (no throttle dependent scaling)

#ifdef AA_pdScaleYawStabilizer
	const float absyaw = fabsf( rxcopy[ 2 ] );
	// const float absyaw = fabsf( gyro[ 2 ] / ( (float)MAX_RATEYAW * DEGTORAD ) );
	pdScaleValue *= 1 + absyaw * ( AA_pdScaleYawStabilizer - 1 ); // Increase Kp and Kd on high yaw speeds to avoid iTerm Rotation related wobbles.
#endif
}

// I vector rotation by joelucid
// https://www.rcgroups.com/forums/showpost.php?p=39354943&postcount=13468
void rotateErrors()
{
	// // rotation around x axis:
	// float temp = gyro[ 0 ] * LOOPTIME * 1E-6f;
	// ierror[ 1 ] -= ierror[ 2 ] * temp;
	// ierror[ 2 ] += ierror[ 1 ] * temp;

	// // rotation around y axis:
	// temp = gyro[ 1 ] * LOOPTIME * 1E-6f;
	// ierror[ 2 ] -= ierror[ 0 ] * temp;
	// ierror[ 0 ] += ierror[ 2 ] * temp;

	// // rotation around z axis:
	// temp = gyro[ 2 ] * LOOPTIME * 1E-6f;
	// ierror[ 0 ] -= ierror[ 1 ] * temp;
	// ierror[ 1 ] += ierror[ 0 ] * temp;

	// ---

	// http://fooplot.com/#W3sidHlwZSI6MCwiZXEiOiJ4KjAuMDMxIiwiY29sb3IiOiIjODA4MDgwIn0seyJ0eXBlIjoyLCJlcXgiOiJjb3MocykiLCJlcXkiOiJzaW4ocykiLCJjb2xvciI6IiMwMDAwMDAiLCJzbWluIjoiMCIsInNtYXgiOiIwLjAzMSIsInNzdGVwIjoiLjAwMDEifSx7InR5cGUiOjIsImVxeCI6IjEiLCJlcXkiOiJzIiwiY29sb3IiOiIjZmYwMDAwIiwic21pbiI6IjAiLCJzbWF4IjoiMC4wMzEiLCJzc3RlcCI6Ii4wMDAxIn0seyJ0eXBlIjoyLCJlcXgiOiIxLXMqcy8yIiwiZXF5IjoicyIsImNvbG9yIjoiI0ZGMDBGRiIsInNtaW4iOiIwIiwic21heCI6IjAuMDMxIiwic3N0ZXAiOiIuMDAwMSJ9LHsidHlwZSI6MTAwMCwid2luZG93IjpbIjAuOTg0IiwiMS4wMTYiLCIwIiwiMC4wMzIiXSwic2l6ZSI6WzY1MCw2NTBdfV0-
	// yaw angle (at 1800?/s -> a2 = 1800/180*3.1416*0.001 = 0.031)
	const float a2 = gyro[ 2 ] * LOOPTIME * 1E-6f;
	// small angle approximations
	const float COS_a2 = 1 - a2 * a2 / 2;
	const float SIN_a2 = a2; // Adding -a2*a2*a2/6 yields no further improvement.
	const float ierror0 = ierror[ 0 ] * COS_a2 - ierror[ 1 ] * SIN_a2;
	const float ierror1 = ierror[ 1 ] * COS_a2 + ierror[ 0 ] * SIN_a2;
	ierror[ 0 ] = ierror0;
	ierror[ 1 ] = ierror1;
}

// below are functions used with gestures for changing pids by a percentage

// Cycle through P / I / D - The initial value is P
// The return value is the currently selected TERM (after setting the next one)
// 1: P
// 2: I
// 3: D
// The return value is used to blink the leds in main.c
int next_pid_term( void )
{
	switch ( current_pid_term ) {
		case 0:
			current_pid_term_pointer = pidki;
			current_pid_term = 1;
			break;
		case 1:
			if ( pidkd[ current_pid_axis ] == 0.0f ) { // Skip a zero D term, and go directly to P
				current_pid_term_pointer = pidkp;
				current_pid_term = 0;
			} else {
				current_pid_term_pointer = pidkd;
				current_pid_term = 2;
			}
			break;
		case 2:
			current_pid_term_pointer = pidkp;
			current_pid_term = 0;
			break;
	}
	return current_pid_term + 1;
}

// Cycle through the axis - Initial is Roll
// Return value is the selected axis, after setting the next one.
// 1: Roll
// 2: Pitch
// 3: Yaw
// The return value is used to blink the leds in main.c
int next_pid_axis( void )
{
	const int size = 3;
	if ( current_pid_axis == size - 1 ) {
		current_pid_axis = 0;
	} else {
#ifdef COMBINE_PITCH_ROLL_PID_TUNING
		if ( current_pid_axis < 2 ) {
			// Skip axis == 1 which is roll, and go directly to 2 (Yaw)
			current_pid_axis = 2;
		}
#else
		++current_pid_axis;
#endif
	}
	return current_pid_axis + 1;
}

#define PID_GESTURES_MULTI 1.1f

static int change_pid_value( int increase )
{
	float multiplier = 1.0f / (float)PID_GESTURES_MULTI;
	if ( increase ) {
		multiplier = (float)PID_GESTURES_MULTI;
		++number_of_increments[ current_pid_term ][ current_pid_axis ];
	} else {
		--number_of_increments[ current_pid_term ][ current_pid_axis ];
	}
	current_pid_term_pointer[ current_pid_axis ] *= multiplier;
#ifdef COMBINE_PITCH_ROLL_PID_TUNING
	if ( current_pid_axis == 0 ) {
		current_pid_term_pointer[ current_pid_axis + 1 ] *= multiplier;
	}
#endif
	return abs( number_of_increments[ current_pid_term ][ current_pid_axis ] );
}

// Increase currently selected term for the currently selected axis by 10%.
// The return value, is absolute number of times the specific term/axis was increased or decreased.
int increase_pid( void )
{
	return change_pid_value( 1 );
}

int decrease_pid( void )
{
	return change_pid_value( 0 );
}
