#include <math.h> // fabsf
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h> // abs

#include "config.h"
#include "drv_time.h"
#include "filter.h"
#include "pid.h"
#include "util.h"

#define RECTANGULAR_RULE_INTEGRAL // a.k.a. Midpoint Rule Integral
//#define TRAPEZOIDAL_RULE_INTEGRAL
//#define SIMPSON_RULE_INTEGRAL

//#define FIRST_DIFFERENCE
#define CENTRAL_DIFFERENCE

// aux_analog[ 0 ] -- aux_analog[ 1 ]

static float pdScaleValue = 1.0f; // updated in pid_precalc()
int pw_sustain_steps = 0; // updated in pid_precalc()

#define AA_KP ( x < 2 ? pdScaleValue * aux_analog[ 0 ] : 1.0f ) // Scale Kp and Kd only for roll and pitch.
#define AA_KI 1.0f
#define AA_KD ( x < 2 ? pdScaleValue * aux_analog[ 1 ] : 1.0f ) // Scale Kp and Kd only for roll and pitch.

// if ( aux[ DEVO_CHAN_11 ] ) { // 3S
// } else { // 4S
// }

// if ( aux[ DEVO_CHAN_8 ] ) { // PID2
// } else { // PID1
// }

// if ( aux[ LEDS_ON ] ) { // ON
// } else { // OFF
// }

float pidkp[ PIDNUMBER ] = PID_KP;
float pidki[ PIDNUMBER ] = PID_KI;
float pidkd[ PIDNUMBER ] = PID_KD;

// "setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
//#define ENABLE_SETPOINT_WEIGHTING
float pidb[ 3 ] = { 1.0, 1.0, 1.0 };

// output limit
const float outlimit[ PIDNUMBER ] = { 0.5, 0.5, 0.25 };

// limit of integral term (abs)
const float integrallimit[ PIDNUMBER ] = { 0.1, 0.1, 0.25 };


// non changable things below
float * pids_array[ 3 ] = { pidkp, pidki, pidkd };
int number_of_increments[ 3 ][ 3 ] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
int current_pid_axis = 0;
int current_pid_term = 0;
uint32_t pid_blink_offset = 0;

float ierror[ PIDNUMBER ] = { 0, 0, 0 };
float pidoutput[ PIDNUMBER ];
extern float setpoint[ PIDNUMBER ]; // control.c
extern float error[ PIDNUMBER ]; // control.c
extern float gyro[ 3 ]; // sixaxis.c
extern float gyro_notch_filtered[ 3 ]; // sixaxis.c
extern int onground; // control.c
extern float vbattfilt_corr; // battery.c
extern float battery_scale_factor; // battery.c
extern float rxcopy[ 4 ]; // control.c
extern float aux_analog[ 2 ]; // rx.c
extern float mixmax; // control.c
extern char aux[ AUXNUMBER ]; // rx.c
extern int packet_period; // rx.c

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

	if ( ( pidoutput[ x ] <= -out_limit ) && ( error[ x ] < 0.0f ) ) {
		i_windup = 1;
	}

	static float avgSetpoint[ 3 ];
	lpf( &avgSetpoint[ x ], setpoint[ x ], ALPHACALC( LOOPTIME, 1e6f / 20.0f ) ); // 20 Hz
	const float absHpfSetpointX = fabsf( setpoint[ x ] - avgSetpoint[ x ] ); // HPF = input - average_input

#ifdef TRANSIENT_WINDUP_PROTECTION
	if ( x < 2 ) { // Only for roll and pitch.
		if ( absHpfSetpointX > 0.1f ) { // 5.7 °/s
			i_windup = 1;
		}
	}
#endif // TRANSIENT_WINDUP_PROTECTION

#ifdef DYNAMIC_ITERM_RESET
	static float lastGyro[ 3 ];
	if ( fabsf( ierror[ x ] ) > integrallimit[ x ] * 0.2f * battery_scale_factor &&
		( ( gyro[ x ] < 0.0f ) != ( lastGyro[ x ] < 0.0f ) ) && // gyro crossed zero
		fabsf( rxcopy[ x ] ) < 0.05f && // stick near center
		// absHpfSetpointX > 0.1f && // and ierror was caused by some recent setpoint change
		true )
	{
		ierror[ x ] *= 0.2f;
	}
	lastGyro[ x ] = gyro[ x ];

	if ( x == 2 ) { // Only for yaw
		static float lastError[ 3 ];
		if ( fabsf( ierror[ x ] ) > integrallimit[ x ] * 0.8f * battery_scale_factor &&
			( ( error[ x ] < 0.0f ) != ( lastError[ x ] < 0.0f ) ) ) // error crossed zero
		{
			ierror[ x ] *= 0.5f;
		}
		lastError[ x ] = error[ x ];
	}
#endif // DYNAMIC_ITERM_RESET

#ifdef RECTANGULAR_RULE_INTEGRAL
	if ( ! i_windup ) {
		ierror[ x ] += error[ x ] * pidki[ x ] * LOOPTIME * 1E-6f * AA_KI;
	}
#endif

#ifdef TRAPEZOIDAL_RULE_INTEGRAL
	static float lasterror[ PIDNUMBER ];
	if ( ! i_windup ) {
		ierror[ x ] += ( error[ x ] + lasterror[ x ] ) * 0.5f * pidki[ x ] * LOOPTIME * 1E-6f * AA_KI;
	}
	lasterror[ x ] = error[ x ];
#endif

#ifdef SIMPSON_RULE_INTEGRAL
	static float lasterror[ PIDNUMBER ];
	static float lasterror2[ PIDNUMBER ];
	if ( ! i_windup ) {
		ierror[ x ] += 0.166666f * ( lasterror2[ x ] + 4 * lasterror[ x ] + error[ x ] ) * pidki[ x ] * LOOPTIME * 1E-6f * AA_KI;
	}
	lasterror2[ x ] = lasterror[ x ];
	lasterror[ x ] = error[ x ];
#endif

	limitf( &ierror[ x ], integrallimit[ x ] * battery_scale_factor );

	float pScaleValueX = 1.0f;
	float iScaleValueX = 1.0f;
	float dScaleValueX = 1.0f;
#ifdef ROLL_FLIP_SMOOTHER
	if ( x < 2 ) { // Only for roll and pitch.
		float absGyroX = ( fabsf( gyro[ x ] ) - (float)RFS_RATE_MIN * DEGTORAD ) / // make a transition from 0 at RFS_RATE_MIN
			( ( (float)RFS_RATE_MAX - (float)RFS_RATE_MIN ) * DEGTORAD ); // to 1.0 at RFS_RATE_MAX
		if ( absGyroX < 0.0f ) { // and limit it to 0.0 below RFS_RATE_MIN
			absGyroX = 0.0f;
		} else if ( absGyroX > 1.0f ) { // and to 1.0 above RFS_RATE_MAX
			absGyroX = 1.0f;
		}
		absGyroX *= 1.0f - rxcopy[ 3 ] / (float)( RFS_THROTTLE_BREAKPOINT ); // less scaling with increasing throttle
		if ( absGyroX < 0.0f ) { // and limit it to 0.0 above RFS_THROTTLE_BREAKPOINT
			absGyroX = 0.0f;
		}
		pScaleValueX = 1.0f - absGyroX * ( 1.0f - (float)RFS_P_SCALER );
		iScaleValueX = 1.0f - absGyroX * ( 1.0f - (float)RFS_I_SCALER );
		dScaleValueX = 1.0f - absGyroX * ( 1.0f - (float)RFS_D_SCALER );
	}
#endif // ROLL_FLIP_SMOOTHER

#ifdef PROP_WASH_REDUCER
	if ( pw_sustain_steps > 0 && x < 2 ) { // Only for roll and pitch.
		pScaleValueX *= (float)PROP_WASH_P_SCALER;
		dScaleValueX *= (float)PROP_WASH_D_SCALER;
	}
#endif // PROP_WASH_REDUCER

	// P term
#ifdef ENABLE_SETPOINT_WEIGHTING
	pidoutput[ x ] = ( setpoint[ x ] * pidb[ x ] - gyro[ x ] ) * pidkp[ x ] * AA_KP * pScaleValueX;
#else // b disabled
	pidoutput[ x ] = error[ x ] * pidkp[ x ] * AA_KP * pScaleValueX;
#endif
	pidoutput[ x ] *= 0.1f; // Since PID_KP is specified 10 times largen compared to traditional SilverWare.
	bb_p[ x ] = pidoutput[ x ];

	// Feedforward term
	if ( x < 2 ) { // Only for roll and pitch.

// https://www.rcgroups.com/forums/showpost.php?p=39606684&postcount=13846
// https://www.rcgroups.com/forums/showpost.php?p=39667667&postcount=13956
#ifdef FEED_FORWARD_STRENGTH

#ifdef RX_SMOOTHING
		static float lastSetpoint[ 2 ];
		float ff = ( setpoint[ x ] - lastSetpoint[ x ] ) * TIMEFACTOR * FEED_FORWARD_STRENGTH * pidkd[ x ] * AA_KD;
		lastSetpoint[ x ] = setpoint[ x ];
#else
		static float lastSetpoint[ 2 ];
		static float setpointDiff[ 2 ];
		static int ffCount[ 2 ];
		if ( setpoint[ x ] != lastSetpoint[ x ] ) {
			const int step_count = packet_period / LOOPTIME; // Spread it evenly over e.g. 5 ms
			setpointDiff[ x ] = ( setpoint[ x ] - lastSetpoint[ x ] ) / step_count;
			ffCount[ x ] = 5;
			lastSetpoint[ x ] = setpoint[ x ];
		}

		float ff = 0.0f;
		if ( ffCount[ x ] > 0 ) {
			--ffCount[ x ];
			ff = setpointDiff[ x ] * TIMEFACTOR * FEED_FORWARD_STRENGTH * pidkd[ x ] * AA_KD;
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
	const float iterm = ierror[ x ] * iScaleValueX;
	pidoutput[ x ] += iterm;
	bb_i[ x ] = iterm;

	// D term
	if ( pidkd[ x ] > 0.0f ) { // skip yaw D term if not set
		float dterm;
#ifdef CASCADE_GYRO_AND_DTERM_FILTER
		const float rate = gyro[ x ];
#else
		const float rate = gyro_notch_filtered[ x ];
#endif // CASCADE_GYRO_AND_DTERM_FILTER
#ifdef FIRST_DIFFERENCE
		static float lastrate[ 3 ];
		dterm = - ( rate - lastrate[ x ] ) * TIMEFACTOR * pidkd[ x ] * AA_KD * dScaleValueX;
		lastrate[ x ] = rate;
#endif // FIRST_DIFFERENCE
#ifdef CENTRAL_DIFFERENCE
		static float lastrate[ PIDNUMBER ][ 2 ];
		dterm = - ( rate - lastrate[ x ][ 1 ] ) / 2.0f * TIMEFACTOR * pidkd[ x ] * AA_KD * dScaleValueX;
		lastrate[ x ][ 1 ] = lastrate[ x ][ 0 ];
		lastrate[ x ][ 0 ] = rate;
#endif // CENTRAL_DIFFERENCE
#ifdef DTERM_LPF_2ND_HZ_BASE
		dterm = dterm_filter( dterm, x );
#endif // DTERM_LPF_2ND_HZ_BASE
#ifdef DTERM_LPF_1ST_A_HZ
		static float dterm1st_a[ 3 ];
		lpf( &dterm1st_a[ x ], dterm, ALPHACALC( LOOPTIME, 1e6f / (float)( DTERM_LPF_1ST_A_HZ ) ) );
		dterm = dterm1st_a[ x ];
#endif // DTERM_LPF_1ST_A_HZ
#ifdef DTERM_LPF_1ST_B_HZ
		static float dterm1st_b[ 3 ];
		lpf( &dterm1st_b[ x ], dterm, ALPHACALC( LOOPTIME, 1e6f / (float)( DTERM_LPF_1ST_B_HZ ) ) );
		dterm = dterm1st_b[ x ];
#endif // DTERM_LPF_1ST_B_HZ
#ifdef BIQUAD_PEAK_HZ
		dterm = peak_filter( dterm, x );
#endif // BIQUAD_PEAK_HZ
		pidoutput[ x ] += dterm;
		bb_d[ x ] = dterm;
	}

#ifdef PID_VOLTAGE_COMPENSATION
	pidoutput[ x ] *= v_compensation;
#endif

	pidoutput[ x ] *= battery_scale_factor;

#ifdef INVERTED_ENABLE
	if ( aux[ FN_INVERTED ] ) {
		pidoutput[ x ] *= (float)( PID_SCALE_INVERTED );
	}
#endif

	limitf( &pidoutput[ x ], out_limit );
}

void pid_precalc()
{
#ifdef PID_VOLTAGE_COMPENSATION
	v_compensation = 4.2f / vbattfilt_corr;
#endif

	pdScaleValue = 1.0f; // constant (no throttle dependent scaling)

#ifdef THROTTLE_PD_ATTENUATION
	extern int pwmdir; // control.c
	extern float bb_throttle; // To include throttle HPF.
	if ( pwmdir == FORWARD ) { // No PD reduction for inverted flying, as prop grip is already weak there.
		pdScaleValue *= 1.0f - bb_throttle * ( 1.0f - (float)TPDA_VALUE ) / (float)TPDA_BREAKPOINT;
		if ( pdScaleValue < (float)TPDA_VALUE ) {
			pdScaleValue = (float)TPDA_VALUE;
		}
	}
#endif // THROTTLE_PD_ATTENUATION

#if 0
	extern float throttle_boost; // control.c
	if ( throttle_boost > 0.0f ) {
		pdScaleValue *= 1.0f - 50.0f * throttle_boost;
		if ( pdScaleValue < 0.5f ) {
			pdScaleValue = 0.5f;
		}
	}
#endif

#ifdef PD_SCALE_YAW_STABILIZER
	const float absyaw = fabsf( rxcopy[ 2 ] );
	// const float absyaw = fabsf( gyro[ 2 ] / ( (float)MAX_RATEYAW * DEGTORAD ) );
	pdScaleValue *= 1.0f + absyaw * ( PD_SCALE_YAW_STABILIZER - 1.0f ); // Scale Kp and Kd on high yaw speeds to avoid oscillations.
#endif

#ifdef PROP_WASH_REDUCER
	extern int pwmdir; // control.c
	extern float bb_throttle; // To include throttle HPF.
	if ( pwmdir == FORWARD ) { // No P and D scaling for inverted flying, as prop grip is already weak there.
		// Only for roll and pitch.
		static int holdoff_steps;
		if ( fabsf( setpoint[ 0 ] ) > 5.0f * DEGTORAD || fabsf( setpoint[ 1 ] ) > 5.0f * DEGTORAD || // 5 °/s roll/pitch setpoint limit
			fabsf( setpoint[ 2 ] ) > 100.0f * DEGTORAD ) // 100 °/s yaw setpoint limit
		{
			holdoff_steps = 70000 / LOOPTIME - 1; // 70 ms holdoff time
		} else if ( holdoff_steps > 0 ) {
			--holdoff_steps;
		}
		if ( holdoff_steps == 0 && // holdoff_steps counted to zero, so setpoint is already 70 ms below the limit.
			bb_throttle > 0.1f && // No scaling when on ground and after throttle punch outs.
			bb_throttle < 0.9f && // Prevent scaling during throttle punch outs.
			( fabsf( gyro[ 0 ] ) > 20.0f * DEGTORAD || fabsf( gyro[ 1 ] ) > 20.0f * DEGTORAD ) ) // 20 °/s gyro limit
		{
			pw_sustain_steps = 25000 / LOOPTIME - 1; // 25 ms sustain time
		} else if ( pw_sustain_steps > 0 ) {
			--pw_sustain_steps;
		}
	}
#endif // PROP_WASH_REDUCER
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

// below are functions used for changing pids by gestures/stick position tuning

void set_current_pid_term( int pid_term )
{
	if ( current_pid_term != pid_term ) {
		pid_blink_offset = gettime();
		current_pid_term = pid_term;
	}
}

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
			set_current_pid_term( 1 );
			break;
		case 1:
			if ( pidkd[ current_pid_axis ] == 0.0f ) { // Skip a zero D term, and go directly to P
				set_current_pid_term( 0 );
			} else {
				set_current_pid_term( 2 );
			}
			break;
		case 2:
			set_current_pid_term( 0 );
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
	pid_blink_offset = gettime() - 200000; // prevent immediate blinking
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

void multiply_current_pid_value( float multiplier )
{
	pid_blink_offset = gettime() - 200000; // prevent blinking while tuning
	pids_array[ current_pid_term ][ current_pid_axis ] *= multiplier;
#ifdef COMBINE_PITCH_ROLL_PID_TUNING
	if ( current_pid_axis == 0 ) {
		pids_array[ current_pid_term ][ current_pid_axis + 1 ] *= multiplier;
	}
#endif
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
	multiply_current_pid_value( multiplier );
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
