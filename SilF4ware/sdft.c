#include <complex.h>
#include <math.h> // powf
#include <stdbool.h>

#include "config.h"
#include "defines.h"
#include "filter.h"
#include "util.h"

#define MIN_HZ 80
#define MAX_HZ 500 // limit: 500 Hz if SAMPLE_PERIOD is 1 ms.
#define SAMPLE_PERIOD 1000 // us. Sampling every 1 ms is enough for MAX_HZ up to 500.
#define SDFT_SIZE ( 100000 / SAMPLE_PERIOD ) // 0.1 seconds time window gives 10 Hz resolution.
#define DAMPING_FACTOR 0.999f
// #define DAMPING_FACTOR nextafterf( 1.0f, 0.0f )

static int idx = 0;
static float x[ SDFT_AXES ][ SDFT_SIZE ];
static float complex X[ SDFT_AXES ][ SDFT_SIZE ];
static float complex coeff[ SDFT_SIZE ];

void sdft_init()
{
	for ( int k = 0; k < SDFT_SIZE; ++k ) {
		const float phi = 2.0f * PI_F * (float)k / (float)SDFT_SIZE;
		coeff[ k ] = DAMPING_FACTOR * ( cos_approx( phi ) + I * sin_approx( phi ) ); // cexpf( I * phi )
	}
}

void sdft_step() // called every LOOPTIME
{
	#define SUBSAMPLES ( SAMPLE_PERIOD / LOOPTIME )

	static float accu[ 3 ]; // Accumulated and then averaged input signal.
	static int subsample_count = 0;

#if 1 // Analyze pidoutput (1) or gyro (0) signal
	extern float pidoutput[ PIDNUMBER ]; // pid.c
	accu[ 0 ] += pidoutput[ 0 ];
	accu[ 1 ] += pidoutput[ 1 ];
	accu[ 2 ] += pidoutput[ 2 ];
#else
	extern float gyro[ 3 ]; // sixaxis.c
	accu[ 0 ] += gyro[ 0 ];
	accu[ 1 ] += gyro[ 1 ];
	accu[ 2 ] += gyro[ 2 ];
#endif

	bool was_in_sync = true;
	extern bool packet_received; // usermain.c
	++subsample_count;
	if ( subsample_count == SUBSAMPLES
		|| packet_received ) // This way we sync SDFT processing to the subsequent loop after receiving a radio packet.
	{
		if ( packet_received && subsample_count != SUBSAMPLES ) {
			was_in_sync = false;
		}
		accu[ 0 ] /= subsample_count;
		accu[ 1 ] /= subsample_count;
		accu[ 2 ] /= subsample_count;
		subsample_count = 0;
	}

	for ( int axis = 0; axis < SDFT_AXES; ++axis ) {
		static int index_1st[ SDFT_AXES ];
		static int index_2nd[ SDFT_AXES ];
		static float value_1st[ SDFT_AXES ];
		static float value_2nd[ SDFT_AXES ];
		static float delta[ SDFT_AXES ];

		const int min_bin_index = (float)MIN_HZ * (float)SDFT_SIZE * ( SAMPLE_PERIOD * 1e-6f ) + 0.5f;
		const int max_bin_index = (float)MAX_HZ * (float)SDFT_SIZE * ( SAMPLE_PERIOD * 1e-6f ) + 0.5f;

		if ( subsample_count == 0 ) {
			// Changing the notch filter frequency is not instantaneous and transitioning to a new frequency
			// takes some time. To avoid long transients for example if the 1st and 2nd filter switch places,
			// we ensure that index_1st is always higher than index_2nd:
			if ( index_1st[ axis ] < index_2nd[ axis ] ) {
				const int temp = index_1st[ axis ];
				index_1st[ axis ] = index_2nd[ axis ];
				index_2nd[ axis ] = temp;
			}

			static float f_1st_Hz = MAX_HZ;
			static float f_2nd_Hz = MAX_HZ;
			if ( was_in_sync ) {
				f_1st_Hz = index_1st[ axis ] / ( SAMPLE_PERIOD * 1e-6f ) / (float)SDFT_SIZE;
				f_2nd_Hz = index_2nd[ axis ] / ( SAMPLE_PERIOD * 1e-6f ) / (float)SDFT_SIZE;
			}

			static float f_Hz_filt[ SDFT_AXES * 2 ];
			extern float sdft_notch_Hz[ SDFT_AXES * 2 ]; // filter.c
			lpf( &f_Hz_filt[ axis * 2 ], f_1st_Hz, ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
			lpf( &sdft_notch_Hz[ axis * 2 ], f_Hz_filt[ axis * 2 ], ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
			lpf( &f_Hz_filt[ axis * 2 + 1 ], f_2nd_Hz, ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
			lpf( &sdft_notch_Hz[ axis * 2 + 1 ], f_Hz_filt[ axis * 2 + 1 ], ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz

			index_1st[ axis ] = max_bin_index;
			index_2nd[ axis ] = max_bin_index;
			value_1st[ axis ] = 0.0f;
			value_2nd[ axis ] = 0.0f;

			const float r_to_N = powf( DAMPING_FACTOR, SDFT_SIZE );
			delta[ axis ] = accu[ axis ] - r_to_N * x[ axis ][ idx ];
			x[ axis ][ idx ] = accu[ axis ];
			accu[ axis ] = 0.0f;
		}

		int bin_chunk = ( max_bin_index - min_bin_index + 1 ) / SUBSAMPLES;
		if ( bin_chunk * SUBSAMPLES < max_bin_index - min_bin_index + 1 ) {
			++bin_chunk;
		}
		const int start_bin_index = min_bin_index + bin_chunk * subsample_count;
		for ( int k = start_bin_index; k < start_bin_index + bin_chunk && k <= max_bin_index; ++k ) {
			X[ axis ][ k ] = coeff[ k ] * ( X[ axis ][ k ] + delta[ axis ] ); // Do the actual SDFT calculation.

			// Find the two most dominant peaks:
			const float re = crealf( X[ axis ][ k ] );
			const float im = cimagf( X[ axis ][ k ] );
			const float mag_squared = re * re + im * im;
			if ( mag_squared > value_1st[ axis ] ) {
				value_2nd[ axis ] = value_1st[ axis ];
				index_2nd[ axis ] = index_1st[ axis ];
				value_1st[ axis ] = mag_squared;
				index_1st[ axis ] = k;
			} else if ( mag_squared > value_2nd[ axis ] ) {
				value_2nd[ axis ] = mag_squared;
				index_2nd[ axis ] = k;
			}
		}
	} // for axis

	if ( subsample_count == 0 ) {
		++idx;
		if ( idx == SDFT_SIZE ) {
			idx = 0;
		}
	}
}
