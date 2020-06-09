#include <complex.h>
#include <math.h> // powf
#include <stdbool.h>

#include "config.h"
#include "defines.h"
#include "filter.h"
#include "util.h"

// #define DAMPING_FACTOR nextafterf( 1.0f, 0.0f )
#define DAMPING_FACTOR 1.0f
#define SAMPLE_PERIOD 1000 // us. Sampling every 1 ms is enough for MAX_HZ up to 500.
#define SDFT_SIZE ( 100000 / SAMPLE_PERIOD ) // 0.1 seconds time window gives 10 Hz resolution.
static float complex coeff[ SDFT_SIZE ];
static float x[ 2 ][ SDFT_SIZE ];
static float complex X[ 2 ][ SDFT_SIZE ];
static int idx = 0;

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

	static float gyro_accu[ 2 ];
	static int subsample_count = 0;

	extern bool packet_received;
	extern float gyro[ 3 ];
	gyro_accu[ 0 ] += gyro[ 0 ];
	gyro_accu[ 1 ] += gyro[ 1 ];
	++subsample_count;
	if ( subsample_count == SUBSAMPLES
		|| packet_received ) // This way we sync SDFT processing to the subsequent loop after receiving a radio packet.
	{
		gyro_accu[ 0 ] /= subsample_count;
		gyro_accu[ 1 ] /= subsample_count;
		subsample_count = 0;
	}

	if ( subsample_count == 0 ) {
		for ( int axis = 0; axis < 2; ++axis ) { // Only for roll and pitch.
			const float r_to_N = powf( DAMPING_FACTOR, SDFT_SIZE );
			const float delta = gyro_accu[ axis ] - r_to_N * x[ axis ][ idx ];
			x[ axis ][ idx ] = gyro_accu[ axis ];
			gyro_accu[ axis ] = 0.0f;

			#define MIN_HZ 90
			#define MAX_HZ 400 // limit: 500 Hz if SAMPLE_PERIOD is 1 ms.
			const int min_bin_index = (float)MIN_HZ * (float)SDFT_SIZE * ( SAMPLE_PERIOD * 1e-6f ) + 0.5f;
			const int max_bin_index = (float)MAX_HZ * (float)SDFT_SIZE * ( SAMPLE_PERIOD * 1e-6f ) + 0.5f;

			int index_1st = max_bin_index, index_2nd = max_bin_index;
			float value_1st = 0.0f, value_2nd = 0.0f;
			for ( int k = min_bin_index; k <= max_bin_index; ++k ) {
				X[ axis ][ k ] = coeff[ k ] * ( X[ axis ][ k ] + delta ); // Do the actual SDFT calculation.

				// Find the two most dominant peaks:
				const float re = crealf( X[ axis ][ k ] );
				const float im = cimagf( X[ axis ][ k ] );
				const float mag_squared = re * re + im * im;
				if ( mag_squared > value_1st ) {
					value_2nd = value_1st;
					index_2nd = index_1st;
					value_1st = mag_squared;
					index_1st = k;
				} else if ( mag_squared > value_2nd ) {
					value_2nd = mag_squared;
					index_2nd = k;
				}
			}

			// Ensure index_1st is the high frequency:
			if ( index_1st < index_2nd ) {
				const int temp = index_1st;
				index_1st = index_2nd;
				index_2nd = temp;
			}

			const float max_Hz_high = index_1st / ( SAMPLE_PERIOD * 1e-6f ) / (float)SDFT_SIZE;
			const float max_Hz_low = index_2nd / ( SAMPLE_PERIOD * 1e-6f ) / (float)SDFT_SIZE;

			static float max_Hz_filt[ 4 ];
			extern float sdft_notch_Hz[ 4 ];
			lpf( &max_Hz_filt[ axis * 2 ], max_Hz_high, ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
			lpf( &sdft_notch_Hz[ axis * 2 ], max_Hz_filt[ axis * 2 ], ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
			lpf( &max_Hz_filt[ axis * 2 + 1 ], max_Hz_low, ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
			lpf( &sdft_notch_Hz[ axis * 2 + 1 ], max_Hz_filt[ axis * 2 + 1 ], ALPHACALC( SAMPLE_PERIOD, 1e6f / 20.0f ) ); // 20 Hz
		}

		++idx;
		if ( idx == SDFT_SIZE ) {
			idx = 0;
		}
	}
}
