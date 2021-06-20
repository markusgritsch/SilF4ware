#include <complex.h>
#include <math.h> // powf

#include "config.h"
#include "defines.h"
#include "util.h"

#define MAX_HZ 200
#define SDFT_SIZE ( 10000 / LOOPTIME ) // 0.01 seconds time window gives 100 Hz resolution.
#define DAMPING_FACTOR 1.0f

static int idx = 0;
static float x[ 3 ][ SDFT_SIZE ];
static float complex X[ 3 ][ SDFT_SIZE ];
static float complex coeff[ SDFT_SIZE ];
static float complex icoeff[ SDFT_SIZE ];

void filter_sdft_init()
{
	for ( int k = 0; k < SDFT_SIZE; ++k ) {
		const float phi = 2.0f * PI_F * (float)k / (float)SDFT_SIZE;
		coeff[ k ] = DAMPING_FACTOR * ( cos_approx( phi ) + I * sin_approx( phi ) ); // cexpf( I * phi )
		const float iphi = 2.0f * PI_F * (float)k * ( SDFT_SIZE - 1 ) / (float)SDFT_SIZE;
		icoeff[ k ] = cos_approx( iphi ) + I * sin_approx( iphi );
	}
}

float filter_sdft_step( float in, int axis )
{
	const float r_to_N = powf( DAMPING_FACTOR, SDFT_SIZE );
	const float delta = in - r_to_N * x[ axis ][ idx ];
	x[ axis ][ idx ] = in;

	const int max_bin_index = (float)MAX_HZ * (float)SDFT_SIZE * ( LOOPTIME * 1e-6f ) + 0.5f;

	for ( int k = 0; k <= max_bin_index; ++k ) {
		X[ axis ][ k ] = coeff[ k ] * ( X[ axis ][ k ] + delta ); // Do the actual SDFT calculation.
		if ( k > 0 ) {
			X[ axis ][ SDFT_SIZE - k ] = conjf( X[ axis ][ k ] );
		}
	}

	if ( axis == 2 ) { // Kludge.
		++idx;
		if ( idx == SDFT_SIZE ) {
			idx = 0;
		}
	}

	// IDFT of newest sample:
	float out = X[ axis ][ 0 ];
	for ( int k = 1; k <= max_bin_index; ++k ) {
		out += X[ axis ][ k ] * icoeff[ k ];
		out += X[ axis ][ SDFT_SIZE - k ] * icoeff[ SDFT_SIZE - k ];
	}
	return out / SDFT_SIZE;
}
