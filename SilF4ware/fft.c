// MIT License, https://github.com/xbarin02/uFFT
// Copyright (c) 2017 David Barina (fft-dit.c)

#include <complex.h>
#include <stddef.h> // size_t

#include "defines.h"
#include "fft.h"
#include "util.h"

static int ctz(size_t N)
{
	int ctz1 = 0;

	while( N ) {
		ctz1++;
		N >>= 1;
	}

	return ctz1-1;
}

static void nop_split(const float complex *x, float complex *X, size_t N)
{
	for(size_t n = 0; n < N/2; n++) {
		X[0/2+n] = x[2*n+0];
		X[N/2+n] = x[2*n+1];
	}
}

static void fft_split(const float complex *x, float complex *X, size_t N, float complex phi)
{
	for(size_t n = 0; n < N/2; n++) {
		// This saves about 2 kB and is almost two times faster than cexpf(-2*(float)M_PI*I*phi)
		const float complex cexp_factor = cos_approx(-2*PI_F*phi) + I * sin_approx(-2*PI_F*phi);
		X[0/2+n] = x[2*n+0] + x[2*n+1] * cexp_factor;
		X[N/2+n] = x[2*n+0] - x[2*n+1] * cexp_factor;
	}
}

static size_t revbits(size_t v, int J)
{
	size_t r = 0;

	for(int j = 0; j < J; j++) {
		r |= ( (v>>j)&1 ) << (J-1-j);
	}

	return r;
}

static int nop_reverse(int b, float complex *buffers[2], size_t N)
{
	int J = ctz(N);

	for(int j = 0; j < J-1; j++, b++) {
		size_t delta = N>>j;

		for(size_t n = 0; n < N; n += delta) {
			nop_split(buffers[b&1]+n, buffers[~b&1]+n, delta);
		}
	}

	return b;
}

static int fft_reverse(int b, float complex *buffers[2], size_t N)
{
	int J = ctz(N);

	for(int j = 0; j < J; j++, b++) {
		size_t delta = N>>j;

		for(size_t n = 0; n < N; n += delta) {
			float complex phi = (float)revbits(n/delta, j) / (float)(2<<j);
			fft_split(buffers[b&1]+n, buffers[~b&1]+n, delta, phi);
		}
	}

	return b;
}

static int fft(float complex *buffers[2], size_t N)
{
	if( !N ) return 0;

	if( N & (N-1) ) return 1;

	int b = 0;

	b = nop_reverse(b, buffers, N);
	b = fft_reverse(b, buffers, N);
	b = nop_reverse(b, buffers, N);

	return 0;
}

static float complex fft_in[ FFT_SIZE ];
static float complex fft_out[ FFT_SIZE ];
int get_max_amplitude_index( float array[], int start_index )
{
	for ( int i = 0; i < FFT_SIZE; ++i ) {
		fft_in[ i ] = array[ start_index ];
		++start_index;
		if ( start_index == FFT_SIZE ) {
			start_index = 0;
		}
	}

	float complex * buffers[ 2 ] = { fft_in, fft_out };
	fft( buffers, FFT_SIZE ); // 4096: 32 ms @ 168 MHz

	int max_index = 0;
	float max_value = 0.0f;
	for ( int i = 1; i <= FFT_SIZE / 2; ++i ) { // skip DC
		// cabsf() uses more space, so we use the squared magnitudes, since we are only looking for the index anyways.
		const float mag_squared = crealf( fft_out[ i ] ) * crealf( fft_out[ i ] ) + cimagf( fft_out[ i ] ) * cimagf( fft_out[ i ] );
		if ( mag_squared > max_value ) {
			max_value = mag_squared;
			max_index = i;
		}
	}

	return max_index;
}
