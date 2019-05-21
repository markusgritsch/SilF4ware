#include "util.h"

// coeff is 1 - alpha
void lpf( float *out, float in, float coeff )
{
	*out = ( *out ) * coeff + in * ( 1 - coeff );
}

float mapf( float x, float in_min, float in_max, float out_min, float out_max )
{
	return ( ( x - in_min ) * ( out_max - out_min ) ) / ( in_max - in_min ) + out_min;
}

void limitf( float * input, const float limit )
{
	if ( *input > limit ) {
		*input = limit;
	} else if ( *input < -limit ) {
		*input = -limit;
	}
}

float rcexpo( float in, float exp )
{
	if ( exp > 1 ) {
		exp = 1;
	} else if ( exp < -1 ) {
		exp = -1;
	}
	float ans = in * in * in * exp + in * ( 1 - exp );
	limitf( &ans, 1.0 );
	return ans;
}

float fastsin( float x )
{
	//always wrap input angle to -PI..PI
	while ( x < -3.14159265f ) {
		x += 6.28318531f;
	}
	while ( x >  3.14159265f ) {
		x -= 6.28318531f;
	}
	float sin1;
	// quadratic approximation
	if ( x < 0 ) {
		sin1 = ( 1.27323954f + 0.405284735f * x ) * x;
	} else {
		sin1 = ( 1.27323954f - 0.405284735f * x ) * x;
	}
	return sin1;
}

float fastcos( float x )
{
	x += 1.57079632f;
	return fastsin( x );
}
