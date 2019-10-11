#include "defines.h"
#include "util.h"

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

// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
// sin_approx maximum absolute error = 2.305023e-06
// cos_approx maximum absolute error = 2.857298e-06
#if 0
	#define sinPolyCoef3 -1.666568107e-1f
	#define sinPolyCoef5  8.312366210e-3f
	#define sinPolyCoef7 -1.849218155e-4f
	#define sinPolyCoef9  0
#else
	#define sinPolyCoef3 -1.666665710e-1f // Double: -1.666665709650470145824129400050267289858e-1
	#define sinPolyCoef5  8.333017292e-3f // Double:  8.333017291562218127986291618761571373087e-3
	#define sinPolyCoef7 -1.980661520e-4f // Double: -1.980661520135080504411629636078917643846e-4
	#define sinPolyCoef9  2.600054768e-6f // Double:  2.600054767890361277123254766503271638682e-6
#endif

float sin_approx( float x )
{
	while ( x > PI_F ) { x -= 2.0f * PI_F; } // always wrap input angle to -PI..PI
	while ( x < -PI_F ) { x += 2.0f * PI_F; }
	if ( x > 0.5f * PI_F ) { // We just pick -90 .. +90 Degree
		x = PI_F - x;
	} else if ( x < -0.5f * PI_F ) {
		x = -PI_F - x;
	}
	const float x2 = x * x;
	return x + x * x2 * ( sinPolyCoef3 + x2 * ( sinPolyCoef5 + x2 * ( sinPolyCoef7 + x2 * sinPolyCoef9 ) ) );
}

float cos_approx( float x )
{
	return sin_approx( x + 0.5f * PI_F );
}
