#include <math.h> // fabsf

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
	while ( x > 3.14159265f ) {
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

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx( float y, float x )
{
	#define atanPolyCoef1 3.14551665884836e-07f
	#define atanPolyCoef2 0.99997356613987f
	#define atanPolyCoef3 0.14744007058297684f
	#define atanPolyCoef4 0.3099814292351353f
	#define atanPolyCoef5 0.05030176425872175f
	#define atanPolyCoef6 0.1471039133652469f
	#define atanPolyCoef7 0.6444640676891548f

	float res;
	const float absX = fabsf( x );
	const float absY = fabsf( y );
	res = MAX( absX, absY );
	if ( res ) {
		res = MIN( absX, absY ) / res;
	} else {
		res = 0.0f;
	}
	res = -( ( ( ( atanPolyCoef5 * res - atanPolyCoef4 ) * res - atanPolyCoef3 ) * res - atanPolyCoef2 ) * res - atanPolyCoef1 ) / ( ( atanPolyCoef7 * res + atanPolyCoef6 ) * res + 1.0f );
	if ( absY > absX ) {
		res = ( PI_F / 2.0f ) - res;
	}
	if ( x < 0 ) {
		res = PI_F - res;
	}
	if ( y < 0 ) {
		res = -res;
	}
	return res;
}

// https://developer.download.nvidia.com/cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
float acos_approx( float x )
{
	float xa = fabsf( x );
	float result = sqrtf( 1.0f - xa ) * ( 1.5707288f + xa * ( -0.2121144f + xa * ( 0.0742610f + ( -0.0187293f * xa ) ) ) );
	if ( x < 0.0f ) {
		return PI_F - result;
	} else {
		return result;
	}
}

float asin_approx( float x )
{
	return 0.5f * PI_F - acos_approx( x );
}
