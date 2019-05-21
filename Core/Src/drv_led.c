#include "defines.h"
#include "drv_time.h"
#include "hardware.h"
#include "main.h"

void ledon( void )
{
	// HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_SET );

#ifdef LED_INVERT
	gpioreset( LED_GPIO_Port, LED_Pin );
#else
	gpioset( LED_GPIO_Port, LED_Pin );
#endif
}

void ledoff( void )
{
	// HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET );

#ifdef LED_INVERT
	gpioset( LED_GPIO_Port, LED_Pin );
#else
	gpioreset( LED_GPIO_Port, LED_Pin );
#endif
}

void ledflash( uint32_t period, uint8_t duty )
{
	if ( gettime() % period > ( period * duty ) >> 4 ) {
		ledon();
	} else {
		ledoff();
	}
}

void led_pwm( uint8_t pwmval )
{
	static uint8_t loopcount = 0;
	++loopcount;
	loopcount &= 0xF;
	if ( pwmval > loopcount ) {
		ledon();
	} else {
		ledoff();
	}
}

// Delta-Sigma first order modulator.
// void led_pwm2( uint8_t pwmval )
// {
// 	static uint32_t lastledtime;
// 	static float lastledbrightness = 0;
// 	static float ds_integrator = 0;
// 	const uint32_t time = gettime();
// 	const uint32_t ledtime = time - lastledtime;
// 	lastledtime = time;
// 	float desiredbrightness = pwmval / 15.0f;
// 	if ( ds_integrator > 2 ) {
// 		ds_integrator = 2;
// 	}
// 	ds_integrator += ( desiredbrightness - lastledbrightness ) * ledtime / (float)LOOPTIME;
// 	if ( ds_integrator > 0.49f ) {
// 		ledon();
// 		lastledbrightness = 1.0f;
// 	} else {
// 		ledoff();
// 		lastledbrightness = 0;
// 	}
// }
