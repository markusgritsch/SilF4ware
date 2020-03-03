#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "drv_led.h"
#include "drv_time.h"

extern bool lowbatt;
extern char aux[ AUXNUMBER ];
extern int rxmode; // bind / normal rx mode
extern bool failsafe; // rx.c

// For LED flash on e.g. gestures.  If ledcommand is set, the LED flashed for 0.5 seconds
// with 10 Hz. Gets automatically reset.
bool ledcommand = false;
int ledblink = 0; // Blink the LED ledblink times at 2 Hz.

void process_led_command( void )
{
	if ( lowbatt ) {
		ledflash( 500000 , 8 );
	} else {
		if ( rxmode == RXMODE_BIND) {
			ledflash( 100000, 12 );
		} else {
			if ( failsafe ) {
				ledflash( 500000, 15 );
			} else {
				static uint32_t ledcommandtime = 0;
				const bool leds_on = aux[ LEDS_ON ];
				if ( ledcommand ) {
					if ( ledcommandtime == 0 ) {
						ledcommandtime = gettime();
					}
					if ( gettime() - ledcommandtime > 500000 ) {
						ledcommand = false;
						ledcommandtime = 0;
					}
					ledflash( 100000, 8 );
				} else if ( ledblink > 0 ) {
					const uint32_t time = gettime();
					if ( ledcommandtime == 0 ) {
						ledcommandtime = time;
						if ( leds_on ) {
							ledoff();
						} else {
							ledon();
						}
					}
					if ( time - ledcommandtime > 500000 ) {
						--ledblink;
						ledcommandtime = 0;
					}
					if ( time - ledcommandtime > 300000 ) {
						if ( leds_on ) {
							ledon();
						} else {
							ledoff();
						}
					}
				} else if ( leds_on ) {
					if ( LED_BRIGHTNESS < 15 ) {
						led_pwm( LED_BRIGHTNESS );
					} else {
						ledon();
					}
				} else {
					ledoff();
				}
			}
		}
	}
}
