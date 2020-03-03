#include <stdint.h>

void pwm_init( void );
void pwm_set( uint8_t number , float pwm ); // pwm: 0 .. 1
void motorbeep( bool motors_failsafe, int channel );
