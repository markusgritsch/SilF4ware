#include "debug.h"
#include "defines.h"
#include "main.h"

void debug_on( void )
{
#ifdef DBG_Pin
	gpioset( DBG_GPIO_Port, DBG_Pin );
#endif
}

void debug_off( void )
{
#ifdef DBG_Pin
	gpioreset( DBG_GPIO_Port, DBG_Pin );
#endif
}

bool is_button_pressed( void )
{
#ifdef BUTTON_Pin
	const GPIO_PinState button_state = HAL_GPIO_ReadPin( BUTTON_GPIO_Port, BUTTON_Pin );
	return button_state == GPIO_PIN_SET;
#else
	return 0;
#endif
}
