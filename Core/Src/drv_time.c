#include "drv_time.h"
#include "main.h"

void time_init( void )
{
	TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t gettime( void )
{
	return TIM2->CNT;
}

void delay( uint32_t us )
{
	volatile uint32_t count;
	count = us * ( SYS_CLOCK_FREQ_MHZ / 6 );
	while ( count-- );
}
