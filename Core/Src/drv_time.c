#include "drv_time.h"
#include "main.h"

void time_init( void )
{
	SysTick->LOAD = 0xFFFFFF & SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL = 0;
	SysTick->CTRL =
		SysTick_CTRL_CLKSOURCE_Msk | // select the core clock
		SysTick_CTRL_ENABLE_Msk; // enable the SysTick timer
}

uint32_t gettime( void )
{
	static uint32_t lastticks;
	static uint32_t globalticks;
	static uint32_t remainder; // carry forward the remainder ticks

	const uint32_t maxticks = SysTick->LOAD;
	const uint32_t ticks = SysTick->VAL;

	uint32_t elapsedticks;
	if ( ticks < lastticks ) {
		elapsedticks = lastticks - ticks;
	} else { // overflow (underflow really)
		elapsedticks = lastticks + ( maxticks - ticks );
	}

	lastticks = ticks;
	elapsedticks += remainder;

	const uint32_t quotient = elapsedticks / SYS_CLOCK_FREQ_MHZ;
	remainder = elapsedticks - quotient * SYS_CLOCK_FREQ_MHZ;

	globalticks = globalticks + quotient;

	return globalticks;
}

void delay( uint32_t us )
{
	volatile uint32_t count;
	count = us * ( SYS_CLOCK_FREQ_MHZ / 6 );
	while ( count-- );
}
