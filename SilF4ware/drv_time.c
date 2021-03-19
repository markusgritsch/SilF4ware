#include "drv_time.h"
#include "main.h"
#include "tm_stm32f4_rcc.h"

void time_init( void )
{
	// Due to the specific PLL configuration, overclocking can be achieved by just changing PLLN.
	TM_RCC_PLL_t PLL;
	TM_RCC_GetPLL( &PLL );
	PLL.PLLN = SYS_CLOCK_FREQ_MHZ;
	TM_RCC_SetPLL( &PLL );
	// For correct UART baudrate settings, UART_Init must be called after this, like in blackbox.c.
	// Timer settings must also take the changed clock rate into account, like below and in drv_dshot_*.c

 	// Counter clock frequency is scaled by PSC + 1, therefore we need to subtract 1.
#if defined(STM32F405xx)
	TIM2->PSC = SYS_CLOCK_FREQ_MHZ / 2 - 1;
#elif defined(STM32F411xE)
	TIM2->PSC = SYS_CLOCK_FREQ_MHZ - 1;
#endif
	TIM2->EGR = TIM_EGR_UG; // Generate an update event to reload the Prescaler immediatly.
	TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t gettime( void )
{
	return TIM2->CNT;
}

void delay( uint32_t us )
{
	const uint32_t wait_until = gettime() + us;
	if ( wait_until < gettime() ) { // check for overflow
		while ( gettime() > wait_until );
	}
	while ( gettime() < wait_until );
}
