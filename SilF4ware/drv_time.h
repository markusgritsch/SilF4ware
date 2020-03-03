#include <stdint.h>

#if defined(STM32F405xx)
	#define SYS_CLOCK_FREQ_MHZ 168
#elif defined(STM32F411xE)
	#define SYS_CLOCK_FREQ_MHZ 100
#else
	#error "Unknown MCU"
#endif

void time_init( void );
uint32_t gettime( void ); // return time in micro seconds from start
void delay( uint32_t us );
