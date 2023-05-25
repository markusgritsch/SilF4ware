#include <stdint.h>

#if defined(STM32F405xx)
	#define SYS_CLOCK_FREQ_MHZ 168 // Betaflight allows setting 192, 216, 240 MHz.
#elif defined(STM32F411xE)
	#define SYS_CLOCK_FREQ_MHZ 100 // Overclocking up to 150 MHz seems to work fine.
#else
	#error "Unknown MCU"
#endif

void time_init( void );
uint32_t gettime( void ); // return time in micro seconds from start
void delay( uint32_t us );
