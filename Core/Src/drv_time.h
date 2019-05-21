#include <stdint.h>

#define SYS_CLOCK_FREQ_MHZ 100

void time_init( void );

// return time in micro seconds from start
// must be called at least once per 99.8 ms or time will overflow
uint32_t gettime( void );

void delay( uint32_t us );

