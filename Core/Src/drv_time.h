#include <stdint.h>

#define SYS_CLOCK_FREQ_MHZ 100

void time_init( void );
uint32_t gettime( void ); // return time in micro seconds from start
void delay( uint32_t us );
