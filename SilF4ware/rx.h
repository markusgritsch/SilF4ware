#include <stdbool.h>

void rx_init( void );
bool checkrx( void ); // returns true in case a packet was received
void notify_telemetry_value( int value ); // 0 .. 511
