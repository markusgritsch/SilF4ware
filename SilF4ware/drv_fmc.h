#include <stdint.h>

void fmc_unlock( void );
void fmc_lock( void );
void fmc_erase( void );
void fmc_write( uint32_t address, uint32_t value );
uint32_t fmc_read( uint32_t address );
void fmc_write_float( uint32_t address, float float_to_write );
float fmc_read_float( uint32_t address );
