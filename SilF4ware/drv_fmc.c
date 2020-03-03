// STM32F40x/STM32F41x Flash memory sectors

// Sectors 0 to 3: 16 Kbyte (0x4000 bytes)
// Sector 4: 64 Kbyte (0x10000 bytes)
// Sectors 5 to 11: 128 Kbyte (0x20000 bytes)

// Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
// Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
// Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
// Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
// Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
// Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
// Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
// Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
// Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
// Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
// Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
// Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes

#include <string.h> // memcpy

#include "drv_fmc.h"
#include "main.h"

#if 0
	// FLASH_Sector_2: 0x08008000 - 0x0800BFFF 16 Kbytes
	#define FLASH_ADDR 0x08008000 // Sector 2
	#define FLASH_ERASE_SECOTR_ID FLASH_SECTOR_2
#else
	// FLASH_Sector_3: 0x0800C000 - 0x0800FFFF 16 Kbytes
	#define FLASH_ADDR 0x0800C000 // Sector 3
	#define FLASH_ERASE_SECOTR_ID FLASH_SECTOR_3
#endif

extern void failloop( int );

void fmc_unlock() {
	HAL_FLASH_Unlock();
}

void fmc_lock() {
	HAL_FLASH_Lock();
}

void fmc_erase( void )
{
	FLASH_Erase_Sector( FLASH_ERASE_SECOTR_ID, FLASH_VOLTAGE_RANGE_3 );
}

// address: 0 .. 4095 (for sector 2)
// value: 32 bit to write
void fmc_write( uint32_t address, uint32_t value )
{
	const HAL_StatusTypeDef status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, FLASH_ADDR + ( address << 2 ), value);
	if ( status != HAL_OK ) {
		HAL_FLASH_Lock();
		failloop( 6 );
	}
}

// reads 32 bit value
uint32_t fmc_read( uint32_t address )
{
	address = FLASH_ADDR + address * 4;
	uint32_t * addressptr = (uint32_t *)address;
	return *addressptr;
}

void fmc_write_float( uint32_t address, float float_to_write ) {
	// warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
	// fmc_write( address, *(uint32_t *)&float_to_write );

	uint32_t temp;
	memcpy( &temp, &float_to_write, 4 );
	fmc_write( address, temp );
}

float fmc_read_float( uint32_t address ) {
	const uint32_t result = fmc_read( address );

	// warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
	//return *(float*)&result;

	float temp;
	memcpy( &temp, &result, 4 );
	return temp;
}
