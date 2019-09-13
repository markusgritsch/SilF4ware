#include "defines.h"
#include "drv_spi2.h"
#include "main.h"

#define MOSIHIGH gpioset(SPI2_MOSI_GPIO_Port, SPI2_MOSI_Pin)
#define MOSILOW gpioreset(SPI2_MOSI_GPIO_Port, SPI2_MOSI_Pin);
#define SCKHIGH gpioset(SPI2_CLK_GPIO_Port, SPI2_CLK_Pin);
#define SCKLOW gpioreset(SPI2_CLK_GPIO_Port, SPI2_CLK_Pin);

#define READMISO (SPI2_MISO_GPIO_Port->IDR & SPI2_MISO_Pin)
// #define DELAY_O3 _NOP_ _NOP_ // necessary when the loop is unrolled i.e. with -O3
#define DELAY_O3

volatile static uint32_t count;

void spi2_sendbyte_slow( int data )
{
	for ( int i =7; i >= 0; --i ) {
		SCKLOW;
		if ( ( data >> i ) & 1 ) {
			MOSIHIGH;
		} else {
			MOSILOW;
		}
		count = 11; while ( count-- );
		SCKHIGH;
		count = 11; while ( count-- );
	}
	SCKLOW;
	count = 11; while ( count-- );
}

void spi2_sendbyte( int data )
{
	for ( int i =7; i >= 0; --i ) {
		SCKLOW;
		if ( ( data >> i ) & 1 ) {
			MOSIHIGH;
		} else {
			MOSILOW;
		}
		SCKHIGH;
		DELAY_O3
	}
	SCKLOW;
}

int spi2_sendzerorecvbyte_slow()
{
	int recv = 0;
	MOSILOW;
	for ( int i = 7; i >= 0; --i ) {
		recv = recv << 1;
		SCKHIGH;
		count = 11; while ( count-- );
		if ( READMISO ) {
			recv = recv | 1;
		}
		SCKLOW;
		count = 11; while ( count-- );
	}
	return recv;
}

int spi2_sendzerorecvbyte()
{
	int recv = 0;
	MOSILOW;
	for ( int i = 7; i >= 0; --i ) {
		recv = recv << 1;
		SCKHIGH;
		DELAY_O3
		if ( READMISO ) {
			recv = recv | 1;
		}
		SCKLOW;
		DELAY_O3
	}
	return recv;
}
