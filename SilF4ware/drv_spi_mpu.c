#include "defines.h"
#include "drv_spi_mpu.h"
#include "main.h"

#define MOSIHIGH gpioset(SPI_MPU_MOSI_GPIO_Port, SPI_MPU_MOSI_Pin);
#define MOSILOW gpioreset(SPI_MPU_MOSI_GPIO_Port, SPI_MPU_MOSI_Pin);
#define SCKHIGH gpioset(SPI_MPU_SCK_GPIO_Port, SPI_MPU_SCK_Pin);
#define SCKLOW gpioreset(SPI_MPU_SCK_GPIO_Port, SPI_MPU_SCK_Pin);

#define READMISO (SPI_MPU_MISO_GPIO_Port->IDR & SPI_MPU_MISO_Pin)

#if 1
	// necessary when using -O3 or -Os (balanced)
	#if defined(STM32F405xx)
		#define DELAY_O3 _NOP_ _NOP_
	#elif defined(STM32F411xE)
		#define DELAY_O3 _NOP_
	#endif
#else
	// works when using -Oz (image size)
	#define DELAY_O3
#endif

// 11 does not always work right after flashing, 13 seems to work fine, so 15 should be on the safe side.
#define DELAY_SLOW count = 15; while ( count-- );

volatile static uint32_t count;

void spi_mpu_sendbyte_slow( int data )
{
	for ( int i = 7; i >= 0; --i ) {
		if ( ( data >> i ) & 1 ) {
			SCKLOW
			MOSIHIGH
		} else {
			SCKLOW
			MOSILOW
		}
		DELAY_SLOW
		SCKHIGH
		DELAY_SLOW
	}
	SCKLOW
	MOSILOW
	DELAY_SLOW
}

void spi_mpu_sendbyte( int data )
{
	for ( int i = 7; i >= 0; --i ) {
		if ( ( data >> i ) & 1 ) {
			SCKLOW
			MOSIHIGH
		} else {
			SCKLOW
			MOSILOW
		}
		SCKHIGH
		DELAY_O3
	}
	SCKLOW
	MOSILOW
}

int spi_mpu_sendzerorecvbyte_slow()
{
	int recv = 0;
	MOSILOW
	for ( int i = 7; i >= 0; --i ) {
		recv = recv << 1;
		SCKHIGH
		DELAY_SLOW
		if ( READMISO ) {
			recv = recv | 1;
		}
		SCKLOW
		DELAY_SLOW
	}
	return recv;
}

int spi_mpu_sendzerorecvbyte()
{
	int recv = 0;
	MOSILOW
	for ( int i = 7; i >= 0; --i ) {
		recv = recv << 1;
		SCKHIGH
		DELAY_O3
		if ( READMISO ) {
			recv = recv | 1;
		}
		SCKLOW
		DELAY_O3
	}
	return recv;
}
