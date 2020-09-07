#include "defines.h"
#include "drv_spi_mpu.h"
#include "hardware.h"
#include "main.h"

#ifdef MPU_HARDSPI

#if MPU_HARDSPI == 1
	#define HARDSPIx SPI1
	#define HARDSPI_INIT_FUNC MX_SPI1_Init
	#if defined(STM32F405xx)
		#define BAUDRATE_FAST_PRESCALER SPI_BAUDRATEPRESCALER_4 // 21 MBit/s
		#define BAUDRATE_SLOW_PRESCALER SPI_BAUDRATEPRESCALER_128 // 0.65625 MBit/s
	#elif defined(STM32F411xE)
		#define BAUDRATE_FAST_PRESCALER SPI_BAUDRATEPRESCALER_4 // 25 MBit/s
		#define BAUDRATE_SLOW_PRESCALER SPI_BAUDRATEPRESCALER_128 // 0.78125 MBit/s
	#endif
#elif MPU_HARDSPI == 2
	#define HARDSPIx SPI2
	#define HARDSPI_INIT_FUNC MX_SPI2_Init
	#if defined(STM32F405xx)
		#define BAUDRATE_FAST_PRESCALER SPI_BAUDRATEPRESCALER_2 // 21 MBit/s
		#define BAUDRATE_SLOW_PRESCALER SPI_BAUDRATEPRESCALER_64 // 0.65625 MBit/s
	#elif defined(STM32F411xE)
		#define BAUDRATE_FAST_PRESCALER SPI_BAUDRATEPRESCALER_2 // 25 MBit/s
		#define BAUDRATE_SLOW_PRESCALER SPI_BAUDRATEPRESCALER_64 // 0.78125 MBit/s
	#endif
#else
	#error "MPU_HARDSPI must be 1 or 2."
#endif

void spi_mpu_init()
{
	extern void HARDSPI_INIT_FUNC( void );
	HARDSPI_INIT_FUNC();
	SET_BIT( HARDSPIx->CR1, SPI_CR1_SPE ); // Enable the specified SPI peripheral
}

void spi_mpu_sendbyte_slow( int data )
{
	MODIFY_REG( HARDSPIx->CR1, SPI_CR1_BR, BAUDRATE_SLOW_PRESCALER );
	spi_mpu_sendbyte( data );
	MODIFY_REG( HARDSPIx->CR1, SPI_CR1_BR, BAUDRATE_FAST_PRESCALER );
}

void spi_mpu_sendbyte( int data )
{
	// while ( ( HARDSPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
	HARDSPIx->DR = data;
	while ( ( HARDSPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
	HARDSPIx->DR;
}

int spi_mpu_sendzerorecvbyte_slow()
{
	MODIFY_REG( HARDSPIx->CR1, SPI_CR1_BR, BAUDRATE_SLOW_PRESCALER );
	return spi_mpu_sendzerorecvbyte();
	MODIFY_REG( HARDSPIx->CR1, SPI_CR1_BR, BAUDRATE_FAST_PRESCALER );
}

int spi_mpu_sendzerorecvbyte()
{
	// while ( ( HARDSPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
	HARDSPIx->DR = 0;
	while ( ( HARDSPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
	return HARDSPIx->DR;
}

#endif // MPU_HARDSPI

// ==========================================================================

#ifdef MPU_SOFTSPI

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

void spi_mpu_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH
	};

	GPIO_InitStruct.Pin = SPI_MPU_MOSI_Pin;
	HAL_GPIO_Init( SPI_MPU_MOSI_GPIO_Port, &GPIO_InitStruct );
	GPIO_InitStruct.Pin = SPI_MPU_SCK_Pin;
	HAL_GPIO_Init( SPI_MPU_SCK_GPIO_Port, &GPIO_InitStruct );

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pin = SPI_MPU_MISO_Pin;
	HAL_GPIO_Init( SPI_MPU_MISO_GPIO_Port, &GPIO_InitStruct );
}

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

#endif // MPU_SOFTSPI
