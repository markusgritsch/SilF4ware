#include "defines.h"
#include "drv_spi_rx.h"
#include "hardware.h"
#include "main.h"

#ifdef HARDSPI

#if HARDSPI == 1
	#define HARDSPIx SPI1
	#define HARDSPI_INIT_FUNC MX_SPI1_Init
#elif HARDSPI == 2
	#define HARDSPIx SPI2
	#define HARDSPI_INIT_FUNC MX_SPI2_Init
#else
	#error "HARDSPI must be 1 or 2."
#endif

void spi_rx_init()
{
	extern void HARDSPI_INIT_FUNC( void );
	HARDSPI_INIT_FUNC();
	SET_BIT( HARDSPIx->CR1, SPI_CR1_SPE ); // Enable the specified SPI peripheral
}

void spi_rx_sendbyte( int data )
{
	// while ( ( HARDSPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
	HARDSPIx->DR = data;
	while ( ( HARDSPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
	HARDSPIx->DR;
}

int spi_rx_sendzerorecvbyte()
{
	// while ( ( HARDSPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
	HARDSPIx->DR = 0;
	while ( ( HARDSPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
	return HARDSPIx->DR;
}

#endif // HARDSPI

// ==========================================================================

#ifdef SOFTSPI_4WIRE

#define MOSIHIGH gpioset(SPI_RX_MOSI_GPIO_Port, SPI_RX_MOSI_Pin);
#define MOSILOW gpioreset(SPI_RX_MOSI_GPIO_Port, SPI_RX_MOSI_Pin);
#define SCKHIGH gpioset(SPI_RX_SCK_GPIO_Port, SPI_RX_SCK_Pin);
#define SCKLOW gpioreset(SPI_RX_SCK_GPIO_Port, SPI_RX_SCK_Pin);

#define READMISO (SPI_RX_MISO_GPIO_Port->IDR & SPI_RX_MISO_Pin)

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

void spi_rx_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH
	};

	GPIO_InitStruct.Pin = SPI_RX_MOSI_Pin;
	HAL_GPIO_Init( SPI_RX_MOSI_GPIO_Port, &GPIO_InitStruct );
	GPIO_InitStruct.Pin = SPI_RX_SCK_Pin;
	HAL_GPIO_Init( SPI_RX_SCK_GPIO_Port, &GPIO_InitStruct );

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pin = SPI_RX_MISO_Pin;
	HAL_GPIO_Init( SPI_RX_MISO_GPIO_Port, &GPIO_InitStruct );
}

void spi_rx_sendbyte( int data )
{
	for ( int i =7; i >= 0; --i ) {
		SCKLOW
		if ( ( data >> i ) & 1 ) {
			MOSIHIGH
		} else {
			MOSILOW
		}
		SCKHIGH
		DELAY_O3
	}
	SCKLOW
	MOSILOW
}

int spi_rx_sendzerorecvbyte()
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

// int spi_rx_sendrecvbyte( int data )
// {
// 	int recv = 0;
// 	for ( int i = 7; i >= 0; --i ) {
// 		recv = recv << 1;
// 		if ( data & ( 1 << 7 ) ) {
// 			MOSIHIGH
// 		} else {
// 			MOSILOW
// 		}
// 		data = data << 1;
// 		SCKHIGH
// 		if ( READMISO ) {
// 			recv = recv | 1;
// 		}
// 		SCKLOW
// 	}
// 	return recv;
// }

// int spi_rx_sendrecvbyte2( int data )
// {
// 	int recv = 0;
// 	for ( int i = 7; i >= 0; --i ) {
// 		if ( data & (1 << 7 ) ) {
// 			MOSIHIGH
// 		} else {
// 			MOSILOW
// 		}
// 		SCKHIGH
// 		data = data << 1;
// 		if ( READMISO ) {
// 			recv = recv | ( 1 << 7 );
// 		}
// 		recv = recv << 1;
// 		SCKLOW
// 	}
// 	recv = recv >> 8;
// 	return recv;
// }

#endif // SOFTSPI_4WIRE
