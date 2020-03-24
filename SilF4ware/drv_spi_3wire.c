#include "defines.h"
#include "drv_spi_rx.h"
#include "drv_time.h"
#include "hardware.h"
#include "main.h"

#ifdef SOFTSPI_3WIRE

static GPIO_InitTypeDef MOSI_InitStruct = {
	.Pin = SPI_RX_MOSI_Pin,
	.Mode = GPIO_MODE_OUTPUT_PP,
	.Pull = GPIO_NOPULL,
	.Speed = GPIO_SPEED_FREQ_HIGH
};
static int mosi_out = 1;

static void mosi_output( void )
{
	mosi_out = 1;
	MOSI_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init( SPI_RX_MOSI_GPIO_Port, &MOSI_InitStruct );
}

static void mosi_input( void )
{
	mosi_out = 0;
	MOSI_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init( SPI_RX_MOSI_GPIO_Port, &MOSI_InitStruct );
}

#define MOSIHIGH gpioset(SPI_RX_MOSI_GPIO_Port, SPI_RX_MOSI_Pin)
#define MOSILOW gpioreset(SPI_RX_MOSI_GPIO_Port, SPI_RX_MOSI_Pin);
#define SCKHIGH gpioset(SPI_RX_SCK_GPIO_Port, SPI_RX_SCK_Pin);
#define SCKLOW gpioreset(SPI_RX_SCK_GPIO_Port, SPI_RX_SCK_Pin);

#define READMOSI (SPI_RX_MOSI_GPIO_Port->IDR & SPI_RX_MOSI_Pin)

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
}

void spi_rx_sendbyte( int data )
{
	if ( ! mosi_out ) {
		mosi_output();
	}
	for ( int i =7; i >= 0; --i ) {
		SCKLOW;
		if ( ( data >> i ) & 1 ) {
			MOSIHIGH;
		} else {
			MOSILOW;
		}
		static int count = 800;
		if ( count ) {  // Very ugly hack. It seems the XN297L needs its configuration bytes
			delay( 1 ); // be written very slow. After that, things can be sped up.
			--count;
		}
		SCKHIGH;
		_NOP_ _NOP_
	}
	SCKLOW;
}

int spi_rx_sendzerorecvbyte()
{
	if ( mosi_out ) {
		mosi_input();
	}
	int recv = 0;
	for ( int i = 7; i >= 0; --i ) {
		recv = recv << 1;
		SCKHIGH;
		_NOP_ _NOP_
		if ( READMOSI ) {
			recv = recv | 1;
		}
		SCKLOW;
		_NOP_ _NOP_
	}
	return recv;
}

#endif
