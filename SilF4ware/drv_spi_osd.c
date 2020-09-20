#include "drv_spi_osd.h"
#include "hardware.h"
#include "main.h"

#ifdef OSD_HARDSPI

#if OSD_HARDSPI == 1
	#define HARDSPIx SPI1
	#define HARDSPI_INIT_FUNC MX_SPI1_Init
	#if defined(STM32F405xx)
		#define BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_8 // 10.5 MBit/s
	#elif defined(STM32F411xE)
		#define BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_8 // 12.5 MBit/s
	#endif
#elif OSD_HARDSPI == 2
	#define HARDSPIx SPI2
	#define HARDSPI_INIT_FUNC MX_SPI2_Init
	#if defined(STM32F405xx)
		#define BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_4 // 10.5 MBit/s
	#elif defined(STM32F411xE)
		#define BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_4 // 12.5 MBit/s
	#endif
#elif OSD_HARDSPI == 3
	#define HARDSPIx SPI3
	#define HARDSPI_INIT_FUNC MX_SPI3_Init
	#if defined(STM32F405xx)
		#define BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_4 // 10.5 MBit/s
	#elif defined(STM32F411xE)
		#define BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_4 // 12.5 MBit/s
	#endif
#else
	#error "OSD_HARDSPI must be 1, 2, or 3."
#endif

void spi_osd_init()
{
#if ( defined RX_HARDSPI && RX_HARDSPI == OSD_HARDSPI ) || ( defined MPU_HARDSPI && MPU_HARDSPI == OSD_HARDSPI )
	// OSD SPI peripheral already initialized by RX or MPU SPI peripheral.
#else
	extern void HARDSPI_INIT_FUNC( void );
	HARDSPI_INIT_FUNC();
	SET_BIT( HARDSPIx->CR1, SPI_CR1_SPE ); // Enable the specified SPI peripheral
#endif
}

static uint32_t previous_prescaler;
void spi_osd_set_baudrate()
{
	previous_prescaler = READ_REG( HARDSPIx->CR1 ) & SPI_CR1_BR;
	MODIFY_REG( HARDSPIx->CR1, SPI_CR1_BR, BAUDRATE_PRESCALER );
}

void spi_osd_restore_baudrate()
{
	MODIFY_REG( HARDSPIx->CR1, SPI_CR1_BR, previous_prescaler );
}

void spi_osd_sendbyte( int data )
{
	// while ( ( HARDSPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
	HARDSPIx->DR = data;
	while ( ( HARDSPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
	HARDSPIx->DR;
}

int spi_osd_sendzerorecvbyte()
{
	// while ( ( HARDSPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
	HARDSPIx->DR = 0;
	while ( ( HARDSPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
	return HARDSPIx->DR;
}

#endif // OSD_HARDSPI
