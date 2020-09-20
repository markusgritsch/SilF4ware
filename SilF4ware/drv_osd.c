#include "binary.h"
#include "drv_osd.h"
#include "drv_spi_osd.h"
#include "drv_time.h"
#include "main.h"

static void spi_osd_cson()
{
	SPI_OSD_NSS_GPIO_Port->BSRR = (uint32_t)SPI_OSD_NSS_Pin << 16U;
}

static void spi_osd_csoff()
{
	SPI_OSD_NSS_GPIO_Port->BSRR = SPI_OSD_NSS_Pin;
}

void drv_osd_init()
{
	spi_osd_init();
	spi_osd_csoff();
	delay( 1 );
}

void osd_set_baudrate()
{
	spi_osd_set_baudrate();
}

void osd_restore_baudrate()
{
	spi_osd_restore_baudrate();
}

void osd_writereg( uint8_t address, uint8_t value )
{
	spi_osd_cson();
	spi_osd_sendbyte( address );
	spi_osd_sendbyte( value );
	spi_osd_csoff();
	delay( 1 );
}

uint8_t osd_readreg( uint8_t address )
{
	spi_osd_cson();
	spi_osd_sendbyte( address | 0x80 );
	uint8_t value = spi_osd_sendzerorecvbyte();
	spi_osd_csoff();
	delay( 1 );
	return value;
}

static void osd_write( uint8_t address, uint8_t value )
{
	spi_osd_sendbyte( address );
	spi_osd_sendbyte( value );
}

void osd_print_display_memory( uint8_t data[], uint32_t size, uint32_t pos )
{
	spi_osd_cson();
	// 16 bit mode, auto increment mode
	osd_write( DMM, B00000001 );
	// memory address
	osd_write( DMAH, 0x01 & ( pos >> 8 ) );
	osd_write( DMAL, 0xFF & pos );
	for ( int i = 0; i < size; ++i ) {
		osd_write( DMDI, data[ i ] );
	}
	// off autoincrement mode
	osd_write( DMDI, 0xFF );
	spi_osd_csoff();
}

void osd_putc_display_memory( uint8_t character, uint32_t pos )
{
	spi_osd_cson();
	static uint8_t last_dmah = 0;
	// memory address
	if ( pos > 0xFF && last_dmah == 0 ) {
		last_dmah = 1;
		osd_write( DMAH, last_dmah );
	} else if ( pos <= 0xFF && last_dmah == 1 ) {
		last_dmah = 0;
		osd_write( DMAH, last_dmah );
	}
	osd_write( DMAL, 0xFF & pos );
	osd_write( DMDI, character );
	spi_osd_csoff();
}
