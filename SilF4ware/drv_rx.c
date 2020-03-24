#include "drv_rx.h"
#include "drv_spi_rx.h"
#include "drv_time.h"
#include "main.h"

static void spi_rx_cson()
{
	SPI_RX_NSS_GPIO_Port->BSRR = (uint32_t)SPI_RX_NSS_Pin << 16U;
}

static void spi_rx_csoff()
{
	SPI_RX_NSS_GPIO_Port->BSRR = SPI_RX_NSS_Pin;
}

void drv_rx_init()
{
	spi_rx_init();
	spi_rx_csoff();
	delay( 1 );
}

void rx_writereg( int reg, int val )
{
	reg = reg & 0x0000003F;
	reg = reg | W_REGISTER;
	spi_rx_cson();
	spi_rx_sendbyte( reg );
	spi_rx_sendbyte( val );
	spi_rx_csoff();
}

int rx_readreg( int reg )
{
	reg = reg & REGISTER_MASK;
	spi_rx_cson();
	spi_rx_sendbyte( reg );
	int val = spi_rx_sendzerorecvbyte();
	spi_rx_csoff();
	return val;
}

// int rx_command_orig( int command )
// {
// 	spi_rx_cson();
// 	int status = spi_rx_sendrecvbyte( command );
// 	spi_rx_csoff();
// 	return status;
// }

void rx_command( int command )
{
	spi_rx_cson();
	spi_rx_sendbyte( command );
	spi_rx_csoff();
}

// void _spi_write_address( int reg, int val )
// {
// 	spi_rx_cson();
// 	spi_rx_sendbyte( reg);
// 	spi_rx_sendbyte( val);
// 	spi_rx_csoff();
// }

void rx_readpayload( int * data, int size )
{
	int index = 0;
	spi_rx_cson();
	spi_rx_sendbyte( R_RX_PAYLOAD ); // read rx payload
	while ( index < size ) {
		data[ index ] = spi_rx_sendzerorecvbyte();
		++index;
	}
	spi_rx_csoff();
}

void rx_writerxaddress( int * addr )
{
	int index = 0;
	spi_rx_cson();
	spi_rx_sendbyte( W_REGISTER | RX_ADDR_P0 );
	while ( index < 5 ) {
		spi_rx_sendbyte( addr[ index ] );
		++index;
	}
	spi_rx_csoff();
}

void rx_writetxaddress( int * addr )
{
	int index = 0;
	spi_rx_cson();
	spi_rx_sendbyte( W_REGISTER | TX_ADDR );
	while ( index < 5 ) {
		spi_rx_sendbyte( addr[ index ] );
		++index;
	}
	spi_rx_csoff();
}

void rx_writepayload( int data[], int size )
{
	int index = 0;
	spi_rx_cson();
	spi_rx_sendbyte( W_TX_PAYLOAD ); // write tx payload
	while ( index < size ) {
		spi_rx_sendbyte( data[ index ] );
		++index;
	}
	spi_rx_csoff();
}

void rx_writeregs( uint8_t data[], uint8_t size )
{
	spi_rx_cson();
	for ( uint8_t i = 0; i < size; ++i ) {
		spi_rx_sendbyte( data[ i ] );
	}
	spi_rx_csoff();
}
