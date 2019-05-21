#include "drv_spi.h"
#include "drv_xn297.h"
#include "hardware.h"
#include "main.h"

// all cases except 3 wires radio
#if !defined (SOFTSPI_3WIRE)

void spi_xn_cson()
{
	SPI_XN_SS_GPIO_Port->BSRR = (uint32_t)SPI_XN_SS_Pin << 16U;
}

void spi_xn_csoff()
{
	SPI_XN_SS_GPIO_Port->BSRR = SPI_XN_SS_Pin;
}

void xn_writereg( int reg, int val )
{
	reg = reg & 0x0000003F;
	reg = reg | W_REGISTER;
	spi_xn_cson();
	spi_sendbyte( reg );
	spi_sendbyte( val );
	spi_xn_csoff();
}

int xn_readreg( int reg )
{
	reg = reg & REGISTER_MASK;
	spi_xn_cson();
	spi_sendbyte( reg );
	int val = spi_sendzerorecvbyte();
	spi_xn_csoff();
	return val;
}

// int xn_command_orig( int command )
// {
// 	spi_xn_cson();
// 	int status = spi_sendrecvbyte( command );
// 	spi_xn_csoff();
// 	return status;
// }

void xn_command( int command )
{
	spi_xn_cson();
	spi_sendbyte( command );
	spi_xn_csoff();
}

// void _spi_write_address( int reg, int val )
// {
// 	spi_xn_cson();
// 	spi_sendbyte( reg);
// 	spi_sendbyte( val);
// 	spi_xn_csoff();
// }

void xn_readpayload( int * data, int size )
{
	int index = 0;
	spi_xn_cson();
	spi_sendbyte( R_RX_PAYLOAD ); // read rx payload
	while ( index < size )	{
		data[ index ] = spi_sendzerorecvbyte();
		++index;
	}
	spi_xn_csoff();
}

void xn_writerxaddress( int * addr )
{
	int index = 0;
	spi_xn_cson();
	spi_sendbyte( W_REGISTER | RX_ADDR_P0 );
	while ( index < 5 ) {
		spi_sendbyte( addr[ index ] );
		++index;
	}
	spi_xn_csoff();
}

void xn_writetxaddress( int * addr )
{
	int index = 0;
	spi_xn_cson();
	spi_sendbyte( W_REGISTER | TX_ADDR );
	while ( index < 5 ) {
		spi_sendbyte( addr[ index ] );
		++index;
	}
	spi_xn_csoff();
}

void xn_writepayload( int data[], int size )
{
	int index = 0;
	spi_xn_cson();
	spi_sendbyte( W_TX_PAYLOAD ); // write tx payload
	while ( index < size ) {
		spi_sendbyte( data[ index ] );
		++index;
	}
	spi_xn_csoff();
}

void xn_writeregs( uint8_t data[], uint8_t size )
{
	spi_xn_cson();
	for ( uint8_t i = 0; i < size; ++i ) {
		spi_sendbyte( data[ i ] );
	}
	spi_xn_csoff();
}

#endif
