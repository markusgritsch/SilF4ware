#include "drv_mpu.h"
#include "drv_spi2.h"
#include "drv_time.h"
#include "main.h"

void spi_mpu_cson()
{
	SPI_MPU_SS_GPIO_Port->BSRR = (uint32_t)SPI_MPU_SS_Pin << 16U;
}

void spi_mpu_csoff()
{
	SPI_MPU_SS_GPIO_Port->BSRR = SPI_MPU_SS_Pin;
}

void mpu_writereg( uint8_t address, uint8_t value )
{
	spi_mpu_cson();
	spi2_sendbyte_slow( address );
	spi2_sendbyte_slow( value );
	spi_mpu_csoff();
	delay( 1 );
}

uint8_t mpu_readreg( uint8_t address )
{
	spi_mpu_cson();
	spi2_sendbyte_slow( address | 0x80 );
	uint8_t value = spi2_sendzerorecvbyte_slow();
	spi_mpu_csoff();
	delay( 1 );
	return value;
}

void mpu_readdata( uint8_t address, uint32_t data[], uint8_t size )
{
	spi_mpu_cson();
	spi2_sendbyte( address | 0x80 );
	for ( uint8_t i = 0; i < size; ++i ) {
		data[ i ] = spi2_sendzerorecvbyte();
	}
	spi_mpu_csoff();
}
