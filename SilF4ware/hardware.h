// NOTE: pin and port defines are in the generated main.h

// Used for LVC stuff
#define ENABLE_ADC

#if defined FC_BOARD_OMNIBUS || defined FC_BOARD_F4XSD
	#define RX_SOFTSPI_4WIRE
	#define MPU_HARDSPI 1
#elif defined FC_BOARD_NOXE
	#define RX_HARDSPI 1
	#define MPU_HARDSPI 2
#elif defined FC_BOARD_NOXE_V1
	#define RX_HARDSPI 2
	#define MPU_HARDSPI 1
#endif

// RX_HARDSPI or 4 wire or 3 wire RX_SOFTSPI (for communication with the radio module)
//#define RX_HARDSPI 1 // only SPI1 and SPI2 implemented
//#define RX_SOFTSPI_4WIRE // used for XN297 and NRF24
//#define RX_SOFTSPI_3WIRE // used for XN297L; SPI_RX_MOSI is used for data output and input

// MPU_HARDSPI or MPU_SOFTSPI (for communication with the gyro chip)
//#define MPU_HARDSPI 1 // only SPI1 and SPI2 implemented
//#define MPU_SOFTSPI

// Choose between DMA or NOP-based-delay version
#define DSHOT_DMA_BIDIR // needed for RPM_FILTER, 4k loop frequency max
//#define DSHOT_DMA_DRIVER // conventional Dshot, consumes less cycles, works for 8k loop frequency
//#define DSHOT_DRIVER // delay version

// Use this if LED is on when pin is low
#define LED_INVERT
