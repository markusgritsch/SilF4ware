// NOTE: pin and port defines are in the generated main.h

// Used for LVC stuff
#define ENABLE_ADC

// 4 wire or 3 wire SPI
#define SOFTSPI_4WIRE // used for XN297 and NRF24
// #define SOFTSPI_3WIRE // used for XN297L; SPI_MOSI is used for data output and input

// Choose between DMA or bitbang version
#define DSHOT_DMA_DRIVER
// #define DSHOT_DRIVER

// Use this if LED is on when pin is low
#define LED_INVERT
