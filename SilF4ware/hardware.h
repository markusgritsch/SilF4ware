// NOTE: pin and port defines are in the generated main.h

// Used for LVC stuff
#define ENABLE_ADC

// 4 wire or 3 wire SPI
#define SOFTSPI_4WIRE // used for XN297 and NRF24
// #define SOFTSPI_3WIRE // used for XN297L; SPI_MOSI is used for data output and input

// Choose between DMA or NOP-based-delay version
#define DSHOT_DMA_BIDIR // needed for RPM_FILTER, 4k loop frequency max
// #define DSHOT_DMA_DRIVER // conventional Dshot, consumes less cycles, works for 8k loop frequency
// #define DSHOT_DRIVER // delay version

// Use this if LED is on when pin is low
#define LED_INVERT
