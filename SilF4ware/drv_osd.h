#include <stdint.h>

void drv_osd_init();
void osd_set_baudrate();
void osd_restore_baudrate();
void osd_writereg( uint8_t address, uint8_t value );
uint8_t osd_readreg( uint8_t address );
void osd_print_display_memory( uint8_t data[], uint32_t size, uint32_t pos );
void osd_putc_display_memory( uint8_t character, uint32_t pos );

// MAX7456 registers:

#define VM0 0x00
#define VM1 0x01
#define DMM 0x04
#define DMAH 0x05
#define DMAL 0x06
#define DMDI 0x07
//#define CMM 0x08
//#define CMAH 0x09
//#define CMAL 0x0A
//#define CMDI 0x0B
// #define RB0 0x10
#define OSDBL 0x6C
#define STAT 0xA0 // Status register read address
// #define DMDO 0XB0
// #define CMDO 0xC0
