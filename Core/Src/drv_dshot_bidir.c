// Enable this for 3D. The 'Motor Direction' setting in BLHeliSuite must be set to 'Bidirectional' (or 'Bidirectional Rev.') accordingly:
#define BIDIRECTIONAL

// IDLE_OFFSET is added to the throttle. Adjust its value so that the motors still spin at minimum throttle.
#define IDLE_OFFSET 30 // 4S

// Select Dshot1200, Dshot600, Dshot300, or Dshot150
// #define DSHOT 1200 // BLHeli_32 only
// #define DSHOT 600 // BLHeli_S BB2 (not supported by BB1)
#define DSHOT 300 // works on BB1
// #define DSHOT 150

// Number of magnets on the motor bell. A correct value is of paramount importance for motor RPM calculation.
#define MOTOR_POLE_COUNT 14 // usually on 22xx motors and above
// #define MOTOR_POLE_COUNT 12 // usually on 18xx motors and below


#include <stdbool.h>

#include "defines.h"
#include "drv_dshot.h"
#include "drv_time.h"
#include "hardware.h"
#include "main.h"

#define DSHOT_BIT_TIME_THIRD ( ( SYS_CLOCK_FREQ_MHZ * 1000 / DSHOT / 3 ) - 1 )
#define GCR_FREQUENCY ( DSHOT * 5 / 4 ) // 375, 750 .. according to specification
#define GCR_BIT_TIME_THIRD ( ( SYS_CLOCK_FREQ_MHZ * 1000 / GCR_FREQUENCY / 3 ) - 1 )
#define GCR_BUFFER_SIZE ( ( 30 + 5 ) * GCR_FREQUENCY * 3 / 1000 + 21 * 3 ) // 30 us delay, 21 bits, 5 us post

#ifdef DSHOT_DMA_BIDIR

extern int onground;

int pwmdir = FORWARD;
static uint32_t dshot_data[ 16 * 3 ] = { 0 };

static uint32_t dshot_data_port1st[ 16 * 3 ] = { 0 }; // DMA buffer
static uint32_t dshot_data_port2nd[ 16 * 3 ] = { 0 };

static uint32_t gcr_data_port1st[ GCR_BUFFER_SIZE ] = { 0 }; // DMA buffer
static uint32_t gcr_data_port2nd[ GCR_BUFFER_SIZE ] = { 0 };

static GPIO_TypeDef * GPIO1st = 0;
static GPIO_TypeDef * GPIO2nd = 0;

typedef enum
{
	WRITE_DSHOT = 0U,
	READ_TELEMETRY
} DmaState;

static DmaState dma_state = WRITE_DSHOT;

static void make_packet( uint8_t number, uint16_t value, bool telemetry );
static void dshot_dma_start( void );
static void dma_write_dshot( void );
static void dma_read_telemetry( void );
static void decode_gcr_telemetry( void );
static float decode_to_hz( uint32_t gcr_data[], uint16_t pin );

static GPIO_InitTypeDef ESC_InitStruct = {
	.Pin = ESC1_Pin,
	.Mode = GPIO_MODE_OUTPUT_PP,
	.Pull = GPIO_PULLUP,
	.Speed = GPIO_SPEED_FREQ_HIGH
};

void pwm_init()
{
	extern void MX_TIM1_Init( void );
	MX_TIM1_Init();

	TIM1->ARR = DSHOT_BIT_TIME_THIRD;
	TIM1->CCR1 = 0; // DMA transfer for both
	TIM1->CCR2 = 0; // ports on timer reset.

	// This driver supports Dshot pins being located on up to two distinct GPIO ports.
	GPIO1st = ESC1_GPIO_Port;
	if ( ESC2_GPIO_Port != GPIO1st ) {
		GPIO2nd = ESC2_GPIO_Port;
	}
	if ( ESC3_GPIO_Port != GPIO1st ) {
		GPIO2nd = ESC3_GPIO_Port;
	}
	if ( ESC4_GPIO_Port != GPIO1st ) {
		GPIO2nd = ESC4_GPIO_Port;
	}
	if ( ( ESC2_GPIO_Port != GPIO1st && ESC2_GPIO_Port != GPIO2nd ) ||
		( ESC3_GPIO_Port != GPIO1st && ESC3_GPIO_Port != GPIO2nd ) ||
		( ESC4_GPIO_Port != GPIO1st && ESC4_GPIO_Port != GPIO2nd ) )
	{
		extern void failloop( int );
		failloop( 7 );
	}

	ESC_InitStruct.Pin = ESC1_Pin;
	HAL_GPIO_Init( ESC1_GPIO_Port, &ESC_InitStruct );
	ESC_InitStruct.Pin = ESC2_Pin;
	HAL_GPIO_Init( ESC2_GPIO_Port, &ESC_InitStruct );
	ESC_InitStruct.Pin = ESC3_Pin;
	HAL_GPIO_Init( ESC3_GPIO_Port, &ESC_InitStruct );
	ESC_InitStruct.Pin = ESC4_Pin;
	HAL_GPIO_Init( ESC4_GPIO_Port, &ESC_InitStruct );
}

int idle_offset = IDLE_OFFSET; // gets corrected by battery_scale_factor in battery.c
void pwm_set( uint8_t number, float pwm )
{
	if ( pwm < 0.0f ) {
		pwm = 0.0f;
	}
	if ( pwm > 0.999f ) {
		pwm = 0.999f;
	}

	uint16_t value = 0;

#ifdef BIDIRECTIONAL

	if ( pwmdir == FORWARD ) {
		// maps 0.0 .. 0.999 to 48 + IDLE_OFFSET .. 1047
		value = 48 + idle_offset + (uint16_t)( pwm * ( 1000 - idle_offset ) );
	} else if ( pwmdir == REVERSE ) {
		// maps 0.0 .. 0.999 to 1048 + IDLE_OFFSET .. 2047
		value = 1048 + idle_offset + (uint16_t)( pwm * ( 1000 - idle_offset ) );
	}

#else

	// maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
	value = 48 + idle_offset * 2 + (uint16_t)( pwm * ( 2001 - idle_offset * 2 ) );

#endif

	if ( onground ) {
		value = 0; // stop the motors
	}

	make_packet( number, value, false );

	if ( number == 3 ) {
		dshot_dma_start();
	}
}

// make Dshot packet
static void make_packet( uint8_t number, uint16_t value, bool telemetry )
{
	uint16_t packet = ( value << 1 ) | ( telemetry ? 1 : 0 ); // Here goes telemetry bit
	// compute checksum
	uint16_t csum = 0;
	uint16_t csum_data = packet;

	for ( uint8_t i = 0; i < 3; ++i ) {
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}
	csum = ~csum; // RPM telemetry Dshot protocol requires complement checksum
	csum &= 0xF;
	// append checksum
	packet = ( packet << 4 ) | csum;

	// generate pulses for whole packet
	for ( uint8_t i = 0; i < 16; ++i ) {
		if ( packet & 0x8000 ) { // MSB first
			dshot_data[ i * 3 + 0 ] |= 0 << number; // inverted signal for RPM telemetry Dshot protocol
			dshot_data[ i * 3 + 1 ] |= 0 << number; // i.e. 1 = __-
			dshot_data[ i * 3 + 2 ] |= 1 << number;
		} else {
			dshot_data[ i * 3 + 0 ] |= 0 << number; // 0 = _--
			dshot_data[ i * 3 + 1 ] |= 1 << number;
			dshot_data[ i * 3 + 2 ] |= 1 << number;
		}
		packet <<= 1;
	}
}

// make Dshot dma packet, then fire
static void dshot_dma_start()
{
	// Decode previous GCR telemetry just before starting the next Dshot DMA transfer.
	// This way we ensure that receiving telemetry has finnished.
	decode_gcr_telemetry(); // Takes about 30 us @168 MHz.

	for ( uint8_t i = 0; i < 16 * 3; ++i ) {
		dshot_data_port1st[ i ] = 0;
		dshot_data_port2nd[ i ] = 0;

		int shift = ( dshot_data[ i ] & 0x01 ) ? 0 : 16;
		if ( ESC1_GPIO_Port == GPIO1st ) {
			dshot_data_port1st[ i ] |= ESC1_Pin << shift;
		} else {
			dshot_data_port2nd[ i ] |= ESC1_Pin << shift;
		}

		shift = ( dshot_data[ i ] & 0x02 ) ? 0 : 16;
		if ( ESC2_GPIO_Port == GPIO1st ) {
			dshot_data_port1st[ i ] |= ESC2_Pin << shift;
		} else {
			dshot_data_port2nd[ i ] |= ESC2_Pin << shift;
		}

		shift = ( dshot_data[ i ] & 0x04 ) ? 0 : 16;
		if ( ESC3_GPIO_Port == GPIO1st ) {
			dshot_data_port1st[ i ] |= ESC3_Pin << shift;
		} else {
			dshot_data_port2nd[ i ] |= ESC3_Pin << shift;
		}

		shift = ( dshot_data[ i ] & 0x08 ) ? 0 : 16;
		if ( ESC4_GPIO_Port == GPIO1st ) {
			dshot_data_port1st[ i ] |= ESC4_Pin << shift;
		} else {
			dshot_data_port2nd[ i ] |= ESC4_Pin << shift;
		}

		dshot_data[ i ] = 0;
	}

	dma_write_dshot();
}

static void dma_write_dshot()
{
	dma_state = WRITE_DSHOT;

	// Change ESC pins to output
	ESC1_GPIO_Port->MODER |= ESC1_Pin * ESC1_Pin;
	ESC2_GPIO_Port->MODER |= ESC2_Pin * ESC2_Pin;
	ESC3_GPIO_Port->MODER |= ESC3_Pin * ESC3_Pin;
	ESC4_GPIO_Port->MODER |= ESC4_Pin * ESC4_Pin;

	extern TIM_HandleTypeDef htim1;
	extern DMA_HandleTypeDef hdma_tim1_ch1;
	extern DMA_HandleTypeDef hdma_tim1_ch2;

	hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
	HAL_DMA_Init( &hdma_tim1_ch1 );
	hdma_tim1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
	HAL_DMA_Init( &hdma_tim1_ch2 );

	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC2 );

	__HAL_DMA_ENABLE_IT( &hdma_tim1_ch2, DMA_IT_TC );

	HAL_DMA_Start( &hdma_tim1_ch1, (uint32_t)dshot_data_port1st, (uint32_t)&GPIO1st->BSRR, 16 * 3 );
	HAL_DMA_Start( &hdma_tim1_ch2, (uint32_t)dshot_data_port2nd, (uint32_t)&GPIO2nd->BSRR, 16 * 3 );

	TIM1->ARR = DSHOT_BIT_TIME_THIRD;
	__HAL_TIM_SetCounter( &htim1, DSHOT_BIT_TIME_THIRD );
	HAL_TIM_Base_Start( &htim1 );
}

void DMA2_Stream2_IRQHandler()
{
	extern DMA_HandleTypeDef hdma_tim1_ch2;
	HAL_DMA_IRQHandler( &hdma_tim1_ch2 );

	extern TIM_HandleTypeDef htim1;
	extern DMA_HandleTypeDef hdma_tim1_ch1;
	extern DMA_HandleTypeDef hdma_tim1_ch2;

	HAL_TIM_Base_Stop( &htim1 );

	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_CC2 );

	// This resets the hdma->State = HAL_DMA_STATE_READY; otherwise, no new transfer is started.
	HAL_DMA_Abort( &hdma_tim1_ch1 );
	HAL_DMA_Abort( &hdma_tim1_ch2 );

	if ( dma_state == WRITE_DSHOT ) {
		dma_read_telemetry();
	} else if ( dma_state == READ_TELEMETRY ) {
		// Do not decode_gcr_telemetry() here in the ISR. If it get's called at the end of main-loop busy-waiting,
		// it could still be running while we should already be starting the next loop cycle, which would ruin a
		// stable loop time. Instead we run decode_gcr_telemetry() before starting the next Dshot DMA transfer.
	}
}

static void dma_read_telemetry()
{
	dma_state = READ_TELEMETRY;

	// Change ESC pins to input
	ESC1_GPIO_Port->MODER &= ~( ESC1_Pin * ESC1_Pin );
	ESC2_GPIO_Port->MODER &= ~( ESC2_Pin * ESC2_Pin );
	ESC3_GPIO_Port->MODER &= ~( ESC3_Pin * ESC3_Pin );
	ESC4_GPIO_Port->MODER &= ~( ESC4_Pin * ESC4_Pin );

	extern TIM_HandleTypeDef htim1;
	extern DMA_HandleTypeDef hdma_tim1_ch1;
	extern DMA_HandleTypeDef hdma_tim1_ch2;

	hdma_tim1_ch1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	HAL_DMA_Init( &hdma_tim1_ch1 );
	hdma_tim1_ch2.Init.Direction = DMA_PERIPH_TO_MEMORY;
	HAL_DMA_Init( &hdma_tim1_ch2 );

	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC2 );

	__HAL_DMA_ENABLE_IT( &hdma_tim1_ch2, DMA_IT_TC );

	HAL_DMA_Start( &hdma_tim1_ch1, (uint32_t)&GPIO1st->IDR, (uint32_t)gcr_data_port1st, GCR_BUFFER_SIZE );
	HAL_DMA_Start( &hdma_tim1_ch2, (uint32_t)&GPIO2nd->IDR, (uint32_t)gcr_data_port2nd, GCR_BUFFER_SIZE );

	TIM1->ARR = GCR_BIT_TIME_THIRD;
	__HAL_TIM_SetCounter( &htim1, GCR_BIT_TIME_THIRD );
	HAL_TIM_Base_Start( &htim1 );
}

float motor_hz[ 4 ];
static void decode_gcr_telemetry()
{
	if ( ESC1_GPIO_Port == GPIO1st ) {
		motor_hz[ 0 ] = decode_to_hz( gcr_data_port1st, ESC1_Pin );
	} else {
		motor_hz[ 0 ] = decode_to_hz( gcr_data_port2nd, ESC1_Pin );
	}

	if ( ESC2_GPIO_Port == GPIO1st ) {
		motor_hz[ 1 ] = decode_to_hz( gcr_data_port1st, ESC2_Pin );
	} else {
		motor_hz[ 1 ] = decode_to_hz( gcr_data_port2nd, ESC2_Pin );
	}

	if ( ESC3_GPIO_Port == GPIO1st ) {
		motor_hz[ 2 ] = decode_to_hz( gcr_data_port1st, ESC3_Pin );
	} else {
		motor_hz[ 2 ] = decode_to_hz( gcr_data_port2nd, ESC3_Pin );
	}

	if ( ESC4_GPIO_Port == GPIO1st ) {
		motor_hz[ 3 ] = decode_to_hz( gcr_data_port1st, ESC4_Pin );
	} else {
		motor_hz[ 3 ] = decode_to_hz( gcr_data_port2nd, ESC4_Pin );
	}
}

// https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
static uint32_t gcr_decode[ 32 ] = {
	0xFF, // 00
	0xFF, // 01
	0xFF, // 02
	0xFF, // 03
	0xFF, // 04
	0xFF, // 05
	0xFF, // 06
	0xFF, // 07
	0xFF, // 08
	0x09, // 09 (01001) -> 9
	0x0A, // 0A (01010) -> A
	0x0B, // 0B (01011) -> B
	0xFF, // 0C
	0x0D, // 0D (01101) -> D
	0x0E, // 0E (01110) -> E
	0x0F, // 0F (01111) -> F
	0xFF, // 10
	0xFF, // 11
	0x02, // 12 (10010) -> 2
	0x03, // 13 (10011) -> 3
	0xFF, // 14
	0x05, // 15 (10101) -> 5
	0x06, // 16 (10110) -> 6
	0x07, // 17 (10111) -> 7
	0xFF, // 18
	0x00, // 19 (11001) -> 0
	0x08, // 1A (11010) -> 8
	0x01, // 1B (11011) -> 1
	0xFF, // 1C
	0x04, // 1D (11101) -> 4
	0x0C, // 1E (11110) -> C
	0xFF, // 1F
};

// #define RPM_TELEMETRY_DEBUG

#ifdef RPM_TELEMETRY_DEBUG
uint32_t rpm_telemetry_no_telemetry_count;
uint32_t rpm_telemetry_bitsize_errors; // if GCR_FREQUENCY is set to a too high value
uint32_t rpm_telemetry_gcr_decode_errors;
uint32_t rpm_telemetry_csum_errors;
uint32_t rpm_telemetry_sample_stats[ 12 ];
#endif

static float decode_to_hz( uint32_t gcr_data[], uint16_t pin )
{
	uint32_t index = 23 * GCR_FREQUENCY * 3 / 1000; // Start looking at 23 us.
	while ( index < GCR_BUFFER_SIZE && ( ( gcr_data[ index ] & pin ) != 0 ) ) { // Find the start bit.
		++index;
	}
	if ( index == GCR_BUFFER_SIZE ) {
#ifdef RPM_TELEMETRY_DEBUG
		++rpm_telemetry_no_telemetry_count;
#endif
		return 0.0f; // No RPM telemetry found, which is allowed in case of overburdened ESC.
	}

	uint32_t gcr_value = 0;
	uint32_t previous_sample = 0;
	uint32_t sample_count = 0;
	uint32_t bits_decoded = 0;
	++index;
	while ( index < GCR_BUFFER_SIZE ) {
		++sample_count;
		if ( ( gcr_data[ index ] & pin ) != previous_sample ) {
			if ( sample_count <= 1 * 3 + 1 ) {
				gcr_value = ( gcr_value << 1 ) | 0x01;
				bits_decoded += 1;
			} else if ( sample_count <= 2 * 3 + 1 ) {
				gcr_value = ( gcr_value << 2 ) | 0x01;
				bits_decoded += 2;
			} else if ( sample_count <= 3 * 3 + 1 ) {
				gcr_value = ( gcr_value << 3 ) | 0x01;
				bits_decoded += 3;
			}
#ifdef RPM_TELEMETRY_DEBUG
			else {
				++rpm_telemetry_bitsize_errors;
			}
			++rpm_telemetry_sample_stats[ sample_count ];
#endif
			sample_count = 0;
		}
		previous_sample = gcr_data[ index ] & pin;
		++index;
	}
	if ( bits_decoded < 20 ) {
		gcr_value <<= 1; // Handle the case when the last quintet ends with a zero.
		++bits_decoded;
	}
#ifdef RPM_TELEMETRY_DEBUG
	if ( bits_decoded != 20 ) {
			++rpm_telemetry_bitsize_errors;
	}
#endif

	const uint32_t nibble1 = gcr_decode[ ( gcr_value & 0xF8000 ) >> 15 ];
	const uint32_t nibble2 = gcr_decode[ ( gcr_value & 0x07c00 ) >> 10 ];
	const uint32_t nibble3 = gcr_decode[ ( gcr_value & 0x003e0 ) >> 5 ];
	const uint32_t telemetry_csum = gcr_decode[ gcr_value & 0x1F ];
	if ( nibble1 > 0xF || nibble2 > 0xF || nibble3 > 0xF || telemetry_csum > 0xF ) {
#ifdef RPM_TELEMETRY_DEBUG
		++rpm_telemetry_gcr_decode_errors;
#endif
		return 0.0f;
	}
	const uint32_t telemetry_data = ( nibble1 << 8 ) | ( nibble2 << 4 ) | nibble3;
	const uint32_t csum = ~( nibble1 ^ nibble2 ^ nibble3 ) & 0xF; // xor nibbles and complement checksum

	if ( csum != telemetry_csum ) {
#ifdef RPM_TELEMETRY_DEBUG
			++rpm_telemetry_csum_errors;
#endif
		return 0.0f;
	}

	const uint32_t mantissa = telemetry_data & 0x1FF; // 9 bit
	const uint32_t exponent = ( telemetry_data & 0xE00 ) >> 9; // 3 bit
	const uint32_t eperiod_us = mantissa << exponent; // 1/erps

	return 1e6f / (float)eperiod_us * 2 / (float)MOTOR_POLE_COUNT; // motor frequency in Hz
}

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

void motorbeep( bool motors_failsafe, int channel )
{
	static unsigned long motor_beep_time = 0;
	unsigned long time = gettime();
	extern char aux[];
	if ( ( motors_failsafe && time > 60e6f ) || aux[ channel ] ) {
		if ( motor_beep_time == 0 ) {
			motor_beep_time = time;
		}
		const unsigned long delta_time = time - motor_beep_time;
		uint8_t beep_command = 0;
		#define INTERVAL 250000
		if ( delta_time % 2000000 < INTERVAL ) {
			beep_command = DSHOT_CMD_BEEP1;
		} else if ( delta_time % 2000000 < INTERVAL * 2 ) {
			beep_command = DSHOT_CMD_BEEP3;
		} else if ( delta_time % 2000000 < INTERVAL * 3 ) {
			beep_command = DSHOT_CMD_BEEP2;
		} else if ( delta_time % 2000000 < INTERVAL * 4 ) {
			beep_command = DSHOT_CMD_BEEP4;
		}
		if ( beep_command != 0 ) {
			make_packet( 0, beep_command, true );
			make_packet( 1, beep_command, true );
			make_packet( 2, beep_command, true );
			make_packet( 3, beep_command, true );
			dshot_dma_start();
		}
	} else {
		motor_beep_time = 0;
	}
}

#endif // DSHOT_DMA_BIDIR
