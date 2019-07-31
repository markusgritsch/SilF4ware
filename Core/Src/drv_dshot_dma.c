// Original BWHOOP DMA code by JazzMac who based his work on http://www.cnblogs.com/shangdawei/p/4762035.html

// Enable this for 3D. The 'Motor Direction' setting in BLHeliSuite must be set to 'Bidirectional' (or 'Bidirectional Rev.') accordingly:
#define BIDIRECTIONAL

// IDLE_OFFSET is added to the throttle. Adjust its value so that the motors still spin at minimum throttle.
#define IDLE_OFFSET 30 // 4S

// Select Dshot1200, Dshot600, Dshot300, or Dshot150
// #define DSHOT 1200 // BLHeli_32 only
#define DSHOT 600 // BLHeli_S BB2 (not supported by BB1)
// #define DSHOT 300 // works on BB1
// #define DSHOT 150 // Dshot150 is too slow if the motors are on two different ports and 8k loop frequency is used.


#include <stdbool.h>

#include "defines.h"
#include "drv_dshot.h"
#include "drv_time.h"
#include "hardware.h"
#include "main.h"

#define DSHOT_BIT_TIME 	( ( SYS_CLOCK_FREQ_MHZ * 1000 / DSHOT ) - 1 )
#define DSHOT_T0H_TIME ( DSHOT_BIT_TIME * 0.30 )
#define DSHOT_T1H_TIME ( DSHOT_BIT_TIME * 0.60 )

#ifdef DSHOT_DMA_DRIVER

extern bool failsafe;
extern int onground;

int pwmdir = FORWARD;

volatile int dshot_dma_phase = 0; // 1: portA, 2: portB, 0: idle
uint16_t dshot_packet[ 4 ]; // 16bits dshot data for 4 motors

uint32_t motor_data_portA[ 16 ] = { 0 }; // DMA buffer: reset output when bit data=0 at TOH timing
uint32_t motor_data_portB[ 16 ] = { 0 };

uint32_t dshot_portA[ 1 ] = { 0 }; // sum of all motor pins at portA
uint32_t dshot_portA_off[ 1 ] = { 0 };
uint32_t dshot_portB[ 1 ] = { 0 }; // sum of all motor pins at portB
uint32_t dshot_portB_off[ 1 ] = { 0 };

void pwm_init()
{
	extern void MX_TIM1_Init( void );
	MX_TIM1_Init();

	TIM1->ARR = DSHOT_BIT_TIME;
	TIM1->CCR1 = DSHOT_T0H_TIME;
	TIM1->CCR2 = DSHOT_T1H_TIME;

	if ( ESC1_GPIO_Port == GPIOA ) {
		*dshot_portA |= ESC1_Pin;
	} else{
		*dshot_portB |= ESC1_Pin;
	}
	if ( ESC2_GPIO_Port == GPIOA ) {
		*dshot_portA |= ESC2_Pin;
	} else{
		*dshot_portB |= ESC2_Pin;
	}
	if ( ESC3_GPIO_Port == GPIOA ) {
		*dshot_portA |= ESC3_Pin;
	} else{
		*dshot_portB |= ESC3_Pin;
	}
	if ( ESC4_GPIO_Port == GPIOA ) {
		*dshot_portA |= ESC4_Pin;
	} else{
		*dshot_portB |= ESC4_Pin;
	}

	*dshot_portA_off = ( *dshot_portA ) << 16;
	*dshot_portB_off = ( *dshot_portB ) << 16;
}

static void dshot_dma_portA()
{
	extern TIM_HandleTypeDef htim1;
	extern DMA_HandleTypeDef hdma_tim1_up;
	extern DMA_HandleTypeDef hdma_tim1_ch1;
	extern DMA_HandleTypeDef hdma_tim1_ch2;

	HAL_TIM_Base_Stop( &htim1 );

	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_UPDATE );
	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_CC2 );

	// This resets the hdma->State = HAL_DMA_STATE_READY; otherwise, no new transfer is started.
	HAL_DMA_Abort( &hdma_tim1_up );
	HAL_DMA_Abort( &hdma_tim1_ch1 );
	HAL_DMA_Abort( &hdma_tim1_ch2 );

	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_UPDATE );
	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC2 );

	__HAL_DMA_ENABLE_IT( &hdma_tim1_ch2, DMA_IT_TC );

	HAL_DMA_Start( &hdma_tim1_up, (uint32_t)dshot_portA, (uint32_t)&GPIOA->BSRR, 16 );
	HAL_DMA_Start( &hdma_tim1_ch1, (uint32_t)motor_data_portA, (uint32_t)&GPIOA->BSRR, 16 );
	HAL_DMA_Start( &hdma_tim1_ch2, (uint32_t)dshot_portA_off, (uint32_t)&GPIOA->BSRR, 16 );

	__HAL_TIM_SetCounter( &htim1, DSHOT_BIT_TIME );
	HAL_TIM_Base_Start( &htim1 );
}

static void dshot_dma_portB()
{
	extern TIM_HandleTypeDef htim1;
	extern DMA_HandleTypeDef hdma_tim1_up;
	extern DMA_HandleTypeDef hdma_tim1_ch1;
	extern DMA_HandleTypeDef hdma_tim1_ch2;

	HAL_TIM_Base_Stop( &htim1 );

	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_UPDATE );
	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_DISABLE_DMA( &htim1, TIM_DMA_CC2 );

	// This resets the hdma->State = HAL_DMA_STATE_READY; otherwise, no new transfer is started.
	HAL_DMA_Abort( &hdma_tim1_up );
	HAL_DMA_Abort( &hdma_tim1_ch1 );
	HAL_DMA_Abort( &hdma_tim1_ch2 );

	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_UPDATE );
	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC1 );
	__HAL_TIM_ENABLE_DMA( &htim1, TIM_DMA_CC2 );

	__HAL_DMA_ENABLE_IT( &hdma_tim1_ch2, DMA_IT_TC );

	HAL_DMA_Start( &hdma_tim1_up, (uint32_t)dshot_portB, (uint32_t)&GPIOB->BSRR, 16 );
	HAL_DMA_Start( &hdma_tim1_ch1, (uint32_t)motor_data_portB, (uint32_t)&GPIOB->BSRR, 16 );
	HAL_DMA_Start( &hdma_tim1_ch2, (uint32_t)dshot_portB_off, (uint32_t)&GPIOB->BSRR, 16 );

	__HAL_TIM_SetCounter( &htim1, DSHOT_BIT_TIME );
	HAL_TIM_Base_Start( &htim1 );
}

// make dshot packet
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

	csum &= 0xf;
	// append checksum
	dshot_packet[ number ] = ( packet << 4 ) | csum;
}

// make dshot dma packet, then fire
static void dshot_dma_start()
{
	// generate dshot dma packet
	for ( uint8_t i = 0; i < 16; ++i ) {
		motor_data_portA[ i ] = 0;
		motor_data_portB[ i ] = 0;

		if ( ! ( dshot_packet[0] & 0x8000 ) ) {
			if ( ESC1_GPIO_Port == GPIOA ) {
				motor_data_portA[ i ] |= ESC1_Pin << 16;
			} else {
				motor_data_portB[ i ] |= ESC1_Pin << 16;
			}
		}
		if ( ! ( dshot_packet[1] & 0x8000 ) ) {
			if ( ESC2_GPIO_Port == GPIOA ) {
				motor_data_portA[ i ] |= ESC2_Pin << 16;
			} else {
				motor_data_portB[ i ] |= ESC2_Pin << 16;
			}
		}
		if ( ! ( dshot_packet[2] & 0x8000 ) ) {
			if ( ESC3_GPIO_Port == GPIOA ) {
				motor_data_portA[ i ] |= ESC3_Pin << 16;
			} else {
				motor_data_portB[ i ] |= ESC3_Pin << 16;
			}
		}
		if ( ! ( dshot_packet[3] & 0x8000 ) ) {
			if ( ESC4_GPIO_Port == GPIOA ) {
				motor_data_portA[ i ] |= ESC4_Pin << 16;
			} else {
				motor_data_portB[ i ] |= ESC4_Pin << 16;
			}
		}

		dshot_packet[ 0 ] <<= 1;
		dshot_packet[ 1 ] <<= 1;
		dshot_packet[ 2 ] <<= 1;
		dshot_packet[ 3 ] <<= 1;
	}

	if ( ESC1_GPIO_Port == GPIOB || ESC2_GPIO_Port == GPIOB || ESC3_GPIO_Port == GPIOB || ESC4_GPIO_Port == GPIOB ) {
		dshot_dma_phase = 2;
	} else {
		dshot_dma_phase = 1;
	}

	dshot_dma_portA();
}

void DMA2_Stream2_IRQHandler(void)
{
	extern DMA_HandleTypeDef hdma_tim1_ch2;
	HAL_DMA_IRQHandler( &hdma_tim1_ch2 );

	switch ( dshot_dma_phase ) {
		case 2:
			dshot_dma_phase = 1;
			dshot_dma_portB();
			break;
		case 1:
			dshot_dma_phase = 0;
			break;
		default:
			dshot_dma_phase = 0;
			break;
	}
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

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

void motorbeep( int channel )
{
	static unsigned long motor_beep_time = 0;
	unsigned long time = gettime();
	extern char aux[];
	if ( ( failsafe && time > 60e6f ) || aux[ channel ] ) {
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

#endif // DSHOT_DMA_DRIVER
