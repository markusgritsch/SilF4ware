#include <stdbool.h>
#include <stdint.h>

#include "blackbox.h"
#include "config.h"
#include "drv_time.h"
#include "main.h" // HAL_UART_Transmit_DMA()

#ifdef BLACKBOX_LOGGING
// 100 bytes maximum at 2 Mbit/s UART speed and a logged data frame every 0.5 ms.
static uint8_t bb_buffer[ 93 ] = "FRAME"; // 98 bytes practical limit
#endif // BLACKBOX_LOGGING

extern int onground;
extern int pw_sustain_steps;
extern float bb_p[ 3 ];
extern float bb_i[ 3 ];
extern float bb_d[ 2 ];
extern float bb_ff[ 3 ];
extern float rx[ 4 ];
extern int pwmdir;
extern float setpoint[ 3 ];
extern float bb_throttle;
extern float vbattadc;
extern int packetrx;
extern float gyro[ 3 ];
extern float gyro_unfiltered[ 3 ];
extern float bb_accel[ 3 ];
extern float bb_mix[ 4 ];

#if defined FC_BOARD_OMNIBUS
	#define BB_UARTx_Init MX_UART4_Init
	#define BB_huartx huart4
#elif defined FC_BOARD_F4XSD
	#define BB_UARTx_Init MX_USART3_UART_Init
	#define BB_huartx huart3
#elif defined FC_BOARD_NOXE || defined FC_BOARD_NOXE_V1
	#define BB_UARTx_Init MX_USART2_UART_Init
	#define BB_huartx huart2
#endif // fc board

void blackbox_init( void )
{
#ifdef BLACKBOX_LOGGING

	extern void BB_UARTx_Init( void );
	BB_UARTx_Init();

#endif // BLACKBOX_LOGGING
}

void blackbox_log( void )
{
#ifdef BLACKBOX_LOGGING

	static uint32_t bb_iteration;

	if ( onground ) {
		bb_iteration = 0;
		return;
	}

	if ( LOOPTIME < 500 ) {
		static int count;
		if ( count == 0 ) {
			count = 500 / LOOPTIME - 1;
		} else {
			--count;
			return;
		}
	}

	int pos = 5; // size of FRAME

	// iteration
	*(uint32_t *)( &bb_buffer[ pos ] ) = bb_iteration; pos += 4;
	// time (not used for FFT, just for time displaying)
	*(uint32_t *)( &bb_buffer[ pos ] ) = gettime() * (float)WALLTIME_CORRECTION_FACTOR; pos += 4;
	// axisP
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_p[ 0 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_p[ 1 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_p[ 2 ] * 1000.0f; pos += 2;
	// axisI
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_i[ 0 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_i[ 1 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_i[ 2 ] * 1000.0f; pos += 2;
	// axisD
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_d[ 0 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_d[ 1 ] * 1000.0f; pos += 2;
	// axisF
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_ff[ 0 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_ff[ 1 ] * 1000.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_ff[ 2 ] * 1000.0f; pos += 2;
	// rcCommand
	*(int16_t *)( &bb_buffer[ pos ] ) = rx[ 0 ] * 500.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = rx[ 1 ] * 500.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = -rx[ 2 ] * 500.0f; pos += 2;
	*(uint16_t *)( &bb_buffer[ pos ] ) = 1500.0f + 500.0f * ( pwmdir == FORWARD ? rx[ 3 ] : -rx[ 3 ] ); pos += 2;
	// setpoint
	*(int16_t *)( &bb_buffer[ pos ] ) = setpoint[ 0 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = setpoint[ 1 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = -setpoint[ 2 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_throttle * 1000.0f; pos += 2;
	// vbatLatest
	*(uint16_t *)( &bb_buffer[ pos ] ) = vbattadc * 100.0f; pos += 2;
	// amperageLatest
	*(int16_t *)( &bb_buffer[ pos ] ) = pw_sustain_steps; pos += 2;
	// rssi
	*(uint16_t *)( &bb_buffer[ pos ] ) = packetrx; pos += 2;
	// gyroADC
	*(int16_t *)( &bb_buffer[ pos ] ) = gyro[ 0 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = gyro[ 1 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = -gyro[ 2 ] * RADTODEG; pos += 2;
	// accSmooth
	*(int16_t *)( &bb_buffer[ pos ] ) = -bb_accel[ 1 ]; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_accel[ 0 ]; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = bb_accel[ 2 ]; pos += 2;
	// debug
	*(int16_t *)( &bb_buffer[ pos ] ) = gyro_unfiltered[ 0 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = gyro_unfiltered[ 1 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = -gyro_unfiltered[ 2 ] * RADTODEG; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = 0; pos += 2;
	// motor
	*(uint16_t *)( &bb_buffer[ pos ] ) = bb_mix[ MOTOR_BR - 1 ] * 1000.0f; pos += 2; // BR
	*(uint16_t *)( &bb_buffer[ pos ] ) = bb_mix[ MOTOR_FR - 1 ] * 1000.0f; pos += 2; // FR
	*(uint16_t *)( &bb_buffer[ pos ] ) = bb_mix[ MOTOR_BL - 1 ] * 1000.0f; pos += 2; // BL
	*(uint16_t *)( &bb_buffer[ pos ] ) = bb_mix[ MOTOR_FL - 1 ] * 1000.0f; pos += 2; // FL
	// pos is 85
#if 1 // select between logging motor Hz (1) or sdft notch Hz (0)
	#ifdef RPM_FILTER
	extern float motor_hz[ 4 ];
	*(int16_t *)( &bb_buffer[ pos ] ) = motor_hz[ MOTOR_BR - 1 ] * 10.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = motor_hz[ MOTOR_FR - 1 ] * 10.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = motor_hz[ MOTOR_BL - 1 ] * 10.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = motor_hz[ MOTOR_FL - 1 ] * 10.0f; pos += 2;
	#endif // RPM_FILTER
#else
	#ifdef BIQUAD_SDFT_NOTCH
	extern float sdft_notch_Hz[ 4 ];
	*(int16_t *)( &bb_buffer[ pos ] ) = sdft_notch_Hz[ 0 ] * 10.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = sdft_notch_Hz[ 1 ] * 10.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = sdft_notch_Hz[ 2 ] * 10.0f; pos += 2;
	*(int16_t *)( &bb_buffer[ pos ] ) = sdft_notch_Hz[ 3 ] * 10.0f; pos += 2;
	#endif // BIQUAD_SDFT_NOTCH
#endif
	// pos is 93
	++bb_iteration;

	extern UART_HandleTypeDef BB_huartx;
	// HAL_UART_IRQHandler( &BB_huartx ); // Resets huart->gState to HAL_UART_STATE_READY
	BB_huartx.gState = HAL_UART_STATE_READY; // Do it directly to save flash space.
	HAL_UART_Transmit_DMA( &BB_huartx, bb_buffer, sizeof( bb_buffer ) );

#endif // BLACKBOX_LOGGING
}
