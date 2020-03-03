#include "drv_adc.h"
#include "drv_time.h"
#include "hardware.h"
#include "main.h"

static uint32_t adc_array[ 2 ];

void adc_init( void )
{
#ifdef ENABLE_ADC
	// ADC and DMA init is done by the generated code in stm32f4xx_hal_msp.c and main.c
	// With the current configuration HAL_DMA_IRQHandler() is called at 84e6/8/(480+12.5) = 21319.8 Hz

	// Since we do not need the interrupt handler, we turn it off.
	HAL_NVIC_DisableIRQ( DMA2_Stream0_IRQn );
	HAL_NVIC_ClearPendingIRQ( DMA2_Stream0_IRQn );

	extern ADC_HandleTypeDef hadc1;
	HAL_ADC_Start_DMA( &hadc1, adc_array, 2 );

	// Wait until the first transfer finnished.
	delay( 100 );
#endif // ENABLE_ADC
}

float adc_read( void )
{
#ifdef ENABLE_ADC
	if ( adc_array[ 1 ] != 0 ) {
		return 1.2f / (float)adc_array[ 1 ] * (float)adc_array[ 0 ]; // true voltage.
	} else { // adc_init() was not called.
		return 0.0f;
	}
#else
	return 4.2f;
#endif // ENABLE_ADC
}
