// Use 0 instead of an actual value if the respective parameter should not be changed.
typedef struct {
	uint16_t PLLM; // This value can be between 2 and 63.
	uint16_t PLLN; // This value can be between 192 and 432.
	uint16_t PLLP; // This value can be 2, 4, 6 or 8.
	uint16_t PLLQ; // This value can be between 2 and 15.
	uint16_t PLLR; // This value can be between 2 and 7 and is only available for STM32F446 devices.
} TM_RCC_PLL_t;

void TM_RCC_SetPLL(TM_RCC_PLL_t* PLL_Settings);
void TM_RCC_GetPLL(TM_RCC_PLL_t* PLL_Settings);
uint8_t TM_RCC_IsPLLReady(void);
