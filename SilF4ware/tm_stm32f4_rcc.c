// http://stm32f4-discovery.net/2015/05/library-59-change-pll-settings-while-stm32f4xx-is-running/
// Copyright (C) Tilen Majerle, 2015

#include <string.h> // memcmp

#include "stm32f4xx.h"

#include "tm_stm32f4_rcc.h"

/* PLL configuration */
#define RCC_PLLR_MASK ((uint32_t)0x70000000)
#define RCC_PLLR_POS 28

void TM_RCC_SetPLL(TM_RCC_PLL_t* PLL_Settings)
{
	uint16_t timeout;
	TM_RCC_PLL_t tmp;

	/* Read PLL settings */
	TM_RCC_GetPLL(&tmp);

	/* Check if structures are equal */
	if (memcmp(PLL_Settings, &tmp, sizeof(TM_RCC_PLL_t)) == 0) {
		/* Don't change PLL settings if settings are the same */
		return;
	}

	/* Enable HSI clock */
	RCC->CR |= RCC_CR_HSION;

	/* Wait till HSI is ready */
	timeout = 0xFFFF;
	while (!(RCC->CR & RCC_CR_HSIRDY) && timeout--);

	/* Select HSI clock as main clock */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSI;

	/* Disable PLL */
	RCC->CR &= ~RCC_CR_PLLON;

	/* Set PLL settings */
	if (PLL_Settings->PLLM) {
		RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM_Msk) | ((PLL_Settings->PLLM << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk);
	}
	if (PLL_Settings->PLLN) {
		RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN_Msk) | ((PLL_Settings->PLLN << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk);
	}
	if (PLL_Settings->PLLP) {
		RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLP_Msk) | ((((PLL_Settings->PLLP >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP_Msk);
	}
	if (PLL_Settings->PLLQ) {
		RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLQ_Msk) | ((PLL_Settings->PLLQ << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk);
	}
	if (PLL_Settings->PLLR) {
		RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLR_MASK) | ((PLL_Settings->PLLR << RCC_PLLR_POS) & RCC_PLLR_MASK);
	}

	/* Enable PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till PLL is ready */
	timeout = 0xFFFF;
	while (!TM_RCC_IsPLLReady() && timeout--);

	/* Enable PLL as main clock */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;

	/* Update system core clock variable */
	SystemCoreClockUpdate();
}

void TM_RCC_GetPLL(TM_RCC_PLL_t* PLL_Settings)
{
	/* Read all PLL settings */
	PLL_Settings->PLLM = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos;
	PLL_Settings->PLLN = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
	PLL_Settings->PLLP = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos) + 1) << 1;
	PLL_Settings->PLLQ = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ_Msk) >> RCC_PLLCFGR_PLLQ_Pos;
	PLL_Settings->PLLR = (RCC->PLLCFGR & RCC_PLLR_MASK) >> RCC_PLLR_POS;
}

uint8_t TM_RCC_IsPLLReady(void)
{
	/* Return PLL ready status */
	return (RCC->CR & RCC_CR_PLLRDY);
}
