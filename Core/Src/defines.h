#ifndef __defines_h__
#define __defines_h__

// defines for things that do not normally need changing

#define PIDNUMBER 3

#define PI_F 3.14159265358979323846f
#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define ROLL 0
#define PITCH 1
#define YAW 2

#define RXMODE_BIND 0
#define RXMODE_NORMAL 1

#define AUXNUMBER 16

// defines for bayang protocol radio
#define CH_ON (AUXNUMBER - 2)
#define CH_OFF (AUXNUMBER - 1)
#define CH_FLIP 0
#define CH_EXPERT 1
#define CH_HEADFREE 2
#define CH_RTH 3
#define CH_AUX1 4
#define CH_AUX2 5
#define CH_EMG 10
#define CH_TO 11
// trims numbers have to be sequential, start at CH_PIT_TRIM
#define CH_PIT_TRIM 6
#define CH_RLL_TRIM 7
#define CH_THR_TRIM 8
#define CH_YAW_TRIM 9
// next 3 channels only when *not* using USE_STOCK_TX
#define CH_INV 6
#define CH_VID 7
#define CH_PIC 8

// devo tx channel mapping -- also for nr24multipro
#define DEVO_CHAN_5 CH_INV
#define DEVO_CHAN_6 CH_FLIP
#define DEVO_CHAN_7 CH_PIC
#define DEVO_CHAN_8 CH_VID
#define DEVO_CHAN_9 CH_HEADFREE
#define DEVO_CHAN_10 CH_RTH
#define DEVO_CHAN_11 CH_TO
#define DEVO_CHAN_12 CH_EMG

// multimodule mapping (taranis)
#define MULTI_CHAN_5 CH_FLIP
#define MULTI_CHAN_6 CH_RTH
#define MULTI_CHAN_7 CH_PIC
#define MULTI_CHAN_8 CH_VID
#define MULTI_CHAN_9 CH_HEADFREE
#define MULTI_CHAN_10 CH_INV

#ifdef USE_DEVO
// devo tx channel mapping -- also for nr24multipro
#define CHAN_5 CH_INV
#define CHAN_6 CH_FLIP
#define CHAN_7 CH_PIC
#define CHAN_8 CH_VID
#define CHAN_9 CH_HEADFREE
#define CHAN_10 CH_RTH
#define CHAN_ON CH_ON
#define CHAN_OFF CH_OFF
#endif

#ifdef USE_MULTI
// multimodule mapping (taranis)
#define CHAN_5 CH_FLIP
#define CHAN_6 CH_RTH
#define CHAN_7 CH_PIC
#define CHAN_8 CH_VID
#define CHAN_9 CH_HEADFREE
#define CHAN_10 CH_INV
#define CHAN_ON CH_ON
#define CHAN_OFF CH_OFF
#endif

#ifdef USE_STOCK_TX
#define CHAN_5 CH_EXPERT
#define CHAN_6 CH_AUX1
#define CHAN_7 CH_HEADFREE
#define CHAN_8 CH_RLL_TRIM
#define CHAN_9 CH_PIT_TRIM
#define CHAN_10 CH_OFF
#define CHAN_ON CH_ON
#define CHAN_OFF CH_OFF
#endif

// for inverted flight motor direction
#define FORWARD 0
#define REVERSE 1

// With constants as parameters this should be precalculated by the compiler.
// This is used to calculate 1 - alpha. filtertime = 1 / filter-cutoff-frequency.
// The approximation is good only for filtertime >> sampleperiod.
#define FILTERCALC( sampleperiod, filtertime ) ( 1.0f - ( 6.0f * (float)sampleperiod ) / ( 3.0f * (float)sampleperiod + (float)filtertime ) )

#ifdef __GNUC__ // Keil compiler verion 6 is also GCC based (ARMCLANG).
	#define _NOP_ __asm("nop;");
#else
	#define _NOP_ __asm{NOP}
#endif

#define gpioset(port, pin) (port->BSRR = pin)
#define gpioreset(port, pin) (port->BSRR = (uint32_t)pin << 16U)

#endif // __defines_h__
