#include "led.h"
#include "MKL25Z4.h"                    // Device header
#include <stdio.h>

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	
	// Enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	// Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_RE_MASK));
	UART2->C2 &= ~((UART_C2_RIE_MASK));
	
	// Set baud rate to desired value
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;		// bus_clock is always half of system_clock
	divisor = bus_clock / (baud_rate * 16);		// Lecture 8 Slide 31
	UART2->BDH = UART_BDH_SBR(divisor >> 8);	// Page 751
	UART2->BDL = UART_BDL_SBR(divisor);				// Page 752
	
	// No Parity, 8-bits
	UART2->C1 = 0;	// Page 752
	UART2->S2 = 0;	// Page 757
	UART2->C3 = 0;	// Page 758
	
	
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	// Enable Tx and Rx
	UART2->C2 |= ((UART_C2_RE_MASK));		// Page 753 - 754
	UART2->C2 |= ((UART_C2_RIE_MASK));
}