#include "led.h"
#include "MKL25Z4.h"                    // Device header
#include <stdio.h>

#define ECHO 13
#define TRIG 9
#define MASK(x) (1 << (x))

void initUS(void) {
	// Enable Clock to PORTA and PORTB
	SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTA_MASK);
	
	PORTB->PCR[TRIG] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[TRIG] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[ECHO] &= ~(PORT_PCR_MUX_MASK | PORT_PCR_PS_MASK);
	PORTA->PCR[ECHO] |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b) | PORT_PCR_PE_MASK;
	
	// Set Data Direction Registers for PortB
	PTB->PDDR |= MASK(TRIG);		// TRIG is output
	PTA->PDDR &= ~MASK(ECHO);  	// ECHO is input
	
	NVIC_SetPriority(PORTA_IRQn,1);
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}