#include "led.h"
#include "MKL25Z4.h"                    // Device header
#include <stdio.h>

#define GREEN_LED_1 1
#define GREEN_LED_2 2
#define GREEN_LED_3 4
#define GREEN_LED_4 12
#define GREEN_LED_5 4
#define GREEN_LED_6 5
#define GREEN_LED_7 8
#define GREEN_LED_8 9
#define RED_LED_1 7
//#define RED_LED_2 0
//#define RED_LED_3 3
//#define RED_LED_4 4
//#define RED_LED_5 5
//#define RED_LED_6 6
//#define RED_LED_7 10
//#define RED_LED_8 11
#define SW_BIT 6
#define MASK(x) (1 << (x))

void InitGPIOA(int port_num) {
	PORTA->PCR[port_num] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[port_num] |= PORT_PCR_MUX(1);
}

void InitGPIOC(int port_num) {
	PORTC->PCR[port_num] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[port_num] |= PORT_PCR_MUX(1);
}

void InitGPIOD(int port_num) {
	PORTD->PCR[port_num] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[port_num] |= PORT_PCR_MUX(1);
}

// Initiate GPIO 
void initLED(void) {
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | (SIM_SCGC5_PORTB_MASK) | SIM_SCGC5_PORTC_MASK | (SIM_SCGC5_PORTD_MASK)); // |= will set the bit
	// Configure MUX settings to make all 3 pins GPIO
//	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; // Clear the bit
//	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); // GPIO setting is 1 so set MUX bits to 001
//	
//	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; // Clear the bit
//	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); // GPIO setting is 1 so set MUX bits to 001
//	
//	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK; // Clear the bit
//	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1); // GPIO setting is 1 so set MUX bits to 001
	
	// 8 Green LEDs
	InitGPIOA(GREEN_LED_1);
	InitGPIOA(GREEN_LED_2);
	InitGPIOA(GREEN_LED_4);
	InitGPIOA(GREEN_LED_5);
	InitGPIOA(GREEN_LED_6);
	InitGPIOD(GREEN_LED_3);
	InitGPIOC(GREEN_LED_7);
	InitGPIOC(GREEN_LED_8);
	
	// 8 Red LEDs
	InitGPIOC(RED_LED_1);
//	InitGPIOC(RED_LED_2);
//	InitGPIOC(RED_LED_3);
//	InitGPIOC(RED_LED_4);
//	InitGPIOC(RED_LED_5);
//	InitGPIOC(RED_LED_6);
//	InitGPIOC(RED_LED_7);
//	InitGPIOC(RED_LED_8);
	
	// PTB->PDDR |= MASK(RED_LED);
	PTD->PDDR |= MASK(GREEN_LED_3);
	// apes together strong !!! - sam1
	PTA->PDDR |= MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6);
	PTC->PDDR |= MASK(GREEN_LED_7) | MASK(GREEN_LED_8);
//	PTC->PDDR |= MASK(RED_LED_1) | MASK(RED_LED_2) | MASK(RED_LED_3) | MASK(RED_LED_4) | MASK(RED_LED_5) |
//			MASK(RED_LED_6) | MASK(RED_LED_7) | MASK(RED_LED_8);
	PTC->PDDR |= MASK(RED_LED_1);
}

void off_green_strip(void) {
	PTA->PDOR &= ~MASK(GREEN_LED_1);
	PTA->PDOR &= ~MASK(GREEN_LED_2);
	PTD->PDOR &= ~MASK(GREEN_LED_3);
	PTA->PDOR &= ~MASK(GREEN_LED_4);
	PTA->PDOR &= ~MASK(GREEN_LED_5);
	PTA->PDOR &= ~MASK(GREEN_LED_6);
	PTC->PDOR &= ~MASK(GREEN_LED_7);
	PTC->PDOR &= ~MASK(GREEN_LED_8);
}

void on_green_strip(void) {
	PTA->PDOR |= MASK(GREEN_LED_1);
	PTA->PDOR |= MASK(GREEN_LED_2);
	PTD->PDOR |= MASK(GREEN_LED_3);
	PTA->PDOR |= MASK(GREEN_LED_4);
	PTA->PDOR |= MASK(GREEN_LED_5);
	PTA->PDOR |= MASK(GREEN_LED_6);
	PTC->PDOR |= MASK(GREEN_LED_7);
	PTC->PDOR |= MASK(GREEN_LED_8);
}