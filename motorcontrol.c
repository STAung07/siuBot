#include "motorcontrol.h"
#include "MKL25Z4.h"                    // Device header
#include <stdio.h>

#define PTB0_Pin 0 // TPM1_CH0 => White wire
#define PTB1_Pin 1 // TPM1_CH1 => Yellow wire
#define PTB2_Pin 2 // TPM2_CH0 => Blue wire
#define PTB3_Pin 3 // TPM2_CH1 => Green wire 

#define DUTY_CYCLE 7500
#define HALF_TURN_DUTY_CYCLE 5000
#define TURN_DUTY_CYCLE 3000
#define SLOW_DUTY_CYCLE 1500


#define REVERSE_DUTY_CYCLE 5625

/* 
4 PWM

Left Forward TPM1_CH0
Left Back Forward B2 Top Chip
Left Front Forward A2 Top Chip

Left Backward TPM1_CH1
Left Back Backward B1 Top Chip
Left Front Backward A1 Top Chip

Right Forward TPM2_CH0
Right Back Forward B1 Bottom Chip
Right Front Forward A2 Bottom Chip

Right Backward TPM2_CH1
Right Back Backward B2 Bottom Chip 
Right Front Backward A1 Bottom Chip
*/

void moveForward(void) {
	TPM1_C0V = DUTY_CYCLE;
	TPM2_C0V = DUTY_CYCLE;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void moveBackward(void) {
	TPM1_C0V = 0;
	TPM2_C0V = 0;
	TPM1_C1V = REVERSE_DUTY_CYCLE;
	TPM2_C1V = REVERSE_DUTY_CYCLE;
}

void moveLeft(void) {
	TPM1_C0V = 0;
	TPM2_C0V = TURN_DUTY_CYCLE;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void moveRight(void) {
	TPM1_C0V = TURN_DUTY_CYCLE;
	TPM2_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void moveLeftForward(void) {
	TPM1_C0V = SLOW_DUTY_CYCLE;
	TPM2_C0V = TURN_DUTY_CYCLE;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void moveRightForward(void) {
	TPM1_C0V = TURN_DUTY_CYCLE;
	TPM2_C0V = SLOW_DUTY_CYCLE;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void moveLeftBackward(void) {
	TPM1_C0V = 0;
	TPM2_C0V = 0;
	TPM1_C1V = SLOW_DUTY_CYCLE;
	TPM2_C1V = TURN_DUTY_CYCLE;
}

void moveRightBackward(void) {
	TPM1_C0V = 0;
	TPM2_C0V = 0;
	TPM1_C1V = TURN_DUTY_CYCLE;
	TPM2_C1V = SLOW_DUTY_CYCLE;
}

void moveFalse(void) {
	TPM1_C0V = 0;
	TPM2_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void moveCircle(void) {
	TPM1_C0V = DUTY_CYCLE * 20 / 33;
	TPM2_C0V = DUTY_CYCLE;
	TPM1_C1V = 0;
	TPM2_C1V = 0;
}

void initMotorPWM(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK); // enable clock gating for timer 1 and timer 2
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// Set modulo Value 4800 0000 / 128 = 37 5000 / 7500 = 50 Hz
	TPM1->MOD = 7500;
	TPM2->MOD = 7500;
	
	// Edge-Aligned PWM for TPM1
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on Tpm1 Channel 0 -> PTB0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK)| (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on Tpm1 Channel 1 -> PTB1
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK)| (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Edge-Aligned PWM for TPM2
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on Tpm2 Channel 0 -> PTB2
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK)| (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on Tpm2 Channel 1 -> PTB3
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK)| (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}