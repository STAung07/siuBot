/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header
#include <stdbool.h>
#include "uart.h"
#include "motorcontrol.h"
#include "led.h"
#include "audio.h"
#include "ultrasonic.h"


/*----------------------------------------------------------------------------
 * LED Constants
 *---------------------------------------------------------------------------*/
#define GREEN_LED_1 1
#define GREEN_LED_2 2
#define GREEN_LED_3 4
#define GREEN_LED_4 12
#define GREEN_LED_5 4
#define GREEN_LED_6 5
#define GREEN_LED_7 8
#define GREEN_LED_8 9
#define RED_LED_1 7
#define SW_BIT 6
#define MASK(x) (1 << (x))

/*----------------------------------------------------------------------------
 * Audio Constants
 *---------------------------------------------------------------------------*/
#define B0  31
#define C1  33
#define CS1 35
#define D1  37
#define DS1 39
#define E1  41
#define F1  44
#define FS1 46
#define G1  49
#define GS1 52
#define A1  55
#define AS1 58
#define B1  62
#define C2  65
#define CS2 69
#define D2  73
#define DS2 78
#define E2  82
#define F2  87
#define FS2 93
#define G2  98
#define GS2 104
#define A2  110
#define AS2 117
#define B2  123
#define C3  131
#define CS3 139
#define D3  147
#define DS3 156
#define E3  165
#define F3  175
#define FS3 185
#define G3  196
#define GS3 208
#define A3  220
#define AS3 233
#define B3  247
#define C4  262
#define CS4 277
#define D4  294
#define DS4 311
#define E4  330
#define F4  349
#define FS4 370
#define G4  392
#define GS4 415
#define A4  440
#define AS4 466
#define B4  494
#define C5  523
#define CS5 554
#define D5  587
#define DS5 622
#define E5  659
#define F5  698
#define FS5 740
#define G5  784
#define GS5 831
#define A5  880
#define AS5 932
#define B5  988
#define C6  1047
#define CS6 1109
#define D6  1175
#define DS6 1245
#define E6  1319
#define F6  1397
#define FS6 1480
#define G6  1568
#define GS6 1661
#define A6  1760
#define AS6 1865
#define B6  1976
#define C7  2093
#define CS7 2217
#define D7  2349
#define DS7 2489
#define E7  2637
#define F7  2794
#define FS7 2960
#define G7  3136
#define GS7 3322
#define A7  3520
#define AS7 3729
#define B7  3951
#define C8  4186
#define CS8 4435
#define D8  4699
#define DS8 4978
#define REST 375001

#define FREQ_TO_MOD(x) (375000 / x)
#define CNV 2 // Larger the number, softer the volume
#define TEMPO_TAKEONME 171
#define TEMPO_MYHEARTWILLGOON 99
#define TEMPO_CONSTANT 60000 // delay = TEMPO_CONSTANT / tempo 

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

/*----------------------------------------------------------------------------
 * Move Cmds
 *---------------------------------------------------------------------------*/
#define COMMAND_MOVE_FORWARD 0x60
#define COMMAND_MOVE_BACKWARD 0x61
#define COMMAND_SHARP_LEFT 0x62
#define COMMAND_SHARP_RIGHT 0x63

#define COMMAND_RIGHT_FORWARD 0x64
#define COMMAND_LEFT_FORWARD 0x65
#define COMMAND_RIGHT_BACKWARD 0x66
#define COMMAND_LEFT_BACKWARD 0x67

#define COMMAND_MOVE_FALSE 0x41
#define COMMAND_CHALLENGE_START 0x70
#define COMMAND_CHALLENGE_DONE 0x72
#define COMMAND_SELF_DRIVE_MODE 0x80

#define COMMAND_CIRCLE_CLOCKWISE 0x68

/*----------------------------------------------------------------------------
 * LED Cmds
 *---------------------------------------------------------------------------*/
#define COMMAND_LED_MOVE_MODE 0x01
#define COMMAND_LED_STATIONARY_MODE 0x02

#define MSG_COUNT 1
#define THREAD_FLAG 0x00000001
#define EVENT_FLAG 0x01
#define EVENT_FLAG_1 0x001
#define EVENT_FLAG_2 0x00000010

/*----------------------------------------------------------------------------
 * Audio Cmds
 *---------------------------------------------------------------------------*/
#define COMMAND_AUDIO 0x50

/*----------------------------------------------------------------------------
 * Thread IDs
 *---------------------------------------------------------------------------*/
osThreadId_t brain_Id, motorControl_Id, redLED_Id, greenLED_Id, audioChallege_Id, audioDone_Id, selfDrive_Id;

/*----------------------------------------------------------------------------
 * Event Flags IDs
 *---------------------------------------------------------------------------*/
osEventFlagsId_t moveLED_flag, stopLED_flag, audio_start_flag, audio_end_flag, self_drive_flag;

/*----------------------------------------------------------------------------
 * Message Queue IDs
 *---------------------------------------------------------------------------*/
osMessageQueueId_t motorMsg, redLEDMsg, greenLEDMsg, audioMsg;

/*----------------------------------------------------------------------------
 * Semaphore
 *---------------------------------------------------------------------------*/
osSemaphoreId_t audioSem;

/*----------------------------------------------------------------------------
 * Volatile constants
 *---------------------------------------------------------------------------*/
volatile uint8_t rx_data = 0x01;
volatile uint32_t echoStart;
volatile uint32_t echoEnd;
volatile uint32_t echoPeriod;
volatile uint32_t echoFreq;
volatile bool isEchoHigh = false;
volatile uint32_t distance;
int greenLED[] = {1, 2, 4, 12, 4, 5, 8, 9};

/*----------------------------------------------------------------------------
 * VConstant constants
 *---------------------------------------------------------------------------*/
#define LEFT_DELAY 500 // was 350
#define FIRST_RIGHT_DELAY 600
#define SECOND_RIGHT_DELAY 800 //change to 550
#define THIRD_RIGHT_DELAY 900 //change to 500
#define LEFT_FORWARD_DELAY 300
#define RIGHT_FORWARD_DELAY 500
#define RIGHT2_FORWARD_DELAY 250
#define RIGHT3_FORWARD_DELAY 300


#define ECHO 13
#define TRIG 9
#define THRESHOLD 50
#define THRESHOLD2 20

typedef struct
{
	uint8_t cmd;
	//uint8_t data;
} myDataPkt;

/*----------------------------------------------------------------------------
 * Interrupts
 *---------------------------------------------------------------------------*/
 
void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;
	}
}

void PORTA_IRQHandler(void) {
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	
	if (!isEchoHigh) {
		isEchoHigh = true;
		echoStart = osKernelGetSysTimerCount();
	} else {
		echoFreq = osKernelGetSysTimerFreq() / 1000000u;
		echoEnd = osKernelGetSysTimerCount();
		echoPeriod = (echoEnd - echoStart) / echoFreq;
		distance = echoPeriod / 29 / 2;
		isEchoHigh = false;
	}
	
	PORTA->ISFR = 0xffffffff;
}

/*----------------------------------------------------------------------------
 * Initialise Message Queues and Event Flags
 *---------------------------------------------------------------------------*/

/* Thread control (message queue, event flag) */
void initMessageQueues () {
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	greenLEDMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	redLEDMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
}

void initEventFlags () {
	moveLED_flag = osEventFlagsNew(NULL);
	stopLED_flag = osEventFlagsNew(NULL);
	audio_start_flag = osEventFlagsNew(NULL);
	audio_end_flag = osEventFlagsNew(NULL);
	self_drive_flag = osEventFlagsNew(NULL);
}

void initSemaphores () {
	audioSem = osSemaphoreNew(1,0,NULL);
}

/*----------------------------------------------------------------------------
 * Decode data from serial port and perform necessary action; tBrain
 *---------------------------------------------------------------------------*/
void tBrain(void *argument) {
	
	myDataPkt motorData;
	myDataPkt ledData;
	myDataPkt audioData;
	uint8_t ledPrio;
	osStatus_t status;
	
	audioData.cmd = COMMAND_AUDIO;
	
	for (;;) {
		switch(rx_data) {
			case COMMAND_MOVE_FORWARD:
				// send to motorcontrol
				motorData.cmd = COMMAND_MOVE_FORWARD;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_MOVE_BACKWARD:
				// send to motorcontrol
				motorData.cmd = COMMAND_MOVE_BACKWARD;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_SHARP_LEFT:
				// send to motorcontrol
				motorData.cmd = COMMAND_SHARP_LEFT;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_SHARP_RIGHT:
				// send to motorcontrol
				motorData.cmd = COMMAND_SHARP_RIGHT;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_RIGHT_FORWARD:
				// send to motorcontrol
				motorData.cmd = COMMAND_RIGHT_FORWARD;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_LEFT_FORWARD:
				// send to motorcontrol
				motorData.cmd = COMMAND_LEFT_FORWARD;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_RIGHT_BACKWARD:
				// send to motorcontrol
				motorData.cmd = COMMAND_RIGHT_BACKWARD;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_LEFT_BACKWARD:
				// send to motorcontrol
				motorData.cmd = COMMAND_LEFT_BACKWARD;
				// send to redled and greenled thread (move mode)
				osEventFlagsSet(moveLED_flag, EVENT_FLAG);
				osEventFlagsClear(stopLED_flag, EVENT_FLAG);
				break;
			case COMMAND_MOVE_FALSE:
				// send to 
				motorData.cmd = COMMAND_MOVE_FALSE;
				// send to redled and greenled thread (dont move)
				osEventFlagsSet(stopLED_flag, EVENT_FLAG);
				osEventFlagsClear(moveLED_flag, EVENT_FLAG);
				break;
			case COMMAND_CHALLENGE_START:
				osEventFlagsSet(audio_start_flag, EVENT_FLAG_1);
				osEventFlagsClear(audio_end_flag, EVENT_FLAG_1);
			  break;
			case COMMAND_CHALLENGE_DONE: 
				osEventFlagsSet(audio_end_flag, EVENT_FLAG_1);
				osEventFlagsClear(audio_start_flag, EVENT_FLAG_1);
				break;
			case COMMAND_SELF_DRIVE_MODE:
				osEventFlagsSet(self_drive_flag, EVENT_FLAG_1);
				osEventFlagsSet(audio_start_flag, EVENT_FLAG_1);
				osEventFlagsClear(audio_end_flag, EVENT_FLAG_1);
				break;
		}
		osMessageQueuePut(motorMsg, &motorData, NULL, 0);
	}
}

/*----------------------------------------------------------------------------
* Movement Control; tMotorControl
 *---------------------------------------------------------------------------*/

void tMotorControl(void *argument) {
	myDataPkt myMotorData;
  for (;;) {
		osMessageQueueGet(motorMsg, &myMotorData, NULL, osWaitForever);
		switch(myMotorData.cmd) {
			case COMMAND_MOVE_FORWARD:
				moveForward();
				break;
			case COMMAND_MOVE_BACKWARD:
				moveBackward();
				break;
			case COMMAND_SHARP_LEFT:
				moveLeft();
				break;
			case COMMAND_SHARP_RIGHT:
				moveRight();
				break;
			case COMMAND_LEFT_FORWARD:
				moveLeftForward();
				break;
			case COMMAND_RIGHT_FORWARD:
				moveRightForward();
				break;
			case COMMAND_LEFT_BACKWARD:
				moveLeftBackward();
				break;
			case COMMAND_RIGHT_BACKWARD:
				moveRightBackward();
				break;
			case COMMAND_MOVE_FALSE:
				moveFalse();
				break;
			case COMMAND_CIRCLE_CLOCKWISE:
				moveCircle();
				break;
		}
	}
}

/*----------------------------------------------------------------------------
 * LED Threads; tRedLED, tGreenLED
 *---------------------------------------------------------------------------*/

void tRedLEDMove(void *argument) {
	for (;;) {
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		osEventFlagsClear(stopLED_flag, EVENT_FLAG);
		PTC->PDOR |= MASK(RED_LED_1);
		osDelay(500);
		PTC->PDOR &= ~MASK(RED_LED_1);
		osDelay(500);
	}
}

void tRedLEDStop(void *argument) {
	for (;;) {
		osEventFlagsWait(stopLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		osEventFlagsClear(moveLED_flag, EVENT_FLAG);
		PTC->PDOR |= MASK(RED_LED_1);
		osDelay(250);
		PTC->PDOR &= ~MASK(RED_LED_1);
		osDelay(250);
	}
}

void tGreenLEDMove(void *argument) {
	for (;;) {
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		osEventFlagsClear(stopLED_flag, EVENT_FLAG);
		
		off_green_strip();
		PTA->PDOR |= MASK(GREEN_LED_1);
		osDelay(250);
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTA->PDOR |= MASK(GREEN_LED_2);
		osDelay(250);	
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTD->PDOR |= MASK(GREEN_LED_3);
		osDelay(250);
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTA->PDOR |= MASK(GREEN_LED_4);
		osDelay(250);			
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTA->PDOR |= MASK(GREEN_LED_5);
		osDelay(250);
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTA->PDOR |= MASK(GREEN_LED_6);
		osDelay(250);
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTC->PDOR |= MASK(GREEN_LED_7);
		osDelay(250);
		osEventFlagsWait(moveLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		off_green_strip();
		PTC->PDOR |= MASK(GREEN_LED_8);
		osDelay(250);
	}
}

void tGreenLEDStop(void *argument) {
	for (;;) {
		osEventFlagsWait(stopLED_flag, EVENT_FLAG, osFlagsNoClear, osWaitForever);
		osEventFlagsClear(moveLED_flag, EVENT_FLAG);
		on_green_strip();
		osDelay(250);
	}
}

/*----------------------------------------------------------------------------
 * Audio Threads; tAudioChallenge, tAudioChallengeEnd
 *---------------------------------------------------------------------------*/
void tAudioChallenge(void *argument) {
	int melody[] = {
      F4, G4, G4, A4,A4, G4, F4, G4, C5, C5, AS4, A4, F4, D4, C4, F4, G4,
      G4, A4, A4, AS4, A4, G4, F4, G4, C5, C5, A4, C5, D5, C5, G4, A4, G4, G4, F4, F4, F4, F4,
      E4, F4, REST, F4, E4, F4, REST, REST, G4, A4, G4, REST, F4, F4, F4, F4, E4, F4, REST, F4, C4,
      C4, C4, REST, F4, F4, F4, F4, E4, F4, REST, F4, E4, F4, REST, REST, G4,
      A4, G4, REST, F4, F4, F4, F4, E4, F4, REST, F4, C4, C4, REST, F4, REST,
      G4, REST, C4, C5, AS4, A4, G4, G4, REST, A4, AS4, A4, G4, REST, F4, E4, F4, REST, REST, E4, D4, D4, E4, D4,
      C4, REST, F4, G4, REST, C4, C5, AS4, A4, G4, G4, REST, A4, AS4, A4, G4, F4,
      E4, F4, REST, E4, E4, F4, G4, A4, AS4, A4, G4, G4, F4, F4, AS4, A4, G4, F4, G4, C5, C5, A4, C5,
      D5, C5, G4, A4, G4, G4  
    };
		
	int notes = sizeof(melody) / sizeof(melody[0]);
	
	int melodyBeat[] = {
		2, 2, 2, -4, 4, 2, 2, 2, -4, 4, 2, 2, 4, -8, -8, 2, 2,
		2, -4, 4, 1, 1, 1, 1, 2, -4, 4, 2, 2, 8, 8, 1, 1, -4, 8, -4, 2, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 2, 2, 8, 4, 4, -4, 2, 4, 4, 4, 4, 4, 2, 2,
		16, 8, 8, -4, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2,
		8, 4, 4, -4, 2, 4, 4, 4, 4, 4, 2, 2, 16, 16, 8, 8,
		8, 4, 4, 8, 4, 2, 2, 4, 4, -4, 2, 8, 4, 2, 2, 4, 4, 4, 2, 2, 8, -4, 1, 1,	
		-8, 4, 16, 8, 4, 4, 8, 4, 2, 2, 4, 4, 4, 4, 8, 4, 4,
		4, 4, 4, 4, 4, 8, 4, 4, 1, 1, 2, 4, 4, -8, 1, 1, 1, 1, 2, -4, 4, 2, 2,
		8, 8, 1, 1, -4, 8			
	};
		
	for(;;) {
		for (int i = 0; i < notes; i++) {
			osEventFlagsWait(audio_start_flag, EVENT_FLAG_1, osFlagsNoClear, osWaitForever);
			osEventFlagsClear(audio_end_flag, EVENT_FLAG_1);
			TPM0->MOD = FREQ_TO_MOD(melody[i]);
			TPM0_C1V = FREQ_TO_MOD(melody[i] / CNV);
						
			if (melodyBeat[i] > 0) {
				osDelay(melodyBeat[i] * TEMPO_CONSTANT / TEMPO_MYHEARTWILLGOON / 4);		
			} else if (melodyBeat[i] < 0) {
				osDelay(- 3 * melodyBeat[i] * TEMPO_CONSTANT / TEMPO_MYHEARTWILLGOON / 8);		
			}			
		}
	}
}

void tAudioChallengeEnd(void *argument) {
	int melody[] = {
      FS5, FS5, D5, B4, REST, B4, REST, E5, REST, E5,
      REST, E5, GS5, GS5, A5, B5, A5, A5, A5, E5,
      REST, D5, REST, FS5, REST, FS5, REST, FS5, E5, E5,
      FS5, E5
    };
		
	int notes = sizeof(melody) / sizeof(melody[0]);
		
	for (;;) {
		for (int i = 0; i < notes; i++) {
			osEventFlagsWait(audio_end_flag, EVENT_FLAG_1, osFlagsNoClear, osWaitForever);
			osEventFlagsClear(audio_start_flag, EVENT_FLAG_1);
			TPM0->MOD = FREQ_TO_MOD(melody[i]);
			TPM0_C1V = FREQ_TO_MOD(melody[i] / CNV);
			osDelay(TEMPO_CONSTANT / TEMPO_TAKEONME / 2);			
		}
	}
}

/*----------------------------------------------------------------------------
 * Self Drive Thread; tSelfDrive
 *---------------------------------------------------------------------------*/

void selfDriveLeft() {
	rx_data = COMMAND_SHARP_LEFT;
	osDelay(LEFT_DELAY);
	rx_data = COMMAND_MOVE_FALSE;
	osDelay(200);
	rx_data = COMMAND_MOVE_FORWARD;
	osDelay(LEFT_FORWARD_DELAY);
	rx_data = COMMAND_MOVE_FALSE;
	osDelay(200);
}

void selfDriveRight(int forward_delay, int delay) {
	rx_data = COMMAND_SHARP_RIGHT;
	osDelay(delay);
	rx_data = COMMAND_MOVE_FALSE;
	osDelay(200);
	rx_data = COMMAND_MOVE_FORWARD;
	osDelay(forward_delay);
	rx_data = COMMAND_MOVE_FALSE;
	osDelay(200);
}

void selfDriveCircle() {
	rx_data = COMMAND_SHARP_LEFT;
	osDelay(LEFT_DELAY);
	rx_data = COMMAND_CIRCLE_CLOCKWISE;
	osDelay(2000);
	rx_data = COMMAND_SHARP_LEFT;
	osDelay(LEFT_DELAY);
}
void tSelfDrive(void *argument) {
	osEventFlagsWait(self_drive_flag, EVENT_FLAG_1, osFlagsNoClear, osWaitForever);
	distance = 100;
	for (;;) {
		//distance = 100;
		// Ultrasonic send pulse
		PTB->PDOR |= MASK(TRIG);
		osDelay(1);
		PTB->PDOR &= ~MASK(TRIG);
		// if first obstacle
		if (distance < THRESHOLD && distance > 0) {
			// STOP
			rx_data = COMMAND_MOVE_BACKWARD;
			osDelay(50);
			rx_data = COMMAND_MOVE_FALSE;
			osDelay(1000);
			selfDriveLeft();
			selfDriveRight(RIGHT2_FORWARD_DELAY, FIRST_RIGHT_DELAY);
			selfDriveRight(RIGHT2_FORWARD_DELAY, SECOND_RIGHT_DELAY); //change forward delay to 300
			selfDriveRight(RIGHT_FORWARD_DELAY, THIRD_RIGHT_DELAY);
			//selfDriveRight(1);
			/*
			rx_data = COMMAND_SHARP_LEFT;
			osDelay(350);
			rx_data = COMMAND_MOVE_FALSE;
			osDelay(200);
			*/
			for (;;) {
				// Ultrasonic send pulse
				PTB->PDOR |= MASK(TRIG);
				osDelay(1);
				PTB->PDOR &= ~MASK(TRIG);
				if (distance < THRESHOLD) {
					rx_data = COMMAND_MOVE_FALSE;
					// off audio
					osEventFlagsSet(audio_end_flag, EVENT_FLAG_1);
					osEventFlagsClear(audio_start_flag, EVENT_FLAG_1);
					osDelay(10);
				}
				rx_data = COMMAND_MOVE_FORWARD;
				osDelay(100);
			}
			//osEventFlagsClear(self_drive_flag, EVENT_FLAG_1);
		} 
		rx_data = COMMAND_MOVE_FORWARD;
		osDelay(30);
	}
}

/* DELAY function */
static void delay(volatile uint32_t nof) {
    while (nof != 0) {
        __asm("NOP");
        nof--;
    }
}

int main (void) {
  // System Initialization
  SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	initMotorPWM();
	initAudioPWM();
	initLED();
	initUS();
	
  osKernelInitialize(); 	               // Initialize CMSIS-RTOS
	initMessageQueues();
	initEventFlags();
	initSemaphores();
	brain_Id = osThreadNew(tBrain, NULL, NULL);
  motorControl_Id = osThreadNew(tMotorControl, NULL, NULL);
	
	osThreadNew(tRedLEDMove, NULL, NULL);
	osThreadNew(tRedLEDStop, NULL, NULL);
	osThreadNew(tGreenLEDMove, NULL, NULL);
	osThreadNew(tGreenLEDStop, NULL, NULL);
	
	audioChallege_Id = osThreadNew(tAudioChallenge, NULL, NULL);
	audioDone_Id = osThreadNew(tAudioChallengeEnd, NULL, NULL);
  selfDrive_Id = osThreadNew(tSelfDrive, NULL, NULL);
	osThreadSetPriority(selfDrive_Id, 32);
	osKernelStart();                      // Start thread execution
  for (;;) {}
}
