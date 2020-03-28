/*
###################################################
# MILESTONE: 5
# PROGRAM: 1
# PROJECT: Final Project
# GROUP: B02
# NAME 1: Bryce Dombrowski, V00873928
# NAME 2: Muqian Gui, V00870654
# DESC: Sort objects quickly and accurately
# DATA:
# REVISED:
#####################################################
*/

// INCLUDES
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "LinkedQueue.h"
#include "lcd.h"

// CALIBRATION VALUES
#define blackHigh 1023
#define blackLow 941
#define whiteHigh 940
#define whiteLow 700
#define steelHigh 699
#define steelLow 351
#define alumHigh 350
#define alumLow 0
#define TOGGLE_DIRECTION 0 // Change to 1 if conveyor is turning the wrong way

// GLOBALS
volatile unsigned int ADC_result = 0xFF;
volatile char flagADCBusy = 0;
volatile char flagProcessing = 0; // Flag to show that the current item is not yet identified
volatile char flagPause = 0; // 1 for pause or un-pause
volatile char flagRampDown = 0; // 1 to ramp down
volatile char flagConveyorStopped = 0; // Lets the program know that the conveyor is stopped and will need restarting.
int speedDCMotor = 0xFF; // What speed to choose? **** TODO
int stepperPos = 0; // Stepper motor position
int step = 0; // Ranges from 1-4, and is how the stepper function determines how to rotate
unsigned int itemsSorted = 0; // # of items sorted
// TODO **** find out exactly what needs to be tracked for displaying later

// FUNCTION DECLARATIONS
void waitToStart();
void initStepperPos();
void setupADC();
void initPWM();
void hwInterrupts(); // Set up hardware interrupts

void displaySorted(link **head, link **tail); // LCD display sorted materials data
int getMaterialType(int reflectivity); // Use reflectivity value to determine material category
void rotate(int count, char cw); // Move stepper motor
void setDCMotorSpeed(char speed); // Change DC motor speed
void updateDCMotorState(); // Pause/un-pause motor from turning
void mTimer(int count); // Delay function

// MAIN PROGRAM
int main(int argc, char *argv[])
{
	link *head; // Sets up head of queue
	link *tail;	// Sets up tail of queue
	setup(&head, &tail); // Set up queue
	link *deQueuedLink; // Creating one pointer handle to be reused multiple times
	
	cli();	// Disables all interrupts
	PORTB = 0b11110010; // Start motor CCW (inverted pins)
	TCCR1B |= _BV(CS10); // mTimer setup
	
	DDRA = 0xFF; // Stepper motor driver pins
	DDRB = 0xFF; // DC motor driver pins
	DDRC = 0xFF; // Output LEDs & LCD display
	DDRD = 0x00; // Button inputs
	DDRF = 0x00; // Sensor input pins (RL, HE, OR, EX) 
	
	InitLCD(0); // Initialize LCD
	initStepperPos(); // Initialize stepper position
	initPWM(); // Start DC motor
	setupADC(); // Set up reflectivity sensor
	sei(); // Enables interrupts
	
	// waitToStart(); // Waits for a button press to start. Maybe unnecessary
	while(1) // Event loop
	{
		// Check OR==high? (optical sensor #1) (Pin D1)
		if((PIND & 0x01)==0x01){
			if(!flagADCBusy && !flagConveyorStopped){ // Run a new ADC if it is available and conveyor is moving
				flagADCBusy = 1;
				flagProcessing = 1; // Lets us know that we currently are trying to identify an object
				ADC_result = 0xFF; // Reset value to highest number
				ADCSRA |= _BV(ADSC); // Triggers new conversion
			}
		}else{ // If object has left optical sensor, it's time to identify the material
			if (flagProcessing){ // Check that we were just processing a material
				// Take optimal value from the ADC values, identify material, and add to FIFO
				link *newLink;
				initLink(newLink);
				newLink->e.value = getMaterialType(ADC_result);
				enqueue(&head,&tail,&newLink);
				flagProcessing = 0;
			}
		}
		
		// Check EX==low? (optical sensor #2) (Pin F3)
		if (((PINF>>3)&1)==0){
			if(stepperPos == firstValue(&head).value){ // Check if stepper is in correct position yet
				dequeue(&head,&deQueuedLink);
				free(deQueuedLink);
				itemsSorted++;
				
				if(flagConveyorStopped){
					setDCMotorSpeed(speedDCMotor);
					flagConveyorStopped = 0;
				}
			}else{
				setDCMotorSpeed(0); // Stop conveyor while we wait for the stepper to rotate to the correct position.
				flagConveyorStopped = 1;
			}
		}
		
		// Check if queue is not empty
		if (!isEmpty(&head)){
			if(stepperPos == firstValue(&head).value){
				rotate(1,1);
				// TODO **** maybe change this later to rotate the optimal direction
				// Also need to figure out how to accelerate and decelerate. Probably requires doing fancier logic than "move one step".
			}
		}else if(flagRampDown) { // If queue is empty, check if we are in ramp down mode
			setDCMotorSpeed(0); // Stop conveyor motor
			displaySorted(&head, &tail); // Display info on LCD
			return 0; // Program end
		}
		
		// Check if we need to pause
		if (flagPause){ // "Pause" routine
			mTimer(100); // Large de-bounce
			flagPause = 0; // Reset pause flag
			displaySorted(&head, &tail); // Display info on LCD
			
			while(!flagPause); // Wait for un-pause button to be pressed
			mTimer(100); // Another large de-bounce
			flagPause = 0; // Reset pause flag
		}
	}
} // main()

// FUNCTIONS

void waitToStart(){ // Runs once at the beginning of program
	// Waits for any button to be pressed
	while(1){
		if (!((PIND>>1) & 1) || !(PIND & 1)){ // Check left and right buttons, active low
			return; // Exit function when any button is pressed
		}
	}
} // waitToStart()

void initStepperPos(){
	// **** TODO
	// Rotates the stepper motor to find the highest HE value, then rotates from there to the starting position. (Black)
	
	rotate(17, 1); // 30 deg cw
} // initStepperPos()

void setupADC(){ // TODO **** Change to use 10-bit conversion
	// ADC conversion is used to interpret analog value on pin F0 for reflectivity sensor
	EICRA |= _BV(ISC21) | _BV(ISC20); // rising edge interrupt
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(ADLAR) | _BV(REFS0);
	
	DIDR0 |= 0x01; // Disable all other pins on port F other than pin 0 for analog read
	
	// ADCSRA |= _BV(ADSC); // initialize the ADC, start one conversion at the beginning
} // setupADC()

void initPWM(){
	// Currently runs at 488 Hz
	
	// Step 1)
	TCCR0A |= _BV(WGM00); // Set WGM to = 7, fast PWM mode
	TCCR0A |= _BV(WGM01);
	
	// Step 2) Enable output compare interrupt enable for Timer 0
	//TIMSK0 |= _BV(OCIE0A);
	
	// Step 3) Compare match output mode: clear on compare match and set when reaches TOP
	TCCR0A |= _BV(COM0A1);
	
	// Step 4) Set prescaler to 8 (chosen to get 488 Hz frequency. Equation is on page 107)
	TCCR0B |= _BV(CS01);
	
	// Step 5) Set duty cycle
	OCR0A = 0x80; // 50% duty cycle
	
	// Step 6)
	DDRB = 0x80; // Set PWM pin to output (pin 7 on register B)
} // initPWM()

void hwInterrupts(){// Hardware interrupts
	
	// Sensors:
	
	/* NOTE: Attempting to do without EX or HE interrupts
	// Set up INT0 (pin D0), EX optical sensor #2
	EIMSK |= _BV(INT0);
	EICRA |= _BV(ISC01); // Falling edge interrupt
	
	// Set up INT2 (pin D2), HE sensor
	EIMSK |= _BV(INT2);
	EICRA |= _BV(ISC21); // Falling edge interrupt
	*/
	
	//Set up INT1 (pin D1), OR optical sensor #1
	EIMSK |= _BV(INT1);
	EICRA |= _BV(ISC11) | _BV(ISC10); // Rising edge interrupt
	
	
	// Buttons:
	// Set up INT3 (pin D3), Pause/un-pause, Left button
	EIMSK |= _BV(INT3);
	EICRA |= _BV(ISC31); // Falling edge interrupt
	
	// Set up INT4 (pin D4), Ramp down, Right button
	EIMSK |= _BV(INT4);
	EICRA |= _BV(ISC41); // Falling edge interrupt

} // hwInterrupts()

void displaySorted(link **head, link **tail){
	// Displays information on LCD about sorted items, and items in the queue
	
	LCDClear();
	LCDHome();
	
	if (!isEmpty(&head)){
		// Display stuff about queue items (if required)
		// **** TODO
	}
	// Display stuff about sorted items
	// TODO ****
	
	LCDWriteString("Testing");
	//LCDGotoXY(0,1);
	//LCDWriteInt(3,4);
} // displaySorted()

int getMaterialType(int reflectivity){
	// Returns stepper position corresponding to Black, White, Steel, or Aluminum material slots
	/*static int bwBorder = (blackLow+whiteHigh)/2;
	static int wsBorder = (whiteLow+steelHigh)/2;
	static int saBorder = (steelLow+alumHigh)/2;*/
	static int bwBorder = blackLow;
	static int wsBorder = whiteLow;
	static int saBorder = steelLow;
	
	if(reflectivity>=bwBorder){
		return 0; // Black
	}else if(reflectivity>=wsBorder){
		return 50; // White
	}else if(reflectivity>=saBorder){
		return 100; // Steel
	}else{
		return 150; // Aluminum
	}
} // getMaterialType()

void rotate(int count, char cw /* 1 rotates cw, 0 rotates ccw */){
	// Rotate the stepper motor by "count" number of steps
	int i;
	// Pin order, MSB first: 0 0 E1 L1 L2 E2 L3 L4
	static int step_array[] = {0b00110000,0b00000110,0b00101000,0b00000101};
	
	for(i=0; i<count; i++){
		if(step>3){ // Overflow condition
			step = 0;
			}else if (step<0){ // Underflow condition
			step = 3;
		} // end if
		
		PORTA = step_array[step]; // Set next step for stepper motor
		cw ? step++ : step--; // Advance to next step in specified direction
		mTimer(20); // 20ms pause
	} // end for
	return;
} // rotate()

void setDCMotorSpeed(char speed){
	// Max speed is 0xFF (100% Duty Cycle)
	OCR0A = speed;
} // setDCMotorSpeed()

void updateDCMotorState(){
	// B0-B4 are IB, IA, EB, EA. Note: EB and EA are always "on" (active low).
	
	if (flagPause){
		PORTB = 0x00; // Turn on the DC motor brake
		flagPause = 0;
	}else{
	// NOTE: This may need to actually be CW depending on the physical setup.
	//		 If so, set TOGGLE_DIRECTION to 1.
		if(!TOGGLE_DIRECTION){
			// Counter-clockwise (IA)
			PORTB = 0b11110001; // IA & EA & EB (active low inputs)
			}else{
			// Clockwise (IB)
			PORTB = 0b11110010; // IB & EA & EB (active low inputs)
		}
	}
} // updateDCMotorState()

void mTimer(int count){
	// Delay by "n" milliseconds
	int i = 0; // Index that represents how many milliseconds have passed.
	TCCR1B |= _BV(WGM12); // Set Waveform Generation mode to Clear Timer on Compare Math mode
	OCR1A = 0x03e8; // Set Output Compare Register for 1000 cycles / 1 ms
	TCNT1 = 0x0000; // Set initial value of Timer Counter
	//TIMSK1 = TIMSK1 | 0x02; // Enable the output compare interrupt
	TIFR1 |= _BV(OCF1A); // Clear timer interrupt flag and begin timer.
	
	while (i<count){ // Poll timer until it reaches the number of cycle specified by the parameter
		if ((TIFR1 & 0x02)==0x02){ // Check if interrupt flag has triggered
			TIFR1 |= _BV(OCF1A); // Clear interrupt flag
			i++; // Continue to next cycle
		} //end if
	} //end while
} // mTimer()


// INTERRUPTS

ISR(INT0_vect){ // EX sensor
	
}

ISR(INT1_vect){ // OR sensor
	
}

ISR(INT2_vect){ // HE sensor
	
}

ISR(INT3_vect){ // Left button pressed
	flagPause = 1;
}

ISR(INT4_vect){ // Right button pressed
	flagRampDown = 1;
}

ISR(ADC_vect){ // Analog to Digital conversion
	if(ADCH < ADC_result){ // Want lowest value for highest reflectivity
		ADC_result = ADCH; // store ADC converted value to ADC_result (0-255)
	}
	flagADCBusy = 0;
}

ISR(BADISR_vect){ // Bad ISR catch statement
	LCDClear();
	LCDWriteStringXY(1,1,"BAD ISR");
	while(1){
		// Stay here, there's an error.
	}
}
