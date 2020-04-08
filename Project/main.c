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

// Step positions of each material bucket
#define BLACK 0
#define ALUM 50
#define WHITE 100
#define STEEL 150

// GLOBALS
int step = 0; // Ranges from 1-4, and is how the stepper function determines how to rotate
int stepperPos = 0; // Stepper motor position
int stepperDestination = 0; // Where the stepper is trying to get to
volatile unsigned int ADC_result; // Current lowest reflectivity value
volatile unsigned int new_ADC_result; // latest reflectivity value
unsigned int itemsSorted = 0; // # of items sorted
unsigned int numBlack = 0; // # black items sorted
unsigned int numAlum = 0; // # aluminum items sorted
unsigned int numWhite = 0; // # white items sorted
unsigned int numSteel = 0;// # steel items sorted
link *head; // Sets up head of queue
link *tail;	// Sets up tail of queue
link *deQueuedLink; // Creating one pointer handle to be reused multiple times

// FLAGS
volatile char flagProcessing = 0; // Flag to show that the current item is not yet identified
volatile char flagPause = 0; // 1 for paused , 0 for un-paused
volatile char flagRampDown = 0; // 1 to ramp down
volatile char flagConveyorStopped = 0; // Lets the program know that the conveyor is stopped and will need restarting.

// CONSTANTS
int speedDCMotor = 0x50; // Speed of conveyor
int stepperPauseTime = 20; // Time between stepper steps (ms). This value changes for acceleration or deceleration.

// FUNCTION DECLARATIONS
void waitToStart();
void initStepperPos();
void setupADC();
void initPWM();
void hwInterrupts(); // Set up hardware interrupts

void displaySorted(link **head, link **tail); // LCD display sorted materials data
int getMaterialType(int reflectivity); // Use reflectivity value to determine material category
void rotate(int count, char cw); // Move stepper motor
void setDCMotorSpeed(char speed); // Change DC motor speed - UNUSED
void updateDCMotorState(char state); // Pause/un-pause motor from turning
void countSorted(int materialStep); // Keep track of what type of item was just sorted
void mTimer(int count); // Delay function

//test functions
char* getMaterialName(int materialStep);

// MAIN PROGRAM
int main(int argc, char *argv[])
{
	setup(&head, &tail); // Set up queue
	
	cli();	// Disables all interrupts
	TCCR1B |= _BV(CS10); // mTimer setup
	
	DDRA = 0xFF; // Stepper motor driver pins
	DDRB = 0xFF; // DC motor driver pins
	DDRC = 0xFF; // Output LEDs & LCD display
	DDRD = 0x00; // Button inputs and EX, OR, HE
	DDRF = 0x00; // Sensor input pin (RL on F1) 
	
	InitLCD(0); // Initialize LCD
	initStepperPos(); // Initialize stepper position
	initPWM(); // Set up DC motor PWM
	updateDCMotorState(1); // Turn on DC motor
	setupADC(); // Set up reflectivity sensor
	hwInterrupts(); // Set up hardware interrupts
	// waitToStart(); // Waits for a button press to start. Maybe unnecessary
	sei(); // Enables interrupts
	
	while(1) // Event loop
	{
		
		// If object has left optical sensor, it's time to identify the material
		// Check - were we just processing a material?
		// Check OR==low? (optical sensor #1) (Pin D1)
		if (flagProcessing && ((PIND & 0x02)==0x00)){
			// Take optimal value from the ADC values, identify material, and add to FIFO
			link *newLink;
			initLink(&newLink);
			newLink->e.value = getMaterialType(ADC_result);
			enqueue(&head,&tail,&newLink); // Add item to FIFO
			flagProcessing = 0; // We have now finished processing the item
			
			// Testing, verbose code. TODO remove later ****
			LCDClear();
			LCDHome();
			LCDWriteString("Material: ");
			LCDGotoXY(10,0);
			LCDWriteString(getMaterialName(getMaterialType(ADC_result)));
		}
				
		// Are there still more items on the conveyor?
		if (!isEmpty(&head)){ // Check if queue is not empty
			stepperDestination = firstValue(&head).value;
			if(stepperPos != stepperDestination){ // Check if stepper has the correct bucket position
				/*if (abs(stepperDestination-stepperPos) > 100){
					stepperPauseTime = 10; // TODO **** these values are arbitrary. Need to use SPS method
				} else if(abs(stepperDestination-stepperPos) > 50){
					stepperPauseTime = 15; // TODO **** these values are arbitrary. Need to use SPS method
				} else{
					stepperPauseTime = 20; // TODO **** these values are arbitrary. Need to use SPS method
				}*/
				rotate(1,(stepperDestination-stepperPos)>0); // rotate the stepper one step in proper direction
				// TODO: possibly set a destination variable and have it rotate in the background using Timer2
				// TODO **** maybe change this later to rotate the optimal direction
				// Also need to figure out how to accelerate and decelerate. Probably requires doing fancier logic than "move one step".
			}
		}else if(flagRampDown) { // If queue is empty, check if we are in ramp down mode
			LCDClear();
			LCDHome();
			LCDWriteString("Ramping down");
			
			mTimer(200); // Give time for last item to make it off of conveyor and into its bucket
			updateDCMotorState(0); // Stop conveyor motor
			cli(); // Stop all interrupts
			
			displaySorted(&head, &tail); // Display info on LCD
			return 0; // Program end
		}
		
		// Check if we need to pause
		if (flagPause){ // "Pause" routine
			updateDCMotorState(0); // Stop DC motor
			LCDClear();
			LCDHome();
			LCDWriteString("Paused");			
			displaySorted(&head, &tail); // Display info on LCD
			
			while(flagPause){
				// Wait for pause/unpause button to be toggled off
			} 
			LCDClear();
			LCDHome();
			LCDWriteString("Unpaused");
			updateDCMotorState(1);
		}
	}
} // main()

// FUNCTIONS

void waitToStart(){ // Runs once at the beginning of program
	// Waits for any button to be pressed
	while(1){
		if (!((PIND>>3) & 1) || !((PIND>>4) & 1)){ // Check left and right buttons, active low
			return; // Exit function when any button is pressed
		}
	}
} // waitToStart()

void initStepperPos(){
	LCDClear();
	LCDHome();
	LCDWriteString("Searching...");
	// Rotates the stepper motor to find the average position between the two "edges" of hall effect sensor data,
	//  then rotates from there to the starting position. (Black bucket)
	stepperPos = 0;
	while((PIND>>2)&1){ // loop while not at black bucket position
		rotate(1, 1); // 1 step cw
	}
	stepperPos = 0; // Now we are centered on the black bucket, tare the value
	
	/*stepperPos = 0;
	int minStep = 0;
	int maxStep = 0;
	int flagHE = (PIND>>2)&1;
	int lastVal = flagHE;
	
	while(stepperPos<200){ // Finding edges of the HE sensor range, then taking an average
		if(flagHE){
			if (lastVal != flagHE){
				if (flagHE){ // HE sensor just turned on
					minStep = stepperPos;
				} else { // HE sensor just turned off
					maxStep = stepperPos;
				}
			}
			lastVal = flagHE;
		}
		rotate(1, 1);// 1 step cw
		flagHE = (PIND>>2)&1; // HE sensor is on pin D2
	}
	if (maxStep < minStep) { // sensor values "wrap around" the bounds of 0-200 steps
		maxStep += 200; // Extend range since it wraps around. It makes taking the average easier
	}
	stepperDestination = (maxStep + minStep)/2;
	if (stepperDestination > 200){
		stepperDestination -= 200;
	}
	if(stepperDestination < stepperPos){
		rotate((stepperPos - stepperDestination), 0);
	}else{
		rotate((stepperDestination - stepperPos), 1);
	}
	stepperPos = 0; // Now we are centered on the black bucket, tare the value
	*/
	LCDClear();
	LCDHome();
	LCDWriteString("Program Start");
} // initStepperPos()

void setupADC(){ // TODO **** Change to use 10-bit conversion. Maybe already done by reading ADCL and ADCH???
	// ADC conversion is used to interpret analog value on pin F1 for reflectivity sensor
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADMUX |= _BV(REFS0) | _BV(MUX0); // enable ADC pin F1
	
	DIDR0 |= 0x02; // Disable all other pins on port F other than pin 1 for analog read
	
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
	OCR0A = speedDCMotor;
	
	// Step 6)
	// DDRB = 0x80; // Set PWM pin to output (pin 7 on register B)
} // initPWM()

void hwInterrupts(){// Hardware interrupts
	
	// Sensors:
	
	// Set up INT0 (pin D0), EX optical sensor #2
	EIMSK |= _BV(INT0);
	EICRA |= _BV(ISC01); // Falling edge interrupt
	
	//Set up INT1 (pin D1), OR optical sensor #1
	EIMSK |= _BV(INT1);
	EICRA |= _BV(ISC11) | _BV(ISC10); // Rising edge interrupt
	
	/* NOTE: No HE interrupts
	// Set up INT2 (pin D2), HE sensor
	EIMSK |= _BV(INT2);
	EICRA |= _BV(ISC21); // Falling edge interrupt
	*/
	
	// Buttons:
	// Set up INT3 (pin D3), Pause/un-pause, Left button
	EIMSK |= _BV(INT3);
	EICRA |= _BV(ISC31); // Falling edge interrupt
	
	// Set up INT4 (pin D4), Ramp down, Right button
	EIMSK |= _BV(INT4);
	EICRB |= _BV(ISC41); // Falling edge interrupt

} // hwInterrupts()

void displaySorted(link **head, link **tail){
	// Displays information on LCD about sorted items, and items in the queue
	
	LCDClear();
	LCDHome();
	LCDWriteString("SRT FD PD");
	
	LCDGotoXY(0,1);
	LCDWriteInt(itemsSorted,4); // Display number items sorted
	
	LCDGotoXY(4,1);
	LCDWriteInt(size(head, tail),3); // Display number items fully detected
	
	LCDGotoXY(7,1);
	LCDWriteInt(flagProcessing,3); // Display number items partially detected. There should only ever be 1 or 0.
	mTimer(2000);
	
	
	LCDClear();
	LCDHome();
	LCDWriteString("BL AL WH ST #OB");
	
	LCDGotoXY(0,1);
	LCDWriteInt(numBlack,3); // Display number of black in the bin

	LCDGotoXY(3,1);
	LCDWriteInt(numAlum,3); // Display number of aluminum in the bin

	LCDGotoXY(6,1);
	LCDWriteInt(numWhite,3); // Display number of white in the bin

	LCDGotoXY(9,1);
	LCDWriteInt(numSteel,3); // Display number of steel in the bin

	LCDGotoXY(13,1);
	LCDWriteInt(size(head,tail),3); // Indicate nothing on the belt right now
	mTimer(3000);
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
		return BLACK; // Black
	}else if(reflectivity>=wsBorder){
		return WHITE; // White
	}else if(reflectivity>=saBorder){
		return STEEL; // Steel
	}else{
		return ALUM; // Aluminum
	}
	
} // getMaterialType()

void rotate(int count, char cw /* 1 rotates cw, 0 rotates ccw */){
	// Rotate the stepper motor by "count" number of steps. 
	// Stepper has 200 steps --> 0 to 199
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
		if (cw){ // Advance to next step in specified direction
			step++;
			if(stepperPos>=200) // Cover overflow condition
				stepperPos = 0;
		} else{
			step--;
			if(stepperPos<0) // Cover underflow condition
				stepperPos = 199;
		}
		mTimer(stepperPauseTime); // **** TODO this may need adjusted as it could throw off timing of control loop
	} // end for
	return;
} // rotate()

void setDCMotorSpeed(char speed){
	// Max speed is 0xFF (100% Duty Cycle)
	OCR0A = speed;
} // setDCMotorSpeed()

void updateDCMotorState(char state){
	// B0-B4 are IB, IA, EB, EA. Note: EB and EA are always "on" (active low).
	if(state == 1){
		PORTB = 0x02; // Go forward - IB & EA & EB (active low inputs)
	} else if(state == 0){
		PORTB = 0x00; // Turn on the DC motor brake
	}
} // updateDCMotorState()

void countSorted(int materialStep){
	switch(materialStep){
		case BLACK:
		numBlack++;
		break;
		case ALUM:
		numAlum++;
		break;
		case WHITE:
		numWhite++;
		break;
		case STEEL:
		numSteel++;
		break;
	}
} // countSorted()

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

char* getMaterialName(int materialStep){ // temporary test function
	switch(materialStep){
		case BLACK:
		return "BLACK";
		break;
		case WHITE:
		return "WHITE";
		break;
		case ALUM:
		return "ALUM";
		break;
		case STEEL:
		return "STEEL";
		break;
	}
	return "";
}


// INTERRUPTS

ISR(INT0_vect){ // EX sensor
	stepperDestination = firstValue(&head).value;
	if(stepperPos != stepperDestination){ // Stepper still needs time to get to destination
		updateDCMotorState(0); // Stop conveyor while we wait for the stepper to rotate to the correct position.
		flagConveyorStopped = 1; // Let other parts of program know we are stopped
		rotate( (stepperDestination-stepperPos), (stepperDestination-stepperPos)>0 /*1 for cw, 0 for ccw*/); // rotate to proper position
		// TODO: Add "width" around each stepper destination for faster bucket dropping ****
	}
	
	// We are now guaranteed to be at the correct position
	dequeue(&head,&tail,&deQueuedLink); // pop item out of queue
	free(deQueuedLink); // Not sure if this will cause errors, may need to re-allocate deQueuedLink next loop?
	itemsSorted++;
	countSorted(stepperDestination); // count the type of item sorted
	
	if(flagConveyorStopped){
		updateDCMotorState(1); // restart conveyor motor
		flagConveyorStopped = 0; // reset flag
	}
}

ISR(INT1_vect){ // OR sensor
	flagProcessing = 1; // Lets us know that we currently are trying to identify an object
	ADC_result = 0x400; // Reset value to highest number (10-bit ADC, so max value of ADC is 0x3FF)
	ADCSRA |= _BV(ADSC); // Triggers new ADC conversion
}

/* NOTE: no interrupt for HE
ISR(INT2_vect){ // HE sensor
	flagHE = 1;
}*/

ISR(INT3_vect){ // Left button pressed
	flagPause = !flagPause;
}

ISR(INT4_vect){ // Right button pressed
	flagRampDown = 1;
}

ISR(ADC_vect){ // Analog to Digital conversion
	new_ADC_result = ADC;
	if(new_ADC_result < ADC_result){ // Want lowest value for highest reflectivity
		ADC_result = new_ADC_result; // store ADC converted value to ADC_result 
	}
	// Check OR==high? (optical sensor #1) (Pin D1)
	if((PIND & 0x02)==0x02){
		ADCSRA |= _BV(ADSC); // Triggers new ADC conversion
	}
}

ISR(BADISR_vect){ // Bad ISR catch statement
	LCDClear();
	LCDWriteStringXY(1,1,"BAD ISR");
	while(1){
		// Stay here, there's an error.
	}
}
