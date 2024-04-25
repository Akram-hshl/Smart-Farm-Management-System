

#include <avr/io.h>
#define F_CPU 1000000UL
#include "util/delay.h"
#include <avr/interrupt.h>
#include "utility.h"
#include <stdio.h>

// For water Supply  
#define ECHO_PIN PORTD0 //define echo pin for ultrasonic distance sensor
#define TRIG_PIN PORTE3	// define trig pin for ultrasonic distance sensor
#define WATER_PUMP_MOTOR_PIN PORTD6	 // Define PD6 for water pump motor


// For security system

#define BIT_BUZZER PORTB7
#define Security_Led (1<<PE6)
#define pir_pin (1<<PE0)
#define FLAME_SENSOR_PIN PE7
#define BUZZER_PIN_1 PB0


// For Food supply 

#define MIXING_MOTOR_PIN (1 << PD5) // Define the pin where the motor is connected (PD5)
#define MIXING_MOTOR_BUTTON_PIN (1 << PC0) // Define the pin where the button is connected (PC4)

#define DELIVERY_MOTOR_PIN (1 << PD4) // Define the pin where the motor is connected (PD5)
#define DELIVERY_MOTOR_BUTTON_PIN (1 << PC3) // Define the pin where the button is connected (PC4)

#define CLOCKWISE_STEPER_BUTTON_PIN (1 << PC1) // Define the pin where the button is connected (PC4)
#define ANTI_CLOCKWISE_STEPER_BUTTON_PIN (1 << PC2) // Define the pin where the button is connected (PC4)


//call the initialization function for security system

void buzzer_init(void);		
void buzzer_tone(void);

void initIO();



volatile uint8_t  INTERRUPT_Falg = 0x00;		// this variable for controlling the state of the interrupt 
volatile uint32_t Time_Counter = 0x00;		// this variable for store the timer1 value for calculate the accurate distance

ISR(INT0_vect)		// Whenever interrupt occurs this time the Interrupt service Routine will be execute 	
{
	INTERRUPT_Falg++;		//This variable value will be increase 
	// check if the rising edge or falling edge
	if(INTERRUPT_Falg == 1)
	{
		// Start Timer by setting the prescaler to 1 (CS10 = 1) so the timer select the system clock    
		SET_BIT(TCCR1B,CS10);
		// Reset Counter
		Time_Counter = 0;
		// // For sense to the falling edge interrupt sense bits ISC00 and ISC01 must be set 0
		CLEAR_BIT(EICRA, ISC00);
		SET_BIT(EICRA, ISC01);
	}
	// Falling edge
	else
	{
		//Stop counter by setting CS10 to ZERO
		CLEAR_BIT(TCCR1B, CS10);
		// Get the counted value, Reset counter and flag to Zero.
		Time_Counter = TCNT1;
		INTERRUPT_Falg = 0x00;
		TCNT1 = 0;
		// Set Interrupt for the rising edge.
		SET_BIT(EICRA, ISC00);
		SET_BIT(EICRA, ISC01);
	}
}



int main(void)
{
	//For security system

	
	DDRE &=~(1<<pir_pin);
	PORTE |=pir_pin;
	
	DDRB |= (1 <<BIT_BUZZER);
	
	DDRE |=Security_Led; 
	PORTE &=~Security_Led;
	buzzer_init();
	initIO();
	
	// For water supply 
	
	DDRA = 0xFF;	// set port A as output for 7 segment display
	
	DDRD |=(1<<WATER_PUMP_MOTOR_PIN); // set port PD6 for output for water pump motor
	PORTD &=~(1<<WATER_PUMP_MOTOR_PIN);
	
		
	
	// Configure Timer1 by the timer counter control register 1
	// use the clock frequency of the uC 1MHz, but for now the timer must be stopped so for that clock select bit( CS10, CS11, CS21 must be set 0 )
	CLEAR_BIT(TCCR1B, CS10);
	CLEAR_BIT(TCCR1B, CS11);
	CLEAR_BIT(TCCR1B, CS21);
	// Configure the Ultrasonic Pins
	// configure PD0 as input, enable Pull-up resistor, and PE3 as output.
	CLEAR_BIT(DDRD, ECHO_PIN);
	SET_BIT(PORTD,ECHO_PIN);
	SET_BIT(DDRE,TRIG_PIN);
	// Configure the external interrupt mask register on the input Echo Pin PDO (INT0)-> Enable INT0 so whenever any change will be occures in echo pin this time ISR will be exicute 
	SET_BIT(EIMSK, INT0);		// External interrupt request0 in enable  by set External interrupt mask register
	// For sense to the Rising edge interrupt sense bits ISC00 and ISC01 must be set 1
	SET_BIT(EICRA, ISC00);
	SET_BIT(EICRA, ISC01);
	// enable global interrupt
	sei();
	//volatile int u32Range = 0;
	int Distance = 0;

	
	
	// for food supply 
	
	DDRD |= MIXING_MOTOR_PIN; // Timer 1, Channel A (Pin PB5)
	PORTD &=~(MIXING_MOTOR_PIN);
	
	DDRC &=~(MIXING_MOTOR_BUTTON_PIN);
	PORTC |= MIXING_MOTOR_BUTTON_PIN; // Enable internal pull-up for the button pin
	
	DDRD |= DELIVERY_MOTOR_PIN; // Timer 1, Channel A (Pin PB5)
	PORTD &=~(DELIVERY_MOTOR_PIN);
	
	DDRC &=~(DELIVERY_MOTOR_BUTTON_PIN);
	PORTC |= DELIVERY_MOTOR_BUTTON_PIN; // Enable internal pull-up for the button pin

	unsigned int y=3;
	DDRC &=~CLOCKWISE_STEPER_BUTTON_PIN;   // For clockwise movement of Stepper motor
	PORTC |=CLOCKWISE_STEPER_BUTTON_PIN;
	
	DDRC &=~(ANTI_CLOCKWISE_STEPER_BUTTON_PIN); // FOR ANTICLOCKWISE MOVEMETN 
	PORTC |=ANTI_CLOCKWISE_STEPER_BUTTON_PIN;
	
	
	DDRF = 0xFF; // set output for stepper motor
	PORTF = 0x00;
	
	while (1)
	{
		// Trigger Pulse for initiating ultrasonic sound
		
		SET_BIT(PORTE, TRIG_PIN);
		_delay_ms(15);
		CLEAR_BIT(PORTE,TRIG_PIN);
		
	
	if (Time_Counter > 0) {
		Distance = (int)(0.017 * Time_Counter);

		switch (Distance) {
			case 0 ... 4:
			PORTA = 0x06;
			_delay_ms(10);
			PORTD &= ~(1 << WATER_PUMP_MOTOR_PIN);
			break;

			case 5 ... 9:
			PORTA = 0x5B;
			_delay_ms(10);
			break;

			case 10 ... 14:
			PORTA = 0x4F;
			_delay_ms(10);
			break;

			case 15 ... 19:
			PORTA = 0x66;
			_delay_ms(10);
			break;

			case 20 ... 24:
			PORTA = 0x6D;
			_delay_ms(10);
			break;

			case 25 ... 29:
			PORTA = 0x7D;
			_delay_ms(10);
			break;

			case 30 ... 34:
			PORTA = 0x07;
			_delay_ms(10);
			PORTD |= (1 << WATER_PUMP_MOTOR_PIN);
			break;
/*
			case 35 ... 39:
			PORTA = 0x7F;
			_delay_ms(10);
			break;

			case 40 ... 44:
			PORTA = 0x6F;
			_delay_ms(10);
			PORTD |= (1 << WATER_PUMP_MOTOR_PIN);
			break;		*/

			default:
			PORTA = 0x3F;
			_delay_ms(10);
			PORTD &= ~(1 << WATER_PUMP_MOTOR_PIN);
			break;
		}
	}


		//stepper motor for food supply
		
		if (!(PINC & CLOCKWISE_STEPER_BUTTON_PIN))
		{
			PORTF = 0x66;
			_delay_ms(y);
			
			PORTF = 0xCC;
			_delay_ms(y);
			
			PORTF = 0x99;
			_delay_ms(y);
			
			PORTF=0x33;
			_delay_ms(y);
			
			
			}else{
			
			PORTF = 0xFF;
		}
		
		if (!(PINC & ANTI_CLOCKWISE_STEPER_BUTTON_PIN))
		{
			PORTF = 0x66;
			_delay_ms(y);
			
			PORTF = 0x33;
			_delay_ms(y);
			
			PORTF = 0x99;
			_delay_ms(y);
			
			PORTF=0xCC;
			_delay_ms(y);
			}else{
			
			PORTF = 0xFF;
		}
		
		//For mixing motor
		
		if (!(PINC & MIXING_MOTOR_BUTTON_PIN)) { // Check if the button is pressed (active low)
			PORTD |= MIXING_MOTOR_PIN; // Turn on the motor
			
			
			}else{
			PORTD &= ~MIXING_MOTOR_PIN; // Turn off the motor after 1 second
		}
		//For mixing motor
		
		if (!(PINC & DELIVERY_MOTOR_BUTTON_PIN)) { // Check if the button is pressed (active low)
			PORTD |= DELIVERY_MOTOR_PIN; // Turn on the motor
			_delay_ms(2000);
			PORTD &= ~DELIVERY_MOTOR_PIN;
			_delay_ms(5000);
			
			}else{
			PORTD &= ~DELIVERY_MOTOR_PIN; // Turn off the motor after 1 second
		}
		
		// for security system
		
		if((PINE & pir_pin)==1)
		{
			
			PORTE |=Security_Led;
			_delay_ms(300);
			
			PORTE &=~Security_Led;
		
			
			buzzer_tone();
			
			
			
		}else
		{
		PORTE &=~Security_Led;
			PORTB &=~ (1 <<7);
		}
		
	if (!(PINE & (1<<FLAME_SENSOR_PIN)))
		//if(((PINE & FLAME_SENSOR_PIN)==1))
		{
			PORTB |=(1<< BUZZER_PIN_1);			//flame detected so active buzzer
			
		}
		else
		{
			PORTB &=~(1<<BUZZER_PIN_1);			// No flame detected so turn of buzzer
			
		}	
		
		
	}
	
	return 0;
	
}

// for security system

void buzzer_init(void)
{
	DDRB |= (1 << BIT_BUZZER); // set Pin as output
	TCCR2 = 0; // Reset Timer Register
	TCCR2 |= (1 << WGM21); // CTC mode
	TCCR2 |= (1 << COM20); // Toggle Mode
	TCCR2 |= (1 << CS20); // no prescaler ==> Timer2 clock is 1MHz
	OCR2 = 0x00;
}

void buzzer_tone(void)
{
	OCR2 = 220;
	_delay_ms(100);
	OCR2 = 0;
	_delay_ms(100);
	OCR2 = 220;
	_delay_ms(100);
	OCR2 = 0;
	_delay_ms(200);
	OCR2 = 220;
	_delay_ms(200);
	OCR2 = 0;
	_delay_ms(200);
	OCR2 = 150;
	_delay_ms(1000);
	OCR2 = 0;
	return;
}


void initIO(){
	
	DDRE &=~(1<< FLAME_SENSOR_PIN);			// set flame sensor pin as output
	PORTE |=(1<< FLAME_SENSOR_PIN);			// Enable pull up resistor for flame sensor
	DDRB |=(1<<BUZZER_PIN_1);				// set Buzzer pin as output
}

