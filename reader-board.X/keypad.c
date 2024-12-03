#include "../common.X/globals.h"
#include "keypad.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

//PORT, PIN, can change
#define Keypad_R0			D, 5
#define Keypad_R1			D, 1
#define Keypad_R2			D, 6
#define Keypad_R3			C, 1    
#define Keypad_C0			C, 0
#define Keypad_C1			A, 3
#define Keypad_C2			A, 2

void keyPadInit() 
{
    // Configure rows as inputs with pull-up resistors
    PORTD.DIR &= ~PIN5_bm;                // Set Keypad_R0 (D5) as input
    PORTD.PIN5CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R0 (D5)
    PORTD.DIR &= ~PIN1_bm;                // Set Keypad_R1 (D1) as input
    PORTD.PIN1CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R1 (D1)
    PORTD.DIR &= ~PIN6_bm;                // Set Keypad_R2 (D6) as input
    PORTD.PIN6CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R2 (D6)
    PORTC.DIR &= ~PIN1_bm;                // Set Keypad_R3 (C1) as input
    PORTC.PIN1CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R3 (C1)
    // Configure columns as outputs and set to low
    PORTC.DIR &= ~PIN0_bm;                 // Set Keypad_C0 (C0) as input
    PORTA.DIR &= ~PIN3_bm;                 // Set Keypad_C1 (A3) as input
    PORTA.DIR &= ~PIN2_bm;                 // Set Keypad_C2 (A2) as input
}

// Get the currently pressed key
enum KeypadKey_t getKey()
{
    unsigned char row , col;	
    
    for (uint8_t col = 0; col < 3; col++) {
        _delay_ms(10);									// wait 10ms for button de-bouncing
        PORTC.DIR &= ~(1 << PIN0_bp);  // Set Keypad_C0 (C0) as input
        PORTA.DIR &= ~(1 << PIN3_bp);  // Set Keypad_C1 (A3) as input
        PORTA.DIR &= ~(1 << PIN2_bp);  // Set Keypad_C2 (A2) as input
		// make only current column as output
        switch (col) {
            case 0:
                PORTC.DIR |= (1 << PIN0_bp);  // Set Keypad_C0 as output
                PORTC.OUT &= ~(1 << PIN0_bp); // Set Keypad_C0 to low
                break;
            case 1:
                PORTA.DIR |= (1 << PIN3_bp);  // Set Keypad_C1 as output
                PORTA.OUT &= ~(1 << PIN3_bp); // Set Keypad_C1 to low
                break;
            case 2:
                PORTA.DIR |= (1 << PIN2_bp);  // Set Keypad_C2 as output
                PORTA.OUT &= ~(1 << PIN2_bp); // Set Keypad_C2 to low
                break;
        }
		// make only current column output is low
		for (row = 0; row < 4; row++) {   
            // Check if the current row reads low, indicating a key press
            switch (row) {
                case 0:
                    if (!(PORTD.IN & (1 << PIN5_bp))) { // Check Keypad_R0 (D5)
                        return kpMap[row][col];
                    }
                    break;
                case 1:
                    if (!(PORTD.IN & (1 << PIN1_bp))) { // Check Keypad_R1 (D1)
                        return kpMap[row][col];
                    }
                    break;
                case 2:
                    if (!(PORTD.IN & (1 << PIN6_bp))) { // Check Keypad_R2 (D6)
                        return kpMap[row][col];
                    }
                    break;
                case 3:
                    if (!(PORTC.IN & (1 << PIN1_bp))) { // Check Keypad_R3 (C1)
                        return kpMap[row][col];
                    }
                    break;
            }
        }
        // Reset the current column
        switch (col) {
            case 0:
                PORTC.DIR &= ~(1 << PIN0_bp); // Set Keypad_C0 to input
                break;
            case 1:
                PORTA.DIR &= ~(1 << PIN3_bp); // Set Keypad_C1 to input
                break;
            case 2:
                PORTA.DIR &= ~(1 << PIN2_bp); // Set Keypad_C2 to input
                break;
        }
    }
    return Key_None; // No key was pressed
}
