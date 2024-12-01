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

void keyPadInit() {
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
    PORTC.DIR |= PIN0_bm;                 // Set Keypad_C0 (C0) as output
    PORTC.OUT &= ~PIN0_bm;                // Set Keypad_C0 (C0) to low
    PORTA.DIR |= PIN3_bm;                 // Set Keypad_C1 (A3) as output
    PORTA.OUT &= ~PIN3_bm;                // Set Keypad_C1 (A3) to low
    PORTA.DIR |= PIN2_bm;                 // Set Keypad_C2 (A2) as output
    PORTA.OUT &= ~PIN2_bm;                // Set Keypad_C2 (A2) to low
}

// Get the currently pressed key
enum KeypadKey_t getKey(){
    for (uint8_t col = 0; col < 3; col++) {
        // Activate the current column
        switch (col) {
            case 0: PORTC.OUT |= PIN0_bm; break; // Set Keypad_C0 high
            case 1: PORTA.OUT |= PIN3_bm; break; // Set Keypad_C1 high
            case 2: PORTA.OUT |= PIN2_bm; break; // Set Keypad_C2 high
        }
        // Check each row for a pressed key
        if (!(PORTD.IN & PIN5_bm)) { return kpMap[0][col]; } // Keypad_R0
        if (!(PORTD.IN & PIN1_bm)) { return kpMap[1][col]; } // Keypad_R1
        if (!(PORTD.IN & PIN6_bm)) { return kpMap[2][col]; } // Keypad_R2
        if (!(PORTC.IN & PIN1_bm)) { return kpMap[3][col]; } // Keypad_R3
        // Deactivate the current column
        switch (col) {
            case 0: PORTC.OUT &= ~PIN0_bm; break; // Set Keypad_C0 low
            case 1: PORTA.OUT &= ~PIN3_bm; break; // Set Keypad_C1 low
            case 2: PORTA.OUT &= ~PIN2_bm; break; // Set Keypad_C2 low
        }
    }
    return Key_None;
}
