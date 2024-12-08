#include "../common.X/globals.h"
#include "keypad.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

// Updated pin definitions
#define Keypad_R0           D, 7
#define Keypad_R1           D, 5
#define Keypad_R3           D, 1
#define Keypad_C0           D, 6
#define Keypad_C1           C, 1
#define Keypad_C2           C, 0

// Keypad Key Mapping for 4x3 Layout (without 3rd row). Had to sacrifice 3rd row because not enough pins :(
const char kpMap[3][3] = {
    {Key_1, Key_2, Key_3}, // Row 0
    {Key_4, Key_5, Key_6}, // Row 1
    {Key_E, Key_0, Key_F}  // Row 3 (R2 is excluded)
};

void keyPadInit() 
{
    // Configure rows as inputs with pull-up resistors
    PORTD.DIR &= ~PIN7_bm;                // Set Keypad_R0 (D7) as input
    PORTD.PIN7CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R0 (D7)
    PORTD.DIR &= ~PIN5_bm;                // Set Keypad_R1 (D5) as input
    PORTD.PIN5CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R1 (D5)
    PORTD.DIR &= ~PIN1_bm;                // Set Keypad_R3 (D1) as input
    PORTD.PIN1CTRL |= PORT_PULLUPEN_bm;  // Enable pull-up resistor on Keypad_R3 (D1)
    
    // Configure columns as inputs initially
    PORTD.DIR &= ~PIN6_bm;                // Set Keypad_C0 (D6) as input
    PORTC.DIR &= ~PIN1_bm;                // Set Keypad_C1 (C1) as input
    PORTC.DIR &= ~PIN0_bm;                // Set Keypad_C2 (C0) as input
}

enum KeypadKey_t getKey()
{
    unsigned char row;	
    
    for (uint8_t col = 0; col < 3; col++) {
        _delay_ms(10);  // wait 10ms for button de-bouncing
        
        // Set all columns to input
        PORTD.DIR &= ~PIN6_bm;  // Keypad_C0 (D6)
        PORTC.DIR &= ~PIN1_bm;  // Keypad_C1 (C1)
        PORTC.DIR &= ~PIN0_bm;  // Keypad_C2 (C0)

        // Make only the current column an output and set it low
        switch (col) {
            case 0:
                PORTD.DIR |= PIN6_bm;     // Set Keypad_C0 as output
                PORTD.OUT &= ~PIN6_bm;    // Set Keypad_C0 to low
                break;
            case 1:
                PORTC.DIR |= PIN1_bm;     // Set Keypad_C1 as output
                PORTC.OUT &= ~PIN1_bm;    // Set Keypad_C1 to low
                break;
            case 2:
                PORTC.DIR |= PIN0_bm;     // Set Keypad_C2 as output
                PORTC.OUT &= ~PIN0_bm;    // Set Keypad_C2 to low
                break;
        }
        
        // Check rows for a pressed key
        for (row = 0; row < 3; row++) {  // Only rows 0, 1, 3 are used
            switch (row) {
                case 0:
                    if (!(PORTD.IN & PIN7_bm)) { // Check Keypad_R0 (D7)
                        return kpMap[row][col];
                    }
                    break;
                case 1:
                    if (!(PORTD.IN & PIN5_bm)) { // Check Keypad_R1 (D5)
                        return kpMap[row][col];
                    }
                    break;
                case 2: // Row 3 mapped to R3
                    if (!(PORTD.IN & PIN1_bm)) { // Check Keypad_R3 (D1)
                        return kpMap[row][col];
                    }
                    break;
            }
        }
        
        // Reset the current column to input
        switch (col) {
            case 0:
                PORTD.DIR &= ~PIN6_bm; // Keypad_C0 to input
                break;
            case 1:
                PORTC.DIR &= ~PIN1_bm; // Keypad_C1 to input
                break;
            case 2:
                PORTC.DIR &= ~PIN0_bm; // Keypad_C2 to input
                break;
        }
    }
    return Key_None; // No key was pressed
}

// Init the keypad with analog read, using 1 pin only (Somehow breaks over night)
void ADCKeyPadInit() 
{
    PORTD.DIRCLR = PIN1_bm;
    PORTD.PIN1CTRL &= ~PORT_PULLUPEN_bm;
    PORTD.PIN1CTRL &= ~PORT_ISC_gm;
    PORTD.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV4_gc;
    ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_FREERUN_bm | ADC_RESSEL_10BIT_gc;
    ADC0.INTCTRL = ADC_RESRDY_bm;
    ADC0.COMMAND = ADC_STCONV_bm;
}


char ADCGetKey(uint16_t adc_value) 
{
    if (adc_value >= 696 && adc_value <= 698) return '1';
    else if (adc_value >= 545 && adc_value <= 547) return '2';
    else if (adc_value >= 120 && adc_value <= 300) return '3';
    else if (adc_value >= 889 && adc_value <= 891) return '4';
    else if (adc_value >= 868 && adc_value <= 870) return '5';
    else if (adc_value >= 841 && adc_value <= 843) return '6';
    else if (adc_value >= 938 && adc_value <= 940) return '7';
    else if (adc_value >= 931 && adc_value <= 933) return '8';
    else if (adc_value >= 922 && adc_value <= 924) return '9';
    else if (adc_value >= 961 && adc_value <= 963) return 'E';
    else if (adc_value >= 957 && adc_value <= 959) return '0';
    else if (adc_value >= 953 && adc_value <= 955) return 'F';
    else return Key_None;
}
