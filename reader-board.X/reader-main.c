#include "../common.X/main.h"
#include "../common.X/button.h"
#include "../common.X/two_wire_interface.h"
#inlcude "../common.X/bluetooth.h"
#include "keypad.h"
#include "bluetooth_reader.h"


#include <avr/io.h>
#include <avr/interrupt.h>

// Setups for the keypad
volatile uint16_t adc_result = 0;

ISR(ADC0_RESRDY_vect) {
    adc_result = ADC0.RES;
    // Get the corresponding key for the ADC value
    char key = ADCGetKey(adc_result);
    // Send the key via USART if it's not Key_None
    if (key != Key_None) {
        usartWriteChar(key);
        _delay_ms(500);
    }
    // Clear ADC interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;
}


int main(void) {
    initializeCommonBoardFunctions("DoorLockReader");
    initializeTwi();

    while (1) {
        if (isButtonPressed(&TEST_BUTTON)) {
            // Do something
        }
    }

    return 0;
}
