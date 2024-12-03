#include "../common.X/main.h"
#include "../common.X/button.h"
#include "rfid-reader.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "../common.X/globals.h"
#include <util/delay.h>

Button TEST_BUTTON = CREATE_BUTTON(PORTA, PIN1_bp);

#define BUTTONS_COUNT 1
Button * const BUTTONS[BUTTONS_COUNT] = {
    &TEST_BUTTON,
};

ISR(PORTA_PORT_vect) {
    for (uint8_t i = 0; i < BUTTONS_COUNT; ++i) {
        checkButtonPress(BUTTONS[i]);
    }
}

int main(void) {
    //    initializeCommonBoardFunctions("DoorLockReader");
    initializeReader();

    PORTD.DIRSET = PIN1_bm;
    
    Uid uid;

    while (1) {
        const bool value = readRfidCard(&uid);
        if (value) {
            // Temporary breakpoint spot
            _delay_ms(1);
        }
        
        PORTD.OUTTGL = PIN1_bm;
        
        _delay_ms(250);
    }

    if (isButtonPressed(&TEST_BUTTON)) {
        return 1;
    }

    return 0;
}
