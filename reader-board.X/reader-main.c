#include "../common.X/main.h"
#include "../common.X/button.h"
#include "../common.X/two_wire_interface.h"
#include "rfid-reader.h"

#include <avr/io.h>
#include <avr/interrupt.h>

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
    initializeCommonBoardFunctions("DoorLockReader");
    initializeTwi();

    while (1) {
        if (isButtonPressed(&TEST_BUTTON)) {
            // Do something
        }
    }

    return 0;
}
