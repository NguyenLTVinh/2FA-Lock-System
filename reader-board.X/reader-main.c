#include "../common.X/main.h"
#include "../common.X/button.h"
#include "../common.X/globals.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "mfrc522.h"

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
    initializeMfrc522();

    PORTD.DIRSET = PIN1_bm;

    Picc card;
    char uid_string[21] = {0};

    while (1) {
        const bool value = readPicc(&card);
        if (value) {
            for (uint8_t i = 0; i < card.size; ++i) {
                sprintf(&uid_string[2 * i], "%02X", card.uidByte[i]);
            }

            // Temporary breakpoint spot
            _delay_ms(100);
        }

        PORTD.OUTTGL = PIN1_bm;

        _delay_ms(250);
    }

    return 0;
}
