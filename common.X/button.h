#ifndef BUTTON_H
#define	BUTTON_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>

    typedef struct Button {
        uint8_t bitmask;
        PORT_t * const port;
        register8_t * const pin_control;
        uint8_t is_pressed;
    } Button;

    /**
     * Creates and returns a button structure for the given port and bit position.
     * 
     * @param port_argument the port this button is connected to
     * @param bit_position_argument the bit position of the pin on the given port 
     * that this button is connected to
     * @return a Button structure for the given port and bit position
     */
#define CREATE_BUTTON(port_argument, bit_position_argument) {\
        .bitmask = 1 << bit_position_argument,\
        .port = &port_argument,\
        .pin_control = &port_argument.PIN0CTRL + sizeof(register8_t) * bit_position_argument,\
        .is_pressed = 0,\
    }

    /**
     * Sets the state of the microcontroller to allow for an interrupt-drive
     * button on the given button's pins.
     * 
     * @param button the button to initialize the microcontroller for
     */
    void initializeButton(const Button * const button);

    /**
     * Updates this button's internal pressed state. Must be called within the
     * ISR of the port that this button is connected to.
     * 
     * @param button the button to update the pressed state for
     */
    void checkButtonPress(Button * const button);

    /**
     * Determines whether this button has been pressed since the last call to
     * this function for the given button. Returns a truthy value only once per
     * button press.
     * 
     * @param button a pointer to the button to determine the pressed status of
     * @return a truthy value if this button has been pressed, a falsy value
     * otherwise
     */
    uint8_t isButtonPressed(Button * const button);

#ifdef	__cplusplus
}
#endif

#endif	/* BUTTON_H */

