#include "button.h"

void initializeButton(const Button * const button) {
    (*button->port).DIRCLR = button->bitmask;
    *button->pin_control |= PORT_PULLUPEN_bm | PORT_ISC_RISING_gc;
}

void checkButtonPress(Button * const button) {
    if (((*button->port).INTFLAGS & button->bitmask) != 0) {
        button->is_pressed = 1;
    }

    (*button->port).INTFLAGS &= button->bitmask;
}

inline uint8_t isButtonPressed(Button * const button) {
    const uint8_t is_pressed = button->is_pressed == 1;
    button->is_pressed = 0;
    return is_pressed;
}