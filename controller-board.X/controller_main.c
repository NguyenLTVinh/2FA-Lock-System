#include "../common.X/main.h"
#include "../common.X/atecc508a.h"
#include <avr/io.h>

const uint8_t SENDER_PUBLIC_KEY[64] = {
    0xAC, 0x11, 0x8B, 0x73, 0x3D, 0x83, 0x50, 0x52, 0x2B, 0xBC, 0xAF, 0x26, 0x83, 0x3E, 0xC4, 0xEF,
    0x2B, 0xBF, 0x91, 0x4C, 0x0E, 0x56, 0x6D, 0xFA, 0xC0, 0xF8, 0xA0, 0xDD, 0xBC, 0xDA, 0x1D, 0x38,
    0xE0, 0xBC, 0xAA, 0x00, 0x29, 0x9D, 0x50, 0x84, 0x07, 0xF0, 0x2C, 0x18, 0xBB, 0xC3, 0x1C, 0xE3,
    0x7C, 0x52, 0x3F, 0xC4, 0x08, 0x64, 0xF7, 0x9B, 0xC2, 0x61, 0xBC, 0x3B, 0x9B, 0xA2, 0xE4, 0x6E
};

int main(void) {
    //    initializeCommonBoardFunctions("DoorLockContoller");

    // Set the pin to output and clear the success and error LEDs
    PORTF.DIRSET = PIN4_bm | PIN5_bm;
    PORTF.OUTSET = PIN4_bm | PIN5_bm;

    if (!initializeAtecc508a()) {
        // failure
        PORTF.OUTCLR = PIN5_bm;
        return 0;
    }

    const uint8_t message[32] = "message to sign!message to sign!";

    uint8_t signature[64];

    if (!atecc508aSignMessage(message, 0, signature)) {
        // failure
        PORTF.OUTCLR = PIN5_bm;
        return 0;
    }

    if (atecc508aIsSignatureValid(message, signature, SENDER_PUBLIC_KEY)) {
        // success
        PORTF.OUTCLR = PIN4_bm;
    } else {
        // failure
        PORTF.OUTCLR = PIN5_bm;
        return 0;
    }

    while (1);

    return 0;
}
