#include "two_wire_interface.h"

#include "globals.h"

#include <avr/io.h>
#include <util/delay.h>

#define TWI_WRITE 0

void initializeTwi(void) {
    // Set the SDA and SCL registers to output
    PORTA.DIRSET = PIN2_bm | PIN3_bm;

    // Standard 100kHz TWI, 4 Cycle Hold, 50ns SDA Hold Time
    TWI0.CTRLA = TWI_SDAHOLD_50NS_gc;

    // Enable run in debug mode
    TWI0.DBGCTRL = TWI_DBGRUN_bm;

    // Clear MSTATUS (write 1 to flags). BUSSTATE set to idle
    TWI0.MSTATUS = TWI_RIF_bm | TWI_WIF_bm | TWI_CLKHOLD_bm | TWI_RXACK_bm |
            TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_BUSSTATE_IDLE_gc;

    // Use the given baud rate corresponding to the integer value 10
    TWI0.MBAUD = 10;

    // Enable TWI in host mode
    TWI0.MCTRLA = TWI_ENABLE_bm;
}

/**
 * Wait for UINT8_MAX iterations for the TWI to be ready to transmit more data.
 * At 100us per iteration, the maximum delay before giving up is 25.6ms.
 * 
 * @return a truthy value if the TWI did not successfully indicate that it is
 * ready to transmit more data, a falsy value otherwise
 */
uint8_t twiWait(void) {
    uint8_t iterations = 0;
    while (!(TWI0.MSTATUS & TWI_CLKHOLD_bm) && ++iterations < UINT8_MAX) {
        _delay_us(100);
    }
    return iterations == UINT8_MAX;
}

void twiWriteBytes(const uint8_t address, const uint8_t * const data, const uint8_t length) {
    // Address the destination microcontroller in write mode
    TWI0.MADDR = (address << 1) | TWI_WRITE;

    if (twiWait()) {
        goto endTransmission;
    }

    // Write the given data
    for (uint8_t i = 0; i < length; ++i) {
        TWI0.MDATA = data[i];

        if (twiWait()) {
            goto endTransmission;
        }
    }

    // Stop the bus (terminate transmission)
endTransmission:
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}