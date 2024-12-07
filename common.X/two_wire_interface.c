#include "two_wire_interface.h"

#include "globals.h"

#include <avr/io.h>
#include <util/delay.h>

const uint8_t TWI_WRITE = 0;
const uint8_t TWI_READ = 1;

void initializeTwi(void) {
    // Set the SDA and SCL registers to output
    PORTA.DIRSET = PIN2_bm | PIN3_bm;

    // Standard 100kHz TWI, 4 Cycle Hold, 50ns SDA Hold Time
    TWI0.CTRLA = TWI_SDASETUP_8CYC_gc | TWI_SDAHOLD_500NS_gc;

    // Enable run in debug mode
    TWI0.DBGCTRL = TWI_DBGRUN_bm;

    // Clear MSTATUS (write 1 to flags). BUSSTATE set to idle
    TWI0.MSTATUS = TWI_CLKHOLD_bm | TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_BUSSTATE_IDLE_gc;

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

uint8_t twiWriteBytes(const uint8_t address, const uint8_t * const data, const uint8_t length) {
    // Address the destination microcontroller in write mode
    TWI0.MADDR = (address << 1) | TWI_WRITE;

    uint8_t bytes_written = 0;

    if (twiWait()) {
        goto endTransmission;
    }

    // Write the given data
    for (uint8_t i = 0; i < length; ++i) {
        TWI0.MDATA = data[i];

        if (twiWait()) {
            goto endTransmission;
        }

        ++bytes_written;
    }

    // Stop the bus (terminate transmission)
endTransmission:
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;

    return bytes_written;
}

uint8_t twiWriteBytesToDeviceAddress(const uint8_t address, const uint8_t register_address, const uint8_t * const data, const uint8_t length) {
    // Address the destination microcontroller in write mode
    TWI0.MADDR = (address << 1) | TWI_WRITE;

    uint8_t bytes_written = 0;

    if (twiWait()) {
        goto endTransmission;
    }

    TWI0.MDATA = register_address;

    if (twiWait()) {
        goto endTransmission;
    }

    // Write the given data
    for (uint8_t i = 0; i < length; ++i) {
        TWI0.MDATA = data[i];

        if (twiWait()) {
            goto endTransmission;
        }

        ++bytes_written;
    }

    // Stop the bus (terminate transmission)
endTransmission:
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;

    return bytes_written;
}

uint8_t twiReadBytes(const uint8_t address, uint8_t * const buffer, const uint8_t length) {
    // Address the destination microcontroller in write mode
    TWI0.MADDR = (address << 1) | TWI_READ;

    uint8_t bytes_read = 0;

    if (twiWait()) {
        goto endTransmission;
    }

    TWI0.MSTATUS = TWI_CLKHOLD_bm;

    while (bytes_read < length) {
        if (twiWait()) {
            goto endTransmission;
        }

        buffer[bytes_read] = TWI0.MDATA;

        ++bytes_read;

        if (bytes_read < length) {
            // If not done, then ACK and read the next byte
            TWI0.MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;
        }
    }

    // Stop the bus (terminate transmission)
endTransmission:
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;

    return bytes_read;
}

uint8_t twiReadBytesFromDeviceAddress(const uint8_t address, const uint8_t register_address, uint8_t * const data, const uint8_t length) {
    // Address the destination microcontroller in write mode
    TWI0.MADDR = (address << 1) | TWI_WRITE;

    uint8_t bytes_read = 0;

    if (twiWait()) {
        goto endTransmission;
    }

    TWI0.MDATA = register_address;

    if (twiWait()) {
        goto endTransmission;
    }

    // Switch to read mode so we can read starting at the selected address
    TWI0.MADDR |= TWI_READ;
    TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;

    // Release the clock hold
    TWI0.MSTATUS = TWI_CLKHOLD_bm;

    while (bytes_read < length) {
        if (twiWait()) {
            goto endTransmission;
        }

        data[bytes_read] = TWI0.MDATA;

        ++bytes_read;

        if (bytes_read < length) {
            // If not done, then ACK and read the next byte
            TWI0.MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;
        }
    }

    // Stop the bus (terminate transmission)
endTransmission:
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;

    return bytes_read;
}