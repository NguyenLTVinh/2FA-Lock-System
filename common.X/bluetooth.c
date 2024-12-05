#include "bluetooth.h"
#include "usart.h"

#include "globals.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define BUF_SIZE 128
#define BLE_RADIO_PROMPT "CMD> "

// Common ble init for both boards
void bluetoothInit() {
    // Put BLE Radio in "Application Mode" by driving F3 high
    PORTF.DIRSET = PIN3_bm;
    PORTF.OUTSET = PIN3_bm;
    // Reset BLE Module - pull PD3 low, then back high after a delay
    PORTD.DIRSET = PIN3_bm | PIN2_bm;
    PORTD.OUTCLR = PIN3_bm;
    _delay_ms(10); // Leave reset signal pulled low
    PORTD.OUTSET = PIN3_bm;
    // The AVR-BLE hardware guide is wrong. Labels this as D3
    // Tell BLE module to expect data - set D2 low
    PORTD.OUTCLR = PIN2_bm;
    _delay_ms(200); // Give time for RN4870 to boot up
}

// Send bluetooth command and read until a response
void sendBluetoothCommand(const char *const command, const char *const readUntil) {
    char buf[BUF_SIZE];
    usartWriteCommand(command);
    usartReadUntil(buf, readUntil);
}

// Write a specific number of bytes to bluetooth stream
void bluetoothWriteBytes(const char *const data, int bytes) {
    for (int i = 0; i < bytes; i++) {
        usartWriteCharacter(data[i]);
    }
}

// Read a specific number of bytes from bluetooth stream
void bluetoothReadBytes(char *const destination, int bytes) {
    for (int i = 0; i < bytes; i++) {
        destination[i] = usartReadCharacter();
    }
    destination[bytes] = 0;
}
