#include "bluetooth.h"
#include "usart.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define BUF_SIZE 128
#define BLE_RADIO_PROMPT "CMD> "

void initializeBluetooth(const char *const name) {
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

    // Put RN4870 in Command Mode
    sendBluetoothCommand("$$$");

    // Change BLE device name to specified value
    // There can be some lag between updating name here and
    // seeing it in the LightBlue phone interface
    char command[BUF_SIZE];
    strcpy(command, "S-,");
    strcat(command, name);
    strcat(command, "\r\n");
    sendBluetoothCommand(command);
}

void sendBluetoothCommand(const char *const command) {
    char buf[BUF_SIZE];
    usartWriteCommand(command);
    usartReadUntil(buf, BLE_RADIO_PROMPT);
}