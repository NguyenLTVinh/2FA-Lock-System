#include "bluetooth.h"
#include "usart.h"

#include "globals.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define BUF_SIZE 128
#define BLE_RADIO_PROMPT "CMD> "

void bluetoothInitController() {
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
    char buf[BUF_SIZE];

    // Put RN4870 in Command Mode
    usartWriteCommand("$$$");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Enable Device Information and UART Transparent services
    usartWriteCommand("SS,C0\r\n");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Reboot for the configuration to take effect
    usartWriteCommand("R,1\r\n");
    usartReadUntil(buf, "%REBOOT%");
    // Need to enter command mode again
    usartWriteCommand("$$$");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Add the peripheral's address to the white list
    usartWriteCommand("JA,0,0491629A182C\r\n");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Scan for the device
    usartWriteCommand("F\r\n");
    usartReadUntil(buf, "0491629A182C"); // Make sure the peripheral is found
    // Stop scan, return to command mode
    usartWriteCommand("X\r\n");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Connect to the peripheral's device
    usartWriteCommand("C,0,0491629A182C\r\n");
    usartReadUntil(buf, "%STREAM_OPEN%");
}

void bluetoothInitReader() {
    // Put BLE Radio in "Application Mode" by driving F3 high
    PORTF.DIRSET = PIN3_bm;
    PORTF.OUTSET = PIN3_bm;
    // Reset BLE Module - pull PD3 low, then back high after a delay
    PORTD.DIRSET = PIN3_bm | PIN2_bm;
    PORTD.OUTCLR = PIN3_bm;
    _delay_ms(10); // Leave reset signal pulled low
    PORTD.OUTSET = PIN3_bm;
    // Tell BLE module to expect data - set D2 low
    PORTD.OUTCLR = PIN2_bm;
    _delay_ms(200); // Give time for RN4870 to boot up
    char buf[BUF_SIZE];
    // Put RN4870 in Command Mode
    usartWriteCommand("$$$");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Display info for MAC
    usartWriteCommand("D\r\n");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Enable advertising
    usartWriteCommand("A\r\n");
    usartReadUntil(buf, BLE_RADIO_PROMPT);
    // Wait until connected by the central board
    usartReadUntil(buf, "%STREAM_OPEN%");

}

void sendBluetoothCommand(const char *const command) {
    char buf[BUF_SIZE];
    usartWriteCommand(command);
    usartReadUntil(buf, BLE_RADIO_PROMPT);
}