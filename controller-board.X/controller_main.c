#include "../common.X/main.h"
#include "../common.X/button.h"
#include "../common.X/bluetooth.h"
#include "../common.X/two_wire_interface.h"
#include "../common.X/usart.h"
#include "bluetooth_controller.h"
#include "doorlock_controller.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include "../common.X/globals.h"

// Define states for the controller
typedef enum {
    WAITING_FOR_RFID,
    VALIDATING_RFID,
    WAITING_FOR_PASSCODE,
    VALIDATING_PASSCODE,
    ACCESS_GRANTED
} ControllerState_t;

// Global variables
volatile ControllerState_t currentState = WAITING_FOR_RFID;
char storedRfids[5][9] = {"378F1803", "92F80B01"}; //  Stored valid RFIDs
char storedPasscodes[5][5] = {"1234", "2003", "4561"}; // Stored valid passcodes
char buffer[128]; // Buffer for receiving data
char rfid[9];
char passcode[5];

// Function prototypes
void setup();
void waitRfidState();
void validateRfidState();
void waitPasscodeState();
void validatePasscodeState();
void unlockDoor();

void setup() {
    initializeUsart();
    bluetoothInit();
    //bluetoothInitController();
    sendBluetoothCommand("$$$", "CMD> ");
    // Enable Device Information and UART Transparent services
    sendBluetoothCommand("SS,C0\r\n", "CMD> ");
    // Reboot for the configuration to take effect
    sendBluetoothCommand("R,1\r\n", "%REBOOT%");
    // Need to enter command mode again
    sendBluetoothCommand("$$$", "CMD> ");
    // Add the peripheral's address to the white list
    sendBluetoothCommand("JA,0,0491629A182C\r\n", "CMD> ");
    // Scan for the device
    sendBluetoothCommand("F\r\n", "0491629A182C"); // Make sure the peripheral is found
    // Stop scan, return to command mode
    sendBluetoothCommand("X\r\n", "CMD> ");
    // Connect to the peripheral's device
    usartWriteCommand("C,0,0491629A182C\r\n");
    doorLockInit();
    lockLock();
}

int main(void) {
    setup();
//    while (1) {
//        usartReadUntil(buffer, ">");
//        extractLastCharacters(buffer, rfid, 8);
//        usartWriteCommand(rfid);
//    }
    while (1) {
        switch (currentState) {
            case WAITING_FOR_RFID:
                waitRfidState();
                break;

            case VALIDATING_RFID:
                validateRfidState();
                break;

            case WAITING_FOR_PASSCODE:
                waitPasscodeState();
                break;

            case VALIDATING_PASSCODE:
                validatePasscodeState();
                break;

            case ACCESS_GRANTED:
                unlockDoor();
                break;
        }
    }

    return 0;
}

void waitRfidState() {
    usartReadUntil(buffer, "|");
    extractLastCharacters(buffer, rfid, 8);
    currentState = VALIDATING_RFID;
}

void validateRfidState() {
    int valid = 0;
    for (int i = 0; i < 5; i++) {
        if (strcmp(rfid, storedRfids[i]) == 0) {
            valid = 1;
            break;
        }
    }
    if (valid) {
        usartWriteCommand("AOK|");
        currentState = WAITING_FOR_PASSCODE; // Proceed to passcode validation
    } else {
        usartWriteCommand("ERR|");
        currentState = WAITING_FOR_RFID; // Restart RFID validation
    }
}

void waitPasscodeState() {
    usartReadUntil(buffer, "|");
    extractLastCharacters(buffer, passcode, 4);
    currentState = VALIDATING_PASSCODE;
}

void validatePasscodeState() {
    int valid = 0;
    for (int i = 0; i < 5; i++) { // Check against stored passcodes
        if (strcmp(passcode, storedPasscodes[i]) == 0) {
            valid = 1;
            break;
        }
    }
    if (valid) {
        usartWriteCommand("AOK|");
        currentState = ACCESS_GRANTED; // Proceed to unlock the door
    } else {
        usartWriteCommand("ERR|");
        currentState = WAITING_FOR_PASSCODE; // Retry passcode validation
    }
}

void unlockDoor() {
    openLock();
    _delay_ms(5000);
    lockLock();
    currentState = WAITING_FOR_RFID;
}
