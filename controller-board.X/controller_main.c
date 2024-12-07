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
char storedRfids[5][9] = {"ABCD1234", "5678EFGH", "9012IJKL", "3456MNOP", "7890QRST"}; // Example stored valid RFIDs
char storedPasscodes[5][5] = {"1234", "5678", "9012", "3456", "7890"}; // Example stored valid passcodes
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
    usartReadUntil(buffer, ">");
    extractLastCharacters(buffer, rfid, 8);
    usartWriteCommand(rfid);
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
        usartWriteCommand("AOK>");
        currentState = WAITING_FOR_PASSCODE; // Proceed to passcode validation
    } else {
        usartWriteCommand("ERR>");
        currentState = WAITING_FOR_RFID; // Restart RFID validation
    }
}

void waitPasscodeState() {
    usartReadUntil(buffer, ">");
    extractLastCharacters(buffer, passcode, 4);
    usartWriteCommand(passcode);
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
        usartWriteCommand("AOK>");
        currentState = ACCESS_GRANTED; // Proceed to unlock the door
    } else {
        usartWriteCommand("ERR>");
        currentState = WAITING_FOR_PASSCODE; // Retry passcode validation
    }
}

void unlockDoor() {
    openLock();
    _delay_ms(5000);
    lockLock();
    currentState = WAITING_FOR_RFID;
}
