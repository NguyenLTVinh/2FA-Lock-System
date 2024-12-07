#include "../common.X/main.h"
#include "../common.X/bluetooth.h"
#include "../common.X/usart.h"
#include "bluetooth_controller.h"
#include "doorlock_controller.h"

#include "../common.X/globals.h"
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>

#define VALID_CREDENTIAL_COUNT 2
#define RFID_STRING_LENGTH 9
#define PASSCODE_STRING_LENGTH 5
const char VALID_RFID_UIDS[VALID_CREDENTIAL_COUNT][RFID_STRING_LENGTH] = {"378F1803", "92F80B01"}; // Stored valid RFIDs
const char VALID_PASSCODES[VALID_CREDENTIAL_COUNT][PASSCODE_STRING_LENGTH] = {"1234", "2003"}; // Stored valid passcodes

// Function prototypes
void unlockDoor(void);

void setup() {
    initializeUsart();
    bluetoothInit();

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

    char buffer[128];

    while (1) {
        usartReadUntil(buffer, "|");
        char rfid_and_passcode[RFID_STRING_LENGTH - 1 + PASSCODE_STRING_LENGTH - 1];
        extractLastCharacters(buffer, rfid_and_passcode, RFID_STRING_LENGTH - 1 + PASSCODE_STRING_LENGTH - 1);

        char rfid[RFID_STRING_LENGTH] = {0};
        char passcode[PASSCODE_STRING_LENGTH] = {0};
        memcpy(rfid, rfid_and_passcode, RFID_STRING_LENGTH - 1);
        memcpy(passcode, rfid_and_passcode + RFID_STRING_LENGTH - 1, PASSCODE_STRING_LENGTH - 1);

        bool isValid = false;
        for (int i = 0; i < VALID_CREDENTIAL_COUNT; ++i) {
            if (strcmp(rfid, VALID_RFID_UIDS[i]) == 0 && strcmp(passcode, VALID_PASSCODES[i]) == 0) {
                isValid = true;
                break;
            }
        }
        if (isValid) {
            const char message[5] = { 0x41, 0x4F, 0x4B, 0x7C, 0x00};
            usartWriteCommand(message);
        } else {
            const char message[5] = { 0x45, 0x52, 0x52, 0x7C, 0x00};
            usartWriteCommand(message);
            continue;
        }

        unlockDoor();
    }

    return 0;
}

void unlockDoor() {
    openLock();
    _delay_ms(5000);
    lockLock();
}
