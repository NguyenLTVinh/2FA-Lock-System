#include "../common.X/main.h"
#include "../common.X/button.h"
#include "../common.X/bluetooth.h"
#include "../common.X/two_wire_interface.h"
#include "../common.X/usart.h"
#include "spi.h"
#include "keypad.h"
#include "bluetooth_reader.h"
#include "lcd.h"
#include "mfrc522.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include "../common.X/globals.h"

#define RFID_READER_BUFFER_SIZE 32
#define BUF_SIZE 32

// Define states
typedef enum {
    IDLE,
    RFIDERR,
    PASSCODE,
    PASSCODEERR,
    ACCESSGRANTED
} LockState_t;

// Global variables
volatile LockState_t currentState = IDLE;
LiquidCrystalDevice_t device;
volatile uint16_t adc_result = 0;

void idleState();
void rfidErrorState();
void passcodeState();
void passcodeErrorState();
void accessGrantedState();

void setup() {
    initializeUsart();
    initializeTwi();
    bluetoothInit();
    _delay_ms(500); // Some time for BLE module to boot up
    // For some reason, calling the init reader function would not work? So I'm doing the rest of the init here.
    char buf[BUF_SIZE];
    // Put RN4870 in Command Mode
    sendBluetoothCommand("$$$", "CMD> ");
    // Enable advertising
    sendBluetoothCommand("A\r\n", "CMD> ");
    // Wait until connected by the central board
    usartReadUntil(buf, "%STREAM_OPEN%");
    initializeMfrc522();
    keyPadInit();
    device = lq_init(0x27, 20, 4, LCD_5x8DOTS);
    lq_turnOnBacklight(&device);
}

// ISR(ADC0_RESRDY_vect) {
//     adc_result = ADC0.RES;
//     // Clear ADC interrupt flag
//     char key = ADCGetKey(adc_result);
//     if (adc_result < 1000) {
//         usartWriteCharacter(key);
//         _delay_ms(500);
//     }
            
//     ADC0.INTFLAGS = ADC_RESRDY_bm;
// }

int main(void) 
{
    setup();
    
    while (1) 
    {   
        switch (currentState) 
        {
            case IDLE:
                idleState();
                break;
            case RFIDERR:
                rfidErrorState();
                break;
            case PASSCODE:
                passcodeState();
                break;
            case PASSCODEERR:
                passcodeErrorState();
                break;
            case ACCESSGRANTED:
                accessGrantedState();
                break;
        }
    }
    return 0;
}

void idleState() {
    lq_clear(&device);
    lq_print(&device, "Scan Keycard");
    Picc card;
    char uid_string[21] = {0}; // Buffer to store UID as a string

    while (currentState == IDLE) { // Stay in IDLE until a card is scanned
        if (readPicc(&card)) {
            // Convert the UID bytes to a hexadecimal string
            for (uint8_t i = 0; i < card.size; ++i) {
                sprintf(&uid_string[2 * i], "%02X", card.uidByte[i]);
            }

            // Send the UID to the controller via Bluetooth
            usartWriteCommand(uid_string);
            lq_clear(&device);
            lq_print(&device, "Reading Card...");
            // Wait for the controller response
            char response[BUF_SIZE];
            bluetoothReadBytes(response, 3);
            if (strcmp(response, "AOK") == 0) {
                currentState = PASSCODE;
            } else {
                currentState = RFIDERR;
            }
        }
    }
}

void rfidErrorState() {
    lq_clear(&device);
    lq_print(&device, "Keycard Not");
    lq_setCursor(&device, 1, 0);
    lq_print(&device, "Recognized!");
    _delay_ms(2000);
    // Transition to IDLE
    currentState = IDLE;
}

void passcodeState() {
    lq_clear(&device);
    lq_print(&device, "Enter Passcode");
    lq_setCursor(&device, 1, 0);

    char passcode[5] = {0};
    uint8_t passcodeIndex = 0;

    while (currentState == PASSCODE) {
        enum KeypadKey_t key = getKey();
        if (key != Key_None) {
            // Wait until the key is released
            while (getKey() != Key_None) {
                _delay_ms(100); // Debounce
            }
            if (key == 'E') {
                // Delete the last digit if the passcode buffer is not empty
                if (passcodeIndex > 0) {
                    passcode[--passcodeIndex] = '\0';
                    lq_setCursor(&device, 1, passcodeIndex);
                    lq_print(&device, " "); // Clear the character on the LCD
                    lq_setCursor(&device, 1, passcodeIndex);
                }
            } else if (key == 'F') {
                // Submit passcode if F is pressed
                if (passcodeIndex == 4) {
                    // Send passcode to the controller via Bluetooth
                    usartWriteCommand(passcode);
                    lq_clear(&device);
                    lq_print(&device, "Checking...");
                    // Read response from the controller
                    char response[BUF_SIZE];
                    bluetoothReadBytes(response, 3);
                    if (strcmp(response, "AOK") == 0) {
                        currentState = ACCESSGRANTED;
                    } else {
                        currentState = PASSCODEERR;
                    }
                }
            } else if (key >= '0' && key <= '9') {
                // Append digit if passcode is not full
                if (passcodeIndex < 4) {
                    passcode[passcodeIndex++] = key;
                    char keystr[2] = {key , '\0'};
                    lq_print(&device, keystr); // Print the digit on the LCD
                }
            }
        }
    }
}

void passcodeErrorState() {
    lq_clear(&device);
    lq_print(&device, "Incorrect");
    lq_setCursor(&device, 1, 0);
    lq_print(&device, "Passcode!");
    _delay_ms(2000);
    currentState = PASSCODE;
}

void accessGrantedState() {
    lq_clear(&device);
    lq_print(&device, "Lock Opened!");
    _delay_ms(2000);
    currentState = IDLE;
}
