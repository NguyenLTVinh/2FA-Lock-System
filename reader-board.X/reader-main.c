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

    char buf[BUF_SIZE];
    // Put RN4870 in Command Mode
    sendBluetoothCommand("$$$", "CMD> ");
    // Enable advertising
    sendBluetoothCommand("A\r\n", "CMD> ");
    usartReadUntil(buf, "%STREAM_OPEN%");
    initializeMfrc522();
    ADCKeyPadInit();
    device = lq_init(0x27, 20, 4, LCD_5x8DOTS);
    lq_turnOnBacklight(&device);
}

ISR(ADC0_RESRDY_vect) {
    adc_result = ADC0.RES;
    // Get the corresponding key for the ADC value
    char key = ADCGetKey(adc_result);
    // Send the key via USART if it's not Key_None
    if (key != Key_None) {
        //usartWriteCharacter(key);
        _delay_ms(500);
    }
    // Clear ADC interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;
}


int main(void) {
    setup();
    sei();
    
    while (1) {
        switch (currentState) {
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
            bluetoothWriteBytes(uid_string, strlen(uid_string));

            // Wait for the controller response
            // NOTE: Some weird issue with BLE encoding, will have to fix later
            char response[BUF_SIZE];
            bluetoothReadBytes(response, 4);
            usartWriteCommand(response);
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
    lq_print(&device, "Keycard Not Recognized");
    _delay_ms(200);
    // Transition to IDLE
    currentState = IDLE;
}

void passcodeState() {
    lq_clear(&device);
    lq_print(&device, "Enter Passcode");
}

void passcodeErrorState() {
    lq_clear(&device);
    lq_print(&device, "Incorrect Passcode");
    _delay_ms(200);
    currentState = PASSCODE;
}

void accessGrantedState() {
    lq_clear(&device);
    lq_print(&device, "Lock Opened!");
    _delay_ms(200);
    currentState = IDLE;
}
