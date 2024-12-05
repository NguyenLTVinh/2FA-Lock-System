#include "../common.X/main.h"
#include "../common.X/button.h"
#include "../common.X/bluetooth.h"
#include "../common.X/two_wire_interface.h"
#include "../common.X/usart.h"
#include "spi.h"
#include "keypad.h"
#include "bluetooth_reader.h"
#include "rfid-reader.h"
#include "lcd.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include "../common.X/globals.h"
#include <util/delay.h>

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
    initializeSpi();
    initializeReader();
    bluetoothInit();
    //bluetoothInitReader();
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
        usartWriteCharacter(key);
        _delay_ms(500);
    }
    // Clear ADC interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;
}


int main(void) {
    //    initializeCommonBoardFunctions("DoorLockReader");
    setup();
    sei();
//    while (1) {
//        bluetoothWriteBytes("TEST", 2);
//    }
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

    Uid card;

    while (currentState == IDLE) { // Stay in IDLE until a card is scanned
        if (readRfidCard(&card)) {
            // Format the UID into a string
            char uidString[RFID_READER_BUFFER_SIZE] = {0};
            for (uint8_t i = 0; i < card.size; i++) {
                sprintf(&uidString[i * 2], "%02X", card.uidByte[i]);
            }
            // Send the UID to the controller via Bluetooth
            bluetoothWriteBytes(uidString, strlen(uidString));
            // Wait for controller response
            char response[BUF_SIZE];
            bluetoothReadBytes(response, 4);
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
    _delay_ms(1000);
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
    _delay_ms(1000);
    currentState = PASSCODE;
}

void accessGrantedState() {
    lq_clear(&device);
    lq_print(&device, "Lock Opened!");
    _delay_ms(1000);
    currentState = IDLE;
}