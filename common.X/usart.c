#include "usart.h"
#include "globals.h"

#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

// This is all based on formula in Table 23-1 of the ATmega 3208 data sheet
// Need to shift value left 6 bits to follow format specified in data sheet:
// 16-bit number, 10 bits are whole number part, bottom 6 are fractional part
// The +0.5 is to force the result to be rounded *up* rather than down.
// SAMPLES_PER_BIT: 16, for normal asynchronous mode. Given in data sheet.
#define SAMPLES_PER_BIT 16
#define USART_BAUD_VALUE(BAUD_RATE) (uint16_t) ((F_CPU << 6) / (((float) SAMPLES_PER_BIT) * (BAUD_RATE)) + 0.5)

void initializeUsart(void) {
    USART0.BAUD = (uint16_t) USART_BAUD_VALUE(9600);
    USART0.CTRLB |= USART_TXEN_bm | USART_RXEN_bm | USART_RXMODE_NORMAL_gc;

    PORTA.DIRSET = PIN0_bm;
    PORTA.DIRCLR = PIN1_bm;
}

void usartWriteCharacter(const char character) {
    while (!(USART0.STATUS & USART_DREIF_bm));
    USART0.TXDATAL = character;
}

void usartWriteCommand(const char *command) {
    for (; *command != '\0'; ++command) {
        usartWriteCharacter(*command);
    }
}

char usartReadCharacter(void) {
    while (!(USART0.STATUS & USART_RXCIF_bm));
    return USART0.RXDATAL;
}

void usartReadUntil(char *const destination, const char *const end_string) {
    memset(destination, 0, 32);
    const uint8_t destination_length = 32 - 1;
    const uint8_t end_len = strlen(end_string);
    uint8_t bytes_read = 0;
    while (bytes_read < destination_length && (bytes_read < end_len || strcmp(destination + bytes_read - end_len, end_string) != 0)) {
        destination[bytes_read] = usartReadCharacter();
        ++bytes_read;
    }
    destination[bytes_read] = 0;
}

void extractLastCharacters(const char *input, char *output, uint8_t numChars) {
    uint8_t len = strlen(input);
    if (len == 0 || input[len - 1] != '|' || numChars > len - 1) {
        output[0] = '\0';
        return;
    }
    // Calculate the starting position for the substring
    uint8_t start = len - 1 - numChars; // Exclude the '>'
    strncpy(output, &input[start], numChars);
    output[numChars] = '\0';
}
