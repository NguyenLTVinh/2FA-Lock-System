#include <avr/io.h>

#ifndef USART_H
#define	USART_H

#ifdef	__cplusplus
extern "C" {
#endif

    void initializeUsart(void);

    void usartWriteCharacter(const char character);

    char usartReadCharacter(void);

    void usartWriteCommand(const char *const command);

    void usartReadUntil(char *const destination, const char *const end_string);
    
    void extractLastCharacters(const char *input, char *output, uint8_t numChars);

#ifdef	__cplusplus
}
#endif

#endif	/* USART_H */
