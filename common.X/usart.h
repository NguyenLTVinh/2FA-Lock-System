#ifndef USART_H
#define	USART_H

#ifdef	__cplusplus
extern "C" {
#endif

    void initializeUsart(void);

    void usartWriteCommand(const char *const command);

    void usartReadUntil(char *const destination, const char *const end_string);

#ifdef	__cplusplus
}
#endif

#endif	/* USART_H */

