/* 
 * File:   mfrc522.h
 * Author: username
 *
 * Created on December 2, 2024, 12:40 PM
 */

#ifndef MFRC522_H
#define	MFRC522_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include <stdbool.h>

    void initializeMfrc522(void);

    uint8_t mfrc522ReadByteAtAddress(const uint8_t address);

    bool mfrc522ReadDataAtAddress(const uint8_t address, uint8_t * const data, const uint8_t length, const uint8_t receive_alignment);

    bool mfrc522WriteByteAtAddress(const uint8_t address, const uint8_t byte);

    bool mfrc522WriteDataAtAddress(const uint8_t address, const uint8_t * const data, const uint8_t length);


#ifdef	__cplusplus
}
#endif

#endif	/* MFRC522_H */

