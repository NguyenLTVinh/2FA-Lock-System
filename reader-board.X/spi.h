/* 
 * File:   spi.h
 * Author: username
 *
 * Created on December 2, 2024, 11:48 AM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

    void initializeSpi(void);
    
    uint8_t spiReadByteAtAddress(const uint8_t address);
    
    void spiReadDataAtAddress(const uint8_t address, uint8_t * const data, const uint8_t length);
    
    void spiWriteByteAtAddress(const uint8_t address, const uint8_t byte);
    
    void spiWriteDataAtAddress(const uint8_t address, const uint8_t * const data, const uint8_t length);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

