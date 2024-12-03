#include "spi.h"

#include <avr/io.h>

#define SPI_PORT	PORTA
#define SPI_MOSI	PIN4_bm
#define SPI_MISO	PIN5_bm
#define SPI_SCK		PIN6_bm
#define SPI_SS		PIN7_bm

inline void enableDevice(void) {
    SPI_PORT.OUT &= ~SPI_SS;
}

inline void disableDevice(void) {
    SPI_PORT.OUT |= SPI_SS;
}

void initializeSpi(void) {
    // Set output ports as output
    SPI_PORT.DIRSET = SPI_MOSI | SPI_SCK | SPI_SS;
    // Set input ports as input
    SPI_PORT.DIRCLR = SPI_MISO;
    SPI0.CTRLA = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESC_DIV4_gc | SPI_CLK2X_bm;
}

uint8_t spiTransferByte(const uint8_t byte) {
    // Load data into the buffer
    SPI0.DATA = byte;

    // Wait until transmission completes
    while (!(SPI0.INTFLAGS & SPI_IF_bm));

    // Return received data
    return SPI0.DATA;
}

uint8_t spiReadByteAtAddress(const uint8_t address) {
    enableDevice();
    spiTransferByte(address);
    
    const uint8_t byte = spiTransferByte(0);
    
    disableDevice();
    
    return byte;
}

bool spiReadDataAtAddress(const uint8_t address, uint8_t * const data, const uint8_t length) {
    enableDevice();
    spiTransferByte(address);

    for (uint8_t i = 0; i < length - 1; ++i) {
        data[i] = spiTransferByte(address);
    }

    data[length - 1] = spiTransferByte(0);

    disableDevice();

    return true;
}

bool spiWriteByteAtAddress(const uint8_t address, const uint8_t byte) {
    enableDevice();
    spiTransferByte(address);
    spiTransferByte(byte);
    disableDevice();

    return true;
}

bool spiWriteDataAtAddress(const uint8_t address, const uint8_t * const data, const uint8_t length) {
    enableDevice();
    spiTransferByte(address);

    for (uint8_t i = 0; i < length - 1; ++i) {
        spiTransferByte(data[i]);
    }

    disableDevice();

    return true;
}