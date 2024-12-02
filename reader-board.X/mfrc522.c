#include "mfrc522.h"

#include "spi.h"

const uint8_t MFRC522_ADDRESS_READ_FLAG = 0x80;
const uint8_t MFRC522_ADDRESS_MASK = 0x7E;

inline void initializeMfrc522(void) {
    initializeSpi();
}

inline uint8_t mfrc522ReadByteAtAddress(const uint8_t address) {
    return spiReadByteAtAddress(((address << 1) & MFRC522_ADDRESS_MASK) | MFRC522_ADDRESS_READ_FLAG);
}

bool mfrc522ReadDataAtAddress(const uint8_t address, uint8_t * const data, const uint8_t length, const uint8_t receive_alignment) {
    const bool success = spiReadDataAtAddress(((address << 1) & MFRC522_ADDRESS_MASK) | MFRC522_ADDRESS_READ_FLAG, data, length);
    
    if (receive_alignment) {
        // Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        uint8_t mask = 0;
        for (uint8_t i = receive_alignment; i <= 7; i++) {
            mask |= (1 << i);
        }
        data[0] &= mask;
    }
    
    return success;
}

inline bool mfrc522WriteByteAtAddress(const uint8_t address, const uint8_t byte) {
    return spiWriteByteAtAddress((address << 1) & MFRC522_ADDRESS_MASK, byte);
}

inline bool mfrc522WriteDataAtAddress(const uint8_t address, const uint8_t * const data, const uint8_t length) {
    return spiWriteDataAtAddress((address << 1) & MFRC522_ADDRESS_MASK, data, length);
}