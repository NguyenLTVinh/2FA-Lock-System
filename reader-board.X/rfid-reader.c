#include "rfid-reader.h"
#include <stddef.h>

#include "../common.X/two_wire_interface.h"

#define RFID_READER_BUFFER_SIZE 2

const uint8_t RFID_READER_I2C_ADDRESS = 0x28;

typedef enum PiccResult {
    PICC_RESULT_OK,
    PICC_RESULT_COLLISION,
    PICC_RESULT_BUFFER_TOO_SHORT,
    PICC_RESULT_ERROR,
    PICC_RESULT_MIFARE_NACK,
    PICC_RESULT_CRC_WRONG,
    PICC_RESULT_TIMEOUT,
} PiccResult;

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void writeReaderRegisterValue(
        const uint8_t register_address, ///< The register to write to. One of the PCD_Register enums.
        const uint8_t value ///< The value to write.
        ) {
    uint8_t buffer[2] = {register_address, value};
    twiWriteBytes(RFID_READER_I2C_ADDRESS, buffer, 2);
}

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void writeReaderRegisterData(
        uint8_t register_address, ///< The register to write to. One of the PCD_Register enums.
        uint8_t *data, ///< The data to write. Byte array.
        uint8_t length ///< The number of bytes to write to the register
        ) {
    twiWriteBytesToDeviceAddress(RFID_READER_I2C_ADDRESS, register_address, data, length);
}

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegister(
        const uint8_t reg, ///< The register to read from. One of the PCD_Register enums.
        const uint8_t count, ///< The number of bytes to read
        const uint8_t *values, ///< Byte array to store the values in.
        const uint8_t rxAlign ///< Only bit positions rxAlign..7 in values[0] are updated.
        ) {
    if (count == 0) {
        return;
    }
    uint8_t index = 0; // Index in values array.
    _TwoWireInstance->beginTransmission(_chipAddress);
    _TwoWireInstance->write(regist);
    _TwoWireInstance->endTransmission();
    _TwoWireInstance->requestFrom(_chipAddress, _count);
    while (_TwoWireInstance->available()) {
        if (index == 0 && rxAlign) { // Only update bit positions rxAlign..7 in values[0]
            // Create bit mask for bit positions rxAlign..7
            uint8_t mask = 0;
            for (uint8_t i = rxAlign; i <= 7; i++) {
                mask |= (1 << i);
            }
            // Read value and tell that we want to read the same address again.
            byte value = _TwoWireInstance->read();
            // Apply mask to both current value of values[0] and the new data in value.
            values[0] = (values[index] & ~mask) | (value & mask);
        } else { // Normal case
            values[index] = _TwoWireInstance->read();
        }
        ++index;
    }
}

/**
 * Sets the bits given in mask in the register at register_address.
 */
void setReaderRegisterBitmask(
        uint8_t register_address, ///< The register to update. One of the PCD_Register enums.
        uint8_t bitmask ///< The bits to set.
        ) {
    const uint8_t register_value = readReaderRegisterValue(register_address);
    writeReaderRegisterValue(register_address, register_value | bitmask);
}

PiccResult PCD_CommunicateWithPICC(uint8_t command, ///< The command to execute. One of the PCD_Command enums.
        uint8_t waitIRq, ///< The bits in the ComIrqReg register that signals successful completion of the command.
        uint8_t *sendData, ///< Pointer to the data to transfer to the FIFO.
        uint8_t sendLen, ///< Number of bytes to transfer to the FIFO.
        uint8_t *backData, ///< NULL or pointer to buffer if data should be read back after executing the command.
        uint8_t *backLen, ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        uint8_t *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
        uint8_t rxAlign, ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
        uint8_t checkCRC ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
        ) {
    uint8_t n, _validBits;
    unsigned int i;

    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    writeReaderRegisterValue(CommandReg, PCD_Idle); // Stop any active command.
    writeReaderRegisterValue(ComIrqReg, 0x7F); // Clear all seven interrupt request bits
    PCD_SetRegisterBitMask(FIFOLevelReg, 0x80); // FlushBuffer = 1, FIFO initialization
    writeReaderRegisterData(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
    writeReaderRegisterValue(BitFramingReg, bitFraming); // Bit adjustments
    writeReaderRegisterValue(CommandReg, command); // Execute the command
    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86�s.
    i = 2000;
    while (1) {
        n = PCD_ReadRegister(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) { // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01) { // Timer interrupt - nothing received in 25ms
            return PICC_RESULT_TIMEOUT;
        }
        if (--i == 0) { // The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
            return PICC_RESULT_TIMEOUT;
        }
    }

    // Stop now if any errors except collisions were detected.
    byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) { // BufferOvfl ParityErr ProtocolErr
        return PICC_RESULT_ERROR;
    }

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        n = PCD_ReadRegister(FIFOLevelReg); // Number of bytes in the FIFO
        if (n > *backLen) {
            return PICC_RESULT_BUFFER_TOO_SHORT;
        }
        *backLen = n; // Number of bytes returned
        PCD_ReadRegister(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO
        _validBits = PCD_ReadRegister(ControlReg) & 0x07; // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits) {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08) { // CollErr
        return PICC_RESULT_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC) {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4) {
            return PICC_RESULT_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*backLen < 2 || _validBits != 0) {
            return PICC_RESULT_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        byte controlBuffer[2];
        MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != PICC_RESULT_OK) {
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
            return STATUS_CRC_WRONG;
        }
    }

    return PICC_RESULT_OK;
}

PiccResulte PCD_TransceiveData(byte *sendData, ///< Pointer to the data to transfer to the FIFO.
        byte sendLen, ///< Number of bytes to transfer to the FIFO.
        byte *backData, ///< NULL or pointer to buffer if data should be read back after executing the command.
        byte *backLen, ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        byte *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
        byte rxAlign, ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
        bool checkCRC ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
        ) {
    byte waitIRq = 0x30; // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

PiccResult piccRequestA(uint8_t * const buffer, const uint8_t buffer_size, const uint8_t * const uid) {
    if (buffer == NULL || buffer_size < 2) {
        // The ATQA response is 2 bytes long
        return PICC_RESULT_BUFFER_TOO_SHORT;
    }

    PCD_ClearRegisterBitMask(CollReg, 0x80);
    // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
    uint8_t valid_bits = 7;
    PiccResult status = PCD_TransceiveData(&PICC_CMD_REQA, 1, buffer, buffer_size, &valid_bits);
    if (status != PICC_RESULT_OK) {
        return status;
    }
    if (buffer_size != 2 || valid_bits != 0) { // ATQA must be exactly 16 bits.
        return PICC_RESULT_ERROR;
    }
    return PICC_RESULT_OK;
}

bool readRfidCard(uint8_t * const card_contents) {
    uint8_t bufferAtqa[RFID_READER_BUFFER_SIZE];

    uint8_t uid;

    const PiccResult requestAResult = piccRequestA(bufferAtqa, RFID_BUFFER_SIZE, &uid);
    if (requestAResult != PICC_RESULT_OK && requestAResult != PICC_RESULT_COLLISION) {
        return false;
    }

    const PiccResult selectResult = piccSelect(&uid, &bufferAtqa, RFID_BUFFER_SIZE);
    return selectResult = PICC_RESULT_OK;
}
