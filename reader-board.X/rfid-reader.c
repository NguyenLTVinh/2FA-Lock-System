#include "rfid-reader.h"
#include <stddef.h>

#include "mfrc522.h"

#define RFID_READER_BUFFER_SIZE 32

typedef enum PiccResult {
    PICC_RESULT_OK,
    PICC_RESULT_COLLISION,
    PICC_RESULT_BUFFER_TOO_SHORT,
    PICC_RESULT_ERROR,
    PICC_RESULT_MIFARE_NACK,
    PICC_RESULT_CRC_WRONG,
    PICC_RESULT_TIMEOUT,
    PICC_RESULT_INVALID,
    PICC_RESULT_INTERNAL_ERROR,
} PiccResult;

// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 Sefttest
#define MFRC522_SELF_TEST_REFERENCE_LENGTH 64

const uint8_t MFRC522_firmware_referenceV0_0[MFRC522_SELF_TEST_REFERENCE_LENGTH] = {
    0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
    0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
    0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
    0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
    0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
    0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
    0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
    0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 Self test
const uint8_t MFRC522_firmware_referenceV1_0[MFRC522_SELF_TEST_REFERENCE_LENGTH] = {
    0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
    0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
    0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
    0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
    0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
    0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
    0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
    0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 Self test
const uint8_t MFRC522_firmware_referenceV2_0[MFRC522_SELF_TEST_REFERENCE_LENGTH] = {
    0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
    0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
    0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
    0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
    0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
    0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
    0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
    0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};

/**
 * Sets the bits given in mask in the register at register_address.
 */
void setReaderRegisterBitmask(
        uint8_t register_address, ///< The register to update. One of the PCD_Register enums.
        uint8_t bitmask ///< The bits to set.
        ) {
    const uint8_t register_value = mfrc522ReadByteAtAddress(register_address);
    mfrc522WriteByteAtAddress(register_address, register_value | bitmask);
}

/**
 * Clears the bits given in mask from register reg.
 */
void clearReaderRegisterBitmask(
        const uint8_t reg, ///< The register to update. One of the PCD_Register enums.
        const uint8_t mask ///< The bits to clear.
        ) {
    const uint8_t tmp = mfrc522ReadByteAtAddress(reg);
    mfrc522WriteByteAtAddress(reg, tmp & (~mask)); // clear bit mask
}

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return PICC_RESULT_OK on success, PICC_RESULT_TIMEOUT otherwise.
 */
PiccResult computeReaderCrc(
        uint8_t *data, ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
        uint8_t length, ///< In: The number of bytes to transfer.
        uint8_t *result ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
        ) {
    mfrc522WriteByteAtAddress(CommandReg, PCD_Idle); // Stop any active command.
    mfrc522WriteByteAtAddress(DivIrqReg, 0x04); // Clear the CRCIRq interrupt request bit
    setReaderRegisterBitmask(FIFOLevelReg, 0x80); // FlushBuffer = 1, FIFO initialization
    mfrc522WriteDataAtAddress(FIFODataReg, data, length); // Write data to the FIFO
    mfrc522WriteByteAtAddress(CommandReg, PCD_CalcCRC); // Start the calculation

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
    uint16_t i = 5000;
    uint8_t n;
    while (1) {
        n = mfrc522ReadByteAtAddress(DivIrqReg); // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        if (n & 0x04) { // CRCIRq bit set - calculation done
            break;
        }
        if (--i == 0) { // The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
            return PICC_RESULT_TIMEOUT;
        }
    }
    mfrc522WriteByteAtAddress(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.

    // Transfer the result from the registers to the result buffer
    result[0] = mfrc522ReadByteAtAddress(CRCResultRegL);
    result[1] = mfrc522ReadByteAtAddress(CRCResultRegH);
    return PICC_RESULT_OK;
}

/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
bool performSelfTest() {
    // 2. Clear the internal buffer by writing 25 bytes of 00h
    uint8_t ZEROES[25] = {0x00};
    setReaderRegisterBitmask(FIFOLevelReg, 0x80); // flush the FIFO buffer
    mfrc522WriteDataAtAddress(FIFODataReg, ZEROES, 25); // write 25 bytes of 00h to FIFO
    mfrc522WriteByteAtAddress(CommandReg, PCD_Mem); // transfer to internal buffer

    // 3. Enable self-test
    mfrc522WriteByteAtAddress(AutoTestReg, 0x09);

    // 4. Write 00h to FIFO buffer
    mfrc522WriteByteAtAddress(FIFODataReg, 0x00);

    // 5. Start self-test by issuing the CalcCRC command
    mfrc522WriteByteAtAddress(CommandReg, PCD_CalcCRC);

    // 6. Wait for self-test to complete
    for (uint16_t i = 0; i < 0xFF; i++) {
        const uint8_t n = mfrc522ReadByteAtAddress(DivIrqReg); // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        if (n & 0x04) { // CRCIRq bit set - calculation done
            break;
        }
    }
    mfrc522WriteByteAtAddress(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.

    // 7. Read out resulting 64 bytes from the FIFO buffer.
    uint8_t result[MFRC522_SELF_TEST_REFERENCE_LENGTH];
    mfrc522ReadDataAtAddress(FIFODataReg, result, MFRC522_SELF_TEST_REFERENCE_LENGTH, 0);

    // Auto self-test done
    // Reset AutoTestReg register to be 0 again. Required for normal operation.
    mfrc522WriteByteAtAddress(AutoTestReg, 0x00);

    // Determine firmware version (see section 9.3.4.8 in spec)
    const uint8_t version = mfrc522ReadByteAtAddress(VersionReg);

    // Pick the appropriate reference values
    const uint8_t *reference;
    switch (version) {
        case 0x90: // Version 0.0
            reference = MFRC522_firmware_referenceV0_0;
            break;
        case 0x91: // Version 1.0
            reference = MFRC522_firmware_referenceV1_0;
            break;
        case 0x92: // Version 2.0
            reference = MFRC522_firmware_referenceV2_0;
            break;
        default: // Unknown version
            return false; // abort test
    }

    // Verify that the results match up to our expectations
    for (uint8_t i = 0; i < 64; i++) {
        if (result[i] != reference[i]) {
            return false;
        }
    }

    // Test passed; all is good.
    return true;
}

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void resetReader() {
    mfrc522WriteByteAtAddress(CommandReg, PCD_SoftReset); // Issue the SoftReset command.
    // The datasheet does not mention how long the SoftRest command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    _delay_ms(50);
    // Wait for the PowerDown bit in CommandReg to be cleared
    while (mfrc522ReadByteAtAddress(CommandReg) & (1 << 4)) {
        // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
    }
}

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void enableReaderAntenna() {
    const uint8_t value = mfrc522ReadByteAtAddress(TxControlReg);
    if ((value & 0x03) != 0x03) {
        mfrc522WriteByteAtAddress(TxControlReg, value | 0x03);
    }
}

/**
 * Initializes the MFRC522 chip.
 */
void initializeReader(void) {
    initializeMfrc522();
    //	reset_pin_port->DIRCLR = reset_pin_bitmask;
    //    const bool is_in_power_down_mode = (reset_pin_port->IN & reset_pin_bitmask) == 0;
    //	reset_pin_port->DIRSET = reset_pin_bitmask;

    //	if (is_in_power_down_mode) {	// The MFRC522 chip is in power down mode.
    //        // Exit power down mode. This triggers a hard reset.
    //        reset_pin_port->OUTSET = reset_pin_bitmask;
    //		// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    //		_delay_ms(50);
    //	} else {
    // Perform a soft reset
    resetReader();
    //	}

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    mfrc522WriteByteAtAddress(TModeReg, 0x80); // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    mfrc522WriteByteAtAddress(TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
    mfrc522WriteByteAtAddress(TReloadRegH, 0x03); // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    mfrc522WriteByteAtAddress(TReloadRegL, 0xE8);

    mfrc522WriteByteAtAddress(TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    mfrc522WriteByteAtAddress(ModeReg, 0x3D); // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    enableReaderAntenna(); // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}

PiccResult PCD_CommunicateWithPICC(const uint8_t command, ///< The command to execute. One of the PCD_Command enums.
        const uint8_t waitIRq, ///< The bits in the ComIrqReg register that signals successful completion of the command.
        const uint8_t * const sendData, ///< Pointer to the data to transfer to the FIFO.
        const uint8_t sendLen, ///< Number of bytes to transfer to the FIFO.
        uint8_t * const backData, ///< NULL or pointer to buffer if data should be read back after executing the command.
        uint8_t * const backLen, ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        uint8_t * const validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
        const uint8_t rxAlign, ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
        const uint8_t checkCRC ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
        ) {
    uint8_t n;
    unsigned int i;

    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    mfrc522WriteByteAtAddress(CommandReg, PCD_Idle); // Stop any active command.
    mfrc522WriteByteAtAddress(ComIrqReg, 0x7F); // Clear all seven interrupt request bits
    setReaderRegisterBitmask(FIFOLevelReg, 0x80); // FlushBuffer = 1, FIFO initialization
    mfrc522WriteDataAtAddress(FIFODataReg, sendData, sendLen); // Write sendData to the FIFO
    mfrc522WriteByteAtAddress(BitFramingReg, bitFraming); // Bit adjustments
    mfrc522WriteByteAtAddress(CommandReg, command); // Execute the command
    if (command == PCD_Transceive) {
        setReaderRegisterBitmask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86�s.
    i = 2000;
    while (1) {
        n = mfrc522ReadByteAtAddress(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
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
    const uint8_t errorRegValue = mfrc522ReadByteAtAddress(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) { // BufferOvfl ParityErr ProtocolErr
        return PICC_RESULT_ERROR;
    }

    // Tell about collisions
    if (errorRegValue & 0x08) { // CollErr
        return PICC_RESULT_COLLISION;
    }

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen) {
        n = mfrc522ReadByteAtAddress(FIFOLevelReg); // Number of bytes in the FIFO
        if (n > *backLen) {
            return PICC_RESULT_BUFFER_TOO_SHORT;
        }
        *backLen = n; // Number of bytes returned
        mfrc522ReadDataAtAddress(FIFODataReg, backData, n, rxAlign); // Get received data from FIFO
        const uint8_t _validBits = mfrc522ReadByteAtAddress(ControlReg) & 0x07; // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits) {
            *validBits = _validBits;
        }

        // Perform CRC_A validation if requested.
        if (checkCRC) {
            // In this case a MIFARE Classic NAK is not OK.
            if (*backLen == 1 && _validBits == 4) {
                return PICC_RESULT_MIFARE_NACK;
            }
            // We need at least the CRC_A value and all 8 bits of the last byte must be received.
            if (*backLen < 2 || _validBits != 0) {
                return PICC_RESULT_CRC_WRONG;
            }
            // Verify CRC_A - do our own calculation and store the control in controlBuffer.
            uint8_t controlBuffer[2];
            PiccResult status = computeReaderCrc(&backData[0], *backLen - 2, &controlBuffer[0]);
            if (status != PICC_RESULT_OK) {
                return status;
            }
            if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
                return PICC_RESULT_CRC_WRONG;
            }
        }
    }

    return PICC_RESULT_OK;
}

PiccResult PCD_TransceiveDataImpl(const uint8_t * const sendData, ///< Pointer to the data to transfer to the FIFO.
        const uint8_t sendLen, ///< Number of bytes to transfer to the FIFO.
        uint8_t * const backData, ///< NULL or pointer to buffer if data should be read back after executing the command.
        uint8_t * const backLen, ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        uint8_t * const validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
        const uint8_t rxAlign, ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
        const bool checkCRC ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
        ) {
    const uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

PiccResult PCD_TransceiveDataWithAlignment(const uint8_t * const sendData, ///< Pointer to the data to transfer to the FIFO.
        const uint8_t sendLen, ///< Number of bytes to transfer to the FIFO.
        uint8_t * const backData, ///< NULL or pointer to buffer if data should be read back after executing the command.
        uint8_t * const backLen, ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        uint8_t * const validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
        const uint8_t rxAlign ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
        ) {
    return PCD_TransceiveDataImpl(sendData, sendLen, backData, backLen, validBits, rxAlign, false);
}

PiccResult PCD_TransceiveData(const uint8_t * const sendData, ///< Pointer to the data to transfer to the FIFO.
        const uint8_t sendLen, ///< Number of bytes to transfer to the FIFO.
        uint8_t * const backData, ///< NULL or pointer to buffer if data should be read back after executing the command.
        uint8_t * const backLen, ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
        uint8_t * const validBits ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
        ) {
    return PCD_TransceiveDataWithAlignment(sendData, sendLen, backData, backLen, validBits, 0);
}

PiccResult piccRequestA(uint8_t * const buffer, uint8_t buffer_size) {
    if (buffer == NULL || buffer_size < 2) {
        // The ATQA response is 2 bytes long
        return PICC_RESULT_BUFFER_TOO_SHORT;
    }

    clearReaderRegisterBitmask(CollReg, 0x80);
    // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
    uint8_t valid_bits = 7;
    const uint8_t command = PICC_CMD_REQA;
    PiccResult status = PCD_TransceiveData(&command, 1, buffer, &buffer_size, &valid_bits);
    if (status != PICC_RESULT_OK) {
        return status;
    }
    if (buffer_size != 2 || valid_bits != 0) { // ATQA must be exactly 16 bits.
        return PICC_RESULT_ERROR;
    }
    return PICC_RESULT_OK;
}

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
PiccResult piccSelect(
        Uid * const uid ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
        ) {
    uint8_t validBits = 0;

    bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    PiccResult result;
    uint8_t count;
    uint8_t index;
    uint8_t uidIndex; // The first index in uid->uidByte[] that is used in the current Cascade Level.
    uint8_t buffer[9]; // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t bufferUsed; // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rxAlign; // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits; // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
    //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
    //		Byte 3: UID-data
    //		Byte 4: UID-data
    //		Byte 5: UID-data
    //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
    //		Byte 7: CRC_A
    //		Byte 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
    //		========	=============	=====	=====	=====	=====
    //		 4 bytes		1			uid0	uid1	uid2	uid3
    //		 7 bytes		1			CT		uid0	uid1	uid2
    //						2			uid3	uid4	uid5	uid6
    //		10 bytes		1			CT		uid0	uid1	uid2
    //						2			CT		uid3	uid4	uid5
    //						3			uid6	uid7	uid8	uid9

    // Sanity checks
    if (validBits > 80) {
        return PICC_RESULT_INVALID;
    }

    // Prepare MFRC522
    clearReaderRegisterBitmask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    bool uidComplete = false;
    while (!uidComplete) {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel) {
            case 1:
                buffer[0] = PICC_CMD_SEL_CL1;
                uidIndex = 0;
                useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
                break;

            case 2:
                buffer[0] = PICC_CMD_SEL_CL2;
                uidIndex = 3;
                useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
                break;

            case 3:
                buffer[0] = PICC_CMD_SEL_CL3;
                uidIndex = 6;
                useCascadeTag = false; // Never used in CL3.
                break;

            default:
                return PICC_RESULT_INTERNAL_ERROR;
                break;
        }

        // The number of known UID bits in the current Cascade Level.
        int8_t currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0) {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag) {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
        if (bytesToCopy) {
            uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes) {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++) {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag) {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        bool selectDone = false;
        do {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = computeReaderCrc(buffer, 7, &buffer[7]);
                if (result != PICC_RESULT_OK) {
                    return result;
                }
                txLastBits = 0; // 0 => All 8 bits are valid.
                bufferUsed = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer = &buffer[6];
                responseLength = 3;
            } else { // This is an ANTICOLLISION.
                //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                txLastBits = currentLevelKnownBits % 8;
                count = currentLevelKnownBits / 8; // Number of whole bytes in the UID part.
                index = 2 + count; // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                bufferUsed = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer = &buffer[index];
                responseLength = sizeof (buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
            mfrc522WriteByteAtAddress(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_TransceiveDataWithAlignment(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
            if (result == PICC_RESULT_COLLISION) { // More than one PICC in the field => collision.
                const uint8_t valueOfCollReg = mfrc522ReadByteAtAddress(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20) { // CollPosNotValid
                    return PICC_RESULT_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0) {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
                    return PICC_RESULT_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count = (currentLevelKnownBits - 1) % 8; // The bit to modify
                index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index] |= (1 << count);
            } else if (result != PICC_RESULT_OK) {
                return result;
            } else { // STATUS_OK
                if (currentLevelKnownBits >= 32) { // This was a SELECT.
                    selectDone = true; // No more anticollision 
                    // We continue below outside the while.
                } else { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } while (!selectDone);

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++) {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return PICC_RESULT_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = computeReaderCrc(responseBuffer, 1, &buffer[2]);
        if (result != PICC_RESULT_OK) {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
            return PICC_RESULT_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        } else {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return PICC_RESULT_OK;
}

bool readRfidCard(Uid * const card) {
    uint8_t bufferAtqa[RFID_READER_BUFFER_SIZE];

    const PiccResult requestAResult = piccRequestA(bufferAtqa, RFID_READER_BUFFER_SIZE);
    if (requestAResult != PICC_RESULT_OK && requestAResult != PICC_RESULT_COLLISION) {
        return false;
    }

    const PiccResult selectResult = piccSelect(card);
    return selectResult == PICC_RESULT_OK;
}
