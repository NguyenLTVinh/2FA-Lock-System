#include "cryptography.h"
#include "two_wire_interface.h"

#include "globals.h"
#include <util/delay.h>
#include <string.h>

const uint8_t ATRCC508A_I2C_ADDRESS = 0x60;

#define RESPONSE_COUNT_SIZE  1
#define RESPONSE_SIGNAL_SIZE 1
#define CRC_SIZE             2

#define ATRCC508A_PROTOCOL_FIELD_COMMAND 0
#define ATRCC508A_PROTOCOL_FIELD_LENGTH  1
#define ATRCC508A_PROTOCOL_FIELD_OPCODE  2
#define ATRCC508A_PROTOCOL_FIELD_PARAM1  3
#define ATRCC508A_PROTOCOL_FIELD_PARAM2  4
#define ATRCC508A_PROTOCOL_FIELD_DATA    6

#define ATRCC508A_PROTOCOL_FIELD_SIZE_COMMAND 1
#define ATRCC508A_PROTOCOL_FIELD_SIZE_LENGTH  1
#define ATRCC508A_PROTOCOL_FIELD_SIZE_OPCODE  1
#define ATRCC508A_PROTOCOL_FIELD_SIZE_PARAM1  1
#define ATRCC508A_PROTOCOL_FIELD_SIZE_PARAM2  2
#define ATRCC508A_PROTOCOL_FIELD_SIZE_CRC     CRC_SIZE
#define ATRCC508A_PROTOCOL_OVERHEAD (ATRCC508A_PROTOCOL_FIELD_SIZE_COMMAND + ATRCC508A_PROTOCOL_FIELD_SIZE_LENGTH + ATRCC508A_PROTOCOL_FIELD_SIZE_OPCODE + ATRCC508A_PROTOCOL_FIELD_SIZE_PARAM1 + ATRCC508A_PROTOCOL_FIELD_SIZE_PARAM2 + ATRCC508A_PROTOCOL_FIELD_SIZE_CRC)

#define ATRCC508A_SUCCESSFUL_TEMPKEY 0x00
#define ATRCC508A_SUCCESSFUL_VERIFY  0x00
#define ATRCC508A_SUCCESSFUL_LOCK    0x00

#define WORD_ADDRESS_VALUE_COMMAND 	0x03

// COMMANDS (aka "opcodes" in the datasheet)
#define COMMAND_OPCODE_LOCK 	0x17 // Lock configuration and/or Data and OTP zones
#define COMMAND_OPCODE_GENKEY 	0x40 // Creates a key (public and/or private) and stores it in a memory key slot
#define COMMAND_OPCODE_NONCE 	0x16
#define COMMAND_OPCODE_SIGN 	0x41 // Create an ECC signature with contents of TempKey and designated key slot
#define COMMAND_OPCODE_VERIFY 	0x45 // takes an ECDSA <R,S> signature and verifies that it is correctly generated from a given message and public key

#define NONCE_MODE_PASSTHROUGH		 0b00000011 // Operate in pass-through mode and Write TempKey with NumIn. datasheet pg 79
#define SIGN_MODE_TEMPKEY			 0b10000000 // The message to be signed is in TempKey. datasheet pg 85
#define VERIFY_MODE_EXTERNAL		 0b00000010 // Use an external public key for verification, pass to command as data post param2, ds pg 89
#define VERIFY_MODE_STORED			 0b00000000 // Use an internally stored public key for verification, param2 = keyID, ds pg 89
#define VERIFY_PARAM2_KEYTYPE_ECC 	 0x0004 // When verify mode external, param2 should be KeyType, ds pg 89
#define VERIFY_PARAM2_KEYTYPE_NONECC 0x0007 // When verify mode external, param2 should be KeyType, ds pg 89

// Data sheet section 9.1.3 Command Opcodes, Short Descriptions, and Execution Times
const uint8_t NONCE_COMMAND_MAXIMUM_EXECUTION_TIME_MILLISECONDS = 7;
const uint8_t SIGN_COMMAND_MAXIMUM_EXECUTION_TIME_MILLISECONDS = 50;

// Data request constants
//const uint8_t ATRCC508A_MAX_REQUEST_SIZE = 32;
//const uint8_t ATRCC508A_MAX_RETRIES = 20;

const uint8_t SIGN_COMMAND_RESPONSE_LENGTH = RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE;

const uint8_t RESPONSE_COUNT_INDEX = 0;
const uint8_t RESPONSE_SIGNAL_INDEX = RESPONSE_COUNT_SIZE;

/** \brief This function calculates CRC.
 * 
 * Original source: https://ww1.microchip.com/downloads/en/AppNotes/Atmel-8936-CryptoAuth-Data-Zone-CRC-Calculation-ApplicationNote.pdf Section 2 CRC Calculation Code.
 * Adapted to return a 16-bit value instead of write to a pointer to two 8-bit values.
 *
 * \param[in] length number of bytes in buffer
 * \param[in] pointer to data for which CRC should be calculated
 * 
 */
uint16_t atca_calculate_crc(const uint8_t length, const uint8_t * const data) {
    uint8_t counter;
    uint16_t crc_register = 0;
    uint16_t polynom = 0x8005;
    uint8_t shift_register;
    uint8_t data_bit, crc_bit;
    for (counter = 0; counter < length; counter++) {
        for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
            data_bit = (data[counter] & shift_register) ? 1 : 0;
            crc_bit = crc_register >> 15;
            crc_register <<= 1;
            if (data_bit != crc_bit)
                crc_register ^= polynom;
        }
    }
    return (crc_register & 0x00FF) | (crc_register & 0xFF00);
}

bool sendCommand(const uint8_t command_opcode, const uint8_t param1, const uint16_t param2, const uint8_t * const data, const size_t length_of_data) {
    // Build packet array (total_transmission) to send a communication to IC, with opcode COMMAND
    // It expects to see: word address, count, command opcode, param1, param2, data (optional), CRC[0], CRC[1]
    uint8_t total_transmission_length;
    uint8_t total_transmission[UINT8_MAX];

    /* Validate no integer overflow */
    if (length_of_data > UINT8_MAX - ATRCC508A_PROTOCOL_OVERHEAD) {
        return false;
    }

    total_transmission_length = length_of_data + ATRCC508A_PROTOCOL_OVERHEAD;

    total_transmission[ATRCC508A_PROTOCOL_FIELD_COMMAND] = WORD_ADDRESS_VALUE_COMMAND; // word address value (type command)
    total_transmission[ATRCC508A_PROTOCOL_FIELD_LENGTH] = total_transmission_length - ATRCC508A_PROTOCOL_FIELD_SIZE_LENGTH; // count, does not include itself, so "-1"
    total_transmission[ATRCC508A_PROTOCOL_FIELD_OPCODE] = command_opcode;
    total_transmission[ATRCC508A_PROTOCOL_FIELD_PARAM1] = param1;
    memcpy(&total_transmission[ATRCC508A_PROTOCOL_FIELD_PARAM2], &param2, sizeof (param2)); // append param2
    memcpy(&total_transmission[ATRCC508A_PROTOCOL_FIELD_DATA], &data[0], length_of_data); // append data

    // update CRCs
    // copy over just what we need to CRC starting at index 1
    const uint8_t transmission_without_crc_length = total_transmission_length - (ATRCC508A_PROTOCOL_FIELD_SIZE_COMMAND + ATRCC508A_PROTOCOL_FIELD_SIZE_CRC);
    const uint16_t crc = atca_calculate_crc(transmission_without_crc_length, &total_transmission[ATRCC508A_PROTOCOL_FIELD_LENGTH]);

    total_transmission[total_transmission_length - ATRCC508A_PROTOCOL_FIELD_SIZE_CRC] = (uint8_t) (crc & 0xFF);
    total_transmission[total_transmission_length - ATRCC508A_PROTOCOL_FIELD_SIZE_CRC + 1] = (uint8_t) (crc >> 8);

    return twiWriteBytes(ATRCC508A_I2C_ADDRESS, total_transmission, total_transmission_length) == total_transmission_length;
}

uint8_t receiveResponseData(uint8_t * const buffer, const uint8_t length) {
    const uint8_t bytes_read = twiReadBytes(ATRCC508A_I2C_ADDRESS, buffer, length);

    //    while (total_bytes_read < length) {
    //        // Pull in data 32 bytes at at time to avoid overflow on atmega328.
    //        const uint8_t request_amount = length - total_bytes_read > ATRCC508A_MAX_REQUEST_SIZE
    //                ? ATRCC508A_MAX_REQUEST_SIZE
    //                : length - total_bytes_read;
    //
    //        // Request bytes from peripheral
    //        const uint8_t bytes_read = twiReadBytes(ATRCC508A_I2C_ADDRESS, buffer + total_bytes_read, request_amount);
    //
    //        ++request_attempts;
    //        total_bytes_read += bytes_read;
    //
    //        if (request_attempts == ATRCC508A_MAX_RETRIES) {
    //            // this probably means that the device is not responding.
    //            return total_bytes_read;
    //        }
    //    }

    if (buffer[RESPONSE_COUNT_INDEX] != bytes_read) {
        // The byte at the RESPONSE_COUNT_INDEX index should be equal to the actual number of bytes in the message
        return 0;
    }

    // Check CRC
    const uint16_t crc = atca_calculate_crc(bytes_read - CRC_SIZE, buffer);
    if ((buffer[bytes_read - (CRC_SIZE - 1)] != ((uint8_t) crc >> 8))
            || (buffer[bytes_read - CRC_SIZE] != ((uint8_t) crc & 0xFF))) {
        return 0;
    }

    return bytes_read;
}

bool signData(uint8_t input_and_output[SIGN_COMMAND_RESPONSE_LENGTH]) {
    if (!sendCommand(COMMAND_OPCODE_NONCE, NONCE_MODE_PASSTHROUGH, 0x0000, input_and_output, 32)) {
        return false;
    }

    // note, param2 is 0x0000 (and param1 is PASSTHROUGH), so OutData will be just a single byte of zero upon completion.
    // see ds pg 77 for more info

    _delay_ms(NONCE_COMMAND_MAXIMUM_EXECUTION_TIME_MILLISECONDS);

    // Now let's read back from the IC.
    if (receiveResponseData(input_and_output, SIGN_COMMAND_RESPONSE_LENGTH) != SIGN_COMMAND_RESPONSE_LENGTH) {
        return false;
    }

    // Ensure that "return value" is successful
    if (input_and_output[RESPONSE_SIGNAL_INDEX] != ATRCC508A_SUCCESSFUL_TEMPKEY) {
        return false;
    }

    return true;
}
