// The code in this file is heavily inspired by the SparkFun ATECCX08A Arduino Library found at https://github.com/sparkfun/SparkFun_ATECCX08a_Arduino_Library/tree/master
#include "two_wire_interface.h"
#include "globals.h"
#include <util/delay.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#define ATECC508A_I2C_ADDRESS 0x60

const uint8_t ATECC508A_MAX_REQUEST_SIZE = 32;
const uint8_t ATRCC508A_MAX_READ_REQUEST_RETRIES = 20;

#define PUBLIC_KEY_SIZE 64
#define SIGNATURE_SIZE  64

#define RESPONSE_COUNT_SIZE  1
#define RESPONSE_SIGNAL_SIZE 1
#define CRC_SIZE             2

const uint8_t ATRCC508A_SUCCESSFUL_WAKEUP = 0x11;

#define RESPONSE_COUNT_INDEX 0
#define RESPONSE_SIGNAL_INDEX RESPONSE_COUNT_SIZE

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

// WORD ADDRESS VALUES
// These are sent in any write sequence to the IC.
// They tell the IC what we are going to do: Reset, Sleep, Idle, Command.
#define WORD_ADDRESS_VALUE_COMMAND 0x03
#define WORD_ADDRESS_VALUE_IDLE 0x02

// COMMANDS (aka "opcodes" in the datasheet)
#define COMMAND_OPCODE_LOCK 	0x17 // Lock configuration and/or Data and OTP zones
#define COMMAND_OPCODE_GENKEY 	0x40 // Creates a key (public and/or private) and stores it in a memory key slot
#define COMMAND_OPCODE_NONCE 	0x16
#define COMMAND_OPCODE_SIGN 	0x41 // Create an ECC signature with contents of TempKey and designated key slot
#define COMMAND_OPCODE_VERIFY 	0x45 // takes an ECDSA <R,S> signature and verifies that it is correctly generated from a given message and public key

#define NONCE_MODE_PASSTHROUGH		 0b00000011 // Operate in pass-through mode and Write TempKey with NumIn. datasheet pg 79
#define SIGN_MODE_TEMPKEY			 0b10000000 // The message to be signed is in TempKey. datasheet pg 85
#define VERIFY_MODE_EXTERNAL		 0b00000010 // Use an external public key for verification, pass to command as data post param2, ds pg 89
#define VERIFY_PARAM2_KEYTYPE_ECC 	0x0004 // When verify mode external, param2 should be KeyType, ds pg 89

#define ATRCC508A_SUCCESSFUL_TEMPKEY 0x00
#define ATRCC508A_SUCCESSFUL_VERIFY  0x00
#define ATRCC508A_SUCCESSFUL_LOCK    0x00

/** \brief This function calculates CRC.
 * 
 * Original source: https://ww1.microchip.com/downloads/en/AppNotes/Atmel-8936-CryptoAuth-Data-Zone-CRC-Calculation-ApplicationNote.pdf Section 2 CRC Calculation Code.
 * Adapted to return a 16-bit value instead of write to a pointer to two 8-bit values.
 *
 * \param[in] length number of bytes in buffer
 * \param[in] pointer to data for which CRC should be calculated
 * 
 */
uint16_t computeAtecc508aCrc(const uint8_t length, const uint8_t * const data) {
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

bool receiveAtecc508aResponseData(uint8_t * const data, const uint8_t length) {
    // keep track of how many times we've attempted to request, to break out if necessary
    uint8_t request_attempts = 0;
    uint8_t bytes_to_read = length;

    while (bytes_to_read > 0) {
        // amount of bytes to request, needed to pull in data 32 bytes at a time
        const uint8_t request_amount = bytes_to_read > ATECC508A_MAX_REQUEST_SIZE
                ? ATECC508A_MAX_REQUEST_SIZE
                : bytes_to_read;

        bytes_to_read -= twiReadBytes(ATECC508A_I2C_ADDRESS, data + (length - bytes_to_read), request_amount);

        if (++request_attempts == ATRCC508A_MAX_READ_REQUEST_RETRIES) {
            // This probably means that the device is not responding
            return false;
        }
    }

    const uint8_t temp1 = data[RESPONSE_COUNT_INDEX];
    const uint8_t temp2 = data[length - (CRC_SIZE - 1)];
    const uint8_t temp3 = data[length - CRC_SIZE];

    if (temp1 != length) {
        return false;
    }

    const uint16_t crc = computeAtecc508aCrc(length - CRC_SIZE, data);

    return (temp2 == (crc >> 8)) && (temp3 == (uint8_t) (crc & 0x00FF));
}

#include <avr/io.h>

bool initializeAtecc508a(void) {
    initializeTwi();

    twiWriteBytes(0x00, NULL, 0); // set up to write to address "0x00",
    // This creates a "wake condition" where SDA is held low for at least tWLO
    // tWLO means "wake low duration" and must be at least 60 uSeconds (which is achieved by writing 0x00 at 100KHz I2C)
    //    PORTA.OUTCLR = PIN2_bm;
    //
    //    _delay_us(100);
    //
    //    PORTA.OUTSET = PIN2_bm;

    _delay_us(1500); // required for the IC to actually wake up.
    // 1500 uSeconds is minimum and known as "Wake High Delay to Data Comm." tWHI, and SDA must be high during this time.


    // Now let's read back from the IC and see if it reports back good things.
    uint8_t response[RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE] = {55, 56, 57, 58};

    if (!receiveAtecc508aResponseData(response, RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE)) {
        return false;
    }

    return response[RESPONSE_SIGNAL_INDEX] == ATRCC508A_SUCCESSFUL_WAKEUP;
}

bool sendCommandToAtecc508a(const uint8_t command_opcode, const uint8_t param1, const uint16_t param2, const uint8_t * const data, const size_t length_of_data) {
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
    memcpy(&total_transmission[ATRCC508A_PROTOCOL_FIELD_DATA], data, length_of_data); // append data

    // update CRCs
    // copy over just what we need to CRC starting at index 1
    const uint8_t transmission_without_crc_length = total_transmission_length - (ATRCC508A_PROTOCOL_FIELD_SIZE_COMMAND + ATRCC508A_PROTOCOL_FIELD_SIZE_CRC);
    const uint16_t crc = computeAtecc508aCrc(transmission_without_crc_length, &total_transmission[ATRCC508A_PROTOCOL_FIELD_LENGTH]);

    total_transmission[total_transmission_length - ATRCC508A_PROTOCOL_FIELD_SIZE_CRC] = (uint8_t) (crc & 0xFF);
    total_transmission[total_transmission_length - ATRCC508A_PROTOCOL_FIELD_SIZE_CRC + 1] = (uint8_t) (crc >> 8);

    return twiWriteBytes(ATECC508A_I2C_ADDRESS, total_transmission, total_transmission_length) == total_transmission_length;
}

bool atecc508aEnterIdleMode() {
    uint8_t data[1] = {WORD_ADDRESS_VALUE_IDLE};
    return twiWriteBytes(ATECC508A_I2C_ADDRESS, data, 1) == 1;
}

bool atecc508aLoadTemporaryKey(const uint8_t * const temporary_key) {
    if (!sendCommandToAtecc508a(COMMAND_OPCODE_NONCE, NONCE_MODE_PASSTHROUGH, 0x0000, temporary_key, 32))
        return false;

    // note, param2 is 0x0000 (and param1 is PASSTHROUGH), so OutData will be just a single byte of zero upon completion.
    // see ds pg 77 for more info

    _delay_ms(7); // time for IC to process command and execute

    uint8_t response[RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE];

    // Now let's read back from the IC.
    {
        const bool isReceiveResponseSuccessful = receiveAtecc508aResponseData(response, RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE);

        atecc508aEnterIdleMode();

        if (!isReceiveResponseSuccessful) {
            return false;
        }
    }

    // If we hear a "0x00", that means it had a successful nonce
    return response[RESPONSE_SIGNAL_INDEX] == ATRCC508A_SUCCESSFUL_TEMPKEY;
}

bool atecc508aSignTemporaryKey(const uint8_t private_key_slot, uint8_t * const signature) {
    if (!sendCommandToAtecc508a(COMMAND_OPCODE_SIGN, SIGN_MODE_TEMPKEY, private_key_slot, NULL, 0)) {
        return false;
    }

    // Wait for IC to process command and execute
    _delay_ms(70);

    uint8_t response[RESPONSE_COUNT_SIZE + SIGNATURE_SIZE + CRC_SIZE];

    {
        const bool isReceiveResponseSuccessful = receiveAtecc508aResponseData(response, RESPONSE_COUNT_SIZE + SIGNATURE_SIZE + CRC_SIZE);

        atecc508aEnterIdleMode();

        if (!isReceiveResponseSuccessful) {
            return false;
        }
    }

    // for loop through to grab all but the first position (which is "count" of the message)
    memcpy(signature, response + RESPONSE_COUNT_SIZE, SIGNATURE_SIZE);

    return true;
}

bool atecc508aSignMessage(const uint8_t * const message, const uint8_t private_key_slot, uint8_t * const signature) {
    return atecc508aLoadTemporaryKey(message) && atecc508aSignTemporaryKey(private_key_slot, signature);
}

bool atecc508aIsSignatureValid(const uint8_t * const message, const uint8_t * const signature, const uint8_t * const public_key) {
    if (!atecc508aLoadTemporaryKey(message)) {
        return false;
    }

    uint8_t signature_and_public_key[128];

    memcpy(signature_and_public_key, signature, SIGNATURE_SIZE);
    memcpy(signature_and_public_key + SIGNATURE_SIZE, public_key, PUBLIC_KEY_SIZE);

    if (!sendCommandToAtecc508a(COMMAND_OPCODE_VERIFY, VERIFY_MODE_EXTERNAL, VERIFY_PARAM2_KEYTYPE_ECC, signature_and_public_key, sizeof (signature_and_public_key))) {
        return false;
    }

    // Wait for the chip to complete processing
    _delay_ms(58);

    uint8_t response[RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE];

    {
        const bool isReceiveResponseSuccessful = receiveAtecc508aResponseData(response, RESPONSE_COUNT_SIZE + RESPONSE_SIGNAL_SIZE + CRC_SIZE);

        atecc508aEnterIdleMode();

        if (!isReceiveResponseSuccessful) {
            return false;
        }
    }

    // If we hear a "0x00", that means it had a successful verify
    return response[RESPONSE_SIGNAL_INDEX] == ATRCC508A_SUCCESSFUL_VERIFY;
}
