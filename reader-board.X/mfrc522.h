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

    typedef enum Mfrc522Register {
        // Page 0: Command and status
        //                             0x00	 // reserved for future use
        MFRC522_REGISTER_COMMAND =     0x01, // starts and stops command execution
        MFRC522_REGISTER_ComIEn =      0x02, // enable and disable interrupt request control bits
        MFRC522_REGISTER_DivIEn =      0x03, // enable and disable interrupt request control bits
        MFRC522_REGISTER_ComIrq =      0x04, // interrupt request bits
        MFRC522_REGISTER_DivIrq =      0x05, // interrupt request bits
        MFRC522_REGISTER_ERROR =       0x06, // error bits showing the error status of the last command executed 
        MFRC522_REGISTER_STATUS1 =     0x07, // communication status bits
        MFRC522_REGISTER_STATUS2 =     0x08, // receiver and transmitter status bits
        MFRC522_REGISTER_FIFO_DATA =   0x09, // input and output of 64 byte FIFO buffer
        MFRC522_REGISTER_FIFO_LEVEL =  0x0A, // number of bytes stored in the FIFO buffer
        MFRC522_REGISTER_WATER_LEVEL = 0x0B, // level for FIFO underflow and overflow warning
        MFRC522_REGISTER_CONTROL =     0x0C, // miscellaneous control registers
        MFRC522_REGISTER_BIT_FRAMING = 0x0D, // adjustments for bit-oriented frames
        MFRC522_REGISTER_COLLISION =   0x0E, // bit position of the first bit-collision detected on the RF interface
        //                             0x0F	 // reserved for future use

        // Page 1: Command
        //                                   0x10  // reserved for future use
        MFRC522_REGISTER_MODE =              0x11, // defines general modes for transmitting and receiving 
        MFRC522_REGISTER_TRANSMIT_MODE =     0x12, // defines transmission data rate and framing
        MFRC522_REGISTER_RECEIVE_MODE =      0x13, // defines reception data rate and framing
        MFRC522_REGISTER_TRANSMIT_CONTROL =  0x14, // controls the logical behavior of the antenna driver pins TX1 and TX2
        MFRC522_REGISTER_TRANSMIT_ASK =      0x15, // controls the setting of the transmission modulation
        MFRC522_REGISTER_TRANSMIT_SELECT =   0x16, // selects the internal sources for the antenna driver
        MFRC522_REGISTER_RECEIVE_SELECT =    0x17, // selects internal receiver settings
        MFRC522_REGISTER_RECEIVE_THRESHOLD = 0x18, // selects thresholds for the bit decoder
        MFRC522_REGISTER_DEMODULATOR =       0x19, // defines demodulator settings
        //                                   0x1A  // reserved for future use
        //                                   0x1B  // reserved for future use
        MFRC522_REGISTER_MIFARE_TRANSMIT =   0x1C, // controls some MIFARE communication transmit parameters
        MFRC522_REGISTER_MIFARE_RECEIVE =    0x1D, // controls some MIFARE communication receive parameters
        //                                   0x1E  // reserved for future use
        MFRC522_REGISTER_UART_SERIAL_SPEED = 0x1F, // selects the speed of the serial UART interface

        // Page 2: Configuration
        //                                          0x20  // reserved for future use
        MFRC522_REGISTER_CRC_RESULT_HIGH_BITS =     0x21, // shows the MSB and LSB values of the CRC calculation
        MFRC522_REGISTER_CRC_RESULT_LOW_BITS =      0x22,
        //                                          0x23  // reserved for future use
        MFRC522_REGISTER_MOD_WIDTH =                0x24, // controls the ModWidth setting
        //                                          0x25  // reserved for future use
        MFRC522_REGISTER_RECEIVER_GAIN =            0x26, // configures the receiver gain
        MFRC522_REGISTER_GsN =                      0x27, // selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
        MFRC522_REGISTER_CWGsP =                    0x28, // defines the conductance of the p-driver output during periods of no modulation
        MFRC522_REGISTER_ModGs =                    0x29, // defines the conductance of the p-driver output during periods of modulation
        MFRC522_REGISTER_TIMER_MODE =               0x2A, // defines settings for the internal timer
        MFRC522_REGISTER_TIMER_PRESCALER_LOW_BITS = 0x2B, // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
        MFRC522_REGISTER_TIMER_RELOAD_HIGH_BITS =   0x2C, // defines the 16-bit timer reload value
        MFRC522_REGISTER_TIMER_RELOAD_LOW_BITS =    0x2D,
        MFRC522_REGISTER_TIMER_VALUE_HIGH_BITS =    0x2E, // shows the 16-bit timer value
        MFRC522_REGISTER_TIMER_VALUE_LOW_BITS =     0x2F,

        // Page 3: Test Registers
        //                                     0x30  // reserved for future use
        MFRC522_REGISTER_TEST_SIGNAL_CONFIG1 = 0x31, // general test signal configuration
        MFRC522_REGISTER_TEST_SIGNAL_CONFIG2 = 0x32, // general test signal configuration
        MFRC522_REGISTER_TEST_PIN_ENABLE =     0x33, // enables pin output driver on pins D1 to D7
        MFRC522_REGISTER_TEST_PIN_VALUES =     0x34, // defines the values for D1 to D7 when it is used as an I/O bus
        MFRC522_REGISTER_TEST_BUS_STATUS =     0x35, // shows the status of the internal test bus
        MFRC522_REGISTER_AUTO_TEST =           0x36, // controls the digital self test
        MFRC522_REGISTER_VERSION =             0x37, // shows the software version
        MFRC522_REGISTER_ANALOG_TEST =         0x38, // controls the pins AUX1 and AUX2
        MFRC522_REGISTER_TEST_DAC1_VALUE =     0x39, // defines the test value for TestDAC1
        MFRC522_REGISTER_TEST_DAC2_VALUE =     0x3A, // defines the test value for TestDAC2
        MFRC522_REGISTER_TEST_ADC_VALUE =      0x3B  // shows the value of ADC I and Q channels
        //                                     0x3C  // reserved for production tests
        //                                     0x3D  // reserved for production tests
        //                                     0x3E  // reserved for production tests
        //                                     0x3F  // reserved for production tests
    } Mfrc522Register;

    /**
     * MFRC522 commands. Described in chapter 10 of the datasheet.
     */
    typedef enum Mfrc522Command {
        MFRC522_COMMAND_IDLE =                0x00, // no action, cancels current command execution
        MFRC522_COMMAND_MEM =                 0x01, // stores 25 bytes into the internal buffer
        MFRC522_COMMAND_GENERATE_RANDOM_ID =  0x02, // generates a 10-byte random ID number
        MFRC522_COMMAND_CALCULATE_CRC =       0x03, // activates the CRC coprocessor or performs a self test
        MFRC522_COMMAND_TRANSMIT =            0x04, // transmits data from the FIFO buffer
        MFRC522_COMMAND_NO_CHANGE_COMMAND =   0x07, // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
        MFRC522_COMMAND_RECEIVE =             0x08, // activates the receiver circuits
        MFRC522_COMMAND_TRANSCEIVE =          0x0C, // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
        MFRC522_COMMAND_MIFARE_AUTHENTICATE = 0x0E, // performs the MIFARE standard authentication as a reader
        MFRC522_COMMAND_SOFT_RESET =          0x0F  // resets the MFRC522
    } Mfrc522Command;
    
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
    
    typedef struct Picc {
        uint8_t size; // Number of bytes in the UID. 4, 7 or 10.
        uint8_t uidByte[10];
        uint8_t sak; // The SAK (Select acknowledge) byte returned from the PICC after successful selection.
    } Picc;

    void initializeMfrc522(void);

    uint8_t mfrc522ReadByteAtAddress(const uint8_t address);

    void mfrc522ReadDataAtAddress(const uint8_t address, uint8_t * const data, const uint8_t length, const uint8_t receive_alignment);

    void mfrc522WriteByteAtAddress(const uint8_t address, const uint8_t byte);

    void mfrc522WriteDataAtAddress(const uint8_t address, const uint8_t * const data, const uint8_t length);

    bool readPicc(Picc * const card);

#ifdef	__cplusplus
}
#endif

#endif	/* MFRC522_H */
