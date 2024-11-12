#ifndef TWOWIREINTERFACE_H
#define	TWOWIREINTERFACE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <inttypes.h>

    /**
     * Initializes the I2C peripherials on the microcontroller.
     */
    void initializeTwi(void);

    /**
     * Writes the given data with the given length to the microcontroller with
     * the given I2C address.
     * 
     * @param address the address of the destination microcontroller
     * @param data the data to send
     * @param length the length of the data to send
     */
    void twiWriteBytes(uint8_t address, const uint8_t * data, uint8_t length);

#ifdef	__cplusplus
}
#endif

#endif	/* TWOWIREINTERFACE_H */

