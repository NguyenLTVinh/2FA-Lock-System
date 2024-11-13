#ifndef CRYPTOGRAPHY_H
#define	CRYPTOGRAPHY_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdbool.h>
#include <inttypes.h>

    bool sign_data(const uint8_t input_and_output[8]);

#ifdef	__cplusplus
}
#endif

#endif	/* CRYPTOGRAPHY_H */
