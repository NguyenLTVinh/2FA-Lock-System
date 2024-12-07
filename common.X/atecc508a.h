/* 
 * File:   atecc508a.h
 * Author: username
 *
 * Created on December 7, 2024, 12:54 AM
 */

#ifndef ATECC508A_H
#define	ATECC508A_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <inttypes.h>
    
    bool initializeAtecc508a(void);
    
    bool atecc508aSignMessage(const uint8_t * const message, const uint8_t private_key_slot, uint8_t * const signature);
    
    bool atecc508aIsSignatureValid(const uint8_t * const message, const uint8_t * const signature, const uint8_t * const public_key);

#ifdef	__cplusplus
}
#endif

#endif	/* ATECC508A_H */

