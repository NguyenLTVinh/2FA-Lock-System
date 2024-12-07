#include "../common.X/globals.h"
#include "doorlock_controller.h"

#include <avr/io.h>
#include <util/delay.h>

void doorLockInit() {
    // Set replay as output
    PORTA.DIR |= PIN5_bm;  
    // Relay starts in the locked state (LOW)
    PORTA.OUT &= ~PIN5_bm; 
}

void openLock() {
    PORTA.OUT |= PIN5_bm;
}

void lockLock() {
    PORTA.OUT &= ~PIN5_bm;
}
