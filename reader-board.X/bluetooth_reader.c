#include "../common.X/bluetooth.h"
#include "../common.X/usart.h"

#include "../common.X/globals.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define BLE_RADIO_PROMPT "CMD> "
#define BUF_SIZE 128

void bluetoothInitReader() {
    bluetoothInit();
    char buf[BUF_SIZE];
    // Put RN4870 in Command Mode
    sendBluetoothCommand("$$$", BLE_RADIO_PROMPT);
    // Display info for MAC
    sendBluetoothCommand("D\r\n", BLE_RADIO_PROMPT);
    // Enable advertising
    sendBluetoothCommand("A\r\n", BLE_RADIO_PROMPT);
    // Wait until connected by the central board
    usartReadUntil(buf, "%STREAM_OPEN%");
}
