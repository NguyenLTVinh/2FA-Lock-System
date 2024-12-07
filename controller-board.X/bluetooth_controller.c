#include "../common.X/bluetooth.h"
#include "../common.X/usart.h"

#include "../common.X/globals.h"

#include "bluetooth_controller.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define BLE_RADIO_PROMPT "CMD> "

void bluetoothInitController() {
    bluetoothInit();
    // Put RN4870 in Command Mode
    sendBluetoothCommand("$$$", BLE_RADIO_PROMPT);
    // Enable Device Information and UART Transparent services
    sendBluetoothCommand("SS,C0\r\n", BLE_RADIO_PROMPT);
    // Reboot for the configuration to take effect
    sendBluetoothCommand("R,1\r\n", "%REBOOT%");
    // Need to enter command mode again
    sendBluetoothCommand("$$$", BLE_RADIO_PROMPT);
    // Add the peripheral's address to the white list
    sendBluetoothCommand("JA,0,0491629A182C\r\n", BLE_RADIO_PROMPT);
    // Scan for the device
    sendBluetoothCommand("F\r\n", "0491629A182C"); // Make sure the peripheral is found
    // Stop scan, return to command mode
    sendBluetoothCommand("X\r\n", BLE_RADIO_PROMPT);
    // Connect to the peripheral's device
    sendBluetoothCommand("C,0,0491629A182C\r\n", "%STREAM_OPEN%");
}
