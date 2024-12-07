#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

#ifdef	__cplusplus
extern "C" {
#endif

    void bluetoothInit();

    void sendBluetoothCommand(const char *const command, const char *const readUntil);

    void bluetoothWriteBytes(const char *const data, int bytes);

    void bluetoothReadBytes(char *const destination, int bytes);

#ifdef	__cplusplus
}
#endif

#endif	/* BLUETOOTH_H */
