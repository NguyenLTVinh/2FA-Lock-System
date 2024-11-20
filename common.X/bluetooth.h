#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

#ifdef	__cplusplus
extern "C" {
#endif

    void bluetoothInitController();

    void bluetoothInitReader();

    void sendBluetoothCommand(const char *const command);

#ifdef	__cplusplus
}
#endif

#endif	/* BLUETOOTH_H */

