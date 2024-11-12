#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

#ifdef	__cplusplus
extern "C" {
#endif

    void initializeBluetooth(const char *const name);

    void sendBluetoothCommand(const char *const command);

#ifdef	__cplusplus
}
#endif

#endif	/* BLUETOOTH_H */

