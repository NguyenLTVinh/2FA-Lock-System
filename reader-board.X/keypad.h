#ifndef KEYPAD_H
#define	KEYPAD_H

#ifdef	__cplusplus
extern "C" {
#endif
    enum KeypadKey_t {
        Key_Pressed	= 0xFF,
        Key_None	= 0x00,
        Key_0		= '0',
        Key_1		= '1',
        Key_2		= '2',
        Key_3		= '3',
        Key_4		= '4',
        Key_5		= '5',
        Key_6		= '6',
        Key_7		= '7',
        Key_8		= '8',
        Key_9       = '9',
        Key_E		= 'E',
        Key_F		= 'F'
    };

    const char kpMap[4][3] = {
        {Key_1, Key_2, Key_3},
        {Key_4, Key_5, Key_6},
        {Key_7, Key_8, Key_9},
        {Key_E, Key_0, Key_F}
    };

    void keyPadInit();
    enum KeypadKey_t getKey();

#ifdef	__cplusplus
}
#endif

#endif	/* KEYPAD_H */

