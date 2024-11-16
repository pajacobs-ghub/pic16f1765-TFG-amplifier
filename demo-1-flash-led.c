/* demo-1-flash-led.c
 * First code to bring up PIC16F1765-I/P on Curiosity board.
 * PJ 2024-11-16 copied from pic16f1619 exercise from 2023-02-19
 */
// Configuration Bit Settings (generated from Config Memory View)
// CONFIG1
#pragma config FOSC = INTOSC
#pragma config WDTE = ON
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF
#pragma config IESO = ON
#pragma config FCMEN = ON

// CONFIG2
#pragma config WRT = OFF
#pragma config PPS1WAY = OFF
#pragma config ZCD = OFF
#pragma config PLLEN = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOR = OFF
#pragma config DEBUG = OFF
#pragma config LVP = ON

#include <xc.h>
#include "./global_defs.h"

#define LED LATCbits.LATC5

int main()
{
    OSCCONbits.IRCF = 0b1111; // FOSC 16 Mhz
    TRISCbits.TRISC5 = 0; // Pin as output for LED D7 on Curiosity.
    LED = 0;
    while (1) {
        LED ^= 1;
        __delay_ms(1000);
        CLRWDT();
    }
    return 0; // We should never arrive here.
}
