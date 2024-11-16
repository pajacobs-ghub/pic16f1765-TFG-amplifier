/* demo-2-i2c-slave.c
 * Code to bring up MCU as an interrupt-driven I2C slave device
 * on the Curiosity board.
 * PJ 2023-02-27, 2023-03-01 PIC16F1619 variant from i2c-LCD exercise
 *    2024-11-16 PIC16F1765 variant for the TFG amplifier2 exercise
 * This code adapted from the old example
 * ~/pic_work/i2c-slave-flash-led-with-states/i2c-slave-with-states.c
 * which was created on 04-Jan-2013
 *
 * Example I2C slave device that lights a LED on command.
 * This is a particularly simple implementation that expects
 * to receive messages with a single byte of data to control
 * the LED.
 * This example implements the state-table approach shown in
 * Microchip App Note 734.
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
#include <string.h>

// Pins:
// RC4 switch input with 10k external pull-up
// RC1 SDA (I2C data)
// RC0 SCL (I2C clock)
// RC5 LED output
#define BUTTON (PORTCbits.RC4)
#define LED (LATCbits.LATC5)

// Our chosen slave address.
// Note that this number appears in a couple of the messages below.
#define I2C_ADDR7  0x51
// The largest Newhaven serial LCD command seems to be 11 bytes.
#define BUFFER_LEN 16
static volatile unsigned char receive_buf[BUFFER_LEN];
static volatile unsigned char send_buf[BUFFER_LEN];
static volatile unsigned char bytes_received = 0;
static volatile unsigned char send_indx = 0;

void i2c_init()
{
    // Configure PPS I2C_SCL=RB6, I2C_SDA=RB4 
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC0 = 1;
    ANSELCbits.ANSC1 = 0;
    ANSELCbits.ANSC0 = 0;
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    SSPCLKPPS = 0b10000; // RC0
    RC0PPS = 0b10010; // SCL
    SSPDATPPS = 0b10001; // RC1
    RC1PPS = 0b10011; // SDA
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    SSP1CON1 = 0; // Clear flags.
    SSP1CON1 |= 0b00000110; // Slave mode with 7-bit address.
    SSP1CON2bits.SEN = 1; // Clock stretching enabled.
    SSP1STATbits.SMP = 1; // Slew rate disabled for 100kHz operation.
    SSP1ADD = I2C_ADDR7 << 1;  // LSB is R_nW bit
    // MSSP interrupt
    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1; // Enable MSSP interrupts
    INTCONbits.PEIE = 1; // Enable peripheral device interrupts
    // MSSP bus collision interrupt (Do not enable presently)
    // PIR2bits.BCL1IF = 0;
    // PIE2bits.BCL1IE = 1;
    SSP1CON1bits.SSPEN = 1; // Enable serial port.
    return;
}

void i2c_close()
{
    di();
    SSP1CON1bits.SSPEN = 0; // Disable serial port.
    return;
}

void __interrupt() i2c_service()
// Implement the state-table approach shown in Microchip App Note 734.
// Should be usable as an interrupt service routine, also.
// Note that this routine runs some time after SSP1IF flags an event.
// A delay can affect the AN734 test for a NACK from the master.
{
    unsigned char junk, i;
    if (PIR1bits.SSP1IF) {
        PIR1bits.SSP1IF = 0;
        // The following mask is used to retain bits D_nA, S, R_nW, BF
#       define i2c_mask 0b00101101
        if ((SSP1STAT & i2c_mask) == 0b00001001) {
            // State 1: last seen start, i2c write operation, incoming byte is an address.
            // D_nA=0, S=1, R_nW=0, BF=1
            junk = SSP1BUF; // discard the address byte
            for ( i = 0; i < BUFFER_LEN; ++i ) receive_buf[i] = 0;
            bytes_received = 0;
            SSP1CON1bits.CKP = 1; // Release clock.
            return;
        }
        if ((SSP1STAT & i2c_mask) == 0b00101001) {
            // State 2: last seen start, i2c write operation, incoming byte is data.
            // D_nA=1, S=1, R_nW=0, BF=1
            if (bytes_received < BUFFER_LEN) {
                receive_buf[bytes_received] = SSP1BUF;
                bytes_received++;
            } else {
                // Buffer is full, discard incoming byte.
                junk = SSP1BUF;
            }
            SSP1CON1bits.CKP = 1; // Release clock.
            return;
        }
        if ((SSP1STAT & 0b00101100) == 0b00001100) {
            // State 3: last seen start, i2c read operation, incoming byte is address.
            // D_nA=0, S=1, R_nW=1
            junk = SSP1BUF; // discard that address byte
            send_indx = 0;
            bytes_received = 0; // Discard the last received message, too.
            SSP1CON1bits.WCOL = 0;
            SSP1BUF = send_buf[send_indx];
            SSP1CON1bits.CKP = 1; // Release clock.
            send_indx++;
            if (send_indx == BUFFER_LEN) send_indx = 0; // wrap around
            return;
        }
        if ((SSP1STAT & i2c_mask) == 0b00101100) {
            // State 4: last seen start, i2c read operation, last byte was data
            // D_nA=1, S=1, R_nW=1, BF=0
            SSP1CON1bits.WCOL = 0;
            if (SSP1CON2bits.ACKSTAT == 0) {
                // Acknowledge received (because the master wants more).
                SSP1BUF = send_buf[send_indx];
                send_indx++;
                if (send_indx == BUFFER_LEN) send_indx = 0; // wrap around
            } else {
                // NACK; no need to send any more data.
            }
            SSP1CON1bits.CKP = 1; // Release clock.
            return;
        }
        if (SSP1STATbits.P || (SSP1STATbits.D_nA && SSP1CON2bits.ACKSTAT)) {
            // State 5: last seen stop or (Master NACK after sending data byte)
            // Nothing special to do.
            return;
        }
        // If we get here, something has gone wrong.
        while ( 1 ) ; // Block until watch dog barks.
    }
} // end i2c_service()

int main()
{
    unsigned char cmd[BUFFER_LEN];
    unsigned char new_cmd = 0;
    //
    OSCCONbits.IRCF = 0b1111; // FOSC 16 Mhz
    i2c_init();
    ei(); // The i2c_service routine is called via interrupt.
    // Put something in the send buffer so that the I2C master can ask for it.
    for (unsigned char i = 0; i < BUFFER_LEN; ++i) send_buf[i] = i;
    // Main loop.
    while (1) {
        // Disable interrupts while we access the I2C buffer.
        di();
        // We will make a copy of the I2C receive buffer to allow
        // the command interpreter to work on it safely.
        if (bytes_received) {
            for (unsigned char i = 0; i < bytes_received; ++i) cmd[i] = receive_buf[i];
            bytes_received = 0; // Indicate that we have taken the bytes.
            new_cmd = 1; // and flag that we have a command to work on.
        } else {
            new_cmd = 0;
        }
        ei();
        if (!new_cmd) {
            // There is no new command, so just waste a bit of time.
            __delay_ms(1);
            CLRWDT();
            continue;
        }
        // Interpret the command bytes.
        switch (cmd[1]) {
            case 0x0: {
                LED = 0;
                break;
            }
            case 0x1: {
                LED = 1;
                break;
            }
            default:
                // Do nothing special.
                break;
        } // end switch
        //
        // Now we have interpreted the command bytes, clean up.
        for (unsigned char i = 0; i < BUFFER_LEN; ++i) cmd[i] = 0;
        new_cmd = 0;
        CLRWDT();
    } // end while 1
    //
    // Don't expect to arrive here...
    i2c_close();
    return 0;
}
