// This example was found on the internet at http://www.microchip.com/forums/m741878.aspx

/* ***********************************************************
*
*  PIC C18 Example I2C SLAVE for PIC18F ()PIC18F45K20)
*  Author:  John Clegg 
*  Date:    26 August 2013
*
*  Read/write to a bank of 8 bit register values.
*  I2C read returns the contents of the current register, multi-byte
*  reads return subsequent register bytes. 
*  I2C write, the first data byte is the register address, subsequent bytes
*  are written, initially to the register address supplied, then to 
*  successive registers addresses.
************************************************************* */

/** C O N F I G U R A T I O N   B I T S ******************************/

#pragma config FOSC = INTIO67, FCMEN = OFF, IESO = OFF                       // CONFIG1H
#pragma config PWRT = OFF, BOREN = SBORDIS, BORV = 30                        // CONFIG2L
#pragma config WDTEN = OFF, WDTPS = 32768                                    // CONFIG2H
#pragma config MCLRE = OFF, LPT1OSC = OFF, PBADEN = ON, CCP2MX = PORTC       // CONFIG3H
#pragma config STVREN = ON, LVP = OFF, XINST = OFF                             // CONFIG4L
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF                    // CONFIG5L
#pragma config CPB = OFF, CPD = OFF                                          // CONFIG5H
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF                // CONFIG6L
#pragma config WRTB = OFF, WRTC = OFF, WRTD = OFF                            // CONFIG6H
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF            // CONFIG7L
#pragma config EBTRB = OFF                                                   // CONFIG7H


/** I N C L U D E S **************************************************/
#include "p18f45k20.h"
#include "delays.h"

/** D E C L A R A T I O N S *******************************************/

#define I2C_ADDR    0xaa    // 8 bit address

typedef unsigned char   byte;
 void low_isr(void);
void high_isr(void);

volatile byte           i2c_reg_addr     = 0;
volatile byte           i2c_reg_map[16 ] = {0,};
volatile byte           i2c_byte_count   = 0;

/*
* For PIC18 devices the high interrupt vector is found at
* 00000008h. The following code will branch to the
* high_interrupt_service_routine function to handle
* interrupts that occur at the high vector.
*/
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
_asm GOTO high_isr _endasm
}

#pragma code /* return to the default code section */
/*
* For PIC18 devices the low interrupt vector is found at
* 00000018h. The following code will branch to the
* low_interrupt_service_routine function to handle
* interrupts that occur at the low vector.
*/#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
_asm GOTO low_isr _endasm
}

#pragma code /* return to the default code section */

void main (void)
{
    OSCCON            = 0x60;       // IRCFx = 110
    OSCTUNEbits.PLLEN = 0;          // x4 PLL disabled

    // Port D used for diagnostic LEDs
    TRISD      = 0b00111111;     // PORTD bit 7 to output (0) ; bits 6:0 are inputs (1)
    LATDbits.LATD7 = 0;             // RED LED 
    LATDbits.LATD6 = 0;             // YLW LED 
        
    // Setup MSSP in 7 bit I2C Slave mode
    TRISC          = 0b00011000;    // TRISC 3&4 (SCL & SDA) inputs
    LATC           = 0b00011000;
    SSPADD         = I2C_ADDR;      // Set I2C address
    SSPCON1        = 0x36;          // SSPEN: Synchronous Serial Port Enable bit - Enables the serial port and configures the SDA and SCL pins as the serial port pins
                                    // CKP: SCK Release Control bit              - Release clock
                                    // SSPM3:SSPM0: SSP Mode Select bits         - 0110 = I2C Slave mode, 7-bit address    
    SSPSTAT        = 0x00;
    SSPCON2        = 0x01;          // GCEN: General Call address (00h) (Slave mode only) 0 = General call address disabled
                                    // SEN: Start Condition Enable/Stretch Enable bit(1) ()Slave mode) 1 = Clock stretching is enabled 
    PIR1bits.SSPIF = 0;             // Clear MSSP interrupt request flag
    PIE1bits.SSPIE = 1;             // Enable MSSP interrupt enable bit
    INTCONbits.GIE_GIEH  = 1;       // GIE/GIEH: Global Interrupt Enable bit
    INTCONbits.PEIE_GIEL = 1;       // PEIE/GIEL: Peripheral Interrupt Enable bit

    while (1)
    {
        Delay1KTCYx(50);    // Delay 50 x 1000 = 50,000 cycles; 200ms @ 1MHz
    }    
}

#pragma interruptlow low_isr
void low_isr (void)
{
} 

#pragma interruptlow high_isr
void high_isr (void)
{
    byte    sspBuf;
    
    if (PIR1bits.SSPIF) {
        
        if (!SSPSTATbits.D_NOT_A) {
            //
            // Slave Address 
            //
            i2c_byte_count = 0;

            if (SSPSTATbits.BF) {
                // Discard slave address 
                sspBuf = SSPBUF;    // Clear BF
            }
            
            if (SSPSTATbits.R_NOT_W) {                
                // Reading - read from register map
                SSPCON1bits.WCOL = 0;
                SSPBUF           = i2c_reg_map[i2c_reg_addr++];
            } 
            
        } else {
            //
            // Data bytes 
            //
            i2c_byte_count++;

            if (SSPSTATbits.BF) {
                sspBuf = SSPBUF;    // Clear BF
            }

            if (SSPSTATbits.R_NOT_W) {

                // Multi-byte read - advance to next address
                SSPCON1bits.WCOL = 0;
                SSPBUF           = i2c_reg_map[i2c_reg_addr++];
                LATDbits.LATD6 = 1;
                
            } else {                

                if (i2c_byte_count == 1) {
                    // First write byte is register address
                    i2c_reg_addr = sspBuf;

                } else {
                    // Write to register address - auto advance
                    //   to allow multiple bytes to be written
                    i2c_reg_map[i2c_reg_addr++] = sspBuf;
                }
            }
        }
        // Limit address to size of register map
        i2c_reg_addr %= sizeof(i2c_reg_map);
        
        // Finally
        PIR1bits.SSPIF  = 0;            // Clear MSSP interrupt flag
        SSPCON1bits.CKP = 1;            // Release clock        
    }    
}
