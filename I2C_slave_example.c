/*
	I2C_slave_example.c
	Author:	Harry Braviner

	This example should receive an 8-bit number over I2C, and copy this into LATA to turn some LEDs on or off
	It should also transmit an 8-bit number over I2C, the LSB of which will indicate whether RC7 is high or low (to be tested using a switch)

*/

// Begin configuration bits
#pragma config OSC=INTIO67, FCMEN=OFF, IESO=OFF			// CONFIG1H
#pragma config PWRT=ON,	BOREN=SBORDIS, BORV=3			// CONFIG2L
#pragma config WDT=OFF, WDTPS=1					// CONFIG2H
#pragma config CCP2MX=PORTC, PBADEN=OFF, LPT1OSC=OFF, MCLRE=OFF	// CONFIG3H
#pragma config STVREN=ON, LVP=ON, XINST=ON, DEBUG=OFF		// CONFIG4L
#pragma config CP0=OFF, CP1=OFF					// CONFIG5L
#pragma config CPB=OFF, CPD=OFF					// CONFIG5H
#pragma config WRT0=OFF, WRT1=OFF				// CONFIG6L
#pragma config WRTC=OFF, WRTB=OFF, WRTD=OFF			// CONFIG6H
#pragma config EBTR0=OFF, EBTR1=OFF				// CONFIG7L
#pragma config EBTRB=OFF					// CONFIG7H
// End configuration bits

void main(void){

}
