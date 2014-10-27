// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>
<<<<<<< HEAD
#include <stdlib.h>
#include "p24FJ64GA002.h"
#include "lcd.h"

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
		 BKBUG_ON & COE_ON & ICS_PGx1 &
		 FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
		 IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Clock constants
#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE      115200
#define BRGVAL        ((FCY/BAUDRATE)/16)-1

// ******************************************************************************************* //
// Varible used to indicate that the current configuration of the keypad has been changed,
// and the KeypadScan() function needs to be called.
volatile int scanKeypad = 0;

// ******************************************************************************************* //
=======
#include "lcd.h"
>>>>>>> 1e42594ed8820eafd73fb56b1d2ad6417351403c


<<<<<<< HEAD
    int ADC_value;
    char value[8];
    double AD_value;

    LCDInitialize( );

    AD1PCFGbits.PCFG5;	 	// AN5 input pin is analog
    AD1CON2 = 0; 		// Configure A/D voltage reference
    AD1CON3 = 0x0101;
    AD1CON1 = 0x80E4;
    AD1CHS = 0; 		// Configure input channels
    AD1CSSL = 0; 		// No inputs is scanned

    

    return 0;
}
=======
// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
		 BKBUG_ON & COE_ON & ICS_PGx1 &
		 FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
		 IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )
>>>>>>> 1e42594ed8820eafd73fb56b1d2ad6417351403c

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)
// ******************************************************************************************* //
int main(void){
	int ADC_value;
 	char value[8];
	double AD_value;
        TMR3 = 0;
        PR3 = 50464;           
        IFS0bits.T3IF = 0;
        IEC0bits.T3IE = 1;
        T3CON = 0x8030;

	LCDInitialize( );

	AD1PCFG &= 0xFFDF;	 	// AN0 input pin is analog
	AD1CON2 = 0; 		// Configure A/D voltage reference
	AD1CON3 = 0x0101;
	AD1CON1 = 0x20E4;
	AD1CHS = 5; 		// Configure input channels
	AD1CSSL = 5; 		// No inputs is scanned
	IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
	AD1CON1bits.ADON = 1; // Turn on A/D
	while(1){
		while (IFS0bits.AD1IF ==0){};     // AD1CON1bits.DONE can be checked instead
		IFS0bits.AD1IF = 0;
		ADC_value = ADC1BUF0;

		sprintf(value, "%6d", 1023 - ADC_value);
		LCDMoveCursor(0,0); LCDPrintString(value);
//		AD_value = (ADC_value * 3.3)/1024;
//		sprintf(value, "%6.2f", AD_value);
//		LCDMoveCursor(1,0); LCDPrintString(value);
	}
	return 0;
}
