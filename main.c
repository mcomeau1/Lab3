// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>
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

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)
// ******************************************************************************************* //
int main(void){
	int ADC_value;
        float temp;
 	char value[8];
        char period1[5];
        char period2[5];
        TMR3 = 0;
        PR3 = 512;
        IFS0bits.T3IF = 0;
        IEC0bits.T3IE = 1;
        T3CON = 0x0030;

        OC1CON = 0x1D;
        OC1R = 0;
        OC1RS = PR3;
        OC2CON = 0x1D;
        OC2R = 0;
        OC2RS = PR3;
        RPOR4bits.RP8R = 18;
        RPOR4bits.RP9R = 19;
	LCDInitialize( );

	AD1PCFG &= 0xFFDF;	 	// AN0 input pin is analog
	AD1CON2 = 0; 		// Configure A/D voltage reference
	AD1CON3 = 0x0101;
	AD1CON1 = 0x20E4;
	AD1CHS = 5; 		// Configure input channels
	AD1CSSL = 5; 		// No inputs is scanned
	IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
	AD1CON1bits.ADON = 1; // Turn on A/D
        temp = PR3 * 1.0;
	while(1){
            TMR3 = 0;
            while (IFS0bits.AD1IF ==0){};     // AD1CON1bits.DONE can be checked instead
            IFS0bits.AD1IF = 0;
            ADC_value = ADC1BUF0;
            sprintf(value, "%6d", 1023 - ADC_value);
            LCDMoveCursor(0,0); LCDPrintString(value);
            
            T3CONbits.TON = 1;
            OC1RS = ADC_value;
            OC2RS = 1023 - ADC_value;

            while(IFS0bits.T3IF == 0);
            T3CONbits.TON = 0;
            IFS0bits.T3IF = 0;
            sprintf(period1, "%.2f", OC1RS/temp);
            sprintf(period2, "%.2f", OC2RS/temp);
            if(strcmp(period1, "1.00") > 0)
            {
                strcpy(period1,"1.00");
            }
            if(strcmp(period2, "1.00") > 0)
            {
                strcpy(period2,"1.00");
            }
            LCDMoveCursor(1,0); LCDPrintString("        ");
            LCDMoveCursor(1,0); LCDPrintString(period1);
            LCDPrintString(period2);
//		AD_value = (ADC_value * 3.3)/1024;
//		sprintf(value, "%6.2f", AD_value);
//		LCDMoveCursor(1,0); LCDPrintString(value);
	}
	return 0;
}

void __attribute__((interrupt,auto_psv)) _T3Interrupt(void){
    IFS0bits.T3IF = 0;

}

