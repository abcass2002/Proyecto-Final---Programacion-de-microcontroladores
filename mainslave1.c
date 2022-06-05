/*
 * File: mainslave1.c
 * Author: Carlos Bucaro & Abner Casasola
 *
 * Created on 18 de mayo de 2022
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#include <xc.h>
#include <stdint.h>
//CONSTANTES
#define _XTAL_FREQ 500000
#define IN_MIN 0
#define IN_MAX 255
#define OUT_MIN 0
#define OUT_MAX 72
//VARIABLES
unsigned short CCPR_UNO = 0;
unsigned short CCPR_DOS = 0;
//PROTOTIPO DE FUNCIONES
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, unsigned short out_min, unsigned short out_max);
void __interrupt() isr (void){
    PORTB ++;
    if(PIR1bits.SSPIF){
        if(PORTAbits.RA0 == 0){
            PORTB = SSPBUF;
            CCPR_UNO = map(SSPBUF, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR1L = (uint8_t)(CCPR_UNO>>2);
            CCP1CONbits.DC1B = CCPR_UNO & 0b11;
            
        }
        else if(PORTAbits.RA0 == 1){
            PORTD = SSPBUF;
            CCPR_DOS = map(SSPBUF, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR2L = (uint8_t)(CCPR_DOS>>2);
            CCP2CONbits.DC2B1 = (CCPR_DOS & 0b10)>>1;
            CCP2CONbits.DC2B0 = CCPR_DOS & 0b01;
        }
        PIR1bits.SSPIF = 0;
    }
}
void setup (void){
    ANSEL = 0b00000010;
    ANSELH = 0;
    TRISA = 0b00100011;
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTA = 0;
    PORTD = 0;
    //CONFIGURACIÓN DEL RELOJ INTERNO
    OSCCONbits.IRCF = 0b011;
    OSCCONbits.SCS = 1;
   
    //CONFIGURACIÓN PWM
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    PR2 = 32;
    //CONFIGURACIÓN CCP
    CCP1CON = 0;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;
    CCPR1L = 13 >> 2;
    CCP1CONbits.DC1B = 13 & 0b11;
    
    CCP2CON = 0;
    CCP2CONbits.CCP2M = 0b1100;
    CCPR2L = 13 >> 2;
    CCP2CONbits.DC2B0 = 13 & 0b01;
    CCP2CONbits.DC2B1 = 13 & 0b10;
    
    
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;
    T2CONbits.TMR2ON = 1;
    while (!PIR1bits.TMR2IF);
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj

   //CONFIGURACIÓN INTERRUPCIONES
    
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    return;
}

void main(void) {
    setup();
    while(1){
   
    }
return;
}
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}