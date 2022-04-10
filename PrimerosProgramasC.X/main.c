/* 
 * File:   main.c
 * Author: Daniel Casasola
 *
 * Created on April 3, 2022, 9:58 PM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

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


/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define BOTON PORTBbits.RB0     // Asignamos un alias a RB0
#define BOTON1 PORTBbits.RB1
uint8_t valores;
uint8_t unidades;
uint8_t decenas;
uint8_t centenas;
uint8_t x;
uint8_t y;
uint8_t disp_control;
//PROTOTIPO DE FUNCION

char tabla[10] = {
    0B00111111,
    0B00000110,
    0B01011011,
    0B01001111,
    0B01100110,
    0B01101101,
    0B01111101,
    0B00000111,
    0B01111111,
    0B01101111
};


void setup(void);
void controldisplays(void);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if(INTCONbits.RBIF){        // Fue interrupción del PORTB
        if (!BOTON)             // Verificamos si fue RB0 quien generó la interrupción
            valores++;            // Incremento del PORTC
            INTCONbits.RBIF = 0;    // Limpiamos bandera de interrupción
        
        if (!BOTON1)             // Verificamos si fue RB0 quien generó la interrupción
            valores--;            // Incremento del PORTC
            INTCONbits.RBIF = 0;    // Limpiamos bandera de interrupción
    }
    
        if (T0IF){
        //DISPLAYS
            
            switch(disp_control){
                case 0:
                    PORTD = 0;
                    PORTDbits.RD0 = 1;
                    PORTC = tabla[centenas];
                    disp_control++;
                    break;
                case 1:
                    PORTD = 0;
                    PORTDbits.RD1 = 1;
                    PORTC = tabla[decenas];
                    disp_control++;
                    break;
                case 2:
                    PORTD = 0;
                    PORTDbits.RD2 = 1;
                    PORTC = tabla[unidades];
                    disp_control = 0;
                    break; 
            }
        //TMR0
            INTCONbits.T0IF = 0;
            TMR0 = 61;
        }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();      
   
    // LOOP
    while(1){  
        PORTA = x;
        controldisplays();
    }
    return;
}

void controldisplays(void){
    x = valores;
    centenas = x/100;
    y = x%100;
    decenas = y/10;
    unidades = y%10;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0b00000000;        // Usaremos solo I/O digitales
    
    TRISC = 0;               // PORTC como salida
    PORTC = 0;                  // Limpiamos PORTC
    TRISA = 0;
    PORTA = 0;
    TRISD = 0;
    PORTD = 0;
    disp_control = 0;
    //TRISB = 0b00000001;       // RB0 como entrada (configurada con binario)
    //TRISB = 1;                // RB0 como entrada (configurada en decimal)
    TRISBbits.TRISB0 = 1;       // RB0 como entrada (configurada con bits de control)
    TRISBbits.TRISB1 = 1; 
    OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
    WPUBbits.WPUB0 = 1;         // Habilitamos resistencia de pull-up de RB0
    WPUBbits.WPUB1 = 1;
  
    OSCCONbits.SCS = 0;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 0;         
    OSCCONbits.IRCF0 = 1;       //2MHz

    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    
    INTCONbits.T0IE = 1;
    INTCONbits.T0IF = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PS0 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS2 = 1;     //Prescaler 256


    INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
    IOCBbits.IOCB0 = 1;         // Habilitamos interrupción por cambio de estado para RB0
    IOCBbits.IOCB1 = 1;
    INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
}