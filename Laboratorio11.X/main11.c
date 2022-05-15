/* 
 * File:   main11.c
 * Author: Daniel Casasola
 *
 * Created on May 10, 2022, 10:25 PM
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

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define BOTON PORTBbits.RB0     // Asignamos un alias a RB0
#define BOTON1 PORTBbits.RB1
#define FLAG_SPI 0xFF

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t cont = 0;           // Contador que env a el maestro al esclavo?
uint8_t valores;
char cont_master = 0;
char cont_slave = 0xFF;
char val_temporal = 0;
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if (PIR1bits.ADIF){
        PIR1bits.ADIF = 0;
    }
    __delay_ms(100);       
    if(SSPSTATbits.BF){     // Revisamos que no haya comunicaci n en ?
        SSPBUF = ADRESH;      // Movemos el valor del contador para enviarlo
        PORTD = ADRESH;
    }   
    return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){   
        if(ADCON0bits.GO == 0){
            __delay_us(80);
            ADCON0bits.GO = 1;}
            SSPBUF = ADRESH;   // Cargamos valor del contador al buffer
            while(!SSPSTATbits.BF){}// Esperamos a que termine el envio
             PORTAbits.RA7 = 1;      // Deshabilitamos el ss del esclavo
            __delay_ms(300);         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
            PORTAbits.RA7 = 0;      // habilitamos nuevamente el escalvo
            
            SSPBUF = FLAG_SPI;      // Se envía cualquier cosa, Esto es para que el maestro  
                                    //  genere los pulsos del reloj que necesita el esclavo
                                    //  para transmitir datos.
            while(!SSPSTATbits.BF){}// Esperamos a que se reciba un dato
            PORTB = SSPBUF;         // Mostramos dato recibido en PORTB
            __delay_ms(400); 
    }
    return;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00000010; //AN0 Y AN1 ENTRADA ANALOGICA
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    TRISA = 0b00100011;         // SS y RA0 como entradas
    PORTA = 0;
    
    TRISD = 0;
    PORTD = 0;
    TRISB = 0;
    PORTB = 0;
    
    TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
    PORTC = 0;
    
        // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = 0;              // Enviamos un dato inicial
        
        
        //CONFIGURACIONES ADC
    ADCON0bits.ADCS = 0b01;   //Fosc/8
    ADCON1bits.VCFG0 = 0;     //VDD
    ADCON1bits.VCFG1 = 0;     //VSS
    ADCON0bits.CHS = 0b0001;       //AN1
    
    ADCON1bits.ADFM = 0;      //JUSTIFICADO A LA IZQUIERDA
    ADCON0bits.ADON = 1;      //HABILITAMOS MODULO ADC
    __delay_us(80);
        
        
        //CONFIGURACION DE INTERRUPCIONES
    PIR1bits.ADIF = 0;        //LIMPIAMOS BANDERA DE INTERRUPCION ADC
    PIE1bits.ADIE = 1;        //HABILITAMOS INTERRUPCIONES ADC
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

