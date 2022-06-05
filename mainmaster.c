/*
 * File:   mainmaster.c
 * Author: Carlos Bucaro & Daniel Casasola
 *
 * Created on 18 de mayo 2022
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
#define OUT_MIN 13
#define OUT_MAX 72
//VARIABLES
uint8_t POT_CONT = 0;
uint8_t CONT = 0;
unsigned short CCPR_UNO = 0;
unsigned short CCPR_DOS = 0;
uint8_t mode = 0;
uint8_t POT1,POT2,POT3,POT4,P1,P2,P3,P4;
uint8_t VAR1,VAR2,VAR3,VAR4;
//PROTOTIPO DE FUNCIONES
void setup(void);
void servo_control(uint8_t posicion);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, unsigned short out_min, unsigned short out_max);
void ESCRIBIR (uint8_t direccion, uint8_t valores);
uint8_t LEER(uint8_t direccion);


void __interrupt() isr (void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){
            POT1 = ADRESH;
            P1 = ADRESH;
            CCPR_UNO = map(VAR1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR1L = (uint8_t)(CCPR_UNO>>2);
            CCP1CONbits.DC1B = CCPR_UNO & 0b11;     
        }
        else if(ADCON0bits.CHS == 2){
            POT2 = ADRESH;
            P2 = ADRESH;
            CCPR_DOS = map(VAR2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR2L = (uint8_t)(CCPR_DOS>>2);
            CCP2CONbits.DC2B1 = (CCPR_DOS & 0b10)>>1;
            CCP2CONbits.DC2B0 = CCPR_DOS & 0b01;
        }
        else if(ADCON0bits.CHS == 3){
            POT3 = ADRESH;
            P3 = ADRESH;
            PORTCbits.RC7 = 0;
            servo_control(VAR3);
            __delay_ms(40);
            //PORTCbits.RC6 = 0;
        }
        else if(ADCON0bits.CHS == 4){
            POT4 = ADRESH;
            P4 = ADRESH;
            PORTCbits.RC7 = 1;
            servo_control(VAR4);
            __delay_ms(40);
        }
        PIR1bits.ADIF = 0;
    }
    if(INTCONbits.RBIF){
        if(!PORTBbits.RB0){
            mode=mode+1;
            if(mode == 2){
                mode = 0;
            }
        }
        INTCONbits.RBIF = 0;
    }
    return;
}

void setup (void){
    mode = 0;
    ANSEL = 0b00011111;
    ANSELH = 0;
    TRISA = 0b00111111;
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTA = 0;
    PORTD = 0;
    TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
    PORTC = 0;
    //CONFIGURACIÓN DEL RELOJ INTERNO
    OSCCONbits.IRCF = 0b011;
    OSCCONbits.SCS = 1;
    //CONFIGURACIÓN ADC
    ADCON0bits.ADCS = 0b01;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.CHS = 0b0000;
    ADCON0bits.CHS = 0b0010;
    ADCON0bits.CHS = 0b0011;
    ADCON1bits.ADFM = 0;
    ADCON0bits.ADON = 1;
    __delay_us(40);
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
    
    //Configuracion PORTB
    TRISBbits.TRISB0 = 1;       // RB<2:0> como entrada (configurada con bits de control)
    TRISBbits.TRISB1 = 1; 
    TRISBbits.TRISB2 = 1;
    OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
    WPUBbits.WPUB0 = 1;         // Habilitamos resistencia de pull-up de RB<2:0>
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
    IOCBbits.IOCB0 = 1;         // Habilitamos interrupción por cambio de estado para RB<2:0>
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
     //CONFIGURACION SPI
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = 0;
          
    
   //CONFIGURACIÓN INTERRUPCIONES
    
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    return;
}

void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0010;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0010)
                ADCON0bits.CHS = 0b0011;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0011)
                ADCON0bits.CHS = 0b0100;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0100)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            
            __delay_us(40);                 // Tiempo de adquisici n?
            
            ADCON0bits.GO = 1;             
        }
        switch(mode){
            case(0): //Modo de control manual y grabado de velocidad
                PORTAbits.RA6 = 1;
                VAR1 = POT1;
                VAR2 = POT2;
                VAR3 = POT3;
                VAR4 = POT4;
                PIE1bits.ADIE = 1;
                PIR1bits.ADIF = 0;
                if(!PORTBbits.RB1){
                    ESCRIBIR(0,POT1);
                    ESCRIBIR(1,POT2);
                    ESCRIBIR(2,POT3);
                    ESCRIBIR(3,POT4);
                    
                }
                if(!PORTBbits.RB2){
                    ESCRIBIR(4,POT1);
                    ESCRIBIR(5,POT2);
                    ESCRIBIR(6,POT3);
                    ESCRIBIR(7,POT4);
                  
                }
                break;
            case(1): //Modo de reproducción de velocidades
                PORTAbits.RA6 = 0;
                if(!PORTBbits.RB1){
                    VAR1 = LEER(0);
                    VAR2 = LEER(1);
                    VAR3 = LEER(2);
                    VAR4 = LEER(3);
                }
                if(!PORTBbits.RB2){
                    VAR1 = LEER(4);
                    VAR2 = LEER(5);
                    VAR3 = LEER(6);
                    VAR4 = LEER(7);
                }
                break;
            default:
                mode = 0;
        }
    }
return;
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

void servo_control(uint8_t posicion){
    SSPBUF = posicion;                  // Cargamos valor del contador al buffer
    while(!SSPSTATbits.BF){}        // Esperamos a que termine el envio
    return;
}
void ESCRIBIR (uint8_t direccion, uint8_t valores){
    EEADR = direccion;
    EEDAT = valores;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    EECON1bits.WREN = 0;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;
}
uint8_t LEER (uint8_t direccion){
    EEADR = direccion;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;
}