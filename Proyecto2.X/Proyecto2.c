/*
 * Archivo:   Proyecto2.c
 * Dispositivo: PIC16F887
 * Autor: Margareth Vela 
 * 
 * Programa: Proyecto Final
 * Hardware:Puente H y leds en PORTA, push buttons en PORTB, servos y
 * módulo UART en PORTC, y potenciómetros en PORTE.
 * 
 * Creado: Mayo 22, 2021
 * Última modificación: Junio, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------    
#include <xc.h>
#include <string.h>
#include <pic16f887.h>

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Para delay

//------------------------------------------------------------------------------
//                          Palabras de configuración
//------------------------------------------------------------------------------    
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)

//------------------------------------------------------------------------------
//                          Variables
//------------------------------------------------------------------------------
int flag2=0;
const char data = 97; //constante valor a
int flag = 1; //bandera de menu con valor inicial 1
char texto[11]; //texto de opcion 1
unsigned char opcion=0; // opcion ingresada por el usuario

int RB3_old;
int eepromVal = 0;

int addressEEPROM = 0x10;
int parpadear = 0;

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup();
void showString(char *var);// funcion para cadena de strings
void writeToEEPROM(int data, int address);

//------------------------------------------------------------------------------
//                          Código Principal
//------------------------------------------------------------------------------
void main(void) {
    setup(); // Configuraciones
    ADCON0bits.GO = 1; //La conversión ADC se ejecuta
    
    while(1)
    {
    return;
}
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt() isr(void){
    
    if (PIR1bits.RCIF){//registra los caracteres ingresados
        opcion = RCREG;
    }

    if (PIR1bits.ADIF){
               
        if(ADCON0bits.CHS == 5) { //Verifica el canal en l que se encuentra
            PORTD = ADRESH;
            CCPR1L = (PORTD>>1) + 128; //Swift y ajuste de señal
            CCP1CONbits.DC1B1 = PORTDbits.RD0;
            CCP1CONbits.DC1B0 = ADRESL>>7;}
        
        else{
            PORTD = ADRESH;
            CCPR2L = (PORTD>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = PORTDbits.RD0;
            CCP2CONbits.DC2B0 = ADRESL>>7;}
        
        PIR1bits.ADIF = 0; //Se limpia la bandera de ADC
    }
    
    if(INTCONbits.RBIF){
        if (PORTBbits.RB0 == 0){ //Primer motor DC -> izquierda
            if (flag2){
                PORTA = 10;
                __delay_ms(250);
                PORTA = 8;
            }
            else {
                PORTA = 2;
                __delay_ms(250);
                PORTA = 0;
            }
        }
        
        if(PORTBbits.RB1 == 0) { //Primer motor DC -> derecha
            if (flag2){
                PORTA = 9;
                __delay_ms(250);
                PORTA = 8;
            }
            else {
                PORTA = 1;
                __delay_ms(250);
                PORTA = 0;
            }
        }
        if(PORTBbits.RB2 == 0) {//Segundo motor DC -> adelante
            PORTA = 8; 
            flag2 = 1;
            PORTAbits.RA4 = 1;
            PORTAbits.RA5 = 1;
        }
        if(PORTBbits.RB3 == 0) {//Primer motor DC -> retroceso                          
           PORTA = 4; 
           flag2 = 1;
           PORTAbits.RA6 = 1;
           PORTAbits.RA7 = 1;       
        }

        if(PORTBbits.RB3 == 1 && PORTBbits.RB2 == 1) { //Carro apagado
            PORTA = 0; 
            flag2 = 0;      
        }
   
    INTCONbits.RBIF = 0;   
}
 return;   
}


void setup(){
    
    //Configuraciones de reloj
    OSCCONbits.IRCF2 =1 ; // IRCF = 111 (8MHz) 
    OSCCONbits.IRCF1 =1 ;
    OSCCONbits.IRCF0 =1 ;
    OSCCONbits.SCS = 1; // Habilitar reloj interno
    
    //Configuraciones de entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0X70; //Primeros dos pines con entradas analógicas
    
    TRISA = 0x00; //Para puente H y leds
    TRISB = 0xFF; // habilitar pines 
    TRISC = 0xB9; //Para servos
    TRISD = 0x00; 
    TRISE = 0x03; //Para entrada de los potenciometros
  
    OPTION_REGbits.nRBPU =  0 ; // se habilita el pull up interno en PORTB
    WPUB = 0xFF;  // se habilita los pull ups para los pines RB0, RB1 y RB2
    
    PORTA = 0x00;
    PORTB = 0x0F; // se limpian las salidas de los puertos y valores iniciales
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00; //Se limpian los puertos    
    
    //Configuracion ADC
    ADCON1bits.ADFM = 0; //Justificar a la izquierda
    ADCON1bits.VCFG0 = 0; //Vss
    ADCON1bits.VCFG1 = 0; //VDD
    
    ADCON0bits.ADCS = 0b10; //ADC oscilador -> Fosc/32
    ADCON0bits.CHS = 5;     //Comenzar en primer canal
    __delay_us(50);        
    ADCON0bits.ADON = 1;    //Habilitar la conversión ADC
    
    //Configuracion PWM
    PR2 = 250; //Valor inicial de PR2
    CCP1CONbits.P1M = 0; //PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; //Se habilita PWM   
    CCP2CONbits.CCP2M = 0b00001100;   
    
    CCPR1L = 0x0F; 
    CCPR2L = 0x0F;
    CCP1CONbits.DC1B = 0; //Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0; //Se limpia la bandera
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); //Se espera una interrupción
    PIR1bits.TMR2IF = 0;
      
    //Configuración de TX y RX
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 207;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1;
    
    //Configuracion Interrupciones
    INTCONbits.GIE = 1; // Se habilitan las interrupciones globales
    
    INTCONbits.RBIE = 1; // habilitar banderas de interrupción puertos B
    INTCONbits.RBIF = 0; 	
    
    INTCONbits.PEIE = 1; //Enable interrupciones periféricas
    PIE1bits.ADIE = 1;   //Enable interrupción ADC
    PIR1bits.ADIF = 0;   //Se limpia bandera de interrupción ADC
       
    PIE1bits.RCIE = 1;// modulo eusart
    PIR1bits.RCIF = 0;
    
    //Interrupt on change
    IOCB = 0x0F; // setear interrupciones en los pines RB0, RB1 y RB2    
    INTCONbits.RBIF = 0;
    
    return;
}

void showString(char *var){ //subrutina de formacion de cadena de caracteres
    int i;
       
    for (i = 0; i < strlen(var); i++) { //bucle en donde lee el array de char y
        TXREG = var[i]; //lo mueve a la consola
        __delay_ms(5);
    }
    
    TXREG = 13;
    __delay_ms(5);
    TXREG = 11;
}