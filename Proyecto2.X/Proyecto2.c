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
 * Última modificación: Junio 04, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------    
#include <xc.h>
#include <string.h>
#include <stdint.h>
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
int flag2 = 0; //bandera para motor DC
int flag = 1; //bandera de menu USART
unsigned char opcion=0; // opcion ingresada por el usuario
unsigned char temp_posicion1; //posicion temporal del servo 1
unsigned char temp_posicion2; //posicion temporal del servo 2
uint8_t valor_pot; //Para mostrar valor del potenciometro
uint8_t var_temp; //Para conversion a decimal
uint8_t centenas; 
uint8_t decenas;
uint8_t decenas_temp;
uint8_t unidades;
int RB3_old; //Antirebote
unsigned char read_EEPROM = 0x10; //valor de direccion de EEPROM
uint8_t eepromVal; //Valor a ingresar a la EEPROM
uint8_t addressEEPROM = 0x10; //valor de direccion de EEPROM

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void); //Configuraciones
void String_Completo(char *var);//Cadena de strings
void writeToEEPROM(uint8_t data, uint8_t address); //Escribir en EEPROM
uint8_t readFromEEPROM(uint8_t address); //Leer en EEPROM
uint8_t decimal(uint8_t val); //Para convertir en decimal

//------------------------------------------------------------------------------
//                          Código Principal
//------------------------------------------------------------------------------
void main(void) {
    setup(); // Configuraciones
    ADCON0bits.GO = 1; //La conversión ADC se ejecuta
    
    while(1)
    { 
        //Módulo ADC
        if(ADCON0bits.GO == 0){ //Si la conversión ya está terminada
            if (ADCON0bits.CHS == 5){ //Si está en el primer canal,
                ADCON0bits.CHS = 6;}  //pasa al segundo canal
            else if (ADCON0bits.CHS == 6){ //Si está en el segundo canal,
                ADCON0bits.CHS = 5;}  //pasa al primer canal
            else if (ADCON0bits.CHS == 7){ //Si está en el tercer canal,
                ADCON0bits.CHS = 5;}  //pasa al primer canal
            __delay_us(50); //Delay para el capacitor sample/hold
            ADCON0bits.GO = 1; //Se vuelve a ejecutar la conversión ADC
        }
        
        //Módulo EUSART
        if (PIR1bits.TXIF){     
            if(flag){ //Menú para entrar al modo control USART
                String_Completo("Si desea ingresar a modo control USART presione 1");
                flag = 0;
            }
            
            if(opcion == 49){//Entra a modo control USART
                flag = 1;
                opcion = 0;
                    while(opcion != 53){

                        if(flag){ // si la bandera esta encendida mostrara el menu
                            String_Completo("Que accion desea ejecutar?");
                            String_Completo("(1)Controlar Grua");
                            String_Completo("(2)Controlar Carro");
                            String_Completo("(3)Controlar Luces");
                            String_Completo("(4)Mostrar valor del potenciometro");
                            String_Completo("(5)Salir de control por USART");
                            flag = 0;
                        }
                        if(opcion==49){ //Para controlar los servomotores
                            String_Completo("Elija la posicion del primer servo:");
                            String_Completo("(1) 0 [grados]");
                            String_Completo("(2) 90 [grados]");
                            String_Completo("(3) 180 [grados]");
                            flag = 1;
                            opcion = 0;

                            while(!opcion){}

                            if(opcion==49){
                                PORTD = 0;
                                CCPR1L = (PORTD>>1) + 128; //Swift y ajuste de señal
                                CCP1CONbits.DC1B1 = PORTDbits.RD0;
                                CCP1CONbits.DC1B0 = ADRESL>>7;
                            }
                            if(opcion==50){
                                PORTD = 128;
                                CCPR1L = (PORTD>>1) + 128; //Swift y ajuste de señal
                                CCP1CONbits.DC1B1 = PORTDbits.RD0;
                                CCP1CONbits.DC1B0 = ADRESL>>7;
                            }
                            if(opcion==51){
                                PORTD = 255;
                                CCPR1L = (PORTD>>1) + 120; //Swift y ajuste de señal
                                CCP1CONbits.DC1B1 = PORTDbits.RD0;
                                CCP1CONbits.DC1B0 = ADRESL>>7;
                            }

                            String_Completo("Elija la posicion del segundo servo:");
                            String_Completo("(1) 0 [grados]");
                            String_Completo("(2) 90 [grados]");
                            String_Completo("(3) 180 [grados]");
                            flag = 1;
                            opcion = 0;

                            while(!opcion){}

                            if(opcion==49){
                                PORTD = 0;
                                CCPR2L = (PORTD>>1) + 128; //Swift y ajuste de señal
                                CCP2CONbits.DC2B1 = PORTDbits.RD0;
                                CCP2CONbits.DC2B0 = ADRESL>>7;
                            }
                            if(opcion==50){
                                PORTD = 128;
                                CCPR2L = (PORTD>>1) + 128; //Swift y ajuste de señal
                                CCP2CONbits.DC2B1 = PORTDbits.RD0;
                                CCP2CONbits.DC2B0 = ADRESL>>7;
                            }
                            if(opcion==51){
                                PORTD = 255;
                                CCPR2L = (PORTD>>1) + 128; //Swift y ajuste de señal
                                CCP2CONbits.DC2B1 = PORTDbits.RD0;
                                CCP2CONbits.DC2B0 = ADRESL>>7;
                            }
                            opcion = 0;
                        }

                        if(opcion==50){ // Controlar carro
                            String_Completo("Elija una de las siguientes acciones:");
                            String_Completo("(1) Avanzar");
                            String_Completo("(2) Retroceder");
                            String_Completo("(3) Girar a la derecha");
                            String_Completo("(4) Girar a la izquierda");

                            flag = 1;
                            opcion = 0;

                            while(!opcion){}
                            if (opcion == 52){ //Primer motor DC -> izquierda
                                    PORTA = 2;
                                    __delay_ms(350);
                                    PORTA = 0;                                    
                            }

                            if(opcion == 51) { //Primer motor DC -> derecha
                                    PORTA = 1;
                                    __delay_ms(350);
                                    PORTA = 0;
                            }
                            if(opcion == 49) {//Segundo motor DC -> adelante
                                PORTA = 8; 
                                __delay_ms(3000);
                                PORTA = 0;
                            }
                            if(opcion == 50) {//Primer motor DC -> retroceso                          
                               PORTA = 4; 
                               __delay_ms(3000);
                               PORTA = 0;      
                            }                          
                            opcion = 0;
                        }

                        if (opcion==51){ //Controlar luces
                            String_Completo("Elija una de las siguientes acciones:");
                            String_Completo("(1) Encender luces delanteras");
                            String_Completo("(2) Encender luces traseras");
                            String_Completo("(3) Parpadeo");

                            flag = 1;
                            opcion = 0;

                            while(!opcion){}

                            if(opcion == 49){
                                PORTA = 0;
                                PORTAbits.RA6 = 1; //Se encienden las luces delanteras
                                __delay_ms(3000);
                                PORTA = 0;
                            }
                            if(opcion == 50){
                                PORTA = 0;
                                PORTAbits.RA7 = 1; //Se encienden las luces traseras
                                __delay_ms(3000);
                                PORTA = 0;
                            }
                            if(opcion == 51){
                                PORTA = 0xF0; //Parpadeo
                                __delay_ms(500);
                                PORTA = 0;
                                __delay_ms(500);
                                PORTA = 0xF0;
                                __delay_ms(500);
                                PORTA = 0;
                                __delay_ms(500);
                                PORTA = 0xF0;
                                __delay_ms(500);
                                PORTA = 0;
                                __delay_ms(500);
                                PORTA = 0xF0;
                                __delay_ms(500);
                                PORTA = 0;
                            }      
                            opcion = 0;
                        } 

                        if (opcion==52){ //Muestra el valor del POT3
                            flag = 1;                           
                            ADCON0bits.CHS = 7; //Ingresa al canal del POT3
                            __delay_us(50);
                            ADCON0bits.GO = 1; //Se vuelve a ejecutar la conversión ADC
                            __delay_us(50); //Delay para el capacitor sample/hold
                            valor_pot = ADRESH;
                            __delay_us(50);

                            var_temp = valor_pot;
                            centenas = var_temp/100; //Se divide por 100 para obtener las centenas
                            decenas_temp = var_temp%100;//El residuo se almacena en la variable temporal de decenas
                            decenas = decenas_temp/10;//Se divide en 10 el valor de decenas_temp 
                            unidades = var_temp%10;//El residuo se almacena en unidades 

                            String_Completo("Valor del potenciometro:");
                            TXREG = decimal(centenas); //Se convierte el valor a decimal
                            __delay_ms(10);
                            TXREG = decimal(decenas);
                            __delay_ms(10);
                            TXREG = decimal(unidades);
                            __delay_ms(10);

                            TXREG = 13;
                            __delay_ms(10);
                            TXREG = 11;

                            opcion = 0;
                        } 
                    }

                    if(opcion==53){//Sale del modo control USART
                       String_Completo("Si desea ingresar a modo control USART presione 1");
                       flag = 0;
                    }   
                }         
        }
    
        if (PORTBbits.RB4 == 0){ //Funciona para antirebote
            RB3_old = 1;
        }
        
        if(PORTBbits.RB4 == 1 && RB3_old==1){
            eepromVal = temp_posicion1;  //Valor del primer servo         
            writeToEEPROM(eepromVal,addressEEPROM); //Se escribe en la EEPROM            
            if(addressEEPROM == 0x17){ 
                addressEEPROM = 0x10;
            }else{
                addressEEPROM = addressEEPROM + 1;
            }           
            __delay_ms(10);
            eepromVal = temp_posicion2; //Valor del segundo servo           
            writeToEEPROM(eepromVal,addressEEPROM); //Se escribe en la EEPROM           
            if(addressEEPROM == 0x17){
                addressEEPROM = 0x10;
            }else{
                addressEEPROM = addressEEPROM + 1;
            }            
            RB3_old = 0;
        }
    }
    return;
}

//------------------------------------------------------------------------------
//                          Interrupciones
//------------------------------------------------------------------------------
void __interrupt() isr(void){
    
    if (PIR1bits.RCIF){//registra los caracteres ingresados
        opcion = RCREG;
    }

    if (PIR1bits.ADIF){
               
        if(ADCON0bits.CHS == 5) { //Entra al canal 1
            PORTD = ADRESH;
            temp_posicion1 = PORTD;
            CCPR1L = (PORTD>>1) + 128; //Swift y ajuste de señal
            CCP1CONbits.DC1B1 = PORTDbits.RD0;
            CCP1CONbits.DC1B0 = ADRESL>>7;}
        
        else if(ADCON0bits.CHS == 6){ //Entra al canal 2
            PORTD = ADRESH;
            temp_posicion2 = PORTD;
            CCPR2L = (PORTD>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = PORTDbits.RD0;
            CCP2CONbits.DC2B0 = ADRESL>>7;}  
        PIR1bits.ADIF = 0; //Se limpia la bandera de ADC
    }
    
    if(INTCONbits.RBIF){
        if (PORTBbits.RB0 == 0){ //Primer motor DC -> izquierda
            if (flag2){ //Se mueve a la izquierda mientras avanza/retrocede
                PORTA = 10;
                __delay_ms(350);
                PORTA = 8;
            }
            else { //Se mueve a la izquierda
                PORTA = 2;
                __delay_ms(350);
                PORTA = 0;
            }
        }
        
        if(PORTBbits.RB1 == 0) { //Primer motor DC -> derecha
            if (flag2){//Se mueve a la derecha mientras avanza/retrocede
                PORTA = 9;
                __delay_ms(350);
                PORTA = 8;
            }
            else { //Se mueve a la izquierda
                PORTA = 1;
                __delay_ms(350);
                PORTA = 0;
            }
        }
        if(PORTBbits.RB2 == 0) {//Segundo motor DC -> adelante
            PORTA = 8; 
            flag2 = 1;
            PORTAbits.RA6 = 1;
        }
        if(PORTBbits.RB3 == 0) {//Primer motor DC -> retroceso                          
           PORTA = 4; 
           flag2 = 1;
           PORTAbits.RA7 = 1;       
        }

        if(PORTBbits.RB3 == 1 && PORTBbits.RB2 == 1) { //Carro apagado
            PORTA = 0; 
            flag2 = 0;      
        }
        
        if(PORTBbits.RB5 == 0){
            
            PORTD = readFromEEPROM(read_EEPROM); //Lee el valor de la dirección en la EEPROM       
            if(read_EEPROM == 0x17){ //Valor máximo de lectura en la dirección 0x17
                    read_EEPROM = 0x10;
            }else{
                read_EEPROM++;//Incrementa la dirección
            }
            //El primer servo se coloca en la posición guardada
            CCPR1L = (PORTD>>1) + 120;//Swift y ajuste de señal
            CCP1CONbits.DC1B1 = PORTDbits.RD0;
            CCP1CONbits.DC1B0 = ADRESL>>7;

            __delay_ms(10);
            
            //Lee el valor de la dirección en la EEPROM
            PORTD = readFromEEPROM(read_EEPROM);
            if(read_EEPROM == 0x17){ //Valor máximo de lectura en la dirección 0x17
                    read_EEPROM = 0x10;
            }else{
                read_EEPROM++; //Incrementa la dirección
            }
            //El segundo servo se coloca en la posición guardada
            CCPR2L = (PORTD>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = PORTDbits.RD0;
            CCP2CONbits.DC2B0 = ADRESL>>7;
            __delay_ms(5000);       
    }
   
    INTCONbits.RBIF = 0;   //Se limpia la bandera
}
 return;   
}

//------------------------------------------------------------------------------
//                          Subrutinas
//------------------------------------------------------------------------------
uint8_t decimal(uint8_t val){ //Conversion a decimal
    if(val==0){
        return 48;
    }else if(val==1){
        return 49;
    }else if(val==2){
        return 50;
    }else if(val==3){
        return 51;
    }else if(val==4){
        return 52;
    }else if(val==5){
        return 53;
    }else if(val==6){
        return 54;
    }else if(val==7){
        return 55;
    }else if(val==8){
        return 56;
    }else if(val==9){
        return 57;
    }
}

void String_Completo(char *var){
    int i;   
    for (i = 0; i < strlen(var); i++) {
         __delay_ms(10); 
        TXREG = var[i]; //Se transmite caracter por caracter en la terminal
    }
    TXREG = 13; //Agrega una nueva línea
    __delay_ms(10);
    TXREG = 11;
} 

void writeToEEPROM(uint8_t data, uint8_t address){
    EEADR = address; //Se guarda en la dirección deseada
    EEDAT = data; //Se guarda el dato deseado
    EECON1bits.EEPGD = 0; //Entra a la memoria de programa
    EECON1bits.WREN = 1; //Bit de escritura encendido
    
    //deshabilitar interrupciones
    INTCONbits.GIE = 0;
    EECON2 = 0x55; //Secuencia requerida para escribir en EEPROM
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1; //Inicia ciclo de escritura
   
    //se termina la escritura y reenciende interrupciones
    EECON1bits.WREN = 0; 
    INTCONbits.GIE = 1;    
    return;
}


uint8_t readFromEEPROM(uint8_t address){
 EEADR = address; //Entra a la dirección
 EECON1bits.EEPGD = 0; //Entra a la memoria de datos
 EECON1bits.RD = 1; //Inicia ciclo de lectura
 uint8_t data = EEDATA; //Se guarda el valor del dato 
 return data; //Regresa el dato almacenado
}

//------------------------------------------------------------------------------
//                          Configuración
//------------------------------------------------------------------------------
void setup(){
    
    //Configuraciones de reloj
    OSCCONbits.IRCF2 =1 ; // IRCF = 111 (8MHz) 
    OSCCONbits.IRCF1 =1 ;
    OSCCONbits.IRCF0 =1 ;
    OSCCONbits.SCS = 1; // Habilitar reloj interno
    
    //Configuraciones de entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0XE0; //Primeros dos pines con entradas analógicas
    
    TRISA = 0x00; //Para puente H y leds
    TRISB = 0xFF; //Habilitar pines 
    TRISC = 0xB9; //Para servos
    TRISD = 0x00; 
    TRISE = 0x07; //Para entrada de los potenciometros
  
    OPTION_REGbits.nRBPU =  0 ; //Se habilita el pull-up interno en PORTB
    WPUB = 0xFF;  //Se habilita los pull ups para los pines
    
    PORTA = 0x00; //Se limpian las salidas de los puertos y valores iniciales
    PORTB = 0xFF; 
    PORTC = 0x00;    
    PORTD = 0x00;
    PORTE = 0x00;    
    
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
       
    PIE1bits.RCIE = 1;// modulo EUSART
    PIR1bits.RCIF = 0;
    
    //Interrupt on change
    IOCB = 0xFF; // setear interrupciones de todos los pines   
    INTCONbits.RBIF = 0;
    
    return;
}