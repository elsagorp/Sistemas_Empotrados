
// DSPIC33FJ32MC204 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = EC            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF          // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


//#define pulso1 PORTCbits.RC0
//#define pulso2 PORTCbits.RC1

#define baudios_9600 1041

//Frecuencia de cpu de 4Mhz

void uart1_config(unsigned short baud){
    
    // Asignar mi módulo UART a un puerto/pin de comunicacion
    TRISBbits.TRISB2 = 1;       // Pin B2 como puerto de entrada (digital)  
    RPINR18bits.U1RXR = 2;     // Pin RB2 conectado al puerto de recepción Uart1  U1RX
    RPOR1bits.RP3R = 3;        // Pin RB3 conectado al pin de transmision de la Uart1  U1TX
   
    //Configuracion del registro U1MODE
        
    //U1MODEbits.UARTEN = 0;     //UART deshabilitado por defecto al principio de la configuración
    U1MODEbits.USIDL  = 0;
    U1MODEbits.IREN   = 0;
    U1MODEbits.RTSMD  = 1;
    U1MODEbits.UEN    = 0;    //Solo usamos pi de TX y Rx para nuestro proyecto
    U1MODEbits.WAKE   = 0;
    U1MODEbits.LPBACK = 0;    // Loopback deshabilitado
    U1MODEbits.ABAUD  = 0;    // Auto baud rate deshabilitado
    
    
    U1MODEbits.URXINV = 0;    // El estado en reoso (IDLE) es un '1'
    U1MODEbits.BRGH   = 1;    // Modo High-Speed
    U1MODEbits.PDSEL  = 0;    // 8 bits de datos (8N) y paridad nula
    U1MODEbits.STSEL  = 0;    // 1-bit de Stop al final de la trama de datos (8N1)

    
    //Configuracion del registro U1STA
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;  //Tema de interrupciones
    

  
    U1STAbits.ADDEN    = 0;
    U1STAbits.UTXBRK   = 0;     // No usar trama de sincronización
    U1STAbits.UTXINV   = 1;     // El estado en reoso (IDLE) es un '1'
    U1STAbits.OERR     = 0;     // Buffer de recepción esta vacío y no hay problema de overflow
    U1STAbits.UTXEN    = 1;
    
    //Configuramos la velocidad de transmisión
    U1BRG = baud;
    
    //Prioridades, subprioridades e interrupciones referentes a la Uart
    //RX
    IPC2bits.U1RXIP = 6; // Prioridad de nivel 6 (máximo 7)
    IFS0bits.U1RXIF = 0; // Reset RX interrupt flag
    IEC0bits.U1RXIE = 1;  // Enable RX interrupt
    
    U1MODEbits.UARTEN = 1;      // UART habilitado
    
}

void output_compare_config(void){
   /* Configuracion de pines OC */
   TRISCbits.TRISC0 = 0;
   TRISCbits.TRISC1 = 0;
   RPOR8bits.RP16R = 0x12;      // OC1 conectado al pin RP16 (RC0)
   RPOR8bits.RP17R = 0x13;     // OC2 conectado al pin RP17 (RC1)

   /* Inícializacion modulo OC */
    OC1CONbits.OCM = 0;               //Deshabilitar PWM
    OC1CONbits.OCTSEL =  0;          // Timer2 coma base de tiempos para el oC
    OC1R = 1599;                      // 40% de ciclo de rabajo
    OC1RS = 1599;                    // ton = 0,4ms 400microsegundos

    OC2CONbits.OCM = 0;               // Deshabilitar PWM
    OC2CONbits.OCTSEL = 0;             // Timer2 como base de tiempos para el OC
    OC2R = 2799;                     // 70% 
    OC2RS = 2799;                     // ton = 0,7ms  700microsegundos

   /* Configuracion del Timer2 */
    T2CONbits.TON  = 0;            // Deshabilitar Timer2
    T2CONbits.TCS   = 0;            // Seleccionar clock interno
    T2CONbits.TGATE  = 0;          // Gated Timer mode deshabilitado
    T2CONbits.T32 = 0;               //Timer2 modo de 16-bit de resolucion
    T2CONbits.TCKPS = 0;           // Prescaler 1:1                              


/* Configurar registros de interrupcion del Timer2*/
IPC1bits.T2IP = 5;   // Prioridad de nivel 5
IFS0bits.T2IF  = 0;       /// Reset flag
IEC0bits.T2IE = 0;   //Deshabilitar interrupcion


PR2 = 3999;           //Periodo de 1 ms tiempo =PRESCALER* (reg + 1)/Fbus
                     //Fbus = 4MHz 
                         
/*Arrancar mdulos */
OC1CONbits.OCM = 6; // Modo PWM
OC2CONbits.OCM = 6; //Modo PWM
T2CONbits. TON = 1; //Iniciar Timer 2
                   
}

void input_capture_config(void)
{
   /* Configuracion de pines  */
   TRISCbits.TRISC5 = 1;
   TRISCbits. TRISC6 = 1;
   
   RPINR7bits.IC1R = 21;      // IC1 conectado al pin RP21 (RC5)
   RPINR7bits.IC2R = 22;     // IC2 conectado al pin RP22 (RC6)

   /* Configurar modulos IC1 e IC2 */
   IC1CONbits.ICM = 0;      //IC dehabilitado
   IC1CONbits.ICSIDL = 0;
   IC1CONbits.ICTMR = 0;    //Timer 3 seleccionado como base de tiempo para IC
   IC1CONbits.ICI = 0;      //Interrupt on every capture
   
   IC2CONbits.ICM = 0;      //IC dehabilitado
   IC2CONbits.ICSIDL = 0;
   IC2CONbits.ICTMR = 0;    //Timer 3 seleccionado como base de tiempo para IC
   IC2CONbits.ICI = 0;      //Interrupt on every capture
   
 

   /* Configuracion del Timer3 */
    T3CONbits.TON  = 0;            // Deshabilitar Timer3
    T3CONbits.TCS   = 0;            // Seleccionar clock interno
    T3CONbits.TGATE  = 0;          // Gated Timer mode deshabilitado
    T3CONbits.TCKPS = 0;           // Prescaler 1:1                              


    
    /* Configurar registros de interrupcion del Timer3*/
    IPC2bits.T3IP = 5;   // Prioridad de nivel 5
    IFS0bits.T3IF  = 0;       /// Reset flag
    IEC0bits.T3IE = 0;   //Deshabilitar interrupcion

    IPC0bits.IC1IP = 5; // Setup IC1 interrupt priority level
    //IPC0bits.IC2IP = 5; // Setup IC1 interrupt priority level
    
    IEC0bits.IC1IE = 1;
    IEC0bits.IC2IE = 1;

    /*Arrancar modulos IC1, IC2 y Timer3 */
    IC1CONbits.ICM = 3; //Por defecto captura primero el flanco de subida
    IC2CONbits.ICM = 3; //Por defecto captura primero el flanco de subida
    T3CONbits. TON = 1; //Iniciar Timer 3
                   
}

    
 
void delay_ms(unsigned long time_ms){
    unsigned long u;
    for(u=0; u< time_ms*450; u++){
        asm("NOP");
    }
}

void EnviarCaracter(char  dato){
    
    while(U1STAbits.UTXBF); // Si esta lleno no continua
    U1TXREG = dato;
    
}

void EnviarString(char * cadena){
    while(*cadena != '\0'){
        EnviarCaracter(*cadena++); // Solo entra en el bucle si no esta vacio, es decir, si no es final de trama
    }

}


char txbuffer[200]; 
unsigned char recieved_char,dummy;

unsigned int flag = 0;

double  tiempo_real_IC1,tiempo_real_IC2; //float
unsigned int pulso1= 0 , pulso2= 0 ; 
float time_pulsoIC1, time_pulsoIC2, rise_pulsoIC1, rise_pulsoIC2;




//Cristal/Reloj externo de 8Mhz ----> 40Mhz frecuencia de la CPU
int main(void) { 

    //Fosc = Fpri*M/(N1*N2) => 8Mhz * 40/(2*2) = 80MHz
    //Fcpu = Fosc/2
    //Fcpu = 40Mhz ---> Fosc = Fcpu * 2 = 40M * 2 = 80Mhz de frecuencia de oscilador
    PLLFBD = 38; 
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;
    while(OSCCONbits.LOCK != 1);
    
    // Control de pin para trabajar analogico/digital
    AD1PCFGL = 0xFFFF; //Todos los pines estan configurados como pines digitales.
    
   
    // Control de direccionalidad de los pines de mi proyecto
    
   TRISBbits.TRISB0 = 0; //Pin B0 configurado como pin de salida (output) conectado a un LED 1
   TRISBbits.TRISB1 = 0; //Pin B1 configurado como pin de salida (output) conectado a un LED 2
   
    
    
    uart1_config(baudios_9600);
    delay_ms(100);
    
    output_compare_config(); //Configura modulo PWM
    delay_ms(100);
    
    input_capture_config();   //Configura modulo input capture
    delay_ms(100);
    
    /*Habilitar Interrupciones Globales*/
    //INTCON1bits.NSTDIS = 0; //Interrupt Nestinf Enabled
    //SRbits.IPL = 0;       //Enable Global Interrupts
    
    sprintf(txbuffer, "\r\n SIMULACION SISTEMAS EMPOTRADOS \r\n");
    EnviarString(txbuffer);
    delay_ms(100);

    while(1){   
     if(U1STAbits.OERR) U1STAbits.OERR = 0; //Si hay overflow en el buffer de recepcion, resetear uart
     if(flag){
         tiempo_real_IC1 = 1.0*((double)time_pulsoIC1)/4000.0;
         tiempo_real_IC2 = 1.0*((double)time_pulsoIC2)/4000.0;
         
         sprintf(txbuffer, "Time_IC1: %05.3fms  Time_IC2: %05.3fms  \r\n", tiempo_real_IC1,tiempo_real_IC2);
         EnviarString(txbuffer);
         
         IFS0bits.U1RXIF = 0;
         IEC0bits.U1RXIE = 1;
         
         flag= 0;
         delay_ms(100);
     }      

    }
    
    return 0;
}


void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt (void){    
        if(pulso1 == 0){      
            rise_pulsoIC1 = IC1BUF; //Buffer con cuatro posiciones
            IC1CONbits.ICM = 2;  // Capture de next falling edge
            pulso1 = 1;
        }else{
            time_pulsoIC1 = IC1BUF - rise_pulsoIC1;
            IC1CONbits.ICM = 3;  // Capture de next rising edge
            pulso1 = 0;
        }
        IFS0bits.IC1IF = 0;     
        
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC2Interrupt (void){
        if(pulso2 == 0){      
            rise_pulsoIC2 = IC2BUF;
            IC2CONbits.ICM = 2;  // Capture de next falling edge
            pulso2 = 1;
        }else{
            time_pulsoIC2 = IC2BUF - rise_pulsoIC2;
            IC2CONbits.ICM = 3;  // Capture de next rising edge
            pulso2 = 0;
        }
        IFS0bits.IC2IF = 0;       
}

void __attribute__((__interrupt__, no_auto_psv))_U1RXInterrupt (void){
        if (flag == 0){
                recieved_char = U1RXREG; // Obtener caracter recibido en el buffer
                flag = 1;
        }else{
            dummy = U1RXREG;
        }
        IFS0bits.U1RXIF =0;        // Reset Rx Interrupt
}

