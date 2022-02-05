
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
#pragma config IOL1WAY = ON          // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
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
#include <stddef.h>     //Defines NULL
#include <stdbool.h>    // Defines true
#include <math.h>


#define LED_VERDE LATAbits.LATA0
#define LED_ROJO LATAbits.LATA1

#define LED_ON_STATE 1
#define LED_OFF_STATE 0
#define baudios_9600 1041

//Frecuencia de cpu de 40Mhz

void timer1_config(void){
    T1CONbits.TON   = 0; //Inicialmente el temporizador lo ponemos apagado
    T1CONbits.TSIDL = 0;
    T1CONbits.TCKPS = 3;    //Prescaler de 256
    T1CONbits.TCS   = 0;
    T1CONbits.TSYNC = 0;
    
    //Prioridades, subprioridades e interrupciones referentes al timer
    IPC0bits.T1IP = 5; 
    IFS0bits.T1IF = 0; 
    IEC0bits.T1IE = 1; 
    
    PR1  = 0x0005;
    TMR1 = 0;
    
    T1CONbits.TON   = 1;   //Inicializar  temporizador
}

void uart1_config(unsigned short baud){
    
    // Asignar mi módulo UART a un puerto/pin de comunicacion
    TRISCbits.TRISC0 = 1;       // Pin C0 como puerto de entrada (digital)  
    RPINR18bits.U1RXR = 16;     // Pin RC0 conectado al puerto de recepción Uart1  U1RX
    RPOR8bits.RP17R = 3;        // Pin RC1 conectado al pin de transmision de la Uart1  U1TX
   
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
            
    //TX
    IPC3bits.U1TXIP = 5; // Prioridad de nivel 5 (máximo 7)
    IFS0bits.U1TXIF = 0; // Reset TX interrupt flag
    IEC0bits.U1TXIE = 0;  // Enable TX interrupt
    
    U1MODEbits.UARTEN = 1;      // UART habilitado
    
}

void delay_ms(unsigned long time_ms){
    unsigned long u;
    for(u=0; u< time_ms*450; u++){
        asm("NOP");
    }
}


unsigned char recieved_char, dummy;

char txbuffer_ISR[200], dataCMD_ISR[50]; 

unsigned int nextchar = 0;
unsigned char BufferLoadDone = 1;

volatile unsigned int data_count = 0;
volatile unsigned int comando_detectado = 0; 

//Método de guardado de comandos

const char cmd1[] = {"set MIC2_ledgreen 1"};
const char cmd2[] = {"set MIC2_ledgreen 0"};
const char cmd3[] = {"set MIC2_ledred 1"};
const char cmd4[] = {"set MIC2_ledred 0"};
const char cmd5[] = {"print MIC2_data"};
const char cmd6[] = {"stop MIC2_data"};


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
    
   TRISAbits.TRISA0 = 0; //Pin A0 configurado como pin de salida (output) conectado a un LED verde
   TRISAbits.TRISA1 = 0; //Pin A1 configurado como pin de salida (output) conectado a un LED rojo
    
    
    timer1_config();
    uart1_config(baudios_9600);

    while(1){   
     if(comando_detectado){
                // Caso del maestro con sus comandos correspondientes
                if ( !strcmp ( ((const char*)dataCMD_ISR), cmd1) ){
                    //0x50 0x01 0xAA
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                     sprintf(txbuffer_ISR,"%c%c%c", 0x50, 0x01, 0xAA);

                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1;   

                }
                else if ( !strcmp ( ((const char*)dataCMD_ISR), cmd2) ){
                    //0x50 0x00 0xAA
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                    sprintf(txbuffer_ISR,"%c%c%c", 0x50, 0x00, 0xAA); 

                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1;                    

                }
                else if ( !strcmp ( ((const char*)dataCMD_ISR), cmd3) ){
                    //0x51 0x01 0xAA
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                    sprintf(txbuffer_ISR,"%c%c%c", 0x51, 0x01, 0xAA);

                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1; 
       
                }
                else if ( !strcmp ( ((const char*)dataCMD_ISR), cmd4) ){
                    //0x51 0x00 0xAA
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                    sprintf(txbuffer_ISR,"%c%c%c", 0x51, 0x00, 0xAA);

                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1; 
      
                }
                else if ( !strcmp (((const char*) dataCMD_ISR), cmd5) ){
                    //0x52 0x01 0xAA
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                    sprintf(txbuffer_ISR,"%c%c%c", 0x52, 0x01, 0xAA); 
                    
                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1; 
                       
                }
                else if ( !strcmp (((const char* )dataCMD_ISR), cmd6) ){
                    //0x52 0x00 0xAA
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                    sprintf(txbuffer_ISR,"%c%c%c", 0x52, 0x00, 0xAA);

                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1; 
                } else{
                     memset(txbuffer_ISR,'\0', sizeof(txbuffer_ISR));
                    sprintf(txbuffer_ISR,"COMANDO NO EXISTE \r\n"); 
                    
                    if(U1STAbits.UTXBF)IFS0bits.U1TXIF = 0; //Reseteo el flag de transmisión ISR
                    asm("nop");
                    IEC0bits.U1TXIE = 1;

                }
                //Se resetea el buffer para evitar problemas al comparar futuras tramas de datos
                memset(dataCMD_ISR,'\0', sizeof(dataCMD_ISR));
                data_count = 0;
                comando_detectado = 0;
     } 

    
        delay_ms(100);

        

    }
    
      
    return 0;
}

//Interrupción persistente
void __attribute__ ((__interrupt__, no_auto_psv)) _U1TXInterrupt (void){ // Actualmente configurado para U1STAbits.UTXISEL=0
        IEC0bits.U1TXIE = 0;      // Disable UART1 Tx Interrupt
        if(!U1STAbits.UTXBF){      // Mientras el buffer de transmisión NO se encuentre completo,
                U1TXREG = txbuffer_ISR[nextchar++]; // Cargar el buffer con un nuevo dato.
                asm ("nop");
                if(U1STAbits.UTXBF){         // Si el buffer de transmision se completó con el último dato incorporado al buffer,
                     IFS0bits.U1TXIF = 0;         // Clear UART1 Tx Interrupt Flag
                }
        }else IFS0bits.U1TXIF = 0;     // Clear UART1 Tx Interrupt Flag
        
        if (nextchar == 3){
            
            LATAbits.LATA0 = !PORTAbits.RA0; // Para comprobar que esta transmitiendo
            nextchar = 0;
            BufferLoadDone = 1;       // Informamos de que se ha terminado de cargar la cadena de texto de 'Ul_TxBuffer_ISF
        }else IEC0bits.U1TXIE = 1;      // Enable. UART1 Tx Interrupt
}

//Interrupción no persistente
void __attribute__ ((__interrupt__, no_auto_psv)) _T1Interrupt (void){ 
    IFS0bits.T1IF = 0;      // Reset Timer1 Interrupt
}
 

//Interrupción persistente
void __attribute__((__interrupt__, no_auto_psv))_U1RXInterrupt (void){
        if (comando_detectado == 0){
                dataCMD_ISR[data_count] = U1RXREG; // Obtener caracter recibido en el buffer
                data_count++;
                if(dataCMD_ISR[data_count-1] == 13 ) // Retorno de carro-Intro detectad
                {   
                        // Eliminar retorno de carro del str
                        dataCMD_ISR[data_count - 1] = '\0';
                        comando_detectado = 1;
                        data_count = 0;
                        
                }
       
        }else{
            dummy = U1RXREG;
        }
        IFS0bits.U1RXIF =0;        // Reset Rx Interrupt
}

