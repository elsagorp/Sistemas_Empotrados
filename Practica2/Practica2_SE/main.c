// DSPIC33FJ32MC204 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = EC   //           // Primary Oscillator Source (EC Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
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
// Use project enums instead of #define for ON and OFF

#include <xc.h>
#include <stdio.h>

#define LED_VERDE LATAbits.LATA0
#define LED_ROJO LATAbits.LATA1

#define LED_ON_STATE 1
#define LED_OFF_STATE 0
#define baudios_9600 1041

//Frecuencia de cpu de 40Mhz
//Polling
void timer1_config(void){
    T1CONbits.TON   = 0; //Inicialmente el temporizador lo ponemos apagado
    T1CONbits.TSIDL = 0;
    T1CONbits.TCKPS = 3;    //Prescaler de 256
    T1CONbits.TCS   = 0;
    T1CONbits.TSYNC = 0;
    
    PR1  = 0xFFFF;
    TMR1 = 0;
    
    T1CONbits.TON   = 1;   //Inicializar  temporizador
}


void uart_config(unsigned short baud){
    
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
    U1STAbits.URXISEL0 = 0;
    U1STAbits.URXISEL1 = 0;  //Tasa de interrupciones
    
    
    U1STAbits.ADDEN    = 0;
    U1STAbits.UTXBRK   = 0;     // No usar trama de sincronización
    U1STAbits.UTXINV   = 1;     // El estado en reoso (IDLE) es un '1'
    U1STAbits.OERR     = 0;     // Buffer de recepción esta vacío y no hay problema de overflow
    U1STAbits.PERR     = 0;
    U1STAbits.FERR     = 0;
    U1STAbits.UTXEN    = 1;
    
    U1BRG = baud;
    
    U1MODEbits.UARTEN = 1;      // UART habilitado
    
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


void parpadeo_rojo(){
    LATAbits.LATA1 = !PORTAbits.RA1;
    delay_ms(250);
}

unsigned char recieved_char;
char txbuffer[200];
unsigned int contador = 0;
unsigned int numh = 0; //contador para saber si es la primera vez que toca la tecla h o no

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
    TRISBbits.TRISB1 = 1; //Pin B1 configurado como pin de entrada (input)
    
    TRISAbits.TRISA0 = 0; //Pin A0 configurado como pin de salida (output) conectado a un LED verde
    TRISAbits.TRISA1 = 0; //Pin A1 configurado como pin de salida (output) conectado a un LED rojo
    
    LED_VERDE = LED_OFF_STATE;
    
    timer1_config();
    uart_config(baudios_9600);

    while(1){
        U1STAbits.OERR = 0;

        sprintf(txbuffer,"ELSA CONTADOR %d \r\n", contador); //sprintf no imprime sino que traduce
        EnviarString(txbuffer);
        contador++;
        
        
        if(numh == 1){ // Se sigue sin pulsar la tecla h por segunda vez
            parpadeo_rojo();
        }
//Permite determinar si existe, al menos un byte en el buffer de recepción(read-only) si es el caso se mete dentro de la condición
        if(U1STAbits.URXDA){ 
            recieved_char = U1RXREG; //El registro
            if(recieved_char == 'e'){
                LED_VERDE = LED_ON_STATE; //LATAbits.LATA0 = 1;
            }else if(recieved_char == 'a'){
                LED_VERDE = LED_OFF_STATE; //LATAbits.LATA0 = 0;
            }
            if(recieved_char == 'h'){
                if(numh == 0){
                    parpadeo_rojo();
                    numh++;
                }else{
                    LED_ROJO = LED_OFF_STATE;
                    numh = 0;
                }
            }
            if(recieved_char == ' '){
                contador = 0;
            }
        }
        
        
        
        TMR1 = 0;
        while(TMR1 <= 62499);

    }
    
    return 0;
}
