
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


#define baudios_9600 1041

//Frecuencia de cpu de 40Mhz

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


void spi_config(void){
    /*Configuracion de pines SPI*/
    TRISCbits.TRISC0 = 1;   //MISO asignado a RC0 (PIN 25)
    TRISCbits.TRISC1 = 0;   //MOSI asignado a RC1 (PIN 26)
    TRISCbits.TRISC2 = 0;   //SCK asignado a RC2 (PIN 27)
    TRISCbits.TRISC3 = 0;   //CS asignado a RC3 (PIN 36)
    
    LATCbits.LATC1 = 0;
    
    RPINR20bits.SDI1R = 16;  //RC0 trabajando como MISO (entrada)
    RPOR8bits.RP17R    = 7;  // (00111);    RC1 es MOSI (salida)
    RPOR9bits.RP18R   =  8;  // (01000);    RC2 es SCK   (salida)
    
    /*Configuracion SPISTAT*/
    SPI1STATbits.SPIEN  = 0;
    SPI1STATbits.SPISIDL  = 0;
    SPI1STATbits.SPIROV  = 0;
    SPI1STATbits.SPITBF  = 0;
    SPI1STATbits.SPIRBF  = 0;
    
    /*Configuracion SPICON1*/
    SPI1CON1bits.DISSCK = 0;
    SPI1CON1bits.DISSDO = 0;
    SPI1CON1bits.MODE16 = 0;
    SPI1CON1bits.SMP = 0;
    
    SPI1CON1bits.CKE = 0;
    SPI1CON1bits.CKP = 0; //SPI MODE 0
    
    SPI1CON1bits.SSEN  = 0;  //HARDWARE SS NOT USED
    SPI1CON1bits.MSTEN = 1;  //MASTER MODE
    
    SPI1CON1bits.PPRE = 1;  // 1:1
    SPI1CON1bits.SPRE = 6;  // 2:1  //Fspi = Fcpu/2 = 1MHz/1Mbps
    
    /*Configuracion SPICON2. Framed Mode Disabled*/
    SPI1CON2bits.FRMEN  = 0;
    SPI1CON2bits.SPIFSD = 0;
    SPI1CON2bits.FRMPOL = 0;
    SPI1CON2bits.FRMDLY = 0;
    
    /*SPI Interrupts*/
    IFS0bits.SPI1IF  = 0;
    IFS0bits.SPI1EIF = 0;
    IEC0bits.SPI1IE  = 0;
    IEC0bits.SPI1EIE = 0;
    IPC2bits.SPI1IP  = 6;
    IPC2bits.SPI1EIP = 6;
    
    LATCbits.LATC3 = 1;
    SPI1STATbits.SPIEN = 1;
}

unsigned char spi_write(unsigned char data){
    while(SPI1STATbits.SPITBF);     //Esperamos a que el buffer de transmision este libre
    SPI1BUF = data;                 //Cargamos dato al buffer
    while(!SPI1STATbits.SPIRBF);     // Esperamos a recibi el dato de vuelta.
    
    return SPI1BUF;
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
char dir[16], recieved_char, dummy, dat[8];

unsigned int letra = 0, cont = 0, loadDone= 0;
 



char exp_data[10] = {1,2,3,4,5,6,7,8,9,10};
char mem_data[10]  = {0,0,0,0,0,0,0,0,0,0}, mem_address[10] = {0,0,0,0,0,0,0,0,0,0};

//4Mhz frecuencia de la CPU
int main(void) { 

    //Fosc = Fin*M/(N1*N2)
    //Fcpu = Fosc/2
    //Fcpu = 4Mhz ---> Fosc = Fcpu * 2 = 4M * 2 = 8Mhz de frecuencia de oscilador
    PLLFBD = 38; 
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;
    while(OSCCONbits.LOCK != 1);
    
    // Control de pin para trabajar analogico/digital
    AD1PCFGL = 0xFFFF; //Todos los pines estan configurados como pines digitales.
    
   
    // Control de direccionalidad de los pines de mi proyecto
    
   TRISBbits.TRISB0 = 0; //Pin B0 configurado como pin de salida (output) conectado a un LED 1
   TRISBbits.TRISB1 = 0; //Pin B1 configurado como pin de salida (output) conectado a un LED 2
   LATBbits.LATB0 = 0;  //Pin RB0 a nivel bajo por defecto(LED1 apagado)
   LATBbits.LATB1 = 0;  //Pin RB1 a nivel bajo por defecto(LED1 apagado)
    
    
    uart1_config(baudios_9600);
    
    spi_config();
    
    /*Habilitar Interrupciones Globales*/
    INTCON1bits.NSTDIS = 0; //Interrupt Nestinf Enabled
    SRbits.IPL = 0;       //Enable Global Interrupts
    
    while(1){   
        if(loadDone == 0){
            //Secuencia de habilitacion (WRITE ENABLE)
            LATCbits.LATC3 = 0;
            delay_ms(5);

            spi_write(0x06);

            delay_ms(5);
            LATCbits.LATC3 = 1;
            
            delay_ms(1000);

            //Operacion de escritura
            LATCbits.LATC3 = 0;
            delay_ms(5);

            spi_write(0x02);  // Instruccion de tipo escritura

            // Direccion de memoria
            spi_write(0x00);  //MSB
            spi_write(cont+1);  //LSB
            mem_address[cont] =cont+1;
            
            //Escribimos el valor
            spi_write(exp_data[cont]);

            delay_ms(5);
            LATCbits.LATC3 = 1;;
            
            delay_ms(5000);
            
            //Operacion de lectura 

            LATCbits.LATC3 = 0;
            delay_ms(5);

            spi_write(0x03);  //Instruccion de tipo lectura

           // Direccion de memoria
            spi_write(0x00);  //MSB
            spi_write(cont+1);  //LSB
            
            
            mem_data[cont] = spi_write(0x00);
            
            delay_ms(5);
            LATCbits.LATC3 = 1;
            
            cont++;
            delay_ms(1000);
            
            if(cont == 10){
                sprintf(txbuffer, "Carga de los 10 datos realizada \r\n");
                EnviarString(txbuffer);
                cont = 0;
                loadDone = 1;
            }
        }
        if(loadDone && letra){

        sprintf(txbuffer, "Dato en memoria: %02X, Address: %04X, Expected: %02X \r\n", mem_data[cont], mem_address[cont], exp_data[cont]);
            EnviarString(txbuffer);
            cont++;
            
            if(mem_data[cont] != exp_data[cont]) LATBbits.LATB0 = 1;
            
            if(cont == 10){
                cont = 0;
                letra = 0;
                loadDone = 0;
                if(PORTBbits.RB0 == 0)LATBbits.LATB1 = !PORTBbits.RB1;;
            }
        }
    }
    
     
    return 0;
}



 
void __attribute__((__interrupt__, no_auto_psv))_U1RXInterrupt (void){
    if(letra== 0){
        recieved_char = U1RXREG;
        letra = 1;
    }else{
         dummy = U1RXREG;
    }
    IFS0bits.U1RXIF = 0;
}



