
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
#pragma config POSCMD = EC              // Primary Oscillator Source (EC Oscillator Mode)
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
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define LED_VERDE LATAbits.LATA0
#define LED_VERDE2 LATAbits.LATA1

#define LED_ON_STATE 1
#define LED_OFF_STATE 0

void delay_ms(unsigned long time_ms){
    unsigned long u;
    for(u=0; u< time_ms*45; u++){
        asm("NOP");
    }
}

//Cristal/Reloj externo de 8Mhz ----> 4Mhz frecuencia de la CPU
int main(void) { 
    //Fosc = Fpri*M/(N1*N2) => 8Mhz * 4/(2*2) = 8MHz
    //Fcpu = Fosc/2
    //Fcpu = 4M ---> Fosc = Fcpu * 2 = 4M * 2 = 8Mhz de frecuencia de oscilador
    PLLFBD = 2; // M es el valor que consigues sumando en este caso 2  +2
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;
    while(OSCCONbits.LOCK != 1);
    
    // Control de pin para trabajar analogico/digital
    AD1PCFGL = 0xFFFF; //Todos los pines estan configurados como pines digitales.
    
   
    // Control de direccionalidad de los pines de mi proyecto
    TRISBbits.TRISB3 = 0; //Pin B3 configurado como pin de salida (output)
    TRISBbits.TRISB4 = 1; //Pin B4 configurado como pin de entrada (input)
    TRISBbits.TRISB7 = 1; //Pin B7 configurado como pin de entrada (input)
    
    TRISAbits.TRISA0 = 0; //Pin A0 configurado como pin de salida (output)
    TRISAbits.TRISA1 = 0; //Pin A1 configurado como pin de salida (output)
    
    // Control de puertos
    
    while(1){
        if( !PORTBbits.RB4){
                LED_VERDE = LED_ON_STATE; //LATAbits.LATA0 = 1;
                //LED_VERDE2 = LED_ON_STATE; //LATAbits.LATA1 = 1;
        }else{
                 LED_VERDE = LED_OFF_STATE; //LATAbits.LATA0 = 0;
                 //LED_VERDE2 = LED_OFF_STATE; //LATAbits.LATA1 = 0;
        }
        if( !PORTBbits.RB7){
                LATAbits.LATA1 = !PORTAbits.RA1;
                delay_ms(2000);
            
        }else{
            LED_VERDE2 = LED_OFF_STATE; //LATAbits.LATA1 = 0;
        }
        LATBbits.LATB3 = !PORTBbits.RB3;
        delay_ms(1000);
    } 

    
     
    return 0;
}
