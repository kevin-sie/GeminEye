/*
 * Code that includes functions that turn on and off LEDs for each blindspot
 * as well as code that will beep the speaker using interrupts and code for
 * an ISR that enables this.
 * 
 * Code inside main() while loop demos functionality by beeping speaker three 
 * times every 5 seconds and swapping which LED is on every 2.5 seconds
 * 
 */
 
// Defines
#define SYSCLK  40000000L
#define _SUPPRESS_PLIB_WARNING 1

// Include Header Files
#include <p32xxxx.h>
#include <plib.h>
#include <xc.h> 
#include <cstdlib>
#include <stdio.h>
#include <string.h>
 
// Configuration Bits
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
                                    // see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select
#pragma config JTAGEN = OFF         // Disable JTAG
#pragma config FSOSCEN = OFF        // Disable Secondary Oscillator
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK

// Macros
#define setPR2(seconds)   (seconds * SYSCLK / 256)
#define Baud2BRG(desired_baud)      ( (SYSCLK / (16*desired_baud))-1)

 
// Note that 612Hz (PR2=0xffff) is the lowest pwm frequency with our configuration
// : To get lower, use a timer prescaler or use the 32-bit timer mode
#define PWM_FREQ    4000
#define DUTY_CYCLE  2 
 
using namespace std;


// Function prototypes
int UART1Configure( int baud);
int UART2Configure( int baud);
int SerialTransmit1(const char *buffer); // unit8_t try
int SerialTransmit2(const char *buffer); // unit8_t try
unsigned int SerialReceive1(char *buffer, unsigned int max_size);
unsigned int SerialReceive2(char *buffer, unsigned int max_size);


// Alert output settings/helper variables
int onOff;          // 1 = PWM on, 0 = PWM off
int numBeeps = 3;   // The number of times the speaker will beep when beep() is called
int beepCNT = 0;    // Tracks number of beeps that have happened so far


// UART global variables
int foundL = 0;
int foundR = 0;
char knotstart[] = "$GPRMC"; // starting point
int num_comma = 7; // after 7 commas, knots is there
int speed = 0;
char RPiReceiveBuffer[8];


/*
 * Alert System Helper Functions
 */

// Turns on Timer2 and Output Compare Module 2
int beep(void)
{
    // Turn on PWM with the following:
    T3CONSET = 0x8000;      // Enable Timer3, prescaler 1:1
    OC2CONSET = 0x8000;     // Enable Output Compare Module 2
    
    onOff = 1;

    // Enable Timer2 and its interrupt with the following:
    T2CONSET = 0x8070;      // Turn on 16-bit Timer2, set prescaler to 1:256 (frequency is Pbclk / 256)
    IEC0SET = 0x0200;       // enable Timer2 int, IEC0<9>
    
}

// Turns on LED corresponding to left blind spot
int leftAlertOn(void)
{
    LATAbits.LATA0 = 0b1;           // Set the LED pin HIGH
}

// Turns off LED corresponding to left blind spot
int leftAlertOff(void)
{
    LATAbits.LATA0 = 0b0;           // Set the LED pin LOW

}

// Turns on LED corresponding to right blind spot
int rightAlertOn(void)
{
    LATBbits.LATB0 = 0b1;           // Set the LED pin HIGH
}

// Turns off LED corresponding to right blind spot
int rightAlertOff(void)
{
    LATBbits.LATB0 = 0b0;           // Set the LED pin LOW
}

// Timer2 Interrupt Service Routine (For turning on and off the speaker)
extern "C" {void __ISR(8, IPL4AUTO) IntHandler_Timer2(void)
{
    if(beepCNT < numBeeps){
        // Continue beeping, increment beepCNT
        
        // Turn off PWM if it is already on, increment beepCNT
        if(onOff == 1){
            T3CONCLR = 0x8000;              // Disable Timer3, prescaler 1:1
            OC2CONCLR = 0x8000;             // Disable Output Compare Module 2

            onOff = 0;
            beepCNT = beepCNT + 1;
        }
        // Turn on PWM if it is off
        else{
            T3CONSET = 0x8000;              // Enable Timer3, prescaler 1:1
            OC2CONSET = 0x8000;             // Enable Output Compare Module 2

            onOff = 1;
        }
    }
    
    // Stop the beeping
    else{
        
        // Turn off PWM with the following:
        T3CONCLR = 0x8000;      // Disable Timer3, prescaler 1:1
        OC2CONCLR = 0x8000;     // Disable Output Compare Module 2

        // Disable Timer2 and its interrupt with the following:
        T2CONCLR = 0x8070;       // Timer off
        IEC0CLR = 0x0200;       // Clear interrupt flag
        
        beepCNT = 0;
        
    }
    
    IFS0CLR = 0x0200;       // clear timer 2 int flag, IFS0<9>
} // END Timer2 ISR
}

// U2RX Interrupt Service Routine (For receiving Pi communications)
extern "C" {void __ISR(37, IPL4AUTO) IntHandler_U2RX(void)
{    
    RPiReceiveBuffer[0] = U2RXREG;      // empty contents of RX buffer into *buffer pointer
    
    // Clear the receive register by raising then clearing OERR bit
    U2STAbits.OERR = 1;
    U2STAbits.OERR = 0;
    
    /*
     * 
     * Key for interpreting RPi communications:
     * 
     * L -- Turn on left LED
     * R -- Turn on right LED
     * M -- Turn off left LED
     * S -- Turn off right LED
     * A -- Play audio alert
     * 
     */
    
    if(RPiReceiveBuffer[0] == 'L'){
        leftAlertOn();
    }
    
    if(RPiReceiveBuffer[0] == 'R'){
        rightAlertOn();
    }
        
    switch(RPiReceiveBuffer[0] ){
        case 'L':
            leftAlertOn();
            break;
        
        case 'R':
            rightAlertOn();
            break;
        
        case 'M':
            leftAlertOff();
            break;
        
        case 'S':
            rightAlertOff();
            break;
            
        case 'A':
            beep();
            break;
          
    }
    
    
    IFS1CLR = 0x400000;     // clear U2RX interrupt flag, IFS1<22>
} // END U2RX ISR
}


/*
 * UART Helper Functions
 */


//Are these two ever used?
int UART1Configure( int desired_baud){
    // M5 doesnt use pragma config here
    //#pragma config U1MODE = 0x8000 // no parity<1-2>, 1 stop bit<0>, <15>, M5 has it as 0
    //#pragma config U1STA = 0x00001000 // enable receive bit <12>
    U1MODE = 0x8000;
    U1STA = 0x00001000;
    // M5 stuff
    // U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    // U2STA = 0x1400;     // enable TX and RX
    U1BRG = Baud2BRG(desired_baud); // U2BRG = (FPb / (16*baud)) - 1    
    // Calculate actual assigned baud rate
    int actual_baud = SYSCLK / (16 * (U1BRG+1));

    return actual_baud;
} // END UART1Configure()

int UART2Configure( int desired_baud){
    // M5 doesnt use pragma config here
    //#pragma config U1MODE = 0x8000 // no parity<1-2>, 1 stop bit<0>, <15>, M5 has it as 0, MODESET 0x8000
    //#pragma config U1STA = 0x00001000 // enable receive bit <12>
    U2MODE = 0x8000; // mine
    //U2MODE = 0;
    U2STA = 0x1400;
    // M5 stuff
    // U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    // U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(desired_baud); // U2BRG = (FPb / (16*baud)) - 1    
    // Calculate actual assigned baud rate
    int actual_baud = SYSCLK / (16 * (U2BRG+1));

    return actual_baud;
} // END UART2Configure()

// SerialTransmit() transmits a string to the UART2 TX pin MSB first
//
// Inputs: *buffer = string to transmit
int SerialTransmit1(const char *buffer)
{

    unsigned int size = strlen(buffer);
    //int size = 2;
    while(size) // different, debugging with (1) currently
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }

    while( !U2STAbits.TRMT);        // wait for last transmission to finish

    //RPB10Rbits.RPB10R = 0b0000; // OFF
    return 0;
}

int SerialTransmit2(const char *buffer)
{

    unsigned int size = strlen(buffer);
    //int size = 2;
    while(size) // different, debugging with (1) currently
    {
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }

    while( !U2STAbits.TRMT);        // wait for last transmission to finish

    //RPB10Rbits.RPB10R = 0b0000; // OFF
    return 0;
}

unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;

    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        // maybe its receiving too slow/fast, try a pause
        //delay_ms(100);
            *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer

             //insert null character to indicate end of string
            if( *buffer == 'L' || *buffer == 'R'){
                if (*buffer == 'L')
                {
                    foundL = 1;
                }
                else
                {
                    foundR = 1;
                }
                // test
                break;
            }
            if(U1STAbits.OERR == 1)
            {// if overflow
                U1STAbits.OERR = 0;
            }
        buffer++;
        num_char++;
    }

    return num_char;
} // END SerialReceive()

unsigned int SerialReceive2(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;

    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while(num_char < max_size)
    {
        while( !U2STAbits.URXDA);   // wait until data available in RX buffer
        // maybe its receiving too slow/fast, try a pause
        //delay_ms(100);
        *buffer = U2RXREG;          // empty contents of RX buffer into *buffer pointer

        if(U2STAbits.OERR == 1)
        {// if overflow
            U2STAbits.OERR = 0;
        }

        if( *buffer == 'L' || *buffer == 'R'){
            if (*buffer == 'L')
            {
                foundL = 1;
            }
            else
            {
                foundR = 1;
            }
            // test
            break;
        }

        buffer++;
        num_char++;
    }

    return num_char;
} // END SerialReceive()


int main(void)
{
    SYSTEMConfigPerformance(SYSCLK);
 /*
  * Setup pins or audio/visual alerts
  */    
    // Visual
    ANSELA = 0;                     // Set all of PortA as digital
    ANSELB = 0;                     // Set all of PortB as digital
    TRISAbits.TRISA0 = 0;           // RA0 -> Output (Right LED)
    TRISBbits.TRISB0 = 0;           // RB0 -> Output (Left LED)
   
    // Audio
    RPA1Rbits.RPA1R = 0x0005;       // Set OC2 (audio output) to pin RA1 (pin 3) with peripheral pin select
    
/*
 * Setup for timer/interrupts using Timer2
 */   
    
    // The slowest the interrupt will occur is every:
    // prescaler * PR2max / Pbclk = 256 * 65535 / 40MHz = 0.42s
    // thus, a setPR2() argument of anything above 0.42 will give undesired results
    PR2 = setPR2(0.4);       // This will cause a full cycle every 0.8s
 
    // Enable multivectored mode for interrupts
    INTEnableSystemMultiVectoredInt();
 
    // Configure Interrupt
    IEC0CLR = 0x0200;       // disable Timer2 int, IEC0<9>
    IFS0CLR = 0x0200;       // clear Timer2 int flag, IFS0<9>
    IPC2CLR = 0x001f;       // clear Timer2 priority/subpriority fields IPC2<4:0>
    IPC2SET = 0x0010;       // set Timer2 int priority = 4, IPC2<4:2>
    IPC2SET = 0x0000;       // set Timer2 int subpriority = 0, IPC2<1:0>

/*
 * Setup for PWM using Timer3 and OC2  
 */    
    // Configure standard PWM mode for output compare module 2, selects Timer3
    OC2CON = 0x000E;  
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] ? 1
    // : note the TMR Prescaler is 1 and is thus ignored
    PR2 = (SYSCLK / PWM_FREQ) - 1;
 
    // A write to OCxRS configures the duty cycle
    // OCxRS / PRy = duty cycle
    OC2RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);
    
/*
 * UART Setup 
 */    
    
    //Peripheral Pin Select configuration
    U1RXRbits.U1RXR = 0b0010; // U1RX for GPS, RPA4, Pin 12
    U2RXRbits.U2RXR = 0b0011; // U2RX for RPi, RPB11, D- or Pin 22 on Pic32MX270F256B
    //Not sure the next line is right, need to do something to this effect though
    RPB10R.U2TXR = 0b0010; // U2TX for RPi, RPB10, D+ or Pin 21 on Pic32MX270F256B 
 
    
    //UART1Configure(9600); // GPS/ 4800
    UART2Configure(4800); // LiDAR, 115200
    
 /*
 * Setup for U2RX Interrupt (For RPi Communication)
 */   
    
    IEC1CLR = 0x400000;     // disable U2RX interrupt, IEC1<22>
    IFS1CLR = 0x400000;     // clear U2RX interrupt flag, IFS1<22>
    IPC9CLR = 0x001f00;     // clear U2RX interrupt priority/subpriority fields IPC9<12:8>
    IPC9SET = 0x001000;     // set U2RX interrupt priority = 4, IPC9<12:10>
    IPC9SET = 0x000100;     // set U2RX interrupt subpriority = 1, IPC9<9:8>
    IEC1SET = 0x400000;     // enable U2RX interrupt, IEC1<22>
    
    // loop indefinitely
    while(1){
       
       
    }
 
    return 1;
}