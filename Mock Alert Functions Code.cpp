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

// Macros
#define setPR2(seconds)   (seconds * SYSCLK / 256)
 
// Note that 612Hz (PR2=0xffff) is the lowest pwm frequency with our configuration
// : To get lower, use a timer prescaler or use the 32-bit timer mode
#define PWM_FREQ    4000
#define DUTY_CYCLE  2 
 

// Alert output settings/helper variables
int onOff;          // 1 = PWM on, 0 = PWM off
int numBeeps = 3;   // The number of times the speaker will beep when beep() is called
int beepCNT = 0;    // Tracks number of beeps that have happened so far


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

// Timer2 Interrupt Service Routine
extern "C" {void __ISR(8, IPL4AUTO) IntHandler(void)
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


// Delays for specified time
void delay_ms( unsigned t)
{
    // Assumes the SYSCLK and PBCLK frequencies are the same
    // Occupies Timer1
    
    T1CON = 0x8000;     // enable Timer1, source PBCLK, 1:1 prescaler
    while( t--)
    {
        TMR1 = 0;
        while( TMR1 < SYSCLK/1000); // wait 1ms
    }
 
    // disable Timer1
    T1CONCLR = 0x8000;
}

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
    //        prescaler * PR2max / Pbclk = 256 * 65535 / 40MHz = 0.42s
    // thus, a setPR2() argument of anything above 0.42 will give undesired results
    PR2 = setPR2(0.4);       // This will cause a full blink every 0.8s
 
    // Only one interrupt source (Timer2) but still in multivectored mode
    // : found in plib.h
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
    
    // loop indefinitely
    while(1){
        
        // Toggle LEDs
        rightAlertOff();
        leftAlertOn();
        
        beep();            // Beep
        delay_ms(2500);    // Wait 2.5 seconds
        
        //Toggle LEDs
        rightAlertOn();
        leftAlertOff();
        
        delay_ms(2500);    // Wait 2.5 seconds
    }
 
    return 1;
}
