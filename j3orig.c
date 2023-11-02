#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


#include "intrinsics.h"
/**
 * main.c
 *
 * Project: Encoder_testing
 * Changed to timer B2.2 for stepper steps
 *
 */

#define PRINTBUFSIZE 4
#define maxSize 50
#define LOWMASK 0x0F
#define HIGHMASK 0XF0

/** MACROS FOR DC MOTOR **/
#define setDC_A1 P3OUT|=BIT7;
#define clrDC_A1 P3OUT&=~BIT7;
#define setDC_A2 P3OUT|=BIT6;
#define clrDC_A2 P3OUT&=~BIT6;

/** MACROS FOR STEPPER **/
#define B1 BIT5
#define B2 BIT4
#define A1 BIT4
#define A2 BIT5
#define STOP 0
#define NORMAL 1
#define CONTP 2
#define CONTN 3
#define STEPP 4
#define STEPN 5
#define BRAKE 6
#define CHOP_PERIOD (100-1)

/** MACROS FOR ENCODER **/
#define ENCODER_UP TA1R
#define ENCODER_DN TA0R

#define TB1_CHOP_ON TB1CTL |= TBSSEL_2 + MC_2 + TBCLR; // TB1 SMCLK, Cont mode for PWM generation
#define TB0_CHOP_ON TB0CTL |= TBSSEL_2 + MC_2 + TBCLR; // TB0 SMCLK, Cont mode for PWM generation and also step generation on TB0.3 I guess. smh
#define TB1_CHOP_OFF TB1CTL &= ~MC_2;
#define TB0_CHOP_OFF TB0CTL &= ~MC_2;

volatile unsigned char buf[maxSize];    // main input buffer acts like a queue
volatile unsigned int head = 0;         // queue head pointer
volatile unsigned int counter = 0;      // queue counter
volatile unsigned int tail = 0;         // tail pointer

volatile unsigned char deQueueOut;      // temporary target for dequeue value
volatile unsigned char command;         // temporary storage for command
volatile unsigned int data;             // temporary storage for data 16bit
volatile unsigned char data_L;          // temporary storage for low data nibble
volatile unsigned char data_H;          // temporary storage for high data nibble
volatile unsigned char esc;             // temporary storage for escape byte
volatile unsigned char byteState = 0;   // byte state tracker

/** STEPPER LOOKUP ARRAYS **/
// To do: microstepping using the DRV8841 built in chopper (or software?)
const unsigned char A1table[] = {1,1,0,0,0,0,0,1};
const unsigned char A2table[] = {0,1,1,1,0,0,0,0};
const unsigned char B1table[] = {0,0,0,1,1,1,0,0};
const unsigned char B2table[] = {0,0,0,0,0,1,1,1};

volatile int stepInterval = 20000-1;
const int timerBCCR0 = CHOP_PERIOD;             // default BCCR0 for software current chopping
const int stepperDuty = CHOP_PERIOD/3;

volatile unsigned int stepState = 0;            // lookup table entry
volatile unsigned int absoluteSteps = 0;        // real absolute stepper steps
volatile unsigned int absoluteTarget = 0;       // programmed absolute stepper steps
volatile unsigned char stepperMode = 0;          // 0 = Stop; 1 = normal; 2 = Continuous+, 3 = Continuous-; 4 = Single step+; 5 = Single step-
                                                // I think continuous mode is dumb. But the documentation calls for it
volatile unsigned char asyncPrintFlag = 0;


/*
 * In progress.
 * Need a switch case for each port. Assume configuration is already done
 */
void digitalWrite(unsigned char port, unsigned char pin, unsigned char data)
{
    switch(port)
    {
    case 1:
    case 2:
    case 3:
    case 4:
    case 'J':
    default:break;
    }
}


/** DC MOTOR FUNCTIONS **/
/*
 * Commands will immediately run motor. Speed is set independently by Timer 2.1
 */
void DC_stop()
{
    clrDC_A1;
    clrDC_A2;
}
void DC_brake()
{
    setDC_A1;
    setDC_A2;
}
void DC_CCW()
{
    clrDC_A1;
    setDC_A2;
}
void DC_CW()
{
    setDC_A1;
    clrDC_A2;
}


/** STEPPER FUNCTIONS **/
void writeStepperState(unsigned int state)
{
    if(B1table[state]) {
        P3SEL0 |= B1;           // Routes chopper timer to coil B1
        //P3OUT |= B1;          // without chopping
        }
    else {
        P3SEL0 &= ~B1;          // Unroute chopper timer
        P3OUT &= ~B1;           // Force output low
        }

    if(B2table[state]) {
        P3SEL0 |= B2;
        //P3OUT |= B2;
        }
    else {
        P3SEL0 &= ~B2;
        P3OUT &= ~B2;
    }

    if(A1table[state]) {
        P1SEL0 |= A1;
        //P1OUT |= A1;
    }
    else {
        P1SEL0 &= ~A1;
        P1OUT &= ~A1;
    }

    if(A2table[state]) {
        P1SEL0 |= A2;
        //P1OUT |= A2;
    }
    else {
        P1SEL0 &= ~A2;
        P1OUT &= ~A2;
    }
}


void stopStepper()
{
    stepperMode = 0;
    P3SEL0 &= ~B1;
    P3OUT &= ~B1;
    P3SEL0 &= ~B2;
    P3OUT &= ~B2;
    P1SEL0 &= ~A1;
    P1OUT &= ~A1;
    P1SEL0 &= ~A2;
    P1OUT &= ~A2;
}
void resetSteps()
{
    absoluteSteps = 0;
    absoluteTarget = 0;
}
void incrementStepper()
{
    switch(stepperMode)
    {
    case STOP:                                                     // Stop and de-energize.
        P3SEL0 &= ~B1;
        P3OUT &= ~B1;
        P3SEL0 &= ~B2;
        P3OUT &= ~B2;
        P1SEL0 &= ~A1;
        P1OUT &= ~A1;
        P1SEL0 &= ~A2;
        P1OUT &= ~A2;
        break;
    case NORMAL:                                                     // normal mode. Not used in Part 3.
        if(absoluteSteps != absoluteTarget){
            if(absoluteSteps<absoluteTarget){
                stepState = (stepState + 1) % 8;
                writeStepperState(stepState);
                absoluteSteps ++;                                   // open loop update
            }
            else{
                stepState = (stepState - 1) % 8;
                writeStepperState(stepState);
                absoluteSteps --;
            }
        }
    break;
    case CONTP:                                                     // Continuous +  (if we wanted continuous motion, we wouldn't use a stepper...)
        stepState = (stepState + 1) % 8;
        writeStepperState(stepState);
        break;
    case CONTN:                                                     // Continuous -
        stepState = (stepState - 1) % 8;
        writeStepperState(stepState);
        break;
    case STEPP:                                                     // Not continuous - no need to handle here
        break;
    case STEPN:
        break;
    case BRAKE:                                                     // Hold the current state.
        writeStepperState(stepState);
        break;
    default:
        break;
    }
}


/** UART PRINT FUNCTIONS **/
void printUART(unsigned int in)
{
    while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = in;
}
void printUARTint_ASCII(int in)
{
    char printbuffer[PRINTBUFSIZE];
    sprintf(printbuffer, "%d", in);
    unsigned int i;
    for(i = 0; i < PRINTBUFSIZE-1; i++){
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = printbuffer[i];
    }
}
void printUARTstring(char * in)
{
    unsigned int i;
    for(i = 0; i < strlen(in); i++){
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = in[i];
    }
}

void waitForUARTbusy()
{
    while((UCA1STATW & UCBUSY));        // wait until UART is done doing whatever it's doing.
}

void printEncoderCounts() // this is terrible.
{
    unsigned char escape = 0;
    printUART(0xFF);
    if((ENCODER_UP&0x00FF) == 0xFF)
    {
        escape |= 1<<1;
        printUART(0xFE);
    }
    else
    {
        printUART(ENCODER_UP&0x00FF);
    }
    if((ENCODER_UP>>8) == 0xFF)
    {
        escape |= 1<<2;
        printUART(0xFE);
    }
    else
    {
        printUART(ENCODER_UP>>8);
    }
    if((ENCODER_DN&0x00FF) == 0xFF)
    {
        escape |= 1<<4;
        printUART(0xFE);
    }
    else
    {
        printUART(ENCODER_DN&0x00FF);
    }
    if((ENCODER_DN>>8) == 0xFF)
    {
        escape |= 1<<5;
        printUART(0xFE);
    }
    else
    {
        printUART(ENCODER_DN>>8);
    }
    printUART(escape);

}

void deQueue()
{
    if(counter > 0)
    {
       //while(!(UCA1IFG & UCTXIFG));
       //UCA1TXBUF = buf[head];       // transmit dequeued byte for debugging

       deQueueOut = buf[head];
       if(counter > 1)
       {
           head = (head+1)%maxSize;
       }
       counter--;
    }
}

void enQueue(unsigned char in)
{
    if(counter != 0)
    {
        tail = (tail+1)%maxSize;
    }
    if(tail==head && counter != 0)
    {
       head = (head+1)%maxSize;
    }
    buf[tail] = in;
    counter = ++counter > maxSize ? maxSize : counter;
}

void executeCommand(unsigned char commandByte)
{
    switch(command)        // main command execution happens here.
    {
    case 0x00:
        DC_stop();
        break;
    case 0x01:              // Case DC CW
        TB2CCR1 = data;
        DC_CW();
        break;
    case 0x02:              // Case DC CCW
        TB2CCR1 = data;
        DC_CCW();
        break;
    case 0x03:
        stopStepper();
        break;
    case 0x04:
        stepperMode = BRAKE;
        stepState = (stepState + 1) % 8;
        writeStepperState(stepState);
        break;
    case 0x05:
        stepperMode = BRAKE;        // BRAKE = HOLD STATE
        stepState = (stepState - 1) % 8;
        writeStepperState(stepState);
        break;
    case 0x06:
        stepInterval = data;
        TB2CCR2 = data;
        stepperMode = CONTP;
        break;
    case 0x07:
        stepInterval = data;
        TB2CCR2 = data;
        stepperMode = CONTN;
        break;
    case 0x08:
        stepperMode = BRAKE;
    default:
      break;
    }
}


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    /** CLOCK SETUP **/
    CSCTL0_H = CSKEY_H;                 // write CS password.
    CSCTL1 |= DCOFSEL_3;                // select 8HMz DCO
    CSCTL2 |= SELS__DCOCLK;             // select DCO for SMCLK
    CSCTL3 &= DIVS__1;                  // force 8MHz

    UCA1CTL1 = UCSWRST;                 // hold UCA1 in software reset
    UCA1CTL1 |= UCSSEL0|UCSSEL1;        // set source to SMCLK
    UCA1MCTLW = UCOS16 + UCBRF0 + 0x4900;
    UCA1BRW = 52;

    P2SEL1 |= BIT5|BIT6;                // route UCA1 to 2.5 2.6
    UCA1CTL1 &= ~UCSWRST;               // take UCA out of software reset
    UCA1IE |= UCRXIE;                   // enable RX interrupt on UCA0

    /** STEPPER MOTOR TIMERS/ PORTS **/
    /*
     * Uses TB1 and TB0 for software current chopping (It's available in the hardware but unused)
     * TB1CCR0 = Main PWM carrier period
     * TB1.1 = B2
     * TB1.2 = B1
     * TB0.1 = A2
     * TB0.2 = A1
     * TB2.2 = step generation
     *
     *
     * Exactly which timer is which doesn't matter that much for half-stepping, they all get the same
     * fixed voltage limit anyways.
     */

    // ROUTING
    P3DIR |= B1|B2;                 // Coil control B2 B1
    P3SEL0 |= B1|B2;                // Route Timer B1 - default off
    P1DIR |= A1|A2;                 // Coil control A1 A2
    P1SEL0 |= A1|A2;                // Route TImer B0 - default off

    P2DIR |= BIT2;                  // Debug CCR0 out
    P2SEL0 |= BIT2;                 // Debug CCR0 out
    P2SEL1 |= BIT2;                 // Debug CCR0 out

    TB1CCR0 = timerBCCR0;              // TB1CCR0 for PWM count target across TB1
    TB1CCTL1 |= OUTMOD_7;              // mode 7 reset/set for PWM
    TB1CCTL2 |= OUTMOD_7;              // mode 7 reset/set for PWM
    TB0CCR0 = timerBCCR0;              // TB0CCR0 for PWM count target across TB0
    TB0CCTL1 |= OUTMOD_7;              // mode 7 reset/set for PWM
    TB0CCTL2 |= OUTMOD_7;              // mode 7 reset/set for PWM

    TB1CCR1 = stepperDuty;
    TB0CCR1 = stepperDuty;
    TB1CCR2 = stepperDuty;
    TB0CCR2 = stepperDuty;

    // starts the stepper chopper signals
    TB1CTL |= TBSSEL_2 + MC_1 + TBCLR; // TB1 SMCLK, up mode for PWM generation
    TB0CTL |= TBSSEL_2 + MC_1 + TBCLR; // TB0 SMCLK, up mode for PWM generation

    TB2CCR2 = stepInterval;            // use TB2.2 for stepper step generation
    TB2CCTL2 |= CCIE;                  // enable TB2.2 interrupts

    PJDIR |= BIT0;                  // Debug LED
    PJOUT &= ~BIT0;

    /** DC MOTOR TIMERS **/
    /*
     * Uses TB2 for PWM speed control
     */
    P3DIR |= BIT6 + BIT7 + BIT1;      // Set motor driver A inputs
    P2DIR |= BIT1;                    // PWM output
    P2SEL0 |= BIT1;                   // Select TB2.1 on P2.1
    TB2CCR1 = 0xFFFF/2;
    TB2CCTL1 |= OUTMOD_7;             // TB2CCR1 mode 7 = reset/set
    TB2CTL = TBSSEL_2 + MC_2 + TBCLR;     // TB2 SMCLK, CONT mode, interrupt on 0xFFFF overflow for data output, divide 2


    /** ENCODER TIMERS **/
    /*
     *
     * Uses TA0 TA1 for reading latched encoder counts up and down
     * Note: this cannot really be tested independent from the encoder latches
     * The pins are pulled low by the latch output, they don't have tri-state.
     */
    P1DIR &= ~(BIT1 + BIT2);            // Set encoder input pins
    P1SEL1 |= BIT1 + BIT2;              // As clock inputs
    P1SEL0 &= ~(BIT1 + BIT2);           // As clock inputs
    TA0CTL |= TASSEL__INCLK + MC_2;     // TA0 TA0CLK, CONT mode
    TA1CTL |= TASSEL__INCLK + MC_2;     // TA1 TA1CLK, CONT mode
    TB2CTL |= TBIE+BIT6;                // TB2 interrupt
                                        // Uses TB2 main overflow as UART output trigger

    __bis_SR_register(GIE);             // enable global interrupts


    while(1)
    {
        if(asyncPrintFlag)              // there is no way this can give me accurate timing. We'll find out.
        {
            printEncoderCounts();
            asyncPrintFlag = 0;
        }
        if(counter>0)       // keep dequeueing as long as stuff is in the buffer.
        {
             waitForUARTbusy();   // wait for UART
             deQueue();
             switch(byteState)
             {
             case 0:
                 if(deQueueOut == 0xFF) byteState = 1;
                 break;
             case 1:
                 command = deQueueOut;
                 byteState = 2;
                 break;
             case 2:
                 data_H = deQueueOut;
                 byteState = 3;
                 break;
             case 3:
                 data_L = deQueueOut;
                 byteState = 4;
                 break;
             case 4:
                 esc = deQueueOut;
             // If we're here, we have the whole packet. Process it
                byteState = 0;
                switch(esc)            // adjust data according to escape
                {
                case 0x02:
                    data_L = 0xFF;break;
                case 0x04:
                    data_H = 0xFF;break;
                case 0x06:
                    data_L = 0xFF;
                    data_H = 0xFF;break;
                default:
                    break;
                }

                data = data_H<<8 | data_L;      // combine data
                executeCommand(command);

             default:
                break;
             }
        }
    }
    return 0;
}

/** TIMER B INTERRUPT **/
/*
 * Stepper step generation from TB2.2
 */
#pragma vector=TIMER2_B1_VECTOR
__interrupt void TIMERB2_2_ISR(void)
{
    switch(__even_in_range(TB2IV, TB2IV_TBIFG))
    {
    case TB2IV_NONE: break; // No interrupt
    case TB2IV_TBCCR1: break; // CCR1 not used
    case TB2IV_TBCCR2:
        PJOUT^=BIT0;
        TB2CCR2 += stepInterval;
        incrementStepper();             // increment stepper (if required) every B2CCR0 overflow
        break;
    case TB2IV_3: break; // reserved
    case TB2IV_4: break; // reserved
    case TB2IV_5: break; // reserved
    case TB2IV_6: break; // reserved
    case TB2IV_TBIFG:
        // send out encoder counts here.
        asyncPrintFlag = 1;
        break;// overflow
    default: break;
    }
}

/** UART HANDLING **/
#pragma vector=USCI_A1_VECTOR
__interrupt void UCA1RX_ISR(void)
{
    char in = UCA1RXBUF;
    // ECHO: slows things down considerably
    //while(!(UCA1IFG&UCTXIFG));        // wait for cleared USCI A0 TX IFG
    //UCA0TXBUF = in;                   // send back received char

    switch(in)
    {
    default:
        enQueue(in);
        break;
    }

}
