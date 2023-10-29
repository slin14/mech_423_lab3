#include <msp430.h> 

/**
 * main.c - l3 ex2
 * modified from l2 ex10
 * - timer in continuous mode
 * - using timer TB2.1 (prev TB1.1)
 * - freq reading on P2.1 (prev P1.6)
 * | MSG_START_BYTE | cmdByte| data_H_Byte | data_L_Byte | escByte | data_modified |
 * |----------------|--------|-------------|-------------|---------|---------------|
 */

// PARAMETERS
#define LEDOUTPUT        0b11111111
#define UART_CHAR0       'a'
// myTB2CCR0 = 1 MHz / PWM frequency (Hz)
#define myTB2CCR0        2000
// milliseconds per timer interrupt (freq = 1000/TIMER_MILLISEC Hz)
//#define TIMER_MILLISEC   40
#define BUF_SIZE         50
#define MSG_SIZE         5

// CONSTANTS
#define LOWMASK         0x0F
#define HIGHMASK        0xF0
#define UART_INT_EN     0b1
#define TIMERB_LED_PORT 0b1
#define X_CH            ADC10INCH_12
#define Y_CH            ADC10INCH_13
#define Z_CH            ADC10INCH_14
#define NTC_CH          ADC10INCH_4
#define NUM_LEDS        8

// VARIABLES (PARAMETERS)
static const int myTB2CCR1 = 1000; // = duty cycle * myTB2CCR0
static const int myTB2CCR2 = 500;  // = duty cycle * myTB2CCR0
static const int myTA0CCR0 = 500;  // = TIMER_MILLISEC * 1000 - 1;

// VARIABLES (CONSTANTS)
static const unsigned char datapacket = 255;
static const unsigned char BUF_DQ_BYTE = 13;
static const unsigned char BUF_EMPTY_BYTE = 0;   // buf error indicator
static const unsigned char BUF_FULL_BYTE  = 255; // buf error indicator
static const unsigned char MSG_START_BYTE = 255;
static const unsigned char FREQ_CMD_BYTE = 0x01; // msg cmd
static const unsigned char LEDS_CMD_BYTE = 0x02; // msg cmd
static const unsigned char DUTY_CMD_BYTE = 0x03; // msg cmd

// VARIABLES (TO BE USED)
volatile unsigned char rxByte = 0;
volatile unsigned int  prevCap = 0; // volatile tells compiler the variable value can be modified at any point outside of this code
volatile unsigned int  cap = 0;
volatile unsigned int  measurement = 0; // time between rising and falling edge of TA0.1 input
volatile unsigned char axByte = 0;
volatile unsigned char ayByte = 0;
volatile unsigned char azByte = 0;
volatile unsigned int  temp = 0;
volatile unsigned int  tempThresh = 194;
volatile unsigned char buf[BUF_SIZE];
volatile unsigned int  head = 0;
volatile unsigned int  tail = 0;
volatile unsigned int  i = 0;
volatile unsigned int  dequeuedItem = 0;
volatile unsigned char dequeuedByte = 0;
volatile unsigned char startByte = 0;
volatile unsigned char cmdByte = 0;
volatile unsigned char data_H_Byte = 0;
volatile unsigned char data_L_Byte = 0;
volatile unsigned char escByte = 0;
volatile unsigned int  data = 0;
volatile unsigned int  byteState = 0;
//volatile unsigned int  msg_byte_count = 0;
volatile unsigned int  packetReceivedFlag = 0;

#include "mech423.h"
// my header file

/////////////////////////////////////////////////
// FUNCTIONS

// [ex5] setup Timer B Up Mode - counts up to "myTB2CCR0"
// led_port    TB2.x    output port
// 1           TB2.1    P3.4
// 1           TB2.2    P3.5
// 0           TB2.1    P1.6
// 0           TB2.2    P1.7
void setup_timerB_UP_mode(int led_port) {
    if (led_port == TIMERB_LED_PORT) {
        P3DIR  |=  (BIT4 + BIT5); // P3.4 and P3.5 as output // OUTDATED
        P3SEL0 |=  (BIT4 + BIT5); // select TB2.1 and TB2.2
        P3SEL1 &= ~(BIT4 + BIT5); // select TB2.1 and TB2.2  // redundant
    } else {
        P2DIR  |=  (BIT1 + BIT2); // P2.1 and P2.2 as output
        P2SEL0 |=  (BIT1 + BIT2); // select TB2.1 and TB2.2
        P2SEL1 &= ~(BIT1 + BIT2); // select TB2.1 and TB2.2  // redundant
    }

    TB2CTL |= TBSSEL__ACLK + // ACLK as clock source (8 MHz)
              MC__CONTINUOUS + // Continuous mode
              ID__8        ; // divide input clock by 8 -> timer clk 1 MHz
    TB2CTL |= TBCLR;         // clr TBR, ensure proper reset of timer divider logic

    TB2CCR0 = myTB2CCR0;     // value to count up to in UP mode
}

// [ex6] setup Timer A CONTINUOUS Mode - counts up to TxR (counter length set by CNTL)
void setup_timerA_CONT_mode() {
    // configure P1.0 as input to timerA TA0.1
    P1DIR &= ~BIT0;
    P1SEL1 &= ~BIT0;
    P1SEL0 |= BIT0;

    // configure Timer A
    TA0CTL   = TASSEL__ACLK   + // ACLK as clock source (8 MHz)
               MC__CONTINUOUS + // Continuous mode
               ID__8          + // divide input clock by 8 -> timer clk 1 MHz
               TACLR          ; // Timer A counter clear, ensure proper reset of timer divider logic
    // Note:   TAIE (overflow interrupt is NOT enabled)
}


// [ex7,8] Set up TA0.1 to interrupt every "TIMER_MILLISEC" (1000/TIMER_MILLISEC Hz)
void timerA_interrupt(int millisec) {
    // configure TA0.1
    TA0CCTL0 |= CCIE;    // Capture/compare interrupt enable
    TA0CCR0 = myTA0CCR0; // overflow at X ms if 1MHz in

    // start Timer A
    TA0CTL  |= TASSEL_2 + // Timer A clock source select: 2 - SMCLK
               MC_1     + // Timer A mode control: 1 - Up Mode
               TACLR    ; // clear TA0R
}


/////////////////////////////////////////////////
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;  // stop watchdog timer

    setup_8MHz_clks();            // 8 MHz DCO on MCLK, ACLK, SMCLK
    setup_UART(UART_INT_EN);   // set up UART with UART RX interrupt enabled

    setup_LEDs();
    display_LEDs(LEDOUTPUT);   // initialize LEDs

    // setup Timer B [ex5] to output PWM on P2.1 and P2.2
    setup_timerB_UP_mode(~TIMERB_LED_PORT); // TB2.1 and TB2.2 on P2.1 and P2.2

    // initialize PWM outputs to default: 500 Hz, 50% duty TB2.1, 25% duty TB2.2
    TB2CCTL1 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
    TB2CCR1 = myTB2CCR1;
    TB2CCTL2 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
    TB2CCR2 = myTB2CCR2;

    //setup_buttons_input();
    //enable_buttons_interrupt();

    /////////////////////////////////////////////////
    _EINT();         // enable global interrupt

    while(1) {
        // loop if there are any items in buffer
        if (head != tail) { // if buffer not empty
            dequeuedByte = dequeue(); // dequeue
            txUART(dequeuedByte); //debug

            // [ex10] detect and store msg packet
            switch(byteState) {
                case 1:
                    cmdByte = dequeuedByte;
                    byteState = 2;
                    break;
                case 2:
                    data_H_Byte = dequeuedByte;
                    byteState = 3;
                    break;
                case 3:
                    data_L_Byte = dequeuedByte;
                    byteState = 4;
                    break;
                case 4:
                    escByte = dequeuedByte;
                    byteState = 0;
                    //packetReceivedFlag = 1; // entire packet received, process packet in main

                    //printBufUART(); // print to UART for debug

                    // revert modified data using escByte
                    switch(escByte) {
                        case 0x01:
                            data_L_Byte = MSG_START_BYTE;
                            break;
                        case 0x02:
                            data_H_Byte = MSG_START_BYTE;
                            break;
                        case 0x03:
                            data_L_Byte = MSG_START_BYTE;
                            data_H_Byte = MSG_START_BYTE;
                            break;
                        default:
                            break;
                    } // switch (escByte)

                    // combine data_H and data_L Bytes
                    data = data_H_Byte << 8 | data_L_Byte;

                    // [ex10] execute commands
                    switch(cmdByte) {
                        case FREQ_CMD_BYTE: // cmd 1: set Timer B CCR0 (period)
                            TB2CCR0 = data;
                            break;
                        case LEDS_CMD_BYTE: // cmd 2: display data_L_Byte on LEDs
                            byteDisplayLED(data_L_Byte);
                            break;
                        case DUTY_CMD_BYTE: // cmd 3: set Timer B CCR1 (duty cycle)
                            TB2CCR1 = data;
                            break;
                        default:
                            break;
                    } // switch (cmdByte)

                    //packetReceivedFlag = 0;
                    break;
                default:
                    if (dequeuedByte == MSG_START_BYTE) {
                        //startByte = dequeuedByte;
                        byteState = 1;
                    }
                    break;
            } // switch (byteState)
        } // if items in buffer
    } // infinite loop

    return 0;
} // end main
/////////////////////////////////////////////////
// ISRs


// [ex7,8] ISR for TA0.1 CCR0 overflow every TIMER_MILLISEC = 40 (250 Hz)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void timerA0()
{
    //// [ex7] sample accelerometer Ax, Ay, Az
    //axByte = adcReadChannel(X_CH) >> 2;
    //ayByte = adcReadChannel(Y_CH) >> 2;
    //azByte = adcReadChannel(Z_CH) >> 2;

    //// [ex8] sample NTC and transmit over UART
    //temp = adcReadChannel(NTC_CH); // no right shift to incr resolution
    //txUART(temp);

    //// [ex7] transmit 255, Ax, Ay, Az over UART
    //txUART(datapacket);
    //txUART(axByte);
    //txUART(ayByte);
    //txUART(azByte);

    TA0CCTL0 &= ~CCIFG; // clear IFG
}

// ISR for capture from TA0.1
// [ex6] overflow is NOT enabled, so this will NOT fire when TAR overflows
#pragma vector=TIMER0_A1_VECTOR
__interrupt void timerA(void)
{
    if (TA0IV & TA0IV_TACCR1){ // TA0CCR1_CCIFG is set
        cap = TA0CCR1;
        if(!(TA0CCTL1 & CCI)){ // current output is low (it was previously high)
            // save the measurement (time now - starting time)
            measurement = cap - prevCap; // time between rising and falling edge
            // TA0CCR2 = measurement; // save to a register (trying to see it in the debugger)
        }
        else if (TA0CCTL1 & CCI) { // current output is high (it was previously low)
            prevCap = cap; // reset the measurement starting time
        }
        TA0CCTL1 &= ~CCIFG; // clear IFG
    }
}

#pragma vector = PORT4_VECTOR
__interrupt void P4_ISR()
{
    switch (P4IV) {
        case P4IV_P4IFG0: // P4.0 (S1)
            //toggle_LED_zeros(LEDOUTPUT);  // toggle the zeros in LEDOUTPUT
            //turn_ON_LED_ones(LEDOUTPUT);  // turn ON the 1's in LEDOUTPUT

            //// [ex8] S1 as temperature calibration button
            //tempThresh = adcReadChannel(NTC_CH); // no right shift to incr resolution

            P4IFG &= ~BIT0; // clear IFG
            break;
        case P4IV_P4IFG1: // P4.1 (S2)
            // toggle_LED_ones(LEDOUTPUT);   // toggle the ones in LEDOUTPUT
            //turn_OFF_LED_ones(LEDOUTPUT); // turn OFF the 1's in LEDOUTPUT

            P4IFG &= ~BIT1; // clear IFG
            break;
        default: break;
    }
}

// ISR for UART receive
#pragma vector=USCI_A0_VECTOR
__interrupt void UCA0RX_ISR()
{
    rxByte = UCA0RXBUF; // get the received byte from UART RX buffer

    enqueue(rxByte); // [ex10]

    //// [ex9] circular buffer
    //if (rxByte == BUF_DQ_BYTE) { // dequeue if receive a carriage return (ASCII 13)
    //  dequeuedItem = dequeue();
    //}
    //else {
    //    enqueue(rxByte);
    //}
    //printBufUART(); // [ex9] debug


    //// [ex4] echo rxByte, rxBtye + 1
    //// transmit back the received byte
    //txUART(rxByte);                   // UART transmit
    //// transmit back rxByte + 1
    //txUART(rxByte + 1);               // UART transmit

    //// [ex4] turn LED ON if rxByte == 'j', OFF if rxByte == 'k'
    //if      (rxByte == 'j') turn_ON_LED_ones(LEDOUTPUT);  // turn ON the 1's in LEDOUTPUT
    //else if (rxByte == 'k') turn_OFF_LED_ones(LEDOUTPUT); // turn OFF the 1's in LEDOUTPUT

    // UART RX IFG is self clearing
}
