#include <msp430.h> 

/**
 * main.c - l3 ex3
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
static const unsigned char E_STOP_CMD_BYTE = 0x00; // msg cmd
static const unsigned char DC_MOTOR_CW_BYTE = 0x01; // msg cmd
static const unsigned char DC_MOTOR_CCW_BYTE = 0x02; // msg cmd
static const unsigned char STEPPER_STOP_BYTE = 0x03; // msg cmd
static const unsigned char STEPPER_STEP_CW_BYTE = 0x04; // msg cmd
static const unsigned char STEPPER_STEP_CCW_BYTE = 0x05; // msg cmd
static const unsigned char STEPPER_CW_BYTE = 0x06; // msg cmd
static const unsigned char STEPPER_CCW_BYTE = 0x07; // msg cmd
static const unsigned char STEPPER_BREAK_BYTE = 0x08; // msg cmd

static const unsigned int CHOP4 = 12437; // chop 12V to 3V  = 16383; // 65535/4

// Stepper CW sequence given in class
const unsigned char A1table[] = {0,1,1,1,0,0,0,0};
const unsigned char A2table[] = {0,0,0,0,0,1,1,1};
const unsigned char B1table[] = {1,1,0,0,0,0,0,1};
const unsigned char B2table[] = {0,0,0,1,1,1,0,0};
volatile unsigned int stepState = 0;            // lookup table index 

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
volatile unsigned int  stepperMode = 0; // 0 = not continuous, 1 = CW, 2 = CCW

#include "mech423PCB.h"
// my header file

/////////////////////////////////////////////////
// FUNCTIONS

// [l3 ex2] setup TB2 CONT Mode
// TB2.x    output port
// TB2.1    P1.6
// TB2.2    P1.7
void setup_TB2_CONT() {
    P2DIR  |=  (BIT1 + BIT2); // P2.1 and P2.2 as output
    P2SEL0 |=  (BIT1 + BIT2); // select TB2.1 and TB2.2
    P2SEL1 &= ~(BIT1 + BIT2); // select TB2.1 and TB2.2  // redundant

    TB2CTL |= TBSSEL__ACLK +   // ACLK as clock source (8 MHz)
              MC__CONTINUOUS;  // Continuous mode
    TB2CTL |= TBCLR;         // clr TBR, ensure proper reset of timer divider logic

    TB2CCR0 = myTB2CCR0;     // value to count up to in UP mode
}

// [l3 ex3] setup TB0 CONT Mode
// TB0.x    output port
// TB0.1    P1.4
// TB0.2    P1.5
void setup_TB0_CONT() {
    P1DIR  |=  (BIT4 + BIT5);
    P1SEL0 |=  (BIT4 + BIT5);
    P1SEL1 &= ~(BIT4 + BIT5);

    TB0CTL |= TBSSEL__ACLK +   // ACLK as clock source (8 MHz)
              MC__CONTINUOUS;  // Continuous mode
    TB0CTL |= TBCLR;         // clr TBR, ensure proper reset of timer divider logic
	TB0CCTL1 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
	TB0CCTL2 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
    TB0CCR0 = 65535;     // counter value to set PWM [0, 65535]

    TB0CCR1 = CHOP4;
    TB0CCR2 = CHOP4;
}

// [l3 ex2] setup TB1 CONT Mode
// TB1.x    output port
// TB1.1    P3.4
// TB1.2    P3.5
void setup_TB1_CONT() {
    P3DIR  |=  (BIT4 + BIT5);
    P3SEL0 |=  (BIT4 + BIT5);
    P3SEL1 &= ~(BIT4 + BIT5);

    TB1CTL |= TBSSEL__ACLK +   // ACLK as clock source (8 MHz)
              MC__CONTINUOUS;  // Continuous mode
    TB1CTL |= TBCLR;         // clr TBR, ensure proper reset of timer divider logic
	TB1CCTL1 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
	TB1CCTL2 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
    TB1CCR0 = 65535;     // counter value to set PWM [0, 65535]

    TB1CCR1 = CHOP4;
    TB1CCR2 = CHOP4;
}

// circular buffer enqueue
void enqueue(int val)
{
    if ((head + 1) % BUF_SIZE == tail) { // buffer FULL (head + 1 == tail)
        txUART(BUF_FULL_BYTE); // Error: buffer full
    }
    else {
        buf[head] = val; // enqueue
        head = (head + 1) % BUF_SIZE; // head++;
    }
}

// circular buffer dequeue (FIFO)
char dequeue() {
    unsigned char result = 0;
    if (head == tail) { // buffer empty
        txUART(BUF_EMPTY_BYTE); // Error: buffer empty
    }
    else {
        result = buf[tail]; // dequeue
        tail = (tail + 1) % BUF_SIZE; // tail++;
    }
    return result;
}

// circular buffer dequeue (LIFO)
char dequeue_LIFO() {
    unsigned char result = 0;
    if (head == tail) { // buffer empty
        txUART(BUF_EMPTY_BYTE); // Error: buffer empty
    }
    else {
        result = buf[head]; // dequeue
        head = (head + BUF_SIZE - 1) % BUF_SIZE; // head--;
    }
    return result;
}

// debugging: print circular buffer contents over UART
void printBufUART()
{
    for (i = tail; i != head; i = (i + 1) % BUF_SIZE) {
        txUART(buf[i]);
    }
}

// stepper control
void writeStepperState(unsigned int state)
{
    if(B1table[state]) {
        P3SEL0 |= BIT5;
        }
    else {
        P3SEL0 &= ~BIT5;
        P3OUT  &= ~BIT5;
        }

    if(B2table[state]) {
        P3SEL0 |= BIT4;
        }
    else {
        P3SEL0 &= ~BIT4;
        P3OUT  &= ~BIT4;
    }

    if(A1table[state]) {
        P1SEL0 |= BIT5;
    }
    else {
        P1SEL0 &= ~BIT5;
        P1OUT  &= ~BIT5;
    }

    if(A2table[state]) {
        P1SEL0 |= BIT4;
    }
    else {
        P1SEL0 &= ~BIT4;
        P1OUT  &= ~BIT4;
    }
}

// de-energize stopper 
void stopStepper()
{
	P1SEL0 &= ~(BIT4 + BIT5);
	P3SEL0 &= ~(BIT4 + BIT5);
	P1SEL1 &= ~(BIT4 + BIT5);
	P3SEL1 &= ~(BIT4 + BIT5);
	P1OUT  &= ~(BIT4 + BIT5);
	P3OUT  &= ~(BIT4 + BIT5);
}

// Set up TA0.1 to interrupt for stepper speed control
void timerA_interrupt() {
	// configure TA0.1
    TA0CCTL0 |= CCIE;    // Capture/compare interrupt enable
    TA0CCR0 = 65535;     // initialized to slowest speed

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
	// [l3 ex2] - output PWM on P2.1 with adjustable duty cycle
    setup_TB2_CONT(); // TB2.1 and TB2.2 on P2.1 and P2.2

    // initialize PWM outputs to default: 15.3 Hz, 20% duty TB2.1     
	TB2CCTL1 |= OUTMOD_7;    // OUTMOD 7 = reset/set (reset at CCRx, set at CCR0)
    TB2CCR0 = 65535;     // counter value to set PWM [0, 65535]
    TB2CCR1 = 16384;     // duty cycle = CCR1 / CCR0

	// [l3 ex2] - output DC Motor direction on P3.6 (AIN2) and P3.7 (AIN1)
	P3DIR  |=   BIT6 + BIT7;
	P3SEL0 &= ~(BIT6 + BIT7);
	P3SEL1 &= ~(BIT6 + BIT7);
	P3OUT  &= ~(BIT6 + BIT7); // DC Motor stop

	// [l3 3x3] - output stepper motor
	setup_TB0_CONT(); // chop PWM to 25% when ON
	setup_TB1_CONT(); // chop PWM to 25% when ON
	P1DIR  |=   BIT4 + BIT5;
	P3DIR  |=   BIT4 + BIT5;
	P1OUT  &= ~(BIT4 + BIT5); // OFF
	P3OUT  &= ~(BIT4 + BIT5); // OFF

	// [l3 ex3] - timer interrupt for speed control
	timerA_interrupt(); // TA0.1 interrupt
	

    /////////////////////////////////////////////////
    _EINT();         // enable global interrupt

    while(1) {
		//__delay_cycles(100000); txUART(99); // periodically transmit "UART_CHAR0" TEST WORKED
		//for (i=0;i<20000;i++)
        //  _NOP();       // nothing

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
                    data = (data_H_Byte << 8) + data_L_Byte;

                    /////////////////////////////////
                    // [l3] execute commands
                    switch(cmdByte) {
						case E_STOP_CMD_BYTE: // cmd 0: E-STOP
							P3OUT &= ~(BIT6 + BIT7); // DC Motor stop
							stepperMode = 0;
                            break;
                        case DC_MOTOR_CW_BYTE: // cmd 1: DC CW, PWM duty cycle on P2.1
                            TB2CCR1 = data; // PWM duty cycle
							P3OUT |=  BIT6;
							P3OUT &= ~BIT7;
                            break;
                        case DC_MOTOR_CCW_BYTE : // cmd 2: DC CCW, PWM duty cycle on P2.1
                            TB2CCR1 = data; // PWM duty cycle
							P3OUT &= ~BIT6;
							P3OUT |=  BIT7;
                            break;
						case STEPPER_STOP_BYTE: // cmd 3:
        					stopStepper();
							stepperMode = 0;
							break;
						case STEPPER_STEP_CW_BYTE: // cmd 4:
        					stepState = (stepState + 1) % 8;
        					writeStepperState(stepState);
							stepperMode = 0;
							break;
						case STEPPER_STEP_CCW_BYTE: // cmd 5:
        					stepState = (stepState - 1) % 8;
        					writeStepperState(stepState);
							stepperMode = 0;
							break;
						case STEPPER_CW_BYTE: // cmd 6:
							TA0CCR0 = data; // adjust frequency
							stepperMode = 1;
							break;
						case STEPPER_CCW_BYTE: // cmd 7:
							TA0CCR0 = data; // adjust frequency
							stepperMode = 2;
							break;
						case STEPPER_BREAK_BYTE: // cmd 8:
							stepperMode = 0;
                        default:
                            break;
                    } // switch (cmdByte)

                    break;
                default:
                    if (dequeuedByte == MSG_START_BYTE) {
                        byteState = 1;
                    }
					stepperMode = 0;
                    break;
            } // switch (byteState)
        } // if items in buffer
    } // infinite loop

    return 0;
} // end main
/////////////////////////////////////////////////
// ISRs


// [ex3] ISR for TA0.1 CCR0 overflow
#pragma vector = TIMER0_A0_VECTOR
__interrupt void timerA0()
{
    if (stepperMode == 1) { 
		stepState = (stepState + 1) % 8;
		writeStepperState(stepState);
	}
	else if (stepperMode == 2) { 
		stepState = (stepState - 1) % 8;
		writeStepperState(stepState);
	}
		

    TA0CCTL0 &= ~CCIFG; // clear IFG
}

// ISR for UART receive
#pragma vector=USCI_A1_VECTOR
__interrupt void UCA1RX_ISR()
{
    rxByte = UCA1RXBUF; // get the received byte from UART RX buffer

    enqueue(rxByte); // [l3]
}

