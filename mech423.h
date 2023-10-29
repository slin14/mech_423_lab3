#ifndef MECH423_LIB
#define MECH423_LIB

/////////////////////////////////////////////////
// FUNCTIONS
// DCO (8 MHz) -> MCLK
//             -> ACLK
//             -> SMCLK
void setup_8MHz_clks() {
    CSCTL0  = CSKEY;            // password to access CS registers
    CSCTL1 |= DCOFSEL_3;        // DCO 8 MHz
    CSCTL2  = SELM__DCOCLK +    // DCO -> MCLK
              SELA__DCOCLK +    // DCO -> ACLK
              SELS__DCOCLK ;    // DCO -> SMCLK
}

// [ex1] divide SMCLK by 32
void div_32_SMCLK() {
    CSCTL3 |= DIVS__32;
}

// output on P3
void output_P3(int bit) {
    P3DIR  |= bit;
    P3SEL1 |= bit;
    P3SEL0 |= bit;
}

// configure LED8 to LED1 as digital outputs (P3.7 to P3.4, PJ.3 to PJ.0)
void setup_LEDs() {
    P3DIR |= BIT7 + BIT6 + BIT5 + BIT4; // LED8 to LED5
    PJDIR |= BIT3 + BIT2 + BIT1 + BIT0; // LED4 to LED1
}

// display "display_byte" on LEDs
void display_LEDs(int display_byte) {
    P3OUT &= ~HIGHMASK;                 // clear LED8 to LED5
    PJOUT &= ~LOWMASK;                  // clear LED4 to LED1
    P3OUT |= (display_byte & HIGHMASK); // display_byte upper 4 bits
    PJOUT |= (display_byte & LOWMASK);  // display_byte lower 4 bits
}

// [ex2] toggle the 0's in "display_byte" on LEDs
void toggle_LED_zeros(int display_byte) {
    P3OUT ^= (~display_byte & HIGHMASK); // toggle the 0s in upper 4 bits
    PJOUT ^= (~display_byte & LOWMASK);  // toggle the 0s in lower 4 bits
}

// [ex2x] toggle the 1's in "display_byte" on LEDs
void toggle_LED_ones(int display_byte) {
    P3OUT ^= (display_byte & HIGHMASK); // toggle the 1s in upper 4 bits
    PJOUT ^= (display_byte & LOWMASK);  // toggle the 1s in lower 4 bits
}

// [ex4x] turn ON the 1's in "display_byte" on LEDs
void turn_ON_LED_ones(int display_byte) {
    P3OUT |= (display_byte & HIGHMASK); // turn ON the 1s in upper 4 bits
    PJOUT |= (display_byte & LOWMASK);  // turn ON the 1s in lower 4 bits
}

// [ex4x] turn OFF the 1's in "display_byte" on LEDs
void turn_OFF_LED_ones(int display_byte) {
    P3OUT &= ~(display_byte & HIGHMASK); // turn OFF the 1s in upper 4 bits
    PJOUT &= ~(display_byte & LOWMASK);  // turn OFF the 1s in lower 4 bits
}

// [ex3] set up buttons S1 and S2 (inputs on P4.0 and P4.1)
void setup_buttons_input() {
    P4DIR &= ~(BIT0 + BIT1);  // P4.0 and P4.1 as input
    P4REN |=  (BIT0 + BIT1);  // enable pullup
    P4OUT |=  (BIT0 + BIT1);  // enable pullup
}

// [ex3] enable buttons S1 and S2 interrupt (on P4.0 and P4.1)
void enable_buttons_interrupt() {
    // on rising edge (when user lets go of button)
    P4IES &= ~(BIT0 + BIT1); // rising edge
    P4IE  |=  (BIT0 + BIT1); // enabel interrupt
}

// [ex10x] display a byte in binary on LEDs
void byteDisplayLED(unsigned char in)
{
    char high = in & ~LOWMASK;
    char low = in & ~HIGHMASK;
    PJOUT &= ~LOWMASK;
    P3OUT &= ~HIGHMASK;
    PJOUT |= low;
    P3OUT |= high;
}

// set up UART 9600 baud from 8MHz
void setup_UART(int int_en) {
    P2SEL0 &= ~(BIT0 + BIT1); // UART ports P2.0 and P2.1 // redundant
    P2SEL1 |=  (BIT0 + BIT1); // UART ports P2.0 and P2.1

    UCA0CTLW0 |= UCSWRST;                   // Put the UART in software reset
    UCA0CTLW0 |= UCSSEL__ACLK;              // Run the UART using ACLK
  //UCA0CTLW0 |= UCSSEL__SMCLK;             // Run the UART using SMCLK
    UCA0MCTLW = UCOS16 + UCBRF0 + 0x4900;   // Baud rate = 9600 from an 8 MHz clock
    UCA0BRW = 52;                           // Baud rate = 9600 from an 8 MHz clock
    UCA0CTLW0 &= ~UCSWRST;                  // release UART for operation

    if (int_en)    UCA0IE |= UCRXIE;        // Enable UART Rx interrupt
}


// transmit "txByte" over UART
void txUART(unsigned char txByte)
{
    while (!(UCA0IFG & UCTXIFG)); // wait until UART not transmitting
    UCA0TXBUF = txByte;           // transmit txByte
}


// [ex5] setup Timer B Up Mode - counts up to "myTB1CCR0"
// led_port    TB1.x    output port
// 1           TB1.1    P3.4
// 1           TB1.2    P3.5
// 0           TB1.1    P1.6
// 0           TB1.2    P1.7
void setup_timerB_UP_mode(int led_port) {
    if (led_port == TIMERB_LED_PORT) {
        P3DIR  |=  (BIT4 + BIT5); // P3.4 and P3.5 as output
        P3SEL0 |=  (BIT4 + BIT5); // select TB1.1 and TB1.2
        P3SEL1 &= ~(BIT4 + BIT5); // select TB1.1 and TB1.2  // redundant
    } else {
        P1DIR  |=  (BIT6 + BIT7); // P1.6 and P1.7 as output
        P1SEL0 |=  (BIT6 + BIT7); // select TB1.1 and TB1.2
        P1SEL1 &= ~(BIT6 + BIT7); // select TB1.1 and TB1.2  // redundant
    }

    TB1CTL |= TBSSEL__ACLK + // ACLK as clock source (8 MHz)
              MC__UP       + // Up mode
              ID__8        ; // divide input clock by 8 -> timer clk 1 MHz
    TB1CTL |= TBCLR;         // clr TBR, ensure proper reset of timer divider logic

    //TB1CCR0 = myTB1CCR0;     // value to count up to in UP mode
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

// Timer B only: can set counter length using TBxCTL.CNTL
// [ex5x] setup Timer A Up Mode - counts up to "myTA1CCR0"
//             TA1.x    output port
//             TA1.1    P1.2
//             TA1.2    P1.3
void setup_timerA_UP_mode() {
    P1DIR  |=  (BIT2 + BIT3); // P1.2 and P1.3 as output
    P1SEL0 |=  (BIT2 + BIT3); // select TA1.1 and TA1.2
    P1SEL1 &= ~(BIT2 + BIT3); // select TA1.1 and TA1.2  // redundant

    TA1CTL |= TASSEL__ACLK + // ACLK as clock source (8 MHz)
              MC__UP       + // Up mode
              ID__8        ; // divide input clock by 8 -> timer clk 1 MHz
    TA1CTL |= TACLR;         // clr TAR, ensure proper reset of timer divider logic

    //TA1CCR0 = myTA1CCR0;     // value to count up to in UP mode
}

// [ex6x] setup Timer B CONTINUOUS Mode - counts up to TxR (counter length set by CNTL)
void setup_timerB_CONT_mode() {
    // configure P1.4 as input to timerB TB0.1
    P1DIR  &= ~BIT4;
    P1SEL1 &= ~BIT4;
    P1SEL0 |=  BIT4;

    // configure Timer B
    TB0CTL   = TBSSEL__ACLK   + // ACLK as clock source (8 MHz)
               MC__CONTINUOUS + // Continuous mode
               ID__8          + // divide input clock by 8 -> timer clk 1 MHz
               TBCLR          ; // Timer B counter clear, ensure proper reset of timer divider logic
    // Note:   TBIE (overflow interrupt is NOT enabled)
}


// Setup ADC (for accelerometer and NTC)
void setup_ADC() {
    // Power the Accelerometer and/or NTC
    P2DIR |= BIT7; // configure P2.7 as output
    P2OUT |= BIT7; // output high to provide power

    // set up A12, A13, A14 (accelerometer X, Y, Z) as input for ADC
    P3SEL0 |= BIT0 + BIT1 + BIT2;
    P3SEL1 |= BIT0 + BIT1 + BIT2;

    // set up A4 (NTC) as input for ADC on P1.4
    P1SEL0 |= BIT4;
    P1SEL1 |= BIT4;

    ADC10CTL0 |= ADC10ON    + // turn on ADC
                 ADC10SHT_2 ; // sample & hold 16 clks
    ADC10CTL1 |= ADC10SHP;    // sampling input is sourced from timer
    ADC10CTL2 |= ADC10RES;    // 10-bit conversion results
}

// ADC to read a specified channel
unsigned int adcReadChannel(int channel)
{
    ADC10MCTL0 |= channel;  // channel select; Vref=AVCC
    // sample ADC (enable conversion)
    ADC10CTL0 |= ADC10ENC + // enable conversion
                 ADC10SC  ; // start conversion

    // wait for ADC conversion complete
    while(ADC10CTL1 & ADC10BUSY);
    int result = ADC10MEM0;

    // ADC disable conversion to switch channel
    ADC10CTL0 &= ~ADC10ENC;

    ADC10MCTL0 &= ~channel; // clear channel selection

    return result;
}


// [ex8] display temp on LEDs
void displayTempOnLEDs()
{
    int lightUpTo = (tempThresh - temp);            // signed
    if (lightUpTo < 1)        lightUpTo = 1;        // min = 1
    if (lightUpTo > NUM_LEDS) lightUpTo = NUM_LEDS; // max = NUM_LEDS

    // PJx = 1<<x * (lightUpTo>=x) x from 1 to 4 // PJ.0-3 -> LED 1-4
    // P3x = 1<<x * (lightUpTo>=x) x from 5 to 8 // P3.4-7 -> LED 5-8
    PJOUT_L = 0b00001000*(lightUpTo>=4) + 0b00000100*(lightUpTo>=3) + 0b00000010*(lightUpTo>=2) + 0b00000001*(lightUpTo>=1);
    P3OUT   = 0b10000000*(lightUpTo>=8) + 0b01000000*(lightUpTo>=7) + 0b00100000*(lightUpTo>=6) + 0b00010000*(lightUpTo>=5);
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


#endif
