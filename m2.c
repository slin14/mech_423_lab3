
#include <msp430.h>


volatile char buffer[50]; //16 bits
volatile int end_index = 0;
volatile unsigned int start_index = 0;
volatile int length = 0;

volatile char state_var = 2; //stop

volatile char lower_byte = 0;
volatile char upper_byte = 0;
volatile unsigned short full_byte = 0x7FFF;
volatile char packetByteCount=0;

void uart_send(int a)
{
    while (!(UCA1IFG & UCTXIFG)); // Wait until the previous Tx is finished
    UCA1TXBUF = a;
}

void setupclock()
{
    //Setup clocks
    CSCTL0 = 0xA500; // Write password

    CSCTL1 |= DCOFSEL_3; //11b DCO = 8 MHz

    CSCTL2 |= SELM__DCOCLK + SELA__DCOCLK + SELS__DCOCLK; // MCLK = DCO, ACLK = DCO, SMCLK = DCO

    CSCTL3 |= DIVS__32; //32 smlk divider ->250kHz
}

void timerB1setup()
{
    //Timer B1 General
    TB1CTL |= TBSSEL_1; //select aclk
    TB1CTL |= MC__UP; //up mode
    TB1CCR0 = 0xffff;    //set (or reset) at max


    //Timer B1.1 Setup
    TB1CCTL1 |= OUTMOD_7;   //reset/set mode
    TB1CCR1 = full_byte;    //reset at inputed byte (control duty cycle)
    //TB1R = 0xffff;          //maximum count (not needed for up_mode. may be useful in continuous mode)


    //Set pins to output the clock
    //(note: leds will also come on!)

    P3DIR |= BIT4; //output TB1.1
    P3SEL0 |= BIT4; //TB1.1 select
    P3SEL1 &= ~BIT4; //TB1.1 select

    P3DIR |= BIT5; //output TB1.2
    P3SEL0 |= BIT5; //TB1.2 select
    P3SEL1 &= ~BIT5; //TB1.2 select



}

void timerB0setup()
{
    //Timer B1 General
    TB0CTL |= TBSSEL_1; //select aclk
    TB0CTL |= MC__UP; //up mode
    TB0CCR0 = 0xffff;    //set (or reset) at max


    //Timer B0.1 Setup
    TB0CCTL0 |= OUTMOD_7;   //reset/set mode
    TB0CCR0 = full_byte;    //reset at inputed byte (control duty cycle)
    //TB1R = 0xffff;          //maximum count (not needed for up_mode. may be useful in continuous mode)


    //Set pins to output the clock
    //(note: leds will also come on!)

    P1DIR |= BIT4; //output TB0.1
    P1SEL0 |= BIT4; //TB0.1 select
    P1SEL1 &= ~BIT4; //TB0.1 select

    P1DIR |= BIT5; //output TB0.2
    P1SEL0 |= BIT5; //TB0.2 select
    P1SEL1 &= ~BIT5; //TB0.2 select

}

void timerB2setup()
{
    //Timer B1 General
    TB2CTL |= TBSSEL_1; //select aclk
    TB2CTL |= MC__UP; //up mode
    TB2CCR0 = 0xffff;    //set (or reset) at max


    //Timer B0.1 Setup
    TB2CCTL1 |= OUTMOD_7;   //reset/set mode
    TB2CCR1 = full_byte;    //reset at inputed byte (control duty cycle)
    //TB1R = 0xffff;          //maximum count (not needed for up_mode. may be useful in continuous mode)


    //Set pins to output the clock
    //(note: leds will also come on!)

    P2DIR |= BIT1; //output TB0.1
    P2SEL0 |= BIT1; //TB0.1 select
    P2SEL1 &= ~BIT1; //TB0.1 select

    /*
    P1DIR |= BIT5; //output TB0.2
    P1SEL0 |= BIT5; //TB0.2 select
    P1SEL1 &= ~BIT5; //TB0.2 select*/

}

void setupuart()
{
    // UART
    // Configure UART pins
    P2SEL0 &= ~(BIT5 + BIT6);
    P2SEL1 |= BIT5 + BIT6;

    // Configure UART
    UCA1CTLW0 |= UCSWRST; // Put the UART in software reset

    UCA1CTLW0 &= ~(UC7BIT + UCPAR + UCSPB); // 8 bit, no parity, 1 stop bit (default)

    UCA1CTLW0 |= UCSSEL0;                   // Run the UART using ACLK
    UCA1MCTLW = UCOS16 + UCBRF0 + 0x4900;   // Baud rate = 9600 from an 8 MHz clock //Table 18-5
    UCA1BRW = 52;                           // Baud rate = 9600 //Table 18-5

    UCA1CTLW0 &= ~UCSWRST; // release UART for operation

    UCA1IE |= UCRXIE; // Enable UART Rx interrupt
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  setupclock();

  setupuart();

 // timerB1setup();
  //timerB0setup();
  timerB2setup();


  //setup led1
  PJDIR |= BIT0;

  P3DIR |= (BIT7 + BIT6); // DC motor pins

  _EINT();

  int i;

  while(1)
  {
      for (i=0;i<20000;i++)
          _NOP();       // nothing

      TB2CCR1 = full_byte;

      switch(state_var)
      {
          case 0:       // CCW command 2
              P3OUT |= BIT6; //AIN2
              P3OUT &= ~BIT7; //AIN1
              break;

          case 1:       // CW command 3
              P3OUT &= ~BIT6; //AIN2
              P3OUT |= BIT7; //AIN1
              break;

          case 2:       // STOP command 4
              P3OUT &= ~BIT6; //AIN2
              P3OUT &= ~BIT7; //AIN1
              break;

      }
  }
}

void process_message(void)
{

    //extract command byte from buffer
    char command = buffer[start_index+1]; //2nd byte

    //extract data bytes
    upper_byte = buffer[start_index+2]; //3rd byte
    lower_byte = buffer[start_index+3]; //4th byte

    //change data bytes if esc byte = 1
    if(buffer[start_index+4] & 0x01)//bit0
        lower_byte = 0xff;
    if(buffer[start_index+4] & 0x02)//bit1
        upper_byte = 0xff;

    //combine bytes into one word
    full_byte = (upper_byte << 8) + lower_byte; //byte is 8 bits, bit shift by 8

    //Commands
    if(command == 0x02)
        state_var = 0;

    if(command == 0x03)
        state_var = 1;

    if(command == 0x01)
        state_var = 2;


    //Remove packet from buffer (adjust buffer indexes)
    length -= 5;
    start_index +=5;

    if(start_index>=50)
        start_index -= 50; //wrap around

}



#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    unsigned char RxByte = 0;
    RxByte = UCA1RXBUF; // Get the new byte from the Rx buffer


    if(RxByte == 0xff) //start byte
        packetByteCount=5;

    //starting packet was received! Store it and the next 4 bytes.
    if(packetByteCount > 0)
    {
        packetByteCount -= 1;

        if(length==50) //this will never be called
        {
            //error
            uart_send(0x00);
            uart_send(0x51); //S
            uart_send(0x00);
        }
        else
        {
            buffer[end_index] = RxByte; //add to buffer

            length +=1; //adjust size
            end_index +=1;

            if(end_index == 50)
                end_index = 0; //wrap around
        }

        //all5 bytes have been added, process message
        //could probably do this in main loop too with length==5;
        if(packetByteCount==0)
            process_message();

    }

}

