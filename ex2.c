#include <msp430.h> 


/**
 * main.c - l3 ex2
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	return 0;
}
