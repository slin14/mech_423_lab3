#include <msp430.h> 


/**
 * main.c - l3 ex4
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	return 0;
}
