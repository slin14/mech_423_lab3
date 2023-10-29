#include <msp430.h> 


/**
 * main.c - l3 ex6
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	return 0;
}
