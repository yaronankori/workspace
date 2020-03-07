
#include <stdint.h>
#include "Board_LED.h"

void delay(void)
{
	unsigned int i = 0;
	for(i=0;i<500000;i++);
}

int main(void)
{
	
	LED_Initialize();
	
	while(1)
	{
		LED_On(0);
		LED_On(1);
		LED_On(2);
		LED_On(3);
		delay();
		
		LED_Off(0);
		LED_Off(1);
		LED_Off(2);
		LED_Off(3);
		delay();
	}
	
	return 0;
}


