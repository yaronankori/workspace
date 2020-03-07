
#include <stdint.h>
#include "Board_LED.h"
#include "Board_Buttons.h"

void delay(void)
{
	unsigned int i = 0;
	for(i=0;i<500000;i++);
}

int main(void)
{
	
	LED_Initialize();
	Buttons_Initialize();
	
	while(1)
	{

    if(Buttons_GetState() == 1)
		{
			LED_On(0);
		}
		else 
		{
			LED_Off(0);
		}
		delay();
	}
	
	return 0;
}


