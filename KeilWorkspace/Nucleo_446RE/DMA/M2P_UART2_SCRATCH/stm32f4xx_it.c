#include <stdint.h>

#include "stm32f446xx.h"


#define is_it_HT()  DMA1->HISR  & (1<<20)
#define is_it_FT()  DMA1->HISR  & (1<<21)
#define is_it_TE()  DMA1->HISR  & (1<<19)
#define is_it_FE()  DMA1->HISR  & (1<<16)
#define is_it_DME() DMA1->HISR  & (1<<18)

extern void HT_Complete_callback(void);
extern void FT_Complete_callback(void);
extern void TE_Complete_callback(void);
extern void FE_Complete_callback(void);
extern void DME_Complete_callback(void);


void clear_exti_pending_bit(void);

/*void EXTI15_10_IRQHandler(void)
{
 // There is a problem with the nucleo since the STLINK is also connected to this pin, so all the time interrupt will be arrived
	clear_exti_pending_bit();
}*/

void EXTI1_IRQHandler(void)
{
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	pUART2->CR3 |= (1 << 7);
	
	
	clear_exti_pending_bit();
}


// IRQ handler for DMA1 stream6 global interupt
void DMA1_Stream6_IRQHandler(void)
{
	// Half Tranfer
	
	if(is_it_HT())
	{
		DMA1->HIFCR |= (1 << 20); // Clear the half interrupt by the software
		HT_Complete_callback();
	}
	else if(is_it_FT())
	{
		DMA1->HIFCR |= (1 << 21); // Clear the full interrupt by the software
		FT_Complete_callback();
	}
	else if(is_it_TE())
	{
		DMA1->HIFCR |= (1 << 19);
		TE_Complete_callback();
	}
	else if(is_it_FE())
	{
		DMA1->HIFCR |= (1 << 16);
		FE_Complete_callback();
	}
	else if(is_it_DME())
	{
		DMA1->HIFCR |= (1 << 18);
		DME_Complete_callback();
	}
	else
	{
		
	}
	
}


void clear_exti_pending_bit(void)
{
  EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	if(pEXTI->PR & (1 << 1))
	{
		pEXTI->PR &= (1 << 1); // Clear the bit (set the bit)
	}
}
/*void clear_exti_pending_bit(void)
{
  EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	if(pEXTI->PR & (1 << 13))
	{
		pEXTI->PR &= (1 << 13); // Clear the bit (set the bit)
	}
}*/