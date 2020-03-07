

#include <stdint.h>

#include "stm32f446xx.h"

void button_init(void);
void uart2_init(void);
void dma1_init(void);
void send_some_data(void);
#define BASE_ADDR_OF_GPIOC_PERI GPIOC
#define BASE_ADDR_OF_GPIOA_PERI GPIOA

char data_stream[] = "Hello World123\r\n";
void enable_dma1_stream6(void);
void dma1_interrupt_configuration(void);
void HT_Complete_callback(void);
void FT_Complete_callback(void);
void TE_Complete_callback(void);
void FE_Complete_callback(void);
void DME_Complete_callback(void);

int main(void)
{
	//Default clock will be 16MHz internal RC clock
	
	button_init();
	uart2_init();
	//send_some_data(); // Sending via the DMA is from the BUTTON interrupt
	dma1_init();
	dma1_interrupt_configuration();
	//14. Finally: Enable the stream
	enable_dma1_stream6();
	
	while(1);
	
	return 0;
}

void button_init(void)
{
	// button is connected to PC13
	GPIO_TypeDef *pGOIOC;	
	pGOIOC = BASE_ADDR_OF_GPIOC_PERI;
	
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	SYSCFG_TypeDef *pSYSCFG;
	pSYSCFG = SYSCFG;
	
	//***********PROBLEM WITH PC13 because of conflict with ST PA1 will be connected instead******************************
	/*//1. Enable the clock of GPIOC peripheral
	pRCC->AHB1ENR |= (1<<2);
	
	//2. Keep the GPIO pin in Input mode
	pGOIOC->MODER &= ~(0x3 <<26);//(pin 13 is 00 -> input)
	
	//3. Enable the interrupt over the gpio pin
	pEXTI->IMR |= (1<<13);
	
	//4. Enable the clock for SYSCFG
	pRCC-> APB2ENR |= (1<14);
	
	//5. Configuring the SYSCFG CR4 register
	pSYSCFG->EXTICR[3] &= ~(0xF <<4); //Clearing
	pSYSCFG->EXTICR[3] |=  (0x2 <<4); //Setting
	
	//6. Configure the edge detection on that gpio pin for the interrupt
	pEXTI->FTSR |= (1<<13);
		
	//7. Enable the IRQ related to that gpio pin in NVIC of the processor
	NVIC_EnableIRQ(EXTI15_10_IRQn);*/
	//***************************************************************************************
	
	// Config PA1 interrupt pin, use and extenal switch
	GPIO_TypeDef *pGOIOA;	
	pGOIOA = BASE_ADDR_OF_GPIOA_PERI;
	
		//1. Enable the clock of GPIOA peripheral
	pRCC->AHB1ENR |= (1<<0);
	
	//2. Keep the GPIO pin in Input mode
	pGOIOA->MODER &= ~(0x3 <<2);//(pin 1 is 00 -> input)
	
	//3. Enable the interrupt over the gpio pin
	pEXTI->IMR |= (1<<1);
	
	//4. Enable the clock for SYSCFG
	pRCC-> APB2ENR |= (1<14);
	
	//5. Configuring the SYSCFG CR4 register
	pSYSCFG->EXTICR[0] &= ~(0xF <<4); //Clearing
	pSYSCFG->EXTICR[0] |=  (0x2 <<4); //Setting
	
	//6. Configure the edge detection on that gpio pin for the interrupt
	pEXTI->FTSR |= (1<<1);
		
	//7. Enable the IRQ related to that gpio pin in NVIC of the processor
	NVIC_EnableIRQ(EXTI1_IRQn);
}



void uart2_init(void)
{
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	GPIO_TypeDef *pGOIOA;	
	pGOIOA = BASE_ADDR_OF_GPIOA_PERI;
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	//1. enable the peripheral clock for the uart2 peripheral
	pRCC->APB1ENR |= ( 1<<17 );
	
	//2. Configure the gpio pins for uart_tx and uart_rx functionality
	
	// first lets configure PA2 as UART2 TX
	
	//2.1  Enable the clock for the GPIOA peripheral 
	//pRCC->AHB1ENR |= (1 << 0 );
	
	//2.2  Change the mode of the PA2 to alternate function
	pGOIOA->MODER  &= ~(0x3 << 4); //Clear the bits
	pGOIOA->MODER  |=  (0x2 << 4); // Set to Alternate function
	pGOIOA->AFR[0] &= ~(0xF << 8); //Clear
	pGOIOA->AFR[0] |= (0x7 << 8); //Set to alternate fucntion 7
	
	//2.3 Enable the pull up resistor
	pGOIOA->PUPDR |= (0x1 << 4);
	
	
	
	// Then lets configure PA3 as UART2 RX
	//2.4  Change the mode of the PA2 to alternate function
	pGOIOA->MODER  &= ~(0x3 << 6); //Clear the bits
	pGOIOA->MODER  |=  (0x2 << 6); // Set to Alternate function
	pGOIOA->AFR[0] &= ~(0xF << 12); //Clear
	pGOIOA->AFR[0] |= (0x7 << 12); //Set to alternate fucntion 7
	
	//2.5 Enable the pull up resistor
	pGOIOA->PUPDR |= (0x1 << 6);
	
	//3. Configure the baudrate
	pUART2->BRR = 0x8B;

  //4. configure the data width , no of stop bits etc
	//everything is by default ok
  //1MBS or 2 MBS, better to have 2 stopbits
	
	//5. Enable the TX engine of the uart peripheral	
	pUART2->CR1 |= (1 << 3);
	
	//6. Enable the UART peripheral
	pUART2->CR1 |= (1 << 13);
	
}

void send_some_data(void)
{
	char somedata[] = "Hello World\r\n";
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	uint32_t len = sizeof(somedata);
	
	for(uint32_t i=0 ;i<len; i++)
	{
		//if TX is 1 put the byte,waiting for TXE to become 1
		while(!(pUART2->SR & ( 1 << 7 )))
		{			
			asm("nop");
		}
		pUART2->DR = somedata[i];
	}
	
	
	
}

void dma1_init(void)
{
  RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	DMA_TypeDef *pDMA;
	pDMA = DMA1;
	
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
  USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	
	
	//1. Enable the peripheral clock for dma1
	pRCC->AHB1ENR |= ( 1<<21);
	
	//2. Identify the streams that is suitable for your peripheral
	//stream 6
	
	//3. Identify the channel number which uart2 peripheral
	//channel 4
	pSTREAM6->CR &= ~(0x7 << 25);// Clear the bits
	pSTREAM6->CR |=  (0x4 << 25);// Set channel 4
	
	 //*********************************
	//4. Program the source address(our source is memory, so we have to program memory address register)
	pSTREAM6->M0AR = (uint32_t)data_stream;
	//5. Program the destination address
	pSTREAM6->PAR = (uint32_t)&USART2->DR;
	//*********************************
	
	//6. Program number of data items to send
	uint32_t len = sizeof(data_stream);
	pSTREAM6->NDTR = len;
	
	//7. The direction of data transfer m2p p2m m2m (p-peripheral m-memory)
	pSTREAM6->CR |= (0x1 << 6);
	
	//8. Program the source and destination data width
	pSTREAM6->CR &= ~(0x3 << 13);
	pSTREAM6->CR &= ~(0x3 << 11);
	// 8. Auto increment of the memory
	pSTREAM6->CR |= (0x1 << 10);
	
	
	//9. Direct mode or fifo mode
	pSTREAM6->FCR |= (0x1 << 2); // Direct mode disable => FIFO mode enable
	
	//10.Select the FIFO threchold
	pSTREAM6->FCR &= ~(0x3 << 0);
	pSTREAM6->FCR |=  (0x3 << 0);
	
	
	//11 Enable the circular mode if requierd
	// circular mode is disabled by default
	
	//12. Single transfer or burst transfer
	// Single transfer by default
	
	//13. Configure the stream priority
	// Low by default
	


	
}


void enable_dma1_stream6(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
	//14. Finally: Enable the stream
	pSTREAM6->CR |= (0x1 << 0);
}


void dma1_interrupt_configuration(void)
{
  DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	//1.lets do half transfer IE
	pSTREAM6->CR |= (1 << 3);
	
	//2.Tranfer complete IE TCIE
	pSTREAM6->CR |= (1 << 4);
	
	//3.Trenafer error IE(FEIE)
	pSTREAM6->CR |= (1 << 2);
	
	//4.FIFO overrun underrun IE(FEIE)
	pSTREAM6->FCR |= (1 << 7);
	
	//5.Direct mode error (DMEIE)
	pSTREAM6->CR |= (1 << 1);
	
	//6. Enable the IRQ for DMA1 stream6 global interrupt in NVIC
	
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	
}


void HT_Complete_callback(void)
{
	asm("nop");
}
void FT_Complete_callback(void)
{
  USART_TypeDef *pUART2;
	pUART2 = USART2;
	
		DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
  uint32_t len = sizeof(data_stream);
 	pSTREAM6->NDTR = len;
	
	pUART2->CR3 &= ~(1 << 7);// Clear the CR3 CMD
	
	enable_dma1_stream6();
	
	
  // Press again the button to get the string
}
void TE_Complete_callback(void)
{
	asm("nop");
}
void FE_Complete_callback(void)
{
	asm("nop");
}
void DME_Complete_callback(void)
{
	asm("nop");
}