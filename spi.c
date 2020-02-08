#include "ht32f5xxxx_01.h"              // Device header
volatile uint32_t msTicks = 0;


const uint8_t digit[10]={0x7e,0x30,0x6d,0x79,0x33,0x5b,0x5f,0x70,0x7f,0x7b};
const uint8_t addr[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

void SysTick_Handler(void)
{
      /* Increment counter necessary in Delay()*/
      msTicks++;
}
void Delay(uint32_t dlyTicks)
{
      uint32_t curTicks;
 
      curTicks = msTicks;
      while ((msTicks - curTicks) < dlyTicks) ;
}

void dataOut(uint8_t addr, uint8_t data){
	uint16_t dataout=(addr<<8)|data;	
	HT_SPI1->DR =dataout;
	while((HT_SPI1->SR & (1<<0))==0);	
}

void MAX7219_Init (void)
{
	
    dataOut (0x0c,0x01); //normal operation
    dataOut (0x0a,0x03); //intensity
    dataOut (0x0b,0x07); //all digits on
    dataOut (0x09,0x00); //decoding
    dataOut (0x0f,0x00); //display test off
}


void SPI_out(int data) {
	for(int i=0;i<8;i++){
		if(data == 0)
			dataOut(addr[i],0x00);
		else
			dataOut(addr[i],digit[data%10]);
	data=data/10;
	}
}

void spi_init(){
	HT_CKCU->APBCCR0 |=(1<<5); // Enable CLK SPI1
	
	HT_CKCU->APBPCSR0 &=~ (3<<6); //Config APB prescaler for SPI1 
	
	
	//Config PA0,PA1,PA2 as SPI
	HT_AFIO->GPACFGR[0] |= (5<<0);//PA0 as SPI1_SCK
	HT_AFIO->GPACFGR[0] |= (5<<4);//PA1 as SPI1_MOSI
	HT_AFIO->GPACFGR[0] |= (5<<12);//PA3 as SPI1_SEL(SEL select line)
	
	
	
	HT_SPI1->CR1 |= (1<<14); // Config MCU work as Master
	HT_SPI1->CR0 |=(1<<3); 
	HT_SPI1->CR0 |=(1<<0);
	HT_SPI1->CR1 |= (1<<13);
	HT_SPI1->CR1 &= ~(1<<11);
	HT_SPI1->CR1 |= (6<<8); // SPI Data Transf? Format CPOL=0, CPHA=0
	HT_SPI1->CPR = 2;   //SCK=1MHz
	HT_SPI1->CR1 &=~(1<<12);
	SysTick_Config(48000); 	
}
