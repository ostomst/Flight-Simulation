#include "ht32f5xxxx_01.h"              // Device header
#include "spi.c"
#include "math.h"
#include "uart.c"
double acce_x, acce_y, acce_z;
double gyro_x, gyro_y, gyro_z;
float x_avg, y_avg;

double theta_x,theta_y;

const double PI=3.14159;
float x1 =0, x2 = 0, x3 = 0, x4 = 0; // used to average values
float y1 =0, y2 = 0, y3 = 0, y4 = 0; // used to average values

int check_PD1, check_PD2, check_PB12, Xnegative, Ynegative;

float ava;


void writeReg(unsigned char address, unsigned char data){
	while(HT_I2C0->SR & (1<<19));      // check I2C busy
	HT_I2C0->TAR &= ~(1<<10);          //Set TAR[10] ->0 send start bit TAR[9:0]=MPU_address
	HT_I2C0->TAR |= 0x68;
	while(!(HT_I2C0->SR & (1<<2)));	   //wait until ADRS=1
	while(!(HT_I2C0->SR & (1<<17)));   //check TXDE	
	HT_I2C0->DR = address;             //DR=address	
	while(!(HT_I2C0->SR & (1<<17)));   //check TXDE
	HT_I2C0->DR = data;         			 //DR=address
	HT_I2C0->CR |= (1<<1);		         //stop bit			
}

int16_t readReg(unsigned char address){
	int16_t data;
	//write mode
	while(HT_I2C0->SR & (1<<19));      // check I2C busy
	HT_I2C0->TAR &= ~(1<<10);          //Set TAR[10] ->0 send start bit TAR[9:0]=MPU_address
	HT_I2C0->TAR |= 0x68 ;
	while(!(HT_I2C0->SR & (1<<2)));    //wait until ADRS=1
	while(!(HT_I2C0->SR & (1<<17)));   //check TXDE
	HT_I2C0->DR = address;             //DR=address
	while(!(HT_I2C0->SR & (1<<17)));
	HT_GPIOC->DOUTR	&= ~(1<<15);
	//read mode
	HT_I2C0->TAR |= (1<<10);             //Set TAR[10] ->0 send start bit TAR[9:0]=MPU_address
	HT_I2C0->TAR |= 0x68 ;
	while(!(HT_I2C0->SR & (1<<2)));      //wait until ADRS=1
	while((HT_I2C0->SR & (1<<21)));      //check if in receive mode (not)
//	HT_GPIOC->DOUTR	&= ~(1<<14);
	HT_I2C0->CR &= ~(1<<0);              //set bit AA=0; wait until RXBF=1
	while(!(HT_I2C0->SR & (1<<16)));
	data = HT_I2C0->DR;
	HT_I2C0->CR |= (1<<1);               //send stop bit
	
	return data;
}

double readTemp(unsigned char high, unsigned char low){

	int16_t value= (readReg(high) << 8) | readReg(low);
	double temp= (double)value/340+36.53;
	return temp;
}

float readAcce_n(unsigned char high, unsigned char low){

	int16_t value= (readReg(high) << 8) | readReg(low);
	float acce= (float)value*4/8192;
	return acce;
}
float readGyro_n(unsigned char high, unsigned char low){

	int16_t value= (readReg(high) << 8) | readReg(low);
	float gyro= (float)value/1000*32.8;
	return gyro;
}

void readValue(void){

		acce_x=readAcce_n(0x3B,0x3C);
		acce_y=readAcce_n(0x3D,0x3E);
		acce_z=readAcce_n(0x3F,0x40);
	  // fuse the data from the IMU using trigonometry
		float accTilt1 = atan2f(acce_y, acce_z)*180/PI; // Tilt angle from accelerometer (in degrees)
		float accTilt2 = atan2f(acce_x, acce_z)*180/PI; // Tilt angle from accelerometer (in degrees)
    
		x4 = x3;
		x3 = x2;
		x2 = x1;
		x1 = accTilt1;
		x_avg = (x4 + x3 + x2 + x1)/4;
		if(x_avg >= 0) Xnegative = 0;
		else Xnegative = 1;
		
		y4 = y3;
		y3 = y2;
		y2 = y1;
		y1 = accTilt2;
		y_avg = (y4 + y3 + y2 + y1)/4;	
		if(y_avg >= 0) Ynegative = 0;
		else Ynegative = 1;
		
//		gyro_x=readGyro_n(0x43,0x44);
//		gyro_y=readGyro_n(0x45,0x46);
//		gyro_z=readGyro_n(0x47,0x48);

//		theta_x= atan2(gyro_x,sqrt(pow(gyro_y,2)+pow(gyro_z,2)))*180/PI;
//		theta_y= atan2(gyro_y,sqrt(pow(gyro_x,2)+pow(gyro_z,2)))*180/PI;
			
}

void I2C_init(void){
	
	
	
	//clock speed I2CSLPGR = 108/2   .   Fpclk=32MHz -> Tpclk= 1/32/10^6s  ....400MHz-> Tsck=1/400/10^6=Tpclk*H with H=[ (SHPG + d) + (SLPG + d) ]
	HT_I2C0->SHPGR = 54;
	HT_I2C0->SLPGR = 54;
	HT_I2C0->CR &= ~(1<<7);       //address mode 7bit addr
	HT_I2C0->CR |= (1<<3);        //enable I2C
}

void MPU_init(void){
	writeReg(0x19,0x07);  //Sample rate devider (0x19)- sample rate 1ms
	writeReg(0x1A,0x00);  //configuration (0x1A)
	writeReg(0x1B,0x10);  //Gyro config (0x1B)-->0x10
	writeReg(0x1C,0x08);  //acce config (0x1C) acce = +-4g
	writeReg(0x38,0x01);  //interrupt enable (0x38)
	writeReg(0x6B,0x01);  //power management (0x6B)
}

void dac_reader(void){
	
	uint16_t value1, value2, value3;
	while(!(HT_ADC->DR[0] & (1<<31)));
	value1 = (HT_ADC->DR[0]) & ~(1<<31);
	while(!(HT_ADC->DR[0] & (1<<31)));
	value2 = (HT_ADC->DR[0]) & ~(1<<31);
	while(!(HT_ADC->DR[0] & (1<<31)));
	value3 = (HT_ADC->DR[0]) & ~(1<<31);
	ava =(float)( value1 + value2 + value3)/3;
	
}
	

void uart_out_btn(void){
	
	readValue();
	
	UU_PutNumber(Xnegative);	//Sign of x
	USART_putc(' ');
	
	if(Xnegative == 0)			// X value
		UU_PutNumber((int)x_avg);
	else 
		UU_PutNumber((int)(-x_avg));
	USART_putc(' ');
	
	UU_PutNumber(Ynegative);	//Sign of x
	USART_putc(' ');
	
	if(Ynegative == 0)			// Y value
		UU_PutNumber((int)y_avg);
	else 
		UU_PutNumber((int)(-y_avg));
	
	USART_putc(' ');
	
	UU_PutNumber(ava);			// Potentiometer value to uart

	USART_putc(' ');
	switch(check_PB12){			// left brake
		case 0:
			USART_puts("0");
			
		break;
		case 1:
			USART_puts("1");
			check_PB12 = 0;
		break;
	}
	
	USART_putc(' ');

	switch(check_PD1){			// right brake 
		case 0:
			USART_puts("0");
		break;
		case 1:
			USART_puts("1");
			check_PD1 = 0;
		break;
		case 2:
			USART_puts("2");
			check_PD1 = 0;
		break;
	}
	USART_putc(' ');			
	switch(check_PD2){			// landing_gear
		case 0:
			USART_puts("0");
		break;
		case 1:
			USART_puts("1");
			check_PD2 = 0;
		break;
		case 2:
			USART_puts("2");
			check_PD2 = 0;
		break;
	}

}


void BFTM0_IRQHandler(void)
{
		dac_reader();

		uart_out_btn();
	
		HT_GPIOC->DOUTR	&= ~(1<<14);
		HT_BFTM0->SR = 0;							// clear the BFTM interrupt flag
	
}

void EXTI0_1_IRQHandler(void) // Handle for EXTI_1
		{	
			if((HT_EXTI->EDGESR & (1<<1)) == 0 ){
					check_PD1 = 1;
				}
			else{
					check_PD1 = 2;
			}
			HT_EXTI->EDGEFLGR |=(1<<1); // Clear interrupt flag of EXTI1
		}

void EXTI2_3_IRQHandler(void) // Handle for EXTI_2
		{
			if((HT_EXTI->EDGESR & (1<<1)) == 0 ){
					check_PD2 = 1;
				}
			HT_EXTI->EDGEFLGR |=(1<<2); // Clear interrupt flag of EXTI2
		}	
	
void EXTI4_15_IRQHandler(void) //  Handle for EXTI12
		{
			check_PB12 = 1;
			HT_EXTI->EDGEFLGR |= (1<<12); // Clear interrupt flag of EXTI12
		}

void adc_init(void){
	HT_CKCU->APBCCR1 |= (1<<24); // ADC
	
	// Config Prescaler 
	HT_CKCU->APBPCSR0 |= (3<<12);  //BFTM0 PCLK = AHB/8 = 6MHz
	HT_CKCU->APBPCSR1 |= (3<<4);   // ADC PCLK = AHB/8 = 6MHz
	
	// Config PA6 as ADC-IN
	HT_AFIO->GPACFGR[0] |= (2<<24);

	// Config ADC
	HT_ADC->CR |= (2<<0); // Continuous mode
	HT_ADC->CR |= (1<<8); // 1 Channel
	HT_ADC->LST[0] |= (6<<0); // ADSEQ0 -- ADC_IN6
	HT_ADC->STR = 79; // Sampling time 10us
	HT_ADC->TCR |= (1<<3); // Enable trigger by BFTM0
	HT_ADC->IER |= (1<<24);// Enable overwrite interrupt
	HT_ADC->TSR &=~ (1<<19); //BFTM0 
	HT_ADC->CR |= (1<<7); // Enable ADC
	
}
		
	
		
int main(void){
	// enable clock AFIO, (EXTI), I2C, GPIO
	HT_CKCU->AHBCCR |= (1<<16);  //Port A 
	HT_CKCU->AHBCCR |= (1<<17);  //Port B 
	HT_CKCU->AHBCCR |= (1<<18);  //Port C 
	HT_CKCU->AHBCCR |= (1<<19);  // Clock for Port D
	
	HT_CKCU->APBCCR0 |= (1<<14); //AFIO EXTI
	HT_CKCU->APBCCR0 |= (1<<15);	

	HT_CKCU->APBCCR0 |= (1<<0); //I2C0
	
		// Select prescaler I2C0
	HT_CKCU->APBPCSR0 &= ~(3<<0);
	
		// convert IOs to AF7/(I2C)
	HT_AFIO->GPBCFGR[0] |= (7<<0); //PB0 --> SCL
	HT_AFIO->GPBCFGR[0] |= (7<<4); //PB1 --> SDA
	HT_AFIO->GPBCFGR[1] |= (1<<16);
	
	// Configure Output cho LED PC14, PC15, PC1 
	HT_GPIOC->DIRCR |= (3<<14); //LED0,1
	HT_GPIOC->DIRCR |= (1<<1);	//LED2
	// LED off 
	HT_GPIOC->DOUTR |= (3<<14);
	HT_GPIOC->DOUTR |= (1<<1);
	
		// Configure PD1/Button1 , PD2/Button2, PB12/WakeUp as input pins
	HT_GPIOD->DIRCR &=~(1<<1);  // Button1
	HT_GPIOD->DIRCR &=~(1<<2);  // Button2
	HT_GPIOB->DIRCR &=~(1<<12); // WakeUp
	HT_GPIOD->INER = 0xFFFF;
	HT_GPIOB->INER |= (1<<12);
	
	I2C_init();                   // Initialize I2C
	MPU_init();                   // Initialize MPU6050
	spi_init();	                  // Initialize SPI Max7219
	uart_init();									// Initialize UART
	adc_init();										// Initialize ADC
	
	
		// Configure BFTM
	HT_BFTM0->CNTR = 0;										// reset counter
	HT_BFTM0->CMP = 3000000;      				// 0.5s timer
	HT_BFTM0->CR = 5;											// enable timer, interrupt, repetitive mode
	
		//Assign EXTI function to GPIO pins
	HT_AFIO->ESSR[0] |= (3<<4);  // PD1-Button1-EXTI1
	HT_AFIO->ESSR[0] |= (3<<8);	 // PD2-Button2-EXTI2
	HT_AFIO->ESSR[1] |= (1<<16); // PB12-WakeUp-EXTI12
 
	//Configure edge detection, de-bounce
	HT_EXTI->CFGR1 |= (4<<28);	// Both edge Trigger for EXT1
	HT_EXTI->CFGR2 |= (4<<28);   // Both edge Trigger for EXT2
	HT_EXTI->CFGR12 |= (4<<28); // Both edge Trigger for EXT12
	
	// Enable EXTI interrupt pins
	HT_EXTI->CR |= (1<<1);  // Enable for EXTI1
	HT_EXTI->CR |= (1<<2);  // Enable for EXTI2
	HT_EXTI->CR |= (1<<12);  // Enable for EXTI12
	
	// Enable EXTI interrupts
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	NVIC_EnableIRQ(BFTM0_IRQn);
	while(1){
	
	}
}
