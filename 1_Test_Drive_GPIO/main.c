/* -----------------------------------------------------------------------------
 * The functions are simply designed for learning and research purposes.
 * perform only simple peripheral input/output functions
 *
 * by: TKÐ
 *
 * $Date:        26. February 2025
 * $Revision:    V0.0
 *
 * Project:      GPIO Driver for ST STM32F103C8T6
 * -------------------------------------------------------------------- */
#include "driverGPIOTest.h"

void delay_ms(unsigned long ms);

int main(void){
	clockPort_config(GPIOD_d, ENA);
	clockPort_config(GPIOC_d, ENA);
	
	GPIOx_config(GPIOC_d, pin13_IO_, modeOutput2Mhz);
	GPIOx_config(GPIOD_d, 9, modeOutput2Mhz);
	
	
	
	while(1){
		GPIOx_write(GPIOC_d, pin13_IO_, 1);
		delay_ms(5000);
		GPIOx_write(GPIOC_d, pin13_IO_, 0);
		delay_ms(5000);
	}
}

void delay_ms(unsigned long ms){
	for(unsigned long i = 0; i< ms*1000; i++){
		 __NOP();
	}
}

