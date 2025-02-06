#include "driverGPIOTest.h"


// ham kiem tra trang thai
/**
  \fn          uint32_t statusClockPort_config(GPIO_TypeDef_t *GPIOx);
  \brief       Kiem tra cau hinh Port Clock
  \param[in]   GPIOx         Pointer to GPIO peripheral
  \return      true  - success
               false - error
*/
uint32_t statusClockPort_config(GPIO_TypeDef_t *GPIOx){
	
	if			(GPIOx == GPIOA_d) 	return ((RCC_d->APB2ENR & (1 << 2)) != 0U) ? 1 : 0;
	else if (GPIOx == GPIOB_d) 	return ((RCC_d->APB2ENR & (1 << 3)) != 0U) ? 1 : 0;
	else if (GPIOx == GPIOC_d) 	return ((RCC_d->APB2ENR & (1 << 4)) != 0U) ? 1 : 0;
	else if (GPIOx == GPIOD_d) 	return ((RCC_d->APB2ENR & (1 << 5)) != 0U) ? 1 : 0;
	else if (GPIOx == GPIOE_d) 	return ((RCC_d->APB2ENR & (1 << 6)) != 0U) ? 1 : 0;
	
	return 0;
}

/**
  \fn          clockPort_config(GPIO_TypeDef_t *GPIOx, 
																				uint8_t enable)
  \brief       Port Clock Control
  \param[in]   GPIOx  Pointer to GPIO peripheral
  \param[in]   enable Enable or disable clock
*/
void clockPort_config(GPIO_TypeDef_t *GPIOx, uint8_t enable){
	// kiem tra trang thai ban dau da bat chua
	uint32_t status = statusClockPort_config(GPIOx);
	
	//chua bat truoc do thi chi co the bat
	if((enable) && (!status)){
		if      (GPIOx == GPIOA_d) 	RCC_d->APB2ENR = RCC_APB2ENR_IOPAEN_;
		else if (GPIOx == GPIOB_d) 	RCC_d->APB2ENR = RCC_APB2ENR_IOPBEN_;
		else if (GPIOx == GPIOC_d) 	RCC_d->APB2ENR = RCC_APB2ENR_IOPCEN_;
		else if (GPIOx == GPIOD_d) 	RCC_d->APB2ENR = RCC_APB2ENR_IOPDEN_;
		else if (GPIOx == GPIOE_d) 	RCC_d->APB2ENR = RCC_APB2ENR_IOPEEN_;
	}
	// da bat trc do thi chi co the tat
	else if((!enable) && (status)){
		if			(GPIOx == GPIOA_d) 	RCC_d->APB2ENR &= ~(uint32_t)(1 << 2);
		else if (GPIOx == GPIOB_d) 	RCC_d->APB2ENR &= ~(uint32_t)(1 << 3);
		else if (GPIOx == GPIOC_d) 	RCC_d->APB2ENR &= ~(uint32_t)(1 << 4);
		else if (GPIOx == GPIOD_d) 	RCC_d->APB2ENR &= ~(uint32_t)(1 << 5);
		else if (GPIOx == GPIOE_d) 	RCC_d->APB2ENR &= ~(uint32_t)(1 << 6);
	}
}




// Ham cau hinh GPIO
/**
  \fn          GPIOx_config(GPIO_TypeDef_t *GPIOx, 
																	uint32_t numPin, 
																	uint32_t mode)
  \brief       Configure port pin
  \param[in]   GPIOx         Pointer to GPIO peripheral
  \param[in]   numPin        Port pin number
  \param[in]   mode          \ref modeInput										0x8
																	modeOutput2Mhz							0x2
																	modeOutput10Mhz							0x1
																	modeOutput50Mhz							0x3
																	resetMode										0xf
*/
void GPIOx_config(GPIO_TypeDef_t *GPIOx, uint32_t numPin, uint32_t mode){
	uint32_t status = statusClockPort_config(GPIOx);
	if (status == 0) return;
	
	// CRL: 0->7,  CRH: 0->7
	volatile uint32_t *reg ;
	uint16_t numPinConver;
	if(numPin < 8){
		reg = (volatile uint32_t *)&GPIOx->CRL;
		numPinConver = 	(uint16_t)numPin;
	}
	else{
		reg = (volatile uint32_t *)&GPIOx->CRH;
		numPinConver = 	(uint16_t)numPin - 8;
	}
	
	// chon che do
	*reg &= ~(uint32_t)(resetMode << (numPinConver*4));
	if(mode == modeInput){
		*reg |= (uint32_t)(modeInput << (numPinConver*4));
	}
	else if(mode == modeOutput2Mhz){
		*reg |= (uint32_t)(modeOutput2Mhz << (numPinConver*4));
	}
	else if(mode == modeOutput10Mhz){
		*reg |= (uint32_t)(modeOutput10Mhz << (numPinConver*4));
	}
	else if(mode == modeOutput50Mhz){
		*reg |= (uint32_t)(modeOutput50Mhz << (numPinConver*4));
	}
	
}


/**
  \fn          GPIOx_write( GPIO_TypeDef_t *GPIOx, 
																	uint16_t numPin, 
																	uint32_t mode)
  \brief       Set output signal
  \param[in]   GPIOx         Pointer to GPIO peripheral
  \param[in]   numPin        Port pin number
  \param[in]   mode          \ref 1: HIGH
																	0: LOW
*/
void GPIOx_write(GPIO_TypeDef_t *GPIOx, uint16_t numPin, uint32_t mode){
	volatile uint32_t *reg;
	reg = &GPIOx->ODR;
	*reg = ((mode != 0) ? (*reg |= (1 << numPin)) : (*reg &= ~(uint32_t)(1 << numPin)));
	
}

