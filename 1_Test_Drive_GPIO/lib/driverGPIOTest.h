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

#ifndef DRIVER_GPIO_TEST_H_
#define DRIVER_GPIO_TEST_H_

#include "stm32f10x.h"                  // Device header
#include <stdint.h>
#include <stdio.h>

/************8******************  RCC  ****************************************/
// thu tu phai sap xep dung voi Referenct
// Struct thanh ghi RCC
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR; 
} RCC_TypeDef_t;

//Dia chi base RCC
#define RCC_BASE_								((uint32_t)0x40021000)
#define RCC_d										((RCC_TypeDef_t *)RCC_BASE_)

/******************  Bit definition for RCC_APB2ENR register  *****************/
//APB2 peripheral clock enable register
#define RCC_APB2ENR_AFIOEN_						(RCC_d->APB2ENR |= (1 << 0))
#define RCC_APB2ENR_IOPAEN_						(RCC_d->APB2ENR |= (1 << 2))
#define RCC_APB2ENR_IOPBEN_						(RCC_d->APB2ENR |= (1 << 3))
#define RCC_APB2ENR_IOPCEN_						(RCC_d->APB2ENR |= (1 << 4))
#define RCC_APB2ENR_IOPDEN_						(RCC_d->APB2ENR |= (1 << 5))
#define RCC_APB2ENR_IOPEEN_						(RCC_d->APB2ENR |= (1 << 6))
#define RCC_APB2ENR_ADC1EN_						(RCC_d->APB2ENR |= (1 << 9))
#define RCC_APB2ENR_ADC2EN_						(RCC_d->APB2ENR |= (1 << 10))
#define RCC_APB2ENR_TIM1EN_						(RCC_d->APB2ENR |= (1 << 11))
#define RCC_APB2ENR_SPI1EN_						(RCC_d->APB2ENR |= (1 << 12))
#define RCC_APB2ENR_USART1EN_					(RCC_d->APB2ENR |= (1 << 14))



/******************************  GPIO ****************************************/
//Thanh ghi CRL
typedef struct{
	volatile uint32_t MODE0		: 4;				// pin0 : CNF0[1:0](2 bit)   và    MODE0[1:0](2 bit)  
	volatile uint32_t MODE1		: 4;				// pin1 : CNF1[1:0](2 bit)   và    MODE1[1:0](2 bit)  
	volatile uint32_t MODE2		: 4;				// pin2 : CNF2[1:0](2 bit)   và    MODE2[1:0](2 bit)  
	volatile uint32_t MODE3		: 4;				// pin3 : CNF3[1:0](2 bit)   và    MODE3[1:0](2 bit)  
	volatile uint32_t MODE4		: 4;				// pin4 : CNF4[1:0](2 bit)   và    MODE4[1:0](2 bit)  
	volatile uint32_t MODE5		: 4;				// pin5 : CNF5[1:0](2 bit)   và    MODE5[1:0](2 bit)  
	volatile uint32_t MODE6		: 4;				// pin6 : CNF6[1:0](2 bit)   và    MODE6[1:0](2 bit)  
	volatile uint32_t MODE7		: 4;				// pin7 : CNF7[1:0](2 bit)   và    MODE7[1:0](2 bit)  
} CRL_TypeDef_t;

//Thanh ghi CRH
typedef struct{
	volatile uint32_t MODE8		  : 4;			// pin8 :  CNF8[1:0](2 bit)    và    MODE8[1:0](2 bit)  
	volatile uint32_t MODE9		  : 4;			// pin9 :  CNF9[1:0](2 bit)    và    MODE9[1:0](2 bit)  
	volatile uint32_t MODE10		: 4;			// pin10 : CNF10[1:0](2 bit)   và    MODE10[1:0](2 bit)  
	volatile uint32_t MODE11		: 4;			// pin11 : CNF11[1:0](2 bit)   và    MODE11[1:0](2 bit)  
	volatile uint32_t MODE12		: 4;			// pin12 : CNF12[1:0](2 bit)   và    MODE12[1:0](2 bit)  
	volatile uint32_t MODE13		: 4;			// pin13 : CNF13[1:0](2 bit)   và    MODE13[1:0](2 bit)  
	volatile uint32_t MODE14		: 4;			// pin14 : CNF14[1:0](2 bit)   và    MODE14[1:0](2 bit)  
	volatile uint32_t MODE15		: 4;			// pin15 : CNF15[1:0](2 bit)   và    MODE15[1:0](2 bit)  
} CRH_TypeDef_t;


// Struct thanh ghi GPIO
typedef struct
{
  CRL_TypeDef_t CRL;
  CRH_TypeDef_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef_t;

// Dia chi base APB2
#define APB2_BASE_							((uint32_t)(0x40010000))
/******************************* Dia chi base cac GPIO ****************************************/
//Dia chi GPIO port A			-				Register				   -            offset
#define GPIOA_BASE_							((uint32_t)(APB2_BASE_	       +  0x800))
#define GPIOA_d									((GPIO_TypeDef_t*)(GPIOA_BASE_))
	
//Dia chi GPIO port B			-				Register				   -            offset
#define GPIOB_BASE_							((uint32_t)(APB2_BASE_	    +     0xC00))
#define GPIOB_d									((GPIO_TypeDef_t*)(GPIOB_BASE_))

//Dia chi GPIO port C			-				Register				   -            offset
#define GPIOC_BASE_							((uint32_t)(APB2_BASE_	    +     0x1000))
#define GPIOC_d									((GPIO_TypeDef_t*)(GPIOC_BASE_))	

//Dia chi GPIO port D			-				Register				   -            offset
#define GPIOD_BASE_							((uint32_t)(APB2_BASE_	    +     0x1400))
#define GPIOD_d									((GPIO_TypeDef_t*)(GPIOD_BASE_))	
	
//Dia chi GPIO port E			-				Register				   -            offset
#define GPIOE_BASE_							((uint32_t)(APB2_BASE_	 +  0x1800))
#define GPIOE_d									((GPIO_TypeDef_t*)(GPIOC_BASE_))	
	
//Dia chi GPIO port F			-				Register				   -            offset
#define GPIOF_BASE_							((uint32_t)(APB2_BASE_	 +  0x1C00))
#define GPIOF_d									((GPIO_TypeDef_t*)(GPIOF_BASE_))	
	
//Dia chi GPIO port G			-				Register				   -            offset
#define GPIOG_BASE_							((uint32_t)(APB2_BASE_	 +  0x2000))
#define GPIOG_d									((GPIO_TypeDef_t*)(GPIOG_BASE_))	

// Che do pin INPUT hay OUTPUT	-	Register
#define modeInput										0x8
#define modeOutput2Mhz							0x2
#define modeOutput10Mhz							0x1
#define modeOutput50Mhz							0x3
#define resetMode										0xf


// Vi tri pin IDR va ODR 0 - 15		-		register
#define pin0_IO_												0
#define pin1_IO_												1
#define pin2_IO_												2
#define pin3_IO_												3
#define pin4_IO_												4
#define pin5_IO_												5
#define pin6_IO_												6
#define pin7_IO_												7
#define pin8_IO_												8
#define pin9_IO_												9
#define pin10_IO_												10
#define pin11_IO_												11
#define pin12_IO_												12
#define pin13_IO_												13
#define pin14_IO_												14
#define pin15_IO_												15


#define ENA															1
#define DIS_ENA													0

/******************************* Ham su dung ****************************************/
// Ham cau hinh RCC 
/**
  \fn          uint32_t statusClockPort_config(GPIO_TypeDef_t *GPIOx);
  \brief       Kiem tra cau hinh Port Clock
  \param[in]   GPIOx         Pointer to GPIO peripheral
  \return      true  - success
               false - error
*/
uint32_t statusClockPort_config(GPIO_TypeDef_t *GPIOx);

/**
  \fn          clockPort_config(GPIO_TypeDef_t *GPIOx, 
																				uint8_t enable)
  \brief       Port Clock Control
  \param[in]   GPIOx  Pointer to GPIO peripheral
  \param[in]   enable Enable or disable clock
*/
void clockPort_config(GPIO_TypeDef_t *GPIOx, uint8_t enable);

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
void GPIOx_config(GPIO_TypeDef_t *GPIOx, uint32_t numPin, uint32_t mode);
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
void GPIOx_write(GPIO_TypeDef_t *GPIOx, uint16_t numPin, uint32_t mode);



#endif	//DRIVER_GPIO_TEST_H_
