/* 
 * File:   RCC_private.h
 * Author: Mohamed_Nagy
 * https://github.com/Ged0oo 
 * https://www.linkedin.com/in/mohamednagyofficial/
 * Created on February 16, 2023, 8:41 PM
 */


#ifndef RCC_PRIVATE_H
#define RCC_PRIVATE_H

#include "BIT_MATH.h"
#include "STD_TYPES.h"


/* Registers definitions */
#define 		RCC_BASE_ADDRESS        0x40021000
#define 		RCC              		((volatile RCC_t*)RCC_BASE_ADDRESS)


typedef struct
{
	uint32_t CR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t APB2RSTR;
	uint32_t APB1RSTR;
	uint32_t AHBENR;
	uint32_t APB2ENR;
	uint32_t APB1ENR;
	uint32_t BDCR;
	uint32_t CSR;
}RCC_t;



#endif







