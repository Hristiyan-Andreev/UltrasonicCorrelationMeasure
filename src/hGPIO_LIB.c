/*
 * hGPIO_LIB.c
 *
 *  Created on: Dec 3, 2017
 *      Author: Chris
 */

#include "hGPIO_LIB.h"


void ConfAnalogIn(GPIO_TypeDef* PORT, uint16_t PIN)
{
	GPIO_InitTypeDef GPIOInitStruct;

	GPIO_StructInit(&GPIOInitStruct); // Default values - Input, PP, NoPull, VeryLow Speed
	GPIOInitStruct.GPIO_Pin = PIN;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PORT, &GPIOInitStruct);

}

void ConfDigitalOut(GPIO_TypeDef* PORT, uint16_t PIN, GPIOOType_TypeDef PushOrDrain,
										GPIOPuPd_TypeDef PULL, GPIOSpeed_TypeDef SPEED)
{
	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct); // Default values - Input, PP, NoPull, VeryLow Speed

	GPIOInitStruct.GPIO_Pin = PIN;   // Specify pin
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;      //Config in output mode
	GPIOInitStruct.GPIO_OType = PushOrDrain;	  //Config Push-Pull or Open Drain
	GPIOInitStruct.GPIO_PuPd = PULL;	  // Pull UP/DOWN or Floating
	GPIOInitStruct.GPIO_Speed = SPEED;   // Chosen speed
	GPIO_Init(PORT, &GPIOInitStruct);
}

void ConfDigitalIn(GPIO_TypeDef* PORT, uint16_t PIN, GPIOPuPd_TypeDef PULL)
{
	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct); // Default values - Input, PP, NoPull, VeryLow Speed

	GPIOInitStruct.GPIO_Pin = PIN;   			// Specify LED2, PA.5
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN;      //Config output mode
	GPIOInitStruct.GPIO_OType = GPIO_OType_PP;	  //Config Push-Pull mode
	GPIOInitStruct.GPIO_PuPd = PULL;	  // Pull down or up
	GPIO_Init(PORT, &GPIOInitStruct);
}
