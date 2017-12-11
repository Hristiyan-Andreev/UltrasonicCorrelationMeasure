#include "stm32l1xx.h"
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_dma.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx_usart.h"

#include "hGPIO_LIB.h"

//********************************** Define user constants ************************************
#define Ain1 GPIO_Pin_1						// Pin and Channel for transmitter signal
#define Ain1Channel ADC_Channel_1
#define Ain2 GPIO_Pin_4						// Pin and channel for receiver signal
#define Ain2Channel ADC_Channel_2
#define ADCPort GPIOA
#define ADCBufferSize 500

#define TriggerMeasurePin GPIO_Pin_10
#define TriggerMeasurePort GPIOA


char SendStartChar = 'S';
char SendEndChar = 'E';

typedef enum STATE {ReadyToMeasure, MeasureStarted, MeasureEnded, SendingStarted, SendingEnded} STATE;
STATE State;

typedef enum USARTSTATE{Idle, SendBufferStart, SendByte1, SendByte2, SendBufferComplete} USARTSTATE;
USARTSTATE USARTState;

static uint32_t ADCDataBuffer[ADCBufferSize];		// Array to store data from ADC Transmit signal
static uint32_t ADCDataBuffer2[ADCBufferSize];		// Array to store data from ADC Received signal
uint32_t GlobalInterruptsDisabled;		// 0 if they are enabled, 1 if they are disabled

void initDMA(void)
{
	DMA_InitTypeDef DMASetup;				// Struct for DMA config
	/*------------------------ DMA1 configuration ------------------------------*/

	/* DMA1 channel1 configuration - transmitted signal*/
	DMA_DeInit(DMA1_Channel1);								// Reset all registers to default values
	DMA_DeInit(DMA1_Channel2);

	DMASetup.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;				// Read from which register
	DMASetup.DMA_MemoryBaseAddr = (uint32_t)&ADCDataBuffer;  // Write to which address
	DMASetup.DMA_DIR = DMA_DIR_PeripheralSRC;					// Read from peripheral write to memory
	DMASetup.DMA_BufferSize = ADCBufferSize;					// How many transfers from register to memory
	DMASetup.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// Don`t increment the peripheral address
	DMASetup.DMA_MemoryInc = DMA_MemoryInc_Enable;			// Increment the memory address Buffer[i] -> Buffer[i+1]

	DMASetup.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// 16 bits
	DMASetup.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;			// 32 bits
	DMASetup.DMA_Mode = DMA_Mode_Normal;						// When all conversions are done, stop
	DMASetup.DMA_Priority = DMA_Priority_High;				// High priority of the channel
	DMASetup.DMA_M2M = DMA_M2M_Disable;						// Disable memory to memory transfer

	DMA_Init(DMA1_Channel1, &DMASetup);
	DMA_SetCurrDataCounter(DMA1_Channel1, ADCBufferSize);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* DMA1 Channel 2 Configuration - to transfer data from ADC channel 2 - received signal*/
	DMASetup.DMA_MemoryBaseAddr = (uint32_t)&ADCDataBuffer2; //Write to second buffer
	DMA_Init(DMA1_Channel2, &DMASetup);
	DMA_SetCurrDataCounter(DMA1_Channel2, ADCBufferSize);

	/* Enable DMA1 channel2 */
	DMA_Cmd(DMA1_Channel2, ENABLE);
}

void initADC1(void)
{
	GPIO_InitTypeDef GPIOPortA;
	ADC_InitTypeDef ADCInit;				// Struct for ADC Config
	ADC_CommonInitTypeDef ADCSetup;			// Struct for ADC prescaler clock value
//	GPIO_StructInit(&GPIOPortA);	// Fill the variable with default settings
//	GPIOPortA.GPIO_Pin = Ain1;
//	GPIOPortA.GPIO_Mode = GPIO_Mode_AN;
//	GPIOPortA.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(ADCPort, &GPIOPortA);

	ConfAnalogIn(ADCPort, Ain1);

	ADC_DeInit(ADC1);    // Reset all ADC settings to default

	ADCInit.ADC_Resolution = ADC_Resolution_8b; // Select resolution
	ADCInit.ADC_ScanConvMode = ENABLE;			// Enable Scan mode - scan multiple input channels
	ADCInit.ADC_ContinuousConvMode = ENABLE;   // Enable continious mode -> measure many times same channel
	ADCInit.ADC_DataAlign = ADC_DataAlign_Right; // Align the data to the right
	ADCInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Don't wait for edge to convert
	ADCInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; // Trigger from Timer2
	ADC_Init(ADC1, &ADCInit);					// Initialize the ADC Init struct for ADC1

	ADCSetup.ADC_Prescaler = ADC_Prescaler_Div1; //Divide the HSI by 1 -> 16 Mhz (1 ADC cycle = )
	ADC_CommonInit(&ADCSetup);

	/* Enable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	  /* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);						// Enable the ADC1


	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);  // Wait until ADC1 is ON -> ADC Flag ADC on
}
void initUSART(void)
{
	GPIO_InitTypeDef GPIOSetup;
	USART_InitTypeDef USARTSetup;
// ************** Configure USART2 Tx (PA.02) and USART Rx (PA.3) as alternate function push-pull ****
	GPIOSetup.GPIO_Pin = GPIO_Pin_2;
	GPIOSetup.GPIO_Speed = GPIO_Speed_2MHz;
	GPIOSetup.GPIO_Mode = GPIO_Mode_AF;
	GPIOSetup.GPIO_OType = GPIO_OType_OD;
	GPIOSetup.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIOSetup);

// ***************************** Map USART2 to PA.2 and PA.3 ******************************
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

// ***************************** Initialize USART2 *****************************************
	USARTSetup.USART_BaudRate = 38400;
	USARTSetup.USART_WordLength = USART_WordLength_8b;
	USARTSetup.USART_StopBits = USART_StopBits_1;
	USARTSetup.USART_Parity = USART_Parity_No;
	USARTSetup.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTSetup.USART_Mode = USART_Mode_Tx;
	USART_Init(USART2, &USARTSetup); // Configure USART
	USART_Cmd(USART2, ENABLE); // Enable the USART

// **************************** Configure interrupts ***************************************
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}
void initTriggerMeasure(void)
{
//***************************** Init trigger pin ************************************************
//	GPIO_StructInit(&GPIOPortA);		 	  // Fill the variable with default settings
//	GPIOPortA.GPIO_Pin = TriggerMeasurePin;   // Specify pin
//	GPIOPortA.GPIO_Mode = GPIO_Mode_OUT;      //Config output mode
//	GPIOPortA.GPIO_OType = GPIO_OType_PP;	  //Config Push-Pull mode
//	GPIOPortA.GPIO_PuPd = GPIO_PuPd_DOWN;	  // Pull down resistor
//	GPIOPortA.GPIO_Speed = GPIO_Speed_2MHz;   // Low speed
//	GPIO_Init(TriggerMeasurePort, &GPIOPortA);			// Initialize Port A with the settings saved in the structure variable

	ConfDigitalOut(TriggerMeasurePort, TriggerMeasurePin, OutputPP, PullDown, HighSpeed);

//***************************** Init pin for debugging ******************************************
//	GPIO_StructInit(&GPIOPortA);		 	  // Fill the variable with default settings
//	GPIOPortA.GPIO_Pin = GPIO_Pin_8;   			// Specify LED2, PA.5
//	GPIOPortA.GPIO_Mode = GPIO_Mode_OUT;      //Config output mode
//	GPIOPortA.GPIO_OType = GPIO_OType_PP;	  //Config Push-Pull mode
//	GPIOPortA.GPIO_PuPd = GPIO_PuPd_DOWN;	  // Pull down resistor
//	GPIOPortA.GPIO_Speed = GPIO_Speed_2MHz;   // Low speed
//	GPIO_Init(GPIOA, &GPIOPortA);			// Initialize Port A with the settings saved in the structure variable

	ConfDigitalOut(GPIOA, GPIO_Pin_8, OutputPP, PullDown, HighSpeed);


//**************************** Init button for starting measure
//	GPIO_StructInit(&GPIOPortA);
//	GPIOPortA.GPIO_Pin = USER_BUTTON_PIN;   			// Specify LED2, PA.5
//	GPIOPortA.GPIO_Mode = GPIO_Mode_IN;      //Config output mode
//	GPIOPortA.GPIO_OType = GPIO_OType_PP;	  //Config Push-Pull mode
//	GPIOPortA.GPIO_PuPd = GPIO_PuPd_DOWN;	  // Pull down resistor
//	GPIOPortA.GPIO_Speed = GPIO_Speed_2MHz;   // Low speed
//	GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIOPortA);

	ConfDigitalIn(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, PullDown);
}

void initTimers(void)
{
	TIM_TimeBaseInitTypeDef TimeBaseSetup; 	// Struct for TimeBase setup
	TIM_ICInitTypeDef InputCaptureSetup;	// Struct for input capture setup
	GPIO_InitTypeDef GPIOPortA;  			// Struct for ADC1 GPIOA Config
// *************************** Set up Parameters ***************************************
// *************************** Set up timer triggering *********************************
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular); // Only underflow/overflow can generate update interrupt
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update); // TRGO event only from update event

	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);	// Select OnePulse mode ->(Counter stops counting at the next update event

//	TIM_ETRConfig(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0); // Timer2, no division, positive polarity, no filtering
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED); // Timer2, select TI1 source as TRGI
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Trigger); // Timer2, configure in trigger mode (Start counting when TRGI is rising edge)

// *************************** Set up time base (ticks) ********************************
	TimeBaseSetup.TIM_Prescaler = 32;					// Divide system core freq 32 times 32Mhz -> 1Mhz
	TimeBaseSetup.TIM_ClockDivision = TIM_CKD_DIV1;		// No further division of the freq
	TimeBaseSetup.TIM_CounterMode = TIM_CounterMode_Down; // Counting up
	TimeBaseSetup.TIM_Period = 10000;		// 10 000 us = 10ms measuring time
	TIM_TimeBaseInit(TIM3, &TimeBaseSetup);

// *************************** Set up input capture *************************************
	InputCaptureSetup.TIM_Channel = TIM_Channel_1;
	InputCaptureSetup.TIM_ICPrescaler = TIM_CKD_DIV1;
	InputCaptureSetup.TIM_ICFilter = 2;
	InputCaptureSetup.TIM_ICPolarity = TIM_ICPolarity_Rising;
	InputCaptureSetup.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &InputCaptureSetup);

	GPIOPortA.GPIO_Pin = GPIO_Pin_6;
	GPIOPortA.GPIO_Mode = GPIO_Mode_AF;
	GPIOPortA.GPIO_Speed = GPIO_Speed_2MHz;
	GPIOPortA.GPIO_OType = GPIO_OType_PP;
	GPIOPortA.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIOPortA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);

// Configure interrupt on reaching target cycles (10ms)
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void initNVIC(void)
{
	NVIC_InitTypeDef NVICSetup;				// Struct for NVIC config
// ****************** Setup the Timer count complete interrupt *****************************
	NVICSetup.NVIC_IRQChannel = TIM3_IRQn; 		// Enable TIM3 Interrupt
	NVICSetup.NVIC_IRQChannelPreemptionPriority = 5;
	NVICSetup.NVIC_IRQChannelSubPriority = 5;
	NVICSetup.NVIC_IRQChannelCmd = ENABLE;			// Enable global interrupts
	NVIC_Init(&NVICSetup);
}

void Init(void)
{
	DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);  // Allow debugging during low power mode

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);    // Enable clock for DMA1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable clock for ADC 1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // Enable clock for GPIOA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // Enable clock for GPIOA
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Enable clock for Timer9
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

// ********************** Configure SysTick Timer ******************************
	if (SysTick_Config(SystemCoreClock / 1000))	{ while(1);	}

// ********************** Configure interrupt controller ****************************
	initNVIC();
// *********************** Configure DMA ***************************************
	initDMA();
// *********************** Configure USART2 ************************************
	initADC1();
// ********************** Configure Trigger for start of measure******************************
	initTriggerMeasure();

// ********************** Configure timer system *************************************
	initTimers();

// ********************** Configure USART******************************
	initUSART();
}

void SendChar(char Data)
{
	USART_SendData(USART2, (uint16_t)Data);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void SendBufferUSART(uint32_t* Buffer)
{
	uint16_t CurrentDigit = 0;

	if((State == SendingStarted) && (USARTState == SendBufferStart))
	{
		SendChar(SendStartChar);
		USARTState = SendByte1;
	}

	while (CurrentDigit <= ADCBufferSize)
	{
		switch(USARTState)
		{
		case SendByte1:
			SendChar(Buffer[CurrentDigit] & 0x00FF);				// Send lower byte
			USARTState = SendByte2;
			/* no break */
		case SendByte2:
			SendChar(Buffer[CurrentDigit] & 0xFF00);				// Send upper byte
			CurrentDigit++;
			USARTState = SendByte1;
			break;
		}
	}
	USARTState = SendBufferComplete;

	if((State == SendingStarted) && (USARTState == SendBufferComplete))
	{
		SendChar(SendEndChar);
		USARTState = Idle;
		State = SendingEnded;
	}
}
static __IO uint32_t TimingDelay;   // Decrementing value for delay

void Delay_ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}


void ClearBuffer (uint32_t Buffer[])
{
	int index = 0;
	uint16_t SizeOfBuffer = sizeof(Buffer)/sizeof(Buffer[0]);
	for(index = 0; index<= sizeof SizeOfBuffer; index++)
	{
		Buffer[index] = 0;
	}
}

void StopADCMeasure(void)
{
	ADC_Cmd(ADC1, DISABLE);						// Disable the ADC1
	ADC1->CR2 &= (uint32_t)(~ADC_CR2_SWSTART);
	State = MeasureEnded;
}

void ResetADC1(void)
{
	ADC_ContinuousModeCmd(ADC1, DISABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == SET);  // Wait untill ADC1 is shutdown

	ADC_Cmd(ADC1, ENABLE);						// Enable the ADC1
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);  // Wait untill ADC1 is ON -> ADC Flag ADC on
	ADC_ContinuousModeCmd(ADC1, ENABLE);
}

void ResetDMA(void)
{
	DMA_Cmd(DMA1_Channel1, DISABLE);
	Delay_ms(1);
	DMA_SetCurrDataCounter(DMA1_Channel1, ADCBufferSize);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}
void TriggerMeasure()
{
	GPIO_SetBits(TriggerMeasurePort, TriggerMeasurePin);
	ADC_SoftwareStartConv(ADC1);
	State = MeasureStarted;
	Delay_ms(1);
	GPIO_ResetBits(TriggerMeasurePort, TriggerMeasurePin);
}

void CheckAndDisableInterrupts(void) { GlobalInterruptsDisabled = __get_PRIMASK();__disable_irq(); }
void CheckAndEnableInterrupts(void) { if(!GlobalInterruptsDisabled) __enable_irq(); }
int main(void)
{
//	CheckAndDisableInterrupts();
	uint32_t readingCycles = 0;
	Init();
	ADC_RegularChannelConfig(ADC1, Ain1Channel , 1, ADC_SampleTime_4Cycles); //Configure the channel (PA.1, to be read)
//	CheckAndEnableInterrupts();
	while(1)
	{
		State = ReadyToMeasure;
		while(GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == Bit_SET );
		TriggerMeasure();
		Delay_ms(50); // while(State != MeasureEnded)
		if (State == MeasureEnded)
		{
			ResetADC1();
			ResetDMA();
			State = SendingStarted;
			USARTState = SendBufferStart;
			SendBufferUSART(ADCDataBuffer);
			ClearBuffer(ADCDataBuffer);
			readingCycles++;
		}

		Delay_ms(1000);
	}

}

void SysTick_Handler(void)
{
	if(TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}


void TIM3_IRQHandler (void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  // If counter update event had occurred
	{
		StopADCMeasure();
		GPIO_SetBits(GPIOA, GPIO_Pin_8);				// For Debugging
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);			// Clear the TIM3 Update event flag
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);		// Clear the TIM3 Update event IT flag -> same as top
	}
}
