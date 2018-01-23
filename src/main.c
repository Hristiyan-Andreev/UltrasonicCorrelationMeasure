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
#define Ain2Channel ADC_Channel_4
#define ADCPort GPIOA

#define ADCBufferSize 500					// Buffer size in samples
#define MeasureTime 5000					// Measure time in us

#define TriggerMeasurePin GPIO_Pin_10
#define TIM3Trig GPIO_Pin_6
#define TriggerMeasurePort GPIOA

char SendStartChar = 'S';
char SendEndChar = 'E';
//********************************* Define Macros ********************************************
#define SIZEARRAY(x)  (sizeof(x) / sizeof((x)[0]))



typedef enum STATE {ReadyToMeasure, MeasureStarted, MeasureEnded, SendingStarted, SendingEnded} STATE;
STATE State;

typedef enum USARTSTATE{Idle, SendBufferStart, SendByte1, SendByte2, SendBufferComplete} USARTSTATE;
USARTSTATE USARTState;

static uint32_t ADCDataBuffer[ADCBufferSize];		// Array to store data from ADC Transmit signal
uint32_t GlobalInterruptsDisabled;		// 0 if they are enabled, 1 if they are disabled

void initDMA(void)
{
	DMA_InitTypeDef DMASetup;				// Struct for DMA config
	/*------------------------ DMA1 configuration ------------------------------*/

	/* DMA1 channel1 configuration - transmitted signal*/
	DMA_DeInit(DMA1_Channel1);								// Reset all registers to default values

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
}

void initADC1(void)
{
	ADC_InitTypeDef ADCInit;				// Struct for ADC Config
	ADC_CommonInitTypeDef ADCSetup;			// Struct for ADC prescaler clock value

	AnalogIn(ADCPort, Ain1);
	AnalogIn(ADCPort, Ain2);

	ADC_DeInit(ADC1);    // Reset all ADC settings to default

	ADCInit.ADC_Resolution = ADC_Resolution_10b; // Select resolution
	ADCInit.ADC_ScanConvMode = ENABLE;			// Enable Scan mode - scan multiple input channels
	ADCInit.ADC_ContinuousConvMode = ENABLE;   // Enable continious mode -> measure many times same channel
	ADCInit.ADC_DataAlign = ADC_DataAlign_Right; // Align the data to the right
	ADCInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Don't wait for edge to convert
	ADCInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; // Trigger from Timer2
	ADCInit.ADC_NbrOfConversion = 2;							// Number of channels to be scanned
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
	USART_InitTypeDef USARTSetup;

	AltFunc1(GPIOA, GPIO_Pin_2, GPIO_AF_USART2);

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
	DigitalOut(TriggerMeasurePort, TriggerMeasurePin, OutputPP, PullUp, HighSpeed);
	GPIO_SetBits(TriggerMeasurePort, TriggerMeasurePin);

//***************************** Init pin for debugging ******************************************
	DigitalOut(GPIOA, GPIO_Pin_8, OutputPP, PullDown, HighSpeed);

//**************************** Init button for starting measure
	DigitalIn(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, PullDown);
}

void initTimers(void)
{
	TIM_TimeBaseInitTypeDef TimeBaseSetup; 	// Struct for TimeBase setup
	TIM_ICInitTypeDef InputCaptureSetup;	// Struct for input capture setup
// *************************** Set up Parameters ***************************************
// *************************** Set up input capture *************************************
	InputCaptureSetup.TIM_Channel = TIM_Channel_1;
	InputCaptureSetup.TIM_ICPrescaler = TIM_CKD_DIV1;
	InputCaptureSetup.TIM_ICFilter = 2;
	InputCaptureSetup.TIM_ICPolarity = TIM_ICPolarity_Falling;
	InputCaptureSetup.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &InputCaptureSetup);

	AltFunc2(GPIOA, TIM3Trig, GPIO_AF_TIM3, OutputPP, PullUp, HighSpeed);
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
	TimeBaseSetup.TIM_Period = MeasureTime;		// 10 000 us = 10ms measuring time
	TIM_TimeBaseInit(TIM3, &TimeBaseSetup);

// Configure interrupt on reaching target cycles
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

// Always use with defined macro - SIZEOFARRAY(x) as a second argument
void ClearBuffer (uint32_t* Buffer, int SizeOfBuffer)
{
	int index = 0;
	for(index = 0; index<= SizeOfBuffer; index++)
	{
		Buffer[index] = 0;
	}
}

void StopADCMeasure(void)
{
	ADC_Cmd(ADC1, DISABLE);						// Disable the ADC1
	ADC1->CR2 &= (uint32_t)(~ADC_CR2_SWSTART);
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
	GPIO_ResetBits(TriggerMeasurePort, TriggerMeasurePin); // Close the analog switch
	ADC_SoftwareStartConv(ADC1);
}

void CheckAndDisableInterrupts(void) { GlobalInterruptsDisabled = __get_PRIMASK();__disable_irq(); }
void CheckAndEnableInterrupts(void) { if(!GlobalInterruptsDisabled) __enable_irq(); }
void ResetMeasure() { ResetADC1(); ResetDMA();}
void SendDataToPC() { State = SendingStarted; USARTState = SendBufferStart; SendBufferUSART(ADCDataBuffer);
													ClearBuffer(ADCDataBuffer, SIZEARRAY(ADCDataBuffer));}
int main(void)
{
//	CheckAndDisableInterrupts();
	uint32_t readingCycles = 0;
	Init();
	ADC_RegularChannelConfig(ADC1, Ain1Channel , 1, ADC_SampleTime_4Cycles); //Configure the channel to be read
	ADC_RegularChannelConfig(ADC1, Ain2Channel , 2, ADC_SampleTime_4Cycles);
//	CheckAndEnableInterrupts();
	while(1)
	{
		State = ReadyToMeasure;
		while(GPIO_ReadInputDataBit(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == Bit_SET );
		TriggerMeasure();
		State = MeasureStarted;
		Delay_ms(50); // while(State != MeasureEnded)
		if (State == MeasureEnded)
		{
			ResetMeasure();
			SendDataToPC();
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
		GPIO_SetBits(TriggerMeasurePort, TriggerMeasurePin); // Open the analog switch
		StopADCMeasure();
		State = MeasureEnded;
		GPIO_SetBits(GPIOA, GPIO_Pin_8);				// For Debugging
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);			// Clear the TIM3 Update event flag
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);		// Clear the TIM3 Update event IT flag -> same as top
	}
}
