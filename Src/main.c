/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lwip.h"
#include "usb_device.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "globaldefine.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SPI3_WriteByte(uint8_t byte);
void HandleCmd(void);
void Spi2WriteToFPGA( uint8_t *pData, uint16_t Size);
void Spi2ReadFromFPGA(uint8_t *pData, uint16_t Size);

void MX_SPI3_Deinit(void);




/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned long int g_timer = 0, g_timer_last = 0;

extern uint8_t RxIndex,TxIndex;//收发计数器 < COMM_BUFFER_NUM
extern uint32_t HandleRxIndex,HandleTxIndex;
extern COMM_FRAM	RxBufferFS[COMM_BUFFER_NUM];
extern COMM_FRAM	TxBufferFS[COMM_BUFFER_NUM];



uint8_t pData[COMM_BUFFER_NUM];
uint16_t Size;
uint8_t ReceiveData[4096];
int WriteIndex;





void USB_sendbyte(uint8_t byte)
{
	CDC_Transmit_FS((uint8_t *)(&byte),1);
}



/*jtag functions by cyh 2017/9/7*/

void JTAG_SetPins(GPIO_PinState tdi, GPIO_PinState tms)
{
	HAL_GPIO_WritePin(JTAG_TDI_GPIO_Port,JTAG_TDI_Pin,tdi);
	HAL_GPIO_WritePin(JTAG_TMS_GPIO_Port,JTAG_TMS_Pin,tms);
	HAL_GPIO_WritePin(JTAG_TCK_GPIO_Port,JTAG_TCK_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(JTAG_TCK_GPIO_Port,JTAG_TCK_Pin,GPIO_PIN_SET);
}


void JTAG_TLR(void)
{
	int i;
	for(i=0;i<6;++i)
		JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_SET);
	return;
}

void JTAG_RTI(int times)
{
	int i;
	for(i=0;i<times;++i)
		JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
	return;
}

void JTAG_IR(uint8_t *instr)
{
	int i;
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_SET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_SET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
	for(i=0;i<5;++i)
		JTAG_SetPins(instr[i],GPIO_PIN_RESET);
	JTAG_SetPins(instr[5],GPIO_PIN_SET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_SET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
}

void JTAG_EnterDR(void)
{
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_SET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
}

void JTAG_Transmit(uint8_t *data,int size)
{
	int i,j;
	uint8_t v,x;
	for(i=0;i<size;++i)
	{
		v = 0x80;
		x = data[i];
		for(j=0;j<8;++j)
		{
			JTAG_SetPins(x/v, GPIO_PIN_RESET);
			x%=v;
			v>>=1;
		}
	}
}

void JTAG_LastByte(uint8_t ch)
{
	int i,j;
	uint8_t v;
	v = 0x80;
	for(j=0;j<7;++j)
	{
		JTAG_SetPins(ch/v,GPIO_PIN_RESET);
		ch%=v;
		v>>=1;
	}
	JTAG_SetPins(ch/v,GPIO_PIN_SET);
}

void JTAG_ExitDR()
{
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_SET);
	JTAG_SetPins(GPIO_PIN_SET,GPIO_PIN_RESET);
}

void FPGA_JTAG_StartConfig()
{
	uint8_t JPROGRAM[6] = {1,1,0,1,0,0};
	uint8_t CFG_IN[6] = {1,0,1,0,0,0};
	JTAG_RTI(6);
	JTAG_IR(JPROGRAM);
	JTAG_RTI(10000);
	JTAG_IR(CFG_IN);
	JTAG_EnterDR();
}

void FPGA_JTAG_FinishConfig()
{
	uint8_t JSTART[6] = {0,0,1,1,0,0};
	JTAG_IR(JSTART);
	JTAG_RTI(2000);
	JTAG_TLR();
}

void FPGA_Jstart()
{
	uint8_t JSTART[6] = {0,0,1,1,0,0};
	JTAG_RTI(10);
	JTAG_IR(JSTART);	
}

void FPGA_Jshutdown()
{
	uint8_t JSHUTDOWN[6] = {1,0,1,1,0,0};
	JTAG_RTI(200);
	JTAG_IR(JSHUTDOWN);	
	JTAG_RTI(200);
	JTAG_IR(JSHUTDOWN);	
}


/*spiflash functions by cyh 2017/7/29*/




void SPI3_WriteByte(uint8_t byte)
{
	HAL_SPI_Transmit(&hspi3, &byte, 1, 5);//Timeout
}


void ReadJEDECID(uint8_t *pData)//test passed
{
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x9f);
	HAL_SPI_Receive(&hspi3, pData, 3, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}

void ReadStatusReg(uint8_t *pData)//test passed
{
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x05);
	HAL_SPI_Receive(&hspi3, pData, 1, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}



void WriteEnable(void)//test passed
{
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x06);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}

void WriteDisable(void)
{
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x04);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}

void Erase4KSector(uint32_t addr)
{
	uint8_t reg1 = 1;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x20);
	SPI3_WriteByte((addr& 0xFF0000) >> 16);
	SPI3_WriteByte((addr& 0xFF00) >> 8);
	SPI3_WriteByte((addr& 0xFF));
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x05);
	while(reg1&1)
		HAL_SPI_Receive(&hspi3, &reg1, 1, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}


void ChipErase(void)//test passed
{
	uint8_t reg1 = 1;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0xc7);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x05);
	while(reg1&1)
		HAL_SPI_Receive(&hspi3, &reg1, 1, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}

void FastRead(uint32_t addr,int size)//test passed
{
	uint8_t ReadFlash[4096];
	uint8_t* buf=ReadFlash;
	int i;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x0b);
	SPI3_WriteByte((addr& 0xFF0000) >> 16);
	SPI3_WriteByte((addr& 0xFF00) >> 8);
	SPI3_WriteByte((addr& 0xFF));
	SPI3_WriteByte(0x00);
	for(i=0;i<size/512;++i)
		HAL_SPI_Receive(&hspi3, buf+i*512, 512, 5);
	HAL_SPI_Receive(&hspi3, buf+i*512, size%512, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	for(i=0;i<4096;++i)
		if(ReadFlash[i]!=ReceiveData[i])
			continue;
}

void Erase64KSector(uint32_t addr)//test passed
{
	uint8_t reg1 = 1;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0xd8);
	SPI3_WriteByte((addr& 0xFF0000) >> 16);
	SPI3_WriteByte((addr& 0xFF00) >> 8);
	SPI3_WriteByte((addr& 0xFF));
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x05);
	while(reg1&1)
		HAL_SPI_Receive(&hspi3, &reg1, 1, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}


void PageProgram(uint8_t* data,uint32_t addr,uint16_t size)//test passed
{
	uint8_t reg1;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x05);
		HAL_SPI_Receive(&hspi3, &reg1, 1, 5);
	while(!(reg1&2))
		HAL_SPI_Receive(&hspi3, &reg1, 1, 5);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x02);
	SPI3_WriteByte((addr& 0xFF0000) >> 16);
	SPI3_WriteByte((addr& 0xFF00) >> 8);
	SPI3_WriteByte((addr& 0xFF));
	HAL_SPI_Transmit(&hspi3,data,size,50);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	SPI3_WriteByte(0x70);
	while(1)
	{
		HAL_SPI_Receive(&hspi3, &reg1, 1, 5);
		if(reg1&0x80)
				break;
	}	
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}
void ClearConfiguation(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_RESET);
}


void StartConfiguation(void)
{
	int i;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_RESET);
	for(i=0;i<1000;++i);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_SET);
}
void CleanSpiFlashSectors(void)
{
	
	int i;
	uint32_t addr;
	WriteIndex = 0;
	addr = 0;
	for(i=0;i<64;++i)
	{
		WriteEnable();
		Erase64KSector(addr);
		addr += 65536;
		USB_sendbyte(i);
	}
}
void WriteSpiFlash(int ad)
{

	int i,j;
	int a;
	for(i=0;i<16;++i)
	{
		a = i*256;
		WriteEnable();
		PageProgram(ReceiveData+a,a+4096*ad,256);
	}
	j = 0;
return;
}

void EraseBulk()
{
	WriteEnable();
	ChipErase();
}
/*spiflash functions by cyh 2017/7/29*/

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LWIP_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 2 */
	//CDC_Transmit_FS((uint8_t*)st,20);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	MX_SPI3_Deinit();
	//for(i=0;i<10000;++i);
	StartConfiguation();
	//for(i=0;i<10000;++i);
	MX_SPI3_Init();
	
	MX_SPI3_Deinit();
	//for(i=0;i<10000;++i);
	StartConfiguation();
	//for(i=0;i<10000;++i);
	MX_SPI3_Init();
	
	
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HandleCmd();
		g_timer = HAL_GetTick();
		if(g_timer >= g_timer_last  + 500)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			g_timer_last = g_timer;
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the Alarm B 
    */
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void MX_SPI3_Deinit(void)
{
	HAL_SPI_DeInit(&hspi3);
}


/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

	
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PHY_RESET_GPIO_Port, PHY_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PHY_RESET_Pin */
  GPIO_InitStruct.Pin = PHY_RESET_Pin|SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_SET);	
  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	/*config gpioe*/
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5|GPIO_PIN_1, GPIO_PIN_RESET);
	/*gpiob*/
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9|GPIO_PIN_8, GPIO_PIN_RESET);	
}

/* USER CODE BEGIN 4 */
void Spi2WriteToFPGA( uint8_t *pData, uint16_t Size)//, uint32_t Timeout)
{
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, pData, Size, 5);//Timeout);
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
}

void Spi2ReadFromFPGA(uint8_t *pData, uint16_t Size)//, uint32_t Timeout)
{
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);//PA5
  HAL_SPI_Receive(&hspi2, pData, Size,  5);//Timeout);
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);
}



void HandleCmd(void)
{
	uint8_t index=0;
	uint32_t tmp = 0;
	const char respond[] = "successfully completed\n\r";
	if(HandleRxIndex > HandleTxIndex)//有新命令
	{
		index=HandleTxIndex%COMM_BUFFER_NUM;	
		//memcpy((uint8_t *)(&TxBufferFS[0]),(uint8_t *)(&RxBufferFS[0]),sizeof(COMM_FRAM));//copy命令到发送缓冲区
		switch(RxBufferFS[0].cmd)
		{
			case 0x03:
				FastRead(RxBufferFS[0].id*4096,4096);
				break;
			case 0x05:
				ClearConfiguation();
				break;
			case 0x06:
				CleanSpiFlashSectors();
				break;			
			case 0x07:
				WriteSpiFlash(WriteIndex);
				WriteIndex++;
				USB_sendbyte(11);
				break;			
			case 0x08:
				StartConfiguation();
				break;
			case 0x0b:
				MX_SPI3_Deinit();
				break;
			case 0x0e:
				MX_SPI3_Init();
				break;
			case 0x16:
				JTAG_Transmit(RxBufferFS[0].Bbuf,RxBufferFS[0].id);
				USB_sendbyte(30);
				break;
			case 0x19:
				JTAG_LastByte(RxBufferFS[0].id);
				USB_sendbyte(32);
				break;
			case 0x1c:
				FPGA_JTAG_StartConfig();
				USB_sendbyte(60);
				break;	
			case 0x1e:
				FPGA_JTAG_FinishConfig();
				break;
			case 0x21:
				FPGA_Jstart();
				break;
			case 0x22:
				FPGA_Jshutdown();
				break;
			case 0x89:
				EraseBulk();
				break;
			case 0xa2:
				USB_sendbyte(17);
				break;
			case 0xab:
				memcpy(ReceiveData+RxBufferFS[0].adr,RxBufferFS[0].Bbuf,32);
				USB_sendbyte(20);
				break;		
			default:
				break;
		}

		/*}
		else if(0x80 == RxBufferFS[index].cmd)//读ARM寄存器
		{
			CDC_Transmit_FS((uint8_t *)(&TxBufferFS[0]),RxBufferFS[index].len+5);
		}*/
		/*else if(0x01 == RxBufferFS[index].cmd)//写FPGA寄存器
		{
			Spi2WriteToFPGA( pData, Size);
			//spi转发地址和数据部分
		}
		else if(0x81 == RxBufferFS[index].cmd)//读FPGA寄存器
		{
			Spi2ReadFromFPGA( pData, Size);
			//spi转发地址，并回读数据
		}*/
		HandleTxIndex++;
	}
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
