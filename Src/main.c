/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 17/06/2015 17:18:39
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "CAN_protocol.h"


//#include "stdlib.h"
//#include "c++/4.8.1/vector"
//#include <c++/4.8.1/bits/stl_vector.h>
//#include "stdio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RXBUFFERSIZE 1000
 #define	sIDLE		0
 #define	sHEAD		1
 #define	sDATA		2
 #define	sEND		3
 #define	sPRECRC		4
 #define	sPOSTCRC	5
 #define	sCHECK		6
 #define	sDONE		7
#define DLEBINR 16
#define ETXBINR 3
#define framedArrSIZE 49
uint16_t DLINA_AT;
char AT_COM[64];
int TxBusy = 0;
char mas[1];

char 	TxNMEA0183[500];			//буфер передатчика
uint8_t		TxCntNMEA0183;				//счетчик передатчика
char 	RxNMEA0183[500];			//буфер передатчика
uint8_t		RxCntNMEA0183;				//счетчик передатчика
bool	reciveNMEA0183;				//флаг принятых новых данных
bool	sendNMEA0183;				//флаг окончания передачи
bool	T7Flag;						//флаг срабатывания таймера №7
char 	TxRS485[500];			//буфер передатчика
uint8_t		TxCntRS485;				//счетчик передатчика
char 	RxRS485[500];			//буфер передатчика
uint8_t		RxCntRS485;				//счетчик передатчика
bool	reciveRS485;				//флаг принятых новых данных
bool	sendRS485;				//флаг окончания передачи
bool	reciveADRRS485;				//флаг совпадения принятого и текущего адреса
uint8_t aRxBuffer[RXBUFFERSIZE];
char 	TxBINR[500];			//буфер передатчика
uint16_t		TxCntBINR;				//счетчик передатчика
char 	RxBINR[2048];			//буфер передатчика
uint16_t		RxCntBINR;				//счетчик передатчика
bool	reciveBINR;				//флаг принятых новых данных
bool	sendBINR;				//флаг окончания передачи
// bool idleCatched = false;
bool	flagnewbinr;
uint16_t 		RxWrIndex;
bool 	RxBinrBuffOverrun;
uint8_t		NumberSatateRxBINR;		//флаг состояния автомата
#ifdef yesCRC
	bool	CRCok;					//флаг совпадения контрольных сумм
	uint8_t		NumberbyteCRC;			//счетчик байт контрольной суммы
	uint8_t		MCRC[5];				//массив байт контрольной суммы
#endif
uint8_t		hE4BINRmas[530];		//массив с принятым пакетом Е4
bool	hE4BINRok;				//флаг притяного массива Е4
uint8_t		h41BINRmas[12];			//массив с принятым пакетом 41
bool	h41BINRok;				//флаг притяного массива 41
uint8_t		h52BINRmas[224];		//массив с принятым пакетом 52
bool	h52BINRok;				//флаг притяного массива 52
uint8_t		h88BINRmas[69];			//массив с принятым пакетом 88
bool	h88BINRok;				//флаг притяного массива 88
int dataArrIndex;
int mesCount =86;
uint8_t modeCANmsg;

unsigned short Serial;//
unsigned char MsgType,Devtype,Priority,MsgMode;
uint8_t framedArrCANTx[framedArrSIZE];
//TIM_Base_InitTypeDef TimHandle;
bool checkMessageFlag;
int idleCounter;
int rxCounter;
bool beginCANTransmitFlag;
int canMessCounter=0;
char inc;
int rpuChannelsQty=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define CAN_SHDN_PORT GPIOB
#define CAN_SHDN_PIN GPIO_PIN_15

#define LED_LEG_PORT GPIOC
#define LED_LEG_PIN GPIO_PIN_0
//#define RESET_CSM_PORT GPIOB
//#define RESET_CSM_PIN GPIO_PIN_7
//
//#define LED_1_PORT GPIOE
//#define LED_1_PIN  GPIO_PIN_2

#define __HAL_CAN_CANCEL_TRANSMIT(__HANDLE__, __TRANSMITMAILBOX__)\
(((__TRANSMITMAILBOX__) == CAN_TXMAILBOX_0)? ((__HANDLE__)->Instance->TSR |= CAN_TSR_ABRQ0) :\
 ((__TRANSMITMAILBOX__) == CAN_TXMAILBOX_1)? ((__HANDLE__)->Instance->TSR |= CAN_TSR_ABRQ1) :\
 ((__HANDLE__)->Instance->TSR |= CAN_TSR_ABRQ2))
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	beginCANTransmitFlag = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
//  HAL_CAN_MspInit(&hcan2);
  inc = 0;
  double tmp = 1.2;
  volatile uint8_t arr[sizeof(tmp)];
  var2ArrConverter((char*)&tmp,sizeof(tmp),arr);
  volatile const uint8_t someval= calcCSofArr(arr,7);
  hcan2.pTxMsg = &TxMessage;
  hcan2.pRxMsg = &RxMessage;
  IMfreqs.MDADFrequency = 20;
  IMfreqs.MDLUFrequency = 300;
  IMfreqs.MDUSFrequency = 180;
  IMfreqs.totalFrequency = 500;
  checkFrequencies();
//  Freq = 500;
  FreqPresc = CoreFreq/(IMfreqs.totalFrequency);
//  FreqMDLU = 240;
//  FreqMDUS = 240;
//  FreqMDAD = 20;
  htim7.Init.Prescaler = FreqPresc;//*0.908;
  htim7.Init.Period = 1;
  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.BankNumber = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0;// ДЛУ
  sFilterConfig.FilterIdLow = 0x000C;
  sFilterConfig.FilterMaskIdHigh = 0x0;
  sFilterConfig.FilterMaskIdLow = 0x0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
//
//

  HAL_CAN_Init(&hcan2);
//  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
//  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
  HAL_TIM_Base_Init(&htim7);
  HAL_TIM_Base_Start_IT(&htim7);
  volatile int clock = HAL_RCC_GetSysClockFreq();
  clock = HAL_RCC_GetPCLK1Freq();
  clock = HAL_RCC_GetHCLKFreq();
  clock = HAL_RCC_GetPCLK2Freq();
  volatile RCC_OscInitTypeDef rccConf;
  HAL_RCC_GetOscConfig(&rccConf);
//  HAL_GPIO_WritePin(RESET_CSM_PORT,RESET_CSM_PIN,1);
 // HAL_UART_Receive_IT(&huart4, mas, 1); //Принимает в массив байты строки


//  HAL_Delay(20000);
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  HAL_GPIO_WritePin(CAN_SHDN_PORT,CAN_SHDN_PIN,0);
  HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
//  CanRxMsgTypeDef *rxmes;

//  HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
  /* Infinite loop */
  while (1)
  {
	  if(HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0)!=HAL_OK)
		{
			Error_Handler();
		}
	  MDLUTransmitData.LAx = 10;
	  MDLUTransmitData.LAy = 14;
	  MDLUTransmitData.LAz = 22;
	  MDLUTransmitData.service =0x12;
//	  MDLUTransmitData.CSL = 0;
//	  MDLUTransmitData.CSL= calcCSofArr((uint8_t*)&MDLUTransmitData,8);
	  MDUSTransmitData.ARx = 100;
	  MDUSTransmitData.ARy = 170;
	  MDUSTransmitData.ARz = 33;
	  MDLUTransmitData.service =0x14;

	  MDADTransmitData.pressure = 100000;
	  MDADTransmitData.temperature = 22.1;
	  MDADTransmitData.service = 0x13;
//	  MDLUTransmitData.CSL = 0;
//	  MDLUTransmitData.CSL= calcCSofArr((uint8_t*)&MDLUTransmitData,8);
//	  HAL_GPIO_TogglePin(LED_LEG_PORT,LED_LEG_PIN);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

}

/* USER CODE BEGIN 4 */
//int checkLattitudeLongtitude(char *arr)
//{
//	volatile char arr1[8],arr2[8],arr3[8];
//	volatile double lattitude = 0,longtitude = 0, altitude=0;
//	volatile double *l1,*l2,*a1;
//	for(int i = 0; i < 8; i++)
//		arr1[i] = *arr++;
//	for(int i = 0; i < 8; i++)
//		arr2[i] = *arr++;
//	for(int i = 0; i < 8; i++)
//		arr3[i] = *arr++;
//	l1 = (double*)arr1;
//	l2 = (double*)arr2;
//	a1 = (double*)arr3;
//	lattitude = *l1;
//	longtitude = *l2;
//	altitude = *a1;
////	if((lattitude==0)&(longtitude==0))//&(altitude==0))
////		HAL_GPIO_WritePin(LED_1_PORT,LED_1_PIN,GPIO_PIN_SET);//(CAN_SHDN_PORT,CAN_SHDN_PIN);
////	else
////		HAL_GPIO_WritePin(LED_1_PORT,LED_1_PIN,GPIO_PIN_RESET);
//	return 0;
//}

/*
 * procedure func calculate which of subdevices'(MDLU,MDUS,MDAD) data should now be transmitted on CAN
 * also checks cycle state: 0 means transmition to be continued, 1 means last subdevice tick, and beginning new cycle
 * of transmition on next step
 */
char freqQueue(char *f)
{
	double MDLUweightGain = (double)((double)IMfreqs.totalFrequency/(double)IMfreqs.MDLUFrequency)*(double)IMfreqs.MDLUtickCounter;
	double MDUSweightGain = (double)((double)IMfreqs.totalFrequency/(double)IMfreqs.MDUSFrequency)*(double)IMfreqs.MDUStickCounter;
	double MDADweightGain = (double)((double)IMfreqs.totalFrequency/(double)IMfreqs.MDADFrequency)*(double)IMfreqs.MDADtickCounter;
	*f = 3;
	if(MDLUweightGain>=MDUSweightGain)
	{
		if(MDUSweightGain<=MDADweightGain)
		{
			if(IMfreqs.MDUStickCounter<IMfreqs.totalFrequency)
			{
				IMfreqs.MDUStickCounter++;
				*f = MDUSFrequency;
				return 0;
			}
			else return 1;
		}
		else
		{
			if(IMfreqs.MDADtickCounter<IMfreqs.totalFrequency)
			{
				IMfreqs.MDADtickCounter++;
				*f = MDADFrequency;
				return 0;
			}
			else return 1;
		}
	}
	else
	{
		if(MDLUweightGain<=MDADweightGain)
		{
			if(IMfreqs.MDLUtickCounter<IMfreqs.totalFrequency)
			{
				IMfreqs.MDLUtickCounter++;
				*f = MDLUFrequency;
				return 0;
			}
			else return 1;
		}
		else
		{
			if(IMfreqs.MDADtickCounter<IMfreqs.totalFrequency)
			{
				IMfreqs.MDADtickCounter++;
				*f = MDADFrequency;
				return 0;
			}
			else return 1;
		}
	}

}
/*
 * check correctness of desired MDAD,MDLU and MDUS frequencies, do their sum is equal to totalFrequency of device
 * returns 0 if ok, and  1 if nope
 */
char checkFrequencies()
{
	if(IMfreqs.totalFrequency == (IMfreqs.MDUSFrequency+IMfreqs.MDLUFrequency+IMfreqs.MDADFrequency))
	{
		IMfreqs.MDLUtickCounter = 0;
		IMfreqs.MDUStickCounter = 0;
		IMfreqs.MDADtickCounter = 0;
		return 0;
	}
	else
		return 1;
}

char parseArray(uint8_t *inArr, uint16_t inArrLength, int arrType, uint8_t *outArr, uint16_t *outArrLength)
{
	char err=0;
//	uint8_t tmpArr[300];
	uint8_t i;
//	 vector<uint8_t> tmpArr;
//	 vector<uint8_t>::iterator tmpIterator;
//
//	 for(i = 0; i < inArrLength; i++)
//	 {
	//	 tmpArr.insert(*inMas);
//		 tmpArr.at(i) = *inMas;
//		 tmpArr.push_back(*inMas);
//		 tmpArr[i]
//		 inArr++;
//	 }
	switch (arrType)
	{

		case 0x41:
		{
			for(i = 0; i < inArrLength; i++)
			{
				if(i<8)
				{
					*outArrLength++;		//incrementing value
					*outArr =  *inArr;
					inArr++;				//incrementing address
				}

			}
			break;
		}
		case 0x88:
		{
			for(i = 0; i < inArrLength; i++)
			{
				if(i<40)
				{
					*outArrLength++;		//incrementing value
					*outArr =  *inArr;
					inArr++;				//incrementing address
				}
				if(i==68)
				{
					*outArrLength++;		//incrementing value
					*outArr =  *inArr;
					inArr++;				//incrementing address
				}

			}
			break;
		}
		default:
		{
			err = 1;
			break;
		}


	}
	return err;
}

char makeFramedCANMessage(int *currentArrIndex, uint8_t *outArr, uint8_t *mode)
{
	char ret=1;
	uint8_t framesQuantity=0;
	uint8_t currentFrame = 0;
	uint8_t locmode = 0;
	uint8_t currentIndex = *currentArrIndex;
	currentFrame = currentIndex/7;
	framesQuantity = sizeof(framedArrCANTx)/7+1;
	if(currentIndex/7+1 <= framesQuantity)
	{
		for(int i=0; i < 8; i++)
		{
			if(i!=7)
			{
				*outArr = framedArrCANTx[currentIndex];
				currentIndex++;

			}
			else
			{
				if(currentIndex==7)
				{
					*outArr = framesQuantity;
					currentFrame++;
					locmode = 1;
											//also need to set correct mode flag in extID
				}
				else
				{
					currentFrame++;
					if(framesQuantity>currentFrame) //not first not last frame
						{
							*outArr = currentFrame;
							locmode = 2;
						}
					else
						{
							if(framesQuantity == currentFrame)
							{
								*outArr = calcCSofArr(framedArrCANTx,sizeof(framedArrCANTx));
								locmode = 3;
							}
							else
							{
								ret = 0;
								locmode = 0;
							}
						}



				}
			}
			outArr++;
		}
	}
	else
		{
			ret = 0;
			locmode = 0;
		}
	*mode = locmode;
	*currentArrIndex = currentIndex;
//	Serial=0x02;
	MsgMode = locmode;
	return ret;
}


void Error_Handler()
{
	volatile uint32_t tmpError=0;
	volatile HAL_CAN_StateTypeDef tmpStatus=0;
	tmpError = HAL_CAN_GetError(&hcan2);
	if(tmpError==0)
	{
		tmpStatus = HAL_CAN_GetState(&hcan2);
	}
	if(tmpStatus!=HAL_CAN_STATE_READY)
	{
		hcan2.State = HAL_CAN_STATE_READY;
//		HAL_CAN_DeInit(&hcan2);
//		HAL_CAN_Init(&hcan2);
	}

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan->pRxMsg->FIFONumber == 0)
	{

	}
	if (hcan->pRxMsg->FIFONumber == 1)
	{

	}
	if(hcan->Instance==CAN2)
	{
//		HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
		counter++;
		HAL_GPIO_TogglePin(LED_LEG_PORT,LED_LEG_PIN);
	}
	HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
//	if(HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0)!= HAL_OK)
//	{
//		Error_Handler();
//	}
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	__IO uint32_t ERR;
	if(hcan->Instance==CAN2)
	{
		  ERR=hcan->ErrorCode;
		  hcan->ErrorCode=0;
		  if ( ((hcan->Instance->TSR&CAN_TSR_TERR0) == CAN_TSR_TERR0) )
		  {
			__HAL_CAN_CANCEL_TRANSMIT(hcan,0);
		  }
//		  if ((hcan->Instance->TSR&CAN_TSR_TME1) == 0)
//		  {
//			  __HAL_CAN_CANCEL_TRANSMIT(hcan,1);
//		  }
//		  if ((hcan->Instance->TSR&CAN_TSR_TME2) == 0)
//		  {
//			  __HAL_CAN_CANCEL_TRANSMIT(hcan,2);
//		  }
		  HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
	}
}

//void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
//{
//	if(hcan->Instance==CAN2)
//	{
//		volatile int status = HAL_CAN_GetState(hcan);
//		HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
//	}
//
//}
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
//	if(hcan->Instance==CAN2)
//	{
//		if(canMessCounter<mesCount)
//			{
//				if(canMessCounter<mesCount-1)
//					prepareSTDID(1,2,3,3);
//				else
//					prepareSTDID(2,2,3,3);
//				hcan2.pTxMsg = &framedMessagesArr[canMessCounter];
//				framedMessagesArr[canMessCounter].IDE = CAN_ID_STD;
//				framedMessagesArr[canMessCounter].DLC = 8;
//				uint32_t tmp = *(uint32_t*)&STDID;
//				framedMessagesArr[canMessCounter++].StdId =tmp;
//				HAL_CAN_Transmit_IT(&hcan2);
//		//		beginCANTransmitFlag = false;
//			}
//		else canMessCounter=0;
//	}
//	if(HAL_CAN_GetState(&hcan2)==HAL_CAN_STATE_READY)
//	{

//			hcan2.pTxMsg = framedMessagesArr[canMessCounter++];
//			prepareSTDID((char)mesMode,2,3,3);
//			hcan2.pTxMsg = &framedMessagesArr[j];
//			framedMessagesArr[j].IDE = CAN_ID_STD;
//			framedMessagesArr[j].DLC = 8;
//			framedMessagesArr[j].StdId = STDID;
//			HAL_CAN_Transmit_IT(&hcan2);
//			HAL_CAN_Transmit_IT()
//		}
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	volatile char f;
	char msgtype = 0x0;
	char devtype;
	short serial = 0x0011;
	char priority = 0, mesMode = 0;
	//	IIMinstr = IIM_disablePolling_MDUS;
	if(freqQueue(&f)==1)
		checkFrequencies();
	else
		switch(f)
		{
			case 0:
			{
				devtype = MDLU;
//				prepareEXTID(serial,msgtype,devtype,priority,modeCANmsg);
				HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
				prepareSTDID(SingleMessage,IIMdata);
				setTxDataMessage(MDLU);
				HAL_CAN_Transmit(&hcan2, 20);
				break;
			}
			case 1:
			{
				devtype = MDUS;
//				prepareEXTID(serial,msgtype,devtype,priority,modeCANmsg);
				HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
				prepareSTDID(SingleMessage,IIMdata);
				setTxDataMessage(MDUS);
				HAL_CAN_Transmit(&hcan2, 20);
				break;
			}
			case 2:
			{
				devtype = MDAD;
//				prepareEXTID(serial,msgtype,devtype,priority,modeCANmsg);
				HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
				prepareSTDID(SingleMessage,IIMdata);
				setTxDataMessage(MDAD);
				HAL_CAN_Transmit(&hcan2, 20);
				break;
			}
			default:
				break;
		}

	HAL_GPIO_TogglePin(LED_LEG_PORT,LED_LEG_PIN);
}




//}

//void SendAT (const char * S)		//Функция выдачи строковой команды модулю
//{
//
//	 TxBusy = 1;
//	 strcpy (AT_COM,S);
//	 DLINA_AT = strlen (AT_COM);
//	 HAL_UART_Transmit_IT(&huart4, AT_COM, DLINA_AT);
//	 HAL_Delay(1);
//
//}
/* USER CODE END 4 */

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
