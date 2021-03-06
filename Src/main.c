/**
  ******************************************************************************
  * File Name          : main.c
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
#include "usart.h"
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

char 	TxNMEA0183[500];			//����� �����������
uint8_t		TxCntNMEA0183;				//������� �����������
char 	RxNMEA0183[500];			//����� �����������
uint8_t		RxCntNMEA0183;				//������� �����������
bool	reciveNMEA0183;				//���� �������� ����� ������
bool	sendNMEA0183;				//���� ��������� ��������
bool	T7Flag;						//���� ������������ ������� �7
char 	TxRS485[500];			//����� �����������
uint8_t		TxCntRS485;				//������� �����������
char 	RxRS485[500];			//����� �����������
uint8_t		RxCntRS485;				//������� �����������
bool	reciveRS485;				//���� �������� ����� ������
bool	sendRS485;				//���� ��������� ��������
bool	reciveADRRS485;				//���� ���������� ��������� � �������� ������
uint8_t aRxBuffer[RXBUFFERSIZE];
char 	TxBINR[500];			//����� �����������
uint16_t		TxCntBINR;				//������� �����������
char 	RxBINR[2048];			//����� �����������
uint16_t		RxCntBINR;				//������� �����������
bool	reciveBINR;				//���� �������� ����� ������
bool	sendBINR;				//���� ��������� ��������
// bool idleCatched = false;
bool	flagnewbinr;
uint16_t 		RxWrIndex;
bool 	RxBinrBuffOverrun;
uint8_t		NumberSatateRxBINR;		//���� ��������� ��������
#ifdef yesCRC
	bool	CRCok;					//���� ���������� ����������� ����
	uint8_t		NumberbyteCRC;			//������� ���� ����������� �����
	uint8_t		MCRC[5];				//������ ���� ����������� �����
#endif
uint8_t		hE4BINRmas[530];		//������ � �������� ������� �4
bool	hE4BINRok;				//���� ��������� ������� �4
uint8_t		h41BINRmas[12];			//������ � �������� ������� 41
bool	h41BINRok;				//���� ��������� ������� 41
uint8_t		h52BINRmas[224];		//������ � �������� ������� 52
bool	h52BINRok;				//���� ��������� ������� 52
uint8_t		h88BINRmas[69];			//������ � �������� ������� 88
bool	h88BINRok;				//���� ��������� ������� 88
//uint8_t
int dataArrIndex;
int mesCount =86;
uint8_t modeCANmsg;
int size = 0;
uint8_t INSstructArr[72];
uint8_t DMU10_mes[68];
volatile uint8_t messg[8];
volatile int cntr=0;
uint8_t headerLSB=0,headerMSB=0;
bool recieveFlag = false;
uint32_t mesCounter=0;
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
INSCoordMessage insdata;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
//  HAL_CAN_MspInit(&hcan2);
  inc = 0;
  double tmp = 1.2;
  volatile uint8_t arr[sizeof(tmp)];
  volatile int sizechecker;
  sizechecker = sizeof(float);
  sizechecker = sizeof(double);
  var2ArrConverter((char*)&tmp,sizeof(tmp),arr);
  volatile const uint8_t someval= calcCSofArr(arr,7);
  hcan2.pTxMsg = &TxMessage;
  hcan2.pRxMsg = &RxMessage;
  TxMessage.StdId = &STDID;
  IMfreqs.MDADFrequency = 20;
  IMfreqs.MDLUFrequency = 300;
  IMfreqs.MDUSFrequency = 180;
  IMfreqs.totalFrequency = 500;
  checkFrequencies();
  HAL_UART_Receive_IT(&huart1,mas,1);
//  Freq = 500;
  FreqPresc = CoreFreq/(IMfreqs.totalFrequency);
  htim7.Init.Prescaler = FreqPresc;//*0.908;
  htim7.Init.Period = 1;
  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.BankNumber = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0;// ���
  sFilterConfig.FilterIdLow = 0x000C;
  sFilterConfig.FilterMaskIdHigh = 0x0;
  sFilterConfig.FilterMaskIdLow = 0x0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
  HAL_CAN_Init(&hcan2);
  volatile int clock = HAL_RCC_GetSysClockFreq();
  clock = HAL_RCC_GetPCLK1Freq();
  clock = HAL_RCC_GetHCLKFreq();
  clock = HAL_RCC_GetPCLK2Freq();
  volatile RCC_OscInitTypeDef rccConf;
  HAL_RCC_GetOscConfig(&rccConf);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  module = INScoords;
  HAL_GPIO_WritePin(CAN_SHDN_PORT,CAN_SHDN_PIN,0);
  size = getArraySize((uint8_t*)&insdata,sizeof(INSCoordMessage));

//  HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
  /* Infinite loop */
  while (1)
  {
	  protocolMessageProcessor((uint8_t*)&insdata,sizeof(INSCoordMessage),INSstructArr);
	  volatile int m;
	  for(int i = 0; i < size/8; i++ )
	  {

		  cntr = mesQueueProcedure(INSstructArr,cntr,messg,size);
		  if(i==0)
			  m=3;
		  else if(i==size/8-1)
			 m=2;
		  else
			 m=1;
		  prepareSTDID(m,module);
		  setTxDataMessage(module,messg);
		  hcan2.pTxMsg = &TxMessage;
		  if(HAL_CAN_Transmit(&hcan2,3)!=HAL_OK)
			  		{
			  			Error_Handler();
			  		}

	  }
	  cntr = 0;
 }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */



/*NOT IN USE NOW!!!!!
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
/*NOT IN USE NOW!!!!
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{

		uint8_t rxbyte = (uint8_t)USART1->DR;
		HAL_UART_Receive_IT(huart, mas, 1);

		if(recieveFlag)
		{

			mesCounter++;
			DMU10_mes[mesCounter] = rxbyte;
			if(mesCounter==67)
			{
//			    DMU10msg = *(DMU10Message*)DMU10_mes;
				uint8_t tmparr[68];
				straightToInverseConverter(DMU10_mes,sizeof(DMU10Message),tmparr);
			    memcpy(&DMU10msg,tmparr,sizeof(DMU10Message));
			    convertDMU10ToINSMessage(&insdata,DMU10msg);
				recieveFlag=0;
				mesCounter = 0;
				headerLSB=0;
				headerMSB=0;
				clearDMU10_mes();
			}
		}
		else
		{
//			headerLSB=0;
			if(rxbyte==0x55)
			{

				headerMSB = 0x55;
				recieveFlag = false;
				clearDMU10_mes();
				mesCounter=0;
				DMU10_mes[mesCounter] = rxbyte;
	//			headerMSB=0;
			}
			else if(rxbyte==0xAA)
			{
//				headerLSB = 0;
				if(headerMSB == 0x55)
				{
					headerLSB=0xAA;
					recieveFlag = true;
//					mesCounter = 1;
					DMU10_mes[++mesCounter] = rxbyte;
				}
			}
		}

	}
	if(huart->Instance==USART2)
	{
		char rxbyte = (char)USART2->DR;
		HAL_UART_Receive_IT(huart, mas, 1);
	}
}

void convertDMU10ToINSMessage(INSCoordMessage *ins,DMU10Message dmu)
{
	ins->aX = dmu.Xacceleration;
	ins->aY = dmu.Yacceleration;
	ins->aZ = dmu.Zacceleration;
	ins->wX = dmu.Xrate;
	ins->wY = dmu.Yrate;
	ins->wZ = dmu.Zrate;
	ins->XIN = 0;
	ins->YIN = 0;
	ins->ZIN = 0;
	ins->angleX = dmu.XDeltaTheta;
	ins->angleY = dmu.YDeltaTheta;
	ins->angleZ = dmu.ZDeltaTheta;
	ins->xDIS = 0;
	ins->yDIS = 0;
	ins->zDIS = 0;

}

int straightToInverseConverter(uint8_t *inarr, uint32_t size, uint8_t *outarr)
{
	if(size==sizeof(DMU10Message))
	{
		outarr[0]=inarr[1];
		outarr[1]=inarr[0];

		outarr[2]=inarr[3];
		outarr[3]=inarr[2];
		for(int i = 4; i < 60; i+=4)
		{
			outarr[i]=inarr[i+3];
			outarr[i+1]=inarr[i+2];
			outarr[i+2]=inarr[i+1];
			outarr[i+3]=inarr[i];
		}

		outarr[60]=inarr[61];
	    outarr[61]=inarr[60];

	    outarr[62]=inarr[63];
	    outarr[63]=inarr[62];

		outarr[64]=inarr[65];
	    outarr[65]=inarr[64];

	    outarr[66]=inarr[67];
	    outarr[67]=inarr[66];

	}
	else return -1;
	return 0;
}

void clearDMU10_mes()
{
	for(int i =0; i < 68; i++)
	{
		DMU10_mes[i] = 0;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
//		huart->ErrorCode = HAL_UART_ERROR_NONE;
		if(HAL_UART_GetError(huart)==HAL_UART_ERROR_ORE)
			__HAL_UART_CLEAR_FLAG(huart,HAL_UART_ERROR_ORE);
		recieveFlag = false;
		mesCounter = 0;
		clearDMU10_mes();
		huart->ErrorCode = HAL_UART_ERROR_NONE;
//		huart->State = HAL_UART_STATE_READY;
		char rxbyte = (char)USART1->DR;
		HAL_UART_Receive_IT(huart, mas, 1);
	}
}


void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan->Instance==CAN2)
	{
		if(HAL_CAN_GetState(&hcan2)==HAL_CAN_STATE_READY)
		  for(int i = 1; i < size/8; i++ )
		  {
			  cntr = mesQueueProcedure(INSstructArr,cntr,messg,size);
			  HAL_CAN_Transmit_IT(&hcan2);
	//		  HAL_Delay(10);
	//		  if(HAL_CAN_Transmit(&hcan2,3)!=HAL_OK)
	//			  		{
	//			  			Error_Handler();
	//			  		}
	//		  if(cntr!=-1)
	//		  	  HAL_CAN_Transmit(&hcan2,2);
	//		  else
	//			  i = size/8;

		  }
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

	}
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
//	 HAL_CAN_IRQHandler(&hcan2);
//	HAL_CAN_Transmit_IT(&hcan2);
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
				prepareSTDID(SingleMessage,INScoords);
//				setTxDataMessage(MDLU);
				HAL_CAN_Transmit(&hcan2, 20);
				break;
			}
			case 1:
			{
				devtype = MDUS;
//				prepareEXTID(serial,msgtype,devtype,priority,modeCANmsg);
				HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
				prepareSTDID(SingleMessage,INScoords);
//				setTxDataMessage(MDUS);
				HAL_CAN_Transmit(&hcan2, 20);
				break;
			}
			case 2:
			{
				devtype = MDAD;
//				prepareEXTID(serial,msgtype,devtype,priority,modeCANmsg);
				HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
				prepareSTDID(SingleMessage,INScoords);
//				setTxDataMessage(MDAD);
				HAL_CAN_Transmit(&hcan2, 20);
				break;
			}
			default:
				break;
		}

	HAL_GPIO_TogglePin(LED_LEG_PORT,LED_LEG_PIN);
}




//}

//void SendAT (const char * S)		//������� ������ ��������� ������� ������
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
