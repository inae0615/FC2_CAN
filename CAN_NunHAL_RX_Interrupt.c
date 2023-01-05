/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_FilterTypeDef sFilterConfig; // filter setting structure variable
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[25];
uint8_t RxID[25];
uint32_t state =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */


int _write(int file, char *ptr, int len)
{
   HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 100);
   HAL_Delay(1000);
   return (len);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	   HAL_CAN_StateTypeDef state = hcan->State;

	   if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) != 0U)//pending이 1개라도 있으면 실행
	   {
	       /* Check that the Rx FIFO 0 is not empty */


	     /* Get the header */
	     RxHeader.IDE = CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
	     if (RxHeader.IDE == CAN_ID_STD)
	     {
	    	 RxHeader.StdId = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	     }
	     else
	     {
	    	 RxHeader.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_RI0R_EXID_Pos;
	     }
	     RxHeader.RTR = (CAN_RI0R_RTR & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR);
	     RxHeader.DLC = (CAN_RDT0R_DLC & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR) >> CAN_RDT0R_DLC_Pos;
	     RxHeader.FilterMatchIndex = (CAN_RDT0R_FMI & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR) >> CAN_RDT0R_FMI_Pos;
	     RxHeader.Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR) >> CAN_RDT0R_TIME_Pos;

	     /* Get the data */
	     RxData[0] = (uint8_t)((CAN_RDL0R_DATA0 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR) >> CAN_RDL0R_DATA0_Pos);
	     RxData[1] = (uint8_t)((CAN_RDL0R_DATA1 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR) >> CAN_RDL0R_DATA1_Pos);
	     RxData[2] = (uint8_t)((CAN_RDL0R_DATA2 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR) >> CAN_RDL0R_DATA2_Pos);
	     RxData[3] = (uint8_t)((CAN_RDL0R_DATA3 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR) >> CAN_RDL0R_DATA3_Pos);
	     RxData[4] = (uint8_t)((CAN_RDH0R_DATA4 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR) >> CAN_RDH0R_DATA4_Pos);
	     RxData[5] = (uint8_t)((CAN_RDH0R_DATA5 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR) >> CAN_RDH0R_DATA5_Pos);
	     RxData[6] = (uint8_t)((CAN_RDH0R_DATA6 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR) >> CAN_RDH0R_DATA6_Pos);
	     RxData[7] = (uint8_t)((CAN_RDH0R_DATA7 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR) >> CAN_RDH0R_DATA7_Pos);


	     /* Release RX FIFO 0 */
	     SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);


	     HAL_UART_Transmit(&huart2, RxData, strlen(RxData), 100);
	     sprintf(RxID,"_RxID: 0x%3X\r\n",RxHeader.StdId);
	     HAL_UART_Transmit(&huart2, RxID, strlen(RxID), 100);
	   }

	state =1;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  //MX_CAN_Init();
   /* USER CODE BEGIN 2 */
   //==========MX_CAN_INIT============
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
   //============HAL_CAN_INIT===========
   uint32_t tickstart;

   if (hcan.State == HAL_CAN_STATE_RESET)
   {
     /* Init the low level hardware: CLOCK, NVIC */
     HAL_CAN_MspInit(&hcan);
   }

   SET_BIT(hcan.Instance->MCR, CAN_MCR_INRQ);

   /* Get tick */
   tickstart = HAL_GetTick();

   while ((hcan.Instance->MSR & CAN_MSR_INAK) == 0U)
   {
     if ((HAL_GetTick() - tickstart) > 10U)
     {
       /* Update error code */
       hcan.ErrorCode |= HAL_CAN_ERROR_TIMEOUT;

       /* Change CAN state */
       hcan.State = HAL_CAN_STATE_ERROR;

       return HAL_ERROR;
     }
   }

   CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_SLEEP);

   tickstart = HAL_GetTick();

   /* Check Sleep mode leave acknowledge */
   while ((hcan.Instance->MSR & CAN_MSR_SLAK) != 0U)
   {
     if ((HAL_GetTick() - tickstart) > 10U)
     {
       /* Update error code */
       hcan.ErrorCode |= HAL_CAN_ERROR_TIMEOUT;

       /* Change CAN state */
       hcan.State = HAL_CAN_STATE_ERROR;

       return HAL_ERROR;
     }
   }

   /* Set the time triggered communication mode */
   if (hcan.Init.TimeTriggeredMode == ENABLE)
   {
     SET_BIT(hcan.Instance->MCR, CAN_MCR_TTCM);
   }
   else
   {
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_TTCM);
   }

   /* Set the automatic bus-off management */
   if (hcan.Init.AutoBusOff == ENABLE)
   {
     SET_BIT(hcan.Instance->MCR, CAN_MCR_ABOM);
   }
   else
   {
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_ABOM);
   }

   /* Set the automatic wake-up mode */
   if (hcan.Init.AutoWakeUp == ENABLE)
   {
     SET_BIT(hcan.Instance->MCR, CAN_MCR_AWUM);
   }
   else
   {
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_AWUM);
   }

   /* Set the automatic retransmission */
   if (hcan.Init.AutoRetransmission == ENABLE)
   {
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_NART);
   }
   else
   {
     SET_BIT(hcan.Instance->MCR, CAN_MCR_NART);
   }

   /* Set the receive FIFO locked mode */
   if (hcan.Init.ReceiveFifoLocked == ENABLE)
   {
     SET_BIT(hcan.Instance->MCR, CAN_MCR_RFLM);
   }
   else
   {
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_RFLM);
   }

   /* Set the transmit FIFO priority */
   if (hcan.Init.TransmitFifoPriority == ENABLE)
   {
     SET_BIT(hcan.Instance->MCR, CAN_MCR_TXFP);
   }
   else
   {
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_TXFP);
   }

   /* Set the bit timing register */
   WRITE_REG(hcan.Instance->BTR, (uint32_t)(hcan.Init.Mode           |
                                             hcan.Init.SyncJumpWidth  |
                                             hcan.Init.TimeSeg1       |
                                             hcan.Init.TimeSeg2       |
                                             (hcan.Init.Prescaler - 1U)));

   /* Initialize the error code */
   hcan.ErrorCode = HAL_CAN_ERROR_NONE;

   /* Initialize the CAN state */
   hcan.State = HAL_CAN_STATE_READY;
   /*
   if (HAL_CAN_Init(&hcan) != HAL_OK)
   {
     Error_Handler();
   }  */
   //==================================

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x321<< 5; // 0110 0100 0010 0000 << 3210
  sFilterConfig.FilterIdLow = 0x322<<5;  // 0110 01
  sFilterConfig.FilterMaskIdHigh = 0xFFFF; //16bit
  sFilterConfig.FilterMaskIdLow = 0x0200;     //16bit
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK){
       printf("filter error\r\n");
       Error_Handler();
  }

 // if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
 // {
  /* Notification Error */
 // Error_Handler();
 // }
///activ=====================================================

  HAL_CAN_StateTypeDef state = hcan.State;
  if ((state == HAL_CAN_STATE_READY) ||
       (state == HAL_CAN_STATE_LISTENING))
   {
     /* Enable the selected interrupts */
     __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

   }

  //======================================

   //HAL_CAN_START
   /*
   if(HAL_CAN_Start(&hcan) != HAL_OK){
      printf("Start error!!\r\n");
      Error_Handler();
   }
   */
   if (hcan.State == HAL_CAN_STATE_READY)
   {
     /* Change CAN peripheral state */
     hcan.State = HAL_CAN_STATE_LISTENING;

     /* Request leave initialisation */
     CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_INRQ);

     /* Get tick */
     tickstart = HAL_GetTick();

     /* Wait the acknowledge */
     while ((hcan.Instance->MSR & CAN_MSR_INAK) != 0U)
     {
       /* Check for the Timeout */
       if ((HAL_GetTick() - tickstart) > 10U)//CAN_TIMEOUT_VALUE=10U
       {
         /* Update error code */
         hcan.ErrorCode |= HAL_CAN_ERROR_TIMEOUT;

         /* Change CAN state */
         hcan.State = HAL_CAN_STATE_ERROR;

         return HAL_ERROR;
       }
     }

     /* Reset the CAN ErrorCode */
     hcan.ErrorCode = HAL_CAN_ERROR_NONE;

     /* Return function status */
   }
   else
   {
     /* Update error code */
     hcan.ErrorCode |= HAL_CAN_ERROR_NOT_READY;

     return HAL_ERROR;
   }
   //==============================================

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(state == 1){
		 printf("\r\cb = 0\r\n");
		HAL_Delay(1000);
		state = 0;
	  }
	  else{
		  printf("\r\nstate = 0\r\n");
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
