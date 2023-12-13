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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Pins redefinition*/
#define PLS1 GPIO_PIN_1
#define DIR1 GPIO_PIN_2
#define ENA1 GPIO_PIN_3
#define PLS2 GPIO_PIN_4
#define DIR2 GPIO_PIN_5
#define ENA2 GPIO_PIN_6

/*ADC related defines*/
#define ADC_RESOLUTION pow(2,12)
#define V_REF 3.3 //This is VDDA maximum VREF voltage according to datasheet

#define NUM_OF_EPRUVETE 10

/*For a 50 mm length, and 5mm per slot => 10*/
#define NUMBER_OF_SLOTS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
enum Estate
{
	idle,
	heating,
	calibrating,
	locating,
	shifting
};

enum EtimerTask
{
	heatM_1,
	calibrateM_1,
	locateM_1CW,
	locateM_1CCW,
	M_2
};
enum Estate state=idle;
enum EtimerTask task;



/* Unesite zeljenu temperaturu*/
int userTemp=50;

int epr;

int arr[]={0,0,3,0,4,0,0,3,0,3};
int dirTemp=1;
int timerStarted=0;

int positionM2=0;
int position=0;
int toggleCounter=0;
int adcSemaphore=0;

int nearest=0;
int current=0;
int resetM_2=0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*Finding the closest in array related functions*/
int findClosest(int current,int epruveta, int* arr)
{
    int cLeft=current;
    int cRight=current;
    int moveTo=0;
    while(1)
        {
            if (arr[cLeft]==epruveta)
            {
                moveTo=cLeft;
                break;
            }
            else
            {
                cLeft--;
                cRight++;
                if (cLeft<0)
                    cLeft=NUM_OF_EPRUVETE-1;
                if (cRight>NUM_OF_EPRUVETE-1)
                    cRight=0;
                if (arr[cLeft]==epruveta)
                {
                    moveTo=cLeft;
                    break;
                }
                if (arr[cRight]==epruveta)
                {
                    moveTo=cRight;
                    break;
                }
            }
        }
    return moveTo;
}
int existsEpruveta(int epruveta,int *arr)
{
    for(int i=0; i< NUM_OF_EPRUVETE-1; i++)
        if(arr[i]==epruveta)
            return 0;
    return 1;
}
int doCCW(int current,int nearest)
{
    int distanceLeft=(current-nearest+NUM_OF_EPRUVETE)%NUM_OF_EPRUVETE;
    int distanceRight=(nearest-current+NUM_OF_EPRUVETE)%NUM_OF_EPRUVETE;
    if(distanceRight>distanceLeft)
        return 1;
    return 0;
}


double convertAdcToVolts(double adcVal)
{
	return adcVal*(V_REF/ADC_RESOLUTION);
}
double convertToCelsius(double V)
{
	return V*100;
}
void startMotor(int Dir,int motor)
{
	switch(motor)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOA,ENA1|PLS1,GPIO_PIN_SET);
			HAL_Delay(3001); //ENA set at least 3s before seting the direction
			HAL_GPIO_WritePin(GPIOA,DIR1,Dir);
			HAL_Delay(1); //Dir set at least 5 microSec before falling edge of PULSE
			HAL_GPIO_WritePin(GPIOA,PLS1,GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA,ENA2|PLS2,GPIO_PIN_SET);
			HAL_Delay(3001); //ENA set at least 3s before seting the direction
			HAL_GPIO_WritePin(GPIOA,DIR2,Dir);
			HAL_Delay(1); //Dir set at least 5 microSec before falling edge of PULSE
			HAL_GPIO_WritePin(GPIOA,PLS2,GPIO_PIN_RESET);
			break;

	}
}
void disableMotor(int motor)
{
	switch(motor)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOA,ENA1|PLS1|DIR1,GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA,ENA2|PLS2|DIR2,GPIO_PIN_RESET);
			break;
	}
}
void changeDirection(int motor, int dir)
{
	switch(motor)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOA,PLS1, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA,PLS1, GPIO_PIN_SET);
			position++;
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA,DIR1, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA,PLS2, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA,PLS2, GPIO_PIN_SET);
			positionM2++;
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA,DIR2, GPIO_PIN_SET);
			break;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_1:
			epr=0;
			break;
		case GPIO_PIN_2:
			epr=1;
			break;
		case GPIO_PIN_3:
			epr=2;
			break;
		case GPIO_PIN_4:
			epr=3;
			break;
		case GPIO_PIN_5:
			epr=4;
			break;
	}
	//If somebody presses a button the motor one activates in heating mode only if there exists more tubes in the system.
	if(existsEpruveta(epr,arr) && (state==idle))
		state=heating;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
	{
		switch(task)
		{
			case heatM_1:
				HAL_GPIO_TogglePin(GPIOA,PLS1);
				toggleCounter++;
				if(toggleCounter%2==1)
					position++;
				if(position==200)
					position=0;
				break;
			case calibrateM_1:
				HAL_GPIO_TogglePin(GPIOA,PLS1);
				toggleCounter++;
				if (dirTemp)
				{
					if(toggleCounter%2==1)
						position++;
				}
				else
				{
					if(toggleCounter%2==1)
						position--;
				}
				break;
			case locateM_1CW:
				HAL_GPIO_TogglePin(GPIOA,PLS1);
				toggleCounter++;
				if(toggleCounter%2==1)
					position++;
				if(position==200)
					position=0;
				break;
			case locateM_1CCW:
				HAL_GPIO_TogglePin(GPIOA,PLS1);
				toggleCounter++;
				if(toggleCounter%2==1)
					position--;
				if(position<0)
					position=199;
				break;
			case M_2:
				HAL_GPIO_TogglePin(GPIOA,PLS2);
				toggleCounter++;
				if(toggleCounter%2==1)
					positionM2++;
				break;
		}
	}
	else if(htim==&htim3)
	{
		adcSemaphore=1;
	}
}

void heat()
{
	int temp;
	if(!timerStarted)
	{
		task=heatM_1;
		startMotor(1, 1);
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
		timerStarted=1;
	}
	if(adcSemaphore)
	{
		HAL_ADC_Start(&hadc1);
		if(!(HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_BUSY))
		{
			int voltsRaw = HAL_ADC_GetValue(&hadc1);
			int volts=convertAdcToVolts(voltsRaw);
			temp =convertToCelsius(volts);
		}

		if(temp>userTemp)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim3);
			toggleCounter=0;
			disableMotor(1);
			state=calibrating;
			timerStarted=0;
			temp=0;
		}
		adcSemaphore=0;
	}
}
void calibrate()
{
	if(!timerStarted)
	{
		task=calibrateM_1;
		if (position>100)
		{
			startMotor(1, 1);
			dirTemp=1;
		}
		else
		{
			startMotor(1, 0);
			dirTemp=0;
		}
		HAL_TIM_Base_Start_IT(&htim2);
		timerStarted=1;
	}

	if (position%200==0)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
		disableMotor(1);
		timerStarted=0;
		toggleCounter=0;
		position=0;
		state=locating;
	}
}
void locate()
{
	 if(!timerStarted)
	 {
		current=position/20;
	 	nearest=findClosest(current,epr,arr);
	 	if(current==nearest)
	 	{
	 		state=shifting;
	 	}
	 	else
	 	{
	 		if(doCCW(current,nearest))
				{
					startMotor(1, 0);
					dirTemp=0;
					task=locateM_1CCW;
				}
				else
				{
					startMotor(1, 1);
					dirTemp=1;
					task=locateM_1CW;
				}
			HAL_TIM_Base_Start_IT(&htim2);
			timerStarted=1;
	 	}
	 }
	 if(position/20==nearest && position%20==0)
	 {
		HAL_TIM_Base_Stop_IT(&htim2);
		disableMotor(1);
		timerStarted=0;
		toggleCounter=0;
		arr[nearest]=-10;
		state=shifting;
	 }
}
void shift()
{
	if(!timerStarted)
	{
		task=M_2;
		startMotor(1, 2); //START M_2
		HAL_TIM_Base_Start_IT(&htim2);
		timerStarted=1;
	}

	if(positionM2==NUMBER_OF_SLOTS)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
		changeDirection(2,0);
		positionM2--;
		toggleCounter=0;
		HAL_TIM_Base_Start_IT(&htim2);
		resetM_2=1;
		if(existsEpruveta(epr,arr))
			state=locating;
		else
			state=idle;
	}
	if(resetM_2)
	{
		if(positionM2==0)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			timerStarted=0;
			disableMotor(2);
			toggleCounter=0;
			resetM_2=0;
			if(existsEpruveta(epr,arr))
				state=locating;
			else
				state=idle;
		}

	}

}

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Calibrate The ADC On Power-Up For Better Accuracy
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  switch(state)
	  {
		  case idle:
			  //Here something else could be done
			  break;
		  case heating:
			  heat();
			  break;
		  case calibrating:
			  calibrate();
			  break;
		  case locating:
			  locate();
			  break;
		  case shifting:
			  shift();
			  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
