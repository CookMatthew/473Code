/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_it.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP_ADDR 0x77 << 1
#define BNO_ADDR 0x28 << 1
#define portNVIC_INT_CTRL_REG		( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT		( 1UL << 28UL )
#define portNVIC_Thread				( 1UL << 11UL)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MemBuf bnoBuf;
MemBuf bmpBuf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void schedule(void);
void contextSwap(void);
void addTask(void* task);
void initTasks(void);
void delayTask(int delay);

void taskOne(void);
void taskTwo(void);

void readBNO(void);
void readBMP(void);
void logBNO(void);
void logBMP(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char* intToChar(uint16_t num) {
	static char asciiNum[5];
	asciiNum[4] = '\0';
	asciiNum[3] = ((num % 10) + '0');
	asciiNum[2] = (((num/10) % 10) + '0');
	asciiNum[1] = (((num/100) % 10) + '0');
	asciiNum[0] = (((num/1000) % 10) + '0');
	if(asciiNum[0] != '0') {
		return &asciiNum[0];
	} else if(asciiNum[1] != '0') {
		return &asciiNum[1];
	} else if(asciiNum[2] != '0') {
		return &asciiNum[2];
	} else {
		return &asciiNum[3];
	}
}

void readBMP(void) {
	/*
	__disable_irq();
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFA,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFB,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFC,1,&result,1,100);
	__enable_irq();
	// Pressure read MSB to XLSB
	__disable_irq();
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xF7,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xF8,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xF9,1,&result,1,100);
	__enable_irq();
	*/
}

void readBNO(void) {
	//BNO IMU MSB to LSB
	//Set UNIT_SEL register to desired values
	__disable_irq();
	uint8_t msg = 0x10;
	HAL_I2C_Mem_Write(&hi2c1,BNO_ADDR,0x3B,1,&msg,1,100);
	__enable_irq();
	while(1) {
		if(bnoBuf.sent == false) {
			bnoBuf.bufLen = 0;
			uint8_t result;
			uint16_t number;
			//Gyro read
			//X
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x15,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x14,1,&result,1,100);


			__enable_irq();
			//Y
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x17,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x16,1,&result,1,100);
			__enable_irq();
			//Z
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x19,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x18,1,&result,1,100);
			__enable_irq();
			//Accel read
			//X
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x09,1,&result,1,100);
			number = (result << 8);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x08,1,&result,1,100);
			number += result;
						char msg[13] = "Gyro X axis: ";
						for(int i = 0; i < 13; i++) {
							bnoBuf.msg[bnoBuf.bufLen++] = msg[i];
						}
						char* msg2 = intToChar(result);
						for(int i = 0; i < 4; i++) {
							if(msg2[i] == '\0')
								break;
							bnoBuf.msg[bnoBuf.bufLen++] = msg2[i];
						}
			__enable_irq();
			//Y
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x0B,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x0A,1,&result,1,100);
			__enable_irq();
			//Z
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x0D,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x0C,1,&result,1,100);
			__enable_irq();
			//Magnetometer read
			//X
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0xFF,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0xFE,1,&result,1,100);
			__enable_irq();
			//Y
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x11,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x10,1,&result,1,100);
			__enable_irq();
			//Z
			__disable_irq();
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x13,1,&result,1,100);
			HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,0x12,1,&result,1,100);
			__enable_irq();
			bnoBuf.sent = true;
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	}
}

void taskOne(void) {

	//uint8_t msg[9] = "Task one\n";
	while(1) {
		if(bnoBuf.sent == true) {
			__disable_irq();
			HAL_UART_Transmit(&huart1,bnoBuf.msg,bnoBuf.bufLen,100);
			bnoBuf.bufLen = 0;
			bnoBuf.sent = false;
			__enable_irq();
			delayTask(500);
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

	}

}

void taskTwo(void) {
	while(1) {
			uint8_t msg[9] = "Task two\n";
			__disable_irq();
			//HAL_UART_Transmit(&huart1,msg,9,100);
			__enable_irq();
			delayTask(500);
		}
}

void delayTask(int delay) {
	__asm volatile (
			"mrs r0, psp \n"
	);
	currentProcess->delay = delay;
	currentProcess->currentState = BLOCKED;
	portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	while(currentProcess->currentState == BLOCKED) {

	}
	__asm volatile (
			"adds r0, #12 \n"
			"msr psp, r0 \n"
			"bx lr \n"
	);

}

void initTasks(void) {
	for(int i = 0; i < NUM_PROC; i++) {
		processes[i].stackTop = &processes[i].stack[STACK_SIZE - 17];
		processes[i].currentState = TERMINATED;
	}
	currentProcess = &processes[NUM_PROC-1];
	__asm volatile (
			"ldr r3, initialContext \n"
			"ldr r0, [r3] \n"
			"ldr r1, [r0] \n"
			"msr psp, r1 \n"
			"initialContext: .word currentProcess \n"
	);
}
void addTask(void* task) {
	for(int i = 0; i < NUM_PROC; i++) {
		if(processes[i].currentState == TERMINATED) {
			*(uint32_t*)(&processes[i].stack[STACK_SIZE - 2]) = (uint32_t)task;
			*(uint32_t*)(&processes[i].stack[STACK_SIZE - 1]) = (uint32_t)0x21000000;
			processes[i].currentState = ACTIVE;
			return;
		}
	}
}

void schedule(void) {
	static int i = 0;
	for(;;) {
		i = (i + 1) % NUM_PROC;
		if(processes[i].currentState == ACTIVE) {
			currentProcess = &processes[i];
			return;
		}
	}
	//uint8_t msg[11] = "Scheduling";
	//HAL_UART_Transmit(&huart1,msg,11,100);
}


void PendSV_Handler(void) {
	//__disable_irq();

	__asm volatile
	        (
	        "mrs r0, msp \n"
	        "ldr r1, =#0x00000008 \n"
	        "add r0, r1 \n"
	        "msr msp, r0 \n"
	        "mrs r0, psp \n"
	        "ldr r3, currentContext \n"
	        "ldr r2, [r3] \n"
	        "stmdb r0!, {r4-r11, r14} \n"
	        "str r0, [r2] \n"
	    );
	schedule();
	//__enable_irq();

	__asm volatile
		(
	    "ldr r3, currentContext \n"
	    "ldr r1, [r3] \n"
	    "ldr r0, [r1] \n"
	    "ldmia r0!, {r4-r11, r14} \n"
		"msr psp, r0 \n"
	    "isb \n"
		"ldr r14, =#0xFFFFFFFD \n"
		"bx lr \n"
		"currentContext: .word currentProcess \n"
	);
	//return;
	//portNVIC_INT_CTRL_REG = portNVIC_Thread;
	__asm volatile
			(
			"bx r14 \n"
	);

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  initTasks();
  bnoBuf.received = false;
  bnoBuf.sent = false;
  bmpBuf.received = false;
  bmpBuf.sent = false;
  addTask(&taskOne);
  addTask(&taskTwo);
  addTask(&readBNO);
  HAL_InitTick(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/*
	result = 0;
	HAL_UART_Transmit(&huart1,msg,12,100);
	HAL_Delay(1000);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFA,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFB,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFC,1,&result,1,100);
	*/
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : VCP_TX_Pin VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin|VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3)
  {
	  for(int j = 0; j < NUM_PROC; j++) {
	  	if(processes[j].currentState == BLOCKED) {
	  		processes[j].delay--;
			if(processes[j].delay == 0) {
	  			processes[j].currentState = ACTIVE;
	  		}
	  	}
	  }
	  //HAL_TIM_Base_Start_IT(&htim3);

	  //HAL_TIM_Base_Stop_IT(&htim3);
	  //TIM_Cmd(ENABLE);
	  //__HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
	  //__HAL_TIM_SET_COUNTER(&htim3, 0);
	  //HAL_TIM_Base_Start_IT(&htim3);
	//while(1) {}
    //contextSwap();
  }
}
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

