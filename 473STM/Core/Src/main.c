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
MemBuf bnoAccelBuf;
MemBuf bnoGyroBuf;
MemBuf bnoMagBuf;
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

char* intToChar(int16_t num) {
	static char asciiNum[6];
	int pos = 0;
	if(num < 0) {
		asciiNum[pos] = '-';
		num = -num;
		pos++;
	}
	uint16_t temp;
	bool nonzero = false;

	for(int i = 1000; i > 1; i /= 10) {
		temp = (num/i % 10);
		if(temp != 0) {
			asciiNum[pos++] = temp + '0';
			nonzero = true;
		}
	}
	asciiNum[pos++] = (num % 10) + '0';
	asciiNum[pos] = '\0';

	return asciiNum;
}

char* uintToChar(uint16_t num) {
	static char asciiNum[5];
	int pos = 0;
	uint16_t temp;
	bool nonzero = false;

	for(int i = 1000; i > 1; i /= 10) {
		temp = (num/i % 10);
		if(temp != 0) {
			asciiNum[pos++] = temp + '0';
			nonzero = true;
		}
	}
	asciiNum[pos++] = (num % 10) + '0';
	asciiNum[pos] = '\0';

	return asciiNum;
}

void readBNOReg(uint8_t reg, MemBuf* buffer) {
	int16_t number;
	uint8_t result;
	HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,reg,1,&result,1,100);
	number = (result << 8);
	HAL_I2C_Mem_Read(&hi2c1,BNO_ADDR,reg-1,1,&result,1,100);
	number += result;
	char* msg2 = intToChar(number);
	for(int i = 0; i < 4; i++) {
		if(msg2[i] == '\0')
			break;
		buffer->msg[buffer->bufLen++] = msg2[i];
	}
	buffer->msg[buffer->bufLen++] = '\r';
	buffer->msg[buffer->bufLen++] = '\n';
}

void readBMP(void) {
	while(1) {
		if(bmpBuf.sent == false) {
			__disable_irq();

			bmpBuf.bufLen = 0;

			uint8_t result;
			uint16_t num;
			HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFA,1,&result,1,100);
			num = result << 8;
			HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFB,1,&result,1,100);
			num += result;

			char msg[6] = "Temp: ";
			char * tmp = uintToChar(num);
			for(int i = 0; i < 6; i++) {
				bmpBuf.msg[bmpBuf.bufLen++] = msg[i];
			}
			for(int i = 0; i < 4; i++) {
				if(tmp[i] == '\0') {
					break;
				}
				bmpBuf.msg[bmpBuf.bufLen++] = tmp[i];
			}
			bmpBuf.msg[bmpBuf.bufLen++] = '\r';
			bmpBuf.msg[bmpBuf.bufLen++] = '\n';

			HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xF7,1,&result,1,100);
			num = result << 8;
			HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xF8,1,&result,1,100);
			num += result;

			char msg2[10] = "Pressure: ";
			tmp = uintToChar(num);
			for(int i = 0; i < 10; i++) {
				bmpBuf.msg[bmpBuf.bufLen++] = msg2[i];
			}
			for(int i = 0; i < 4; i++) {
				if(tmp[i] == '\0') {
					break;
				}
				bmpBuf.msg[bmpBuf.bufLen++] = tmp[i];
			}
			bmpBuf.msg[bmpBuf.bufLen++] = '\r';
			bmpBuf.msg[bmpBuf.bufLen++] = '\n';

			bmpBuf.sent = true;
			__enable_irq();
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	}

}

void readBNOAccel(void) {
	while(1) {
		if(bnoAccelBuf.sent == false) {
			//Accel read
			//X
			bnoAccelBuf.bufLen = 0;

			__disable_irq();

			char msg2[13] = "Accel  axis: ";
			for(int i = 0; i < 6; i++) {
				bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = msg2[i];
			}
			bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = 'X';
			for(int i = 6; i < 13; i++) {
				bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = msg2[i];
			}
			readBNOReg(0x09,&bnoAccelBuf);

			//Y
			for(int i = 0; i < 6; i++) {
				bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = msg2[i];
			}
			bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = 'Y';
			for(int i = 6; i < 13; i++) {
				bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = msg2[i];
			}
			readBNOReg(0x0B,&bnoAccelBuf);

			//Z

			for(int i = 0; i < 6; i++) {
				bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = msg2[i];
			}
			bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = 'Z';
			for(int i = 6; i < 13; i++) {
				bnoAccelBuf.msg[bnoAccelBuf.bufLen++] = msg2[i];
			}
			readBNOReg(0x09,&bnoAccelBuf);

			bnoAccelBuf.sent = true;
			__enable_irq();
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	}
}

void readBNOMag(void) {
	while(1) {
		if(bnoMagBuf.sent == false) {
			bnoMagBuf.bufLen = 0;

			__disable_irq();
			char msg1[11] = "Mag  axis: ";
			for(int i = 0; i < 4; i++) {
				bnoMagBuf.msg[bnoMagBuf.bufLen++] = msg1[i];
			}
			bnoMagBuf.msg[bnoMagBuf.bufLen++] = 'X';
			for(int i = 4; i < 11; i++) {
				bnoMagBuf.msg[bnoMagBuf.bufLen++] = msg1[i];
			}
			readBNOReg(0xFF,&bnoMagBuf);

			for(int i = 0; i < 4; i++) {
				bnoMagBuf.msg[bnoMagBuf.bufLen++] = msg1[i];
			}
			bnoMagBuf.msg[bnoMagBuf.bufLen++] = 'Y';
			for(int i = 4; i < 11; i++) {
				bnoMagBuf.msg[bnoMagBuf.bufLen++] = msg1[i];
			}
			readBNOReg(0x11,&bnoMagBuf);

			for(int i = 0; i < 4; i++) {
				bnoMagBuf.msg[bnoMagBuf.bufLen++] = msg1[i];
			}
			bnoMagBuf.msg[bnoMagBuf.bufLen++] = 'Z';
			for(int i = 4; i < 11; i++) {
				bnoMagBuf.msg[bnoMagBuf.bufLen++] = msg1[i];
			}
			readBNOReg(0x13,&bnoMagBuf);
			bnoMagBuf.sent = true;
			__enable_irq();
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	}

}

void readBNOGyro(void) {
	//BNO IMU MSB to LSB
	//Set UNIT_SEL register to desired values

	while(1) {
		if(bnoGyroBuf.sent == false) {
			bnoGyroBuf.bufLen = 0;
			//Gyro read
			//X

			__disable_irq();

			char msg1[12] = "Gyro  axis: ";
			for(int i = 0; i < 5; i++) {
				bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = msg1[i];
			}
			bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = 'X';
			for(int i = 5; i < 12; i++) {
				bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = msg1[i];
			}
			readBNOReg(0x15,&bnoGyroBuf);


			//Y

			for(int i = 0; i < 5; i++) {
				bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = msg1[i];
			}
			bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = 'Y';
			for(int i = 5; i < 12; i++) {
				bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = msg1[i];
			}
			readBNOReg(0x17,&bnoGyroBuf);



			//Z
			for(int i = 0; i < 5; i++) {
				bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = msg1[i];
			}
			bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = 'Z';
			for(int i = 5; i < 12; i++) {
				bnoGyroBuf.msg[bnoGyroBuf.bufLen++] = msg1[i];
			}
			readBNOReg(0x19,&bnoGyroBuf);
			__enable_irq();


			bnoGyroBuf.sent = true;
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
	}
}

void bmpPrint(void) {

	//uint8_t msg[9] = "Task one\n";
	while(1) {
		if(bmpBuf.sent == true) {
			__disable_irq();
			HAL_UART_Transmit(&huart1,bmpBuf.msg,bmpBuf.bufLen,100);
			bmpBuf.bufLen = 0;
			bmpBuf.sent = false;
			__enable_irq();
			delayTask(500);
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

	}

}

void bnoMagPrint(void) {

	//uint8_t msg[9] = "Task one\n";
	while(1) {
		if(bnoMagBuf.sent == true) {
			__disable_irq();
			HAL_UART_Transmit(&huart1,bnoMagBuf.msg,bnoMagBuf.bufLen,100);
			bnoMagBuf.bufLen = 0;
			bnoMagBuf.sent = false;
			__enable_irq();
			delayTask(500);
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

	}

}

void bnoAccelPrint(void) {

	//uint8_t msg[9] = "Task one\n";
	while(1) {
		if(bnoAccelBuf.sent == true) {
			__disable_irq();
			HAL_UART_Transmit(&huart1,bnoAccelBuf.msg,bnoAccelBuf.bufLen,100);
			bnoAccelBuf.bufLen = 0;
			bnoAccelBuf.sent = false;
			__enable_irq();
			delayTask(500);
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

	}

}

void bnoGyroPrint(void) {
	while(1) {
		if(bnoGyroBuf.sent == true) {
			__disable_irq();
			HAL_UART_Transmit(&huart1,bnoGyroBuf.msg,bnoGyroBuf.bufLen,100);
			bnoGyroBuf.bufLen = 0;
			bnoGyroBuf.sent = false;
			__enable_irq();
			delayTask(500);
		}
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

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
	__disable_irq();
	initTasks();
	bnoAccelBuf.sent = false;
	bnoGyroBuf.sent = false;
	bmpBuf.sent = false;
	bnoMagBuf.sent = false;
	uint8_t msg = 0x10;
	HAL_I2C_Mem_Write(&hi2c1,BNO_ADDR,0x3B,1,&msg,1,100);
	msg = 0x08;
	HAL_I2C_Mem_Write(&hi2c1,BNO_ADDR,0x3D,1,&msg,1,100);
	msg = 0x27;
	HAL_I2C_Mem_Write(&hi2c1,BMP_ADDR,0xF4,1,&msg,1,100);
	addTask(&bnoGyroPrint);
	addTask(&bnoAccelPrint);
	addTask(&bnoMagPrint);
	addTask(&bmpPrint);
	addTask(&readBNOGyro);
	addTask(&readBNOAccel);
	addTask(&readBNOMag);
	addTask(&readBMP);
	__enable_irq();
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

	// BMP Temp and Pressure Sensor
	// Temp read MSB to XLSB
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFA,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFB,1,&result,1,100);
	HAL_I2C_Mem_Read(&hi2c1,BMP_ADDR,0xFC,1,&result,1,100);
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

