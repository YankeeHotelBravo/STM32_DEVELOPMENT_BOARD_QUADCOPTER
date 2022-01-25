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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "w25qxx.h"
#include "MadgwickAHRS.h"
#include "FS-iA6B.h"
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

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
//Timer
extern uint8_t tim1_2ms_flag;
extern uint8_t tim1_10ms_flag;
extern uint8_t tim1_20ms_flag;

//MPU9250
extern int MPU9250_DRDY;
uint8_t Mag_Calib[12];
extern uint8_t MPU9250_ASAX;
extern uint8_t MPU9250_ASAY;
extern uint8_t MPU9250_ASAZ;
float test_float;
extern float System_Roll, System_Pitch, System_Yaw;

//FTDI
extern uint8_t uart1_rx_flag;
extern uint8_t uart1_rx_data;
uint8_t print_mode = 0;
uint8_t mag_calibration_enable = 0;

//Receiver
extern uint8_t uart2_rx_flag;
extern uint8_t uart2_rx_data;
extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;
unsigned char motor_arming_flag = 0;
unsigned char is_throttle_middle = 0;
unsigned char is_yaw_middle = 0;
unsigned short iBus_SwA_Prev = 0;
float yaw_heading_reference = 0;

//Motor
unsigned int ccr1 ,ccr2, ccr3, ccr4;
unsigned char failsafe_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Compass_Calibration(uint8_t mag_calibration_enable);
void ESC_Calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char* p, int len)
{
	HAL_UART_Transmit_DMA(&huart1, p, len);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU9250.Gx_Offset = 0.43;
	MPU9250.Gy_Offset = -0.49;
	MPU9250.Gz_Offset = -1.40;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  //General Timer HAL
  HAL_TIM_Base_Start_IT(&htim7);
  //General PWM LL
//  LL_TIM_EnableCounter(TIM7);
//  LL_TIM_EnableIT_UPDATE(TIM7);

  //USART Channels HAL
  HAL_UART_Receive_DMA(&huart1, &uart1_rx_data, 1); //FTDI
  HAL_UART_Receive_DMA(&huart2, &uart2_rx_data, 1); //Receiver

	//Motor PWM HAL
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
  //Motor PWM LL
//  LL_TIM_EnableCounter(TIM3);
//  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
//  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
//  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
//  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//Initialize MPU9250
	while(MPU9250_Init(&hi2c1, 3, 3, 3, 3) == 0)
	{
		if(tim1_20ms_flag == 1)
		{
			tim1_20ms_flag = 0;
			printf("MPU9250 Initialing \n");
		}
	}
	MPU9250_Bypass(&hi2c1);
	while(MPU9250_AK8963_Setup(&hi2c1, &MPU9250) == 0)
	{
		if(tim1_20ms_flag == 1)
		{
			tim1_20ms_flag = 0;
			printf("AK8963 Initialing \n");
		}
	}
	MPU9250_Master(&hi2c1);
	MPU9250_Slave0_Enable(&hi2c1);

	//Initialize MS5611

	//EEPROM
	W25qxx_Init();
	W25qxx_ReadSector(Mag_Calib, 0, 0, 12);
	MPU9250.Mx_Offset = *(float*)&Mag_Calib[0];
	MPU9250.My_Offset = *(float*)&Mag_Calib[4];
	MPU9250.Mz_Offset = *(float*)&Mag_Calib[8];

	//Receiver Check
	printf("Receiver Status Check \n"); HAL_Delay(10);
	while(Is_iBus_Received(ibus_rx_cplt_flag) == 0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
	}
	while(Is_Throttle_Min() == 0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_GPIO_Port, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_GPIO_Port, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
	printf("Receiver OK \n"); HAL_Delay(10);

	//ESC Calibration
	if(iBus.SwB == 2000 && iBus.SwC == 2000)
	{
		ESC_Calibration();
		while(iBus.SwB != 1000) Is_iBus_Received(ibus_rx_cplt_flag);
	}

	while (1)
	{
		//UART Message Check
		Receive_Command();
		Compass_Calibration(mag_calibration_enable);

		//Receiver Channel Check
		Is_iBus_Received(ibus_rx_cplt_flag);
		if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
		{
			if(iBus.LV < 1010)
			{
				motor_arming_flag = 1;
				yaw_heading_reference = System_Yaw;
			}
		}
		iBus_SwA_Prev = iBus.SwA;
		if(iBus.SwA != 2000)
		{
			motor_arming_flag = 0;
		}
		if(motor_arming_flag == 1)
				{
					if(failsafe_flag == 0)
					{
						if(iBus.LV > 1010)
						{
							TIM3->CCR1 = ccr1 > 20000 ? 19900 : ccr1 < 10000 ? 10000 : ccr1;
							TIM3->CCR2 = ccr2 > 20000 ? 19900 : ccr2 < 10000 ? 10000 : ccr2;
							TIM3->CCR3 = ccr3 > 20000 ? 19900 : ccr3 < 10000 ? 10000 : ccr3;
							TIM3->CCR4 = ccr4 > 20000 ? 19900 : ccr4 < 10000 ? 10000 : ccr4;

						}
						else
						{
							TIM3->CCR1 = 10000;
							TIM3->CCR2 = 10000;
							TIM3->CCR3 = 10000;
							TIM3->CCR4 = 10000;
						}
					}
					else
					{
						TIM3->CCR1 = 10000;
						TIM3->CCR2 = 10000;
						TIM3->CCR3 = 10000;
						TIM3->CCR4 = 10000;
					}
				}
				else
				{
					TIM3->CCR1 = 10000;
					TIM3->CCR2 = 10000;
					TIM3->CCR3 = 10000;
					TIM3->CCR4 = 10000;
				}

		//Read MPU9250 + Motor PID
		if(tim1_2ms_flag == 1)
		{
			tim1_2ms_flag = 0;
			MPU9250_Read_All(&hi2c1);
			MPU9250_Parsing(&MPU9250);
			MadgwickAHRSupdate(MPU9250.Gx_Rad, MPU9250.Gy_Rad, MPU9250.Gz_Rad, MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Mx, MPU9250.My, MPU9250.Mz);

			ccr1 = 10000 + (iBus.LV-1000)*10;
			ccr2 = 10000 + (iBus.LV-1000)*10;
			ccr3 = 10000 + (iBus.LV-1000)*10;
			ccr4 = 10000 + (iBus.LV-1000)*10;
		}

		//Print According to the Input
		if(tim1_20ms_flag == 1)
		{
			tim1_20ms_flag = 0;
//			printf("%.2f \t %.2f \t %.2f \t \n", System_Roll, System_Pitch, System_Yaw);

			switch(print_mode)
			{
			case 1: printf("%.2f \t %.2f \t %.2f \t \n", System_Roll, System_Pitch, System_Yaw); break; //Roll, Pitch, Yaw
			case 2: /* printf("%.2f \t %.2f \t \m", Alt, Alt_Filt); */ break; //Alt Raw, Alt Filt
			case 3: printf("%.2f \t %.2f \t %.2f \t \n", MPU9250.Gx, MPU9250.Gy, MPU9250.Gz); break; //Gyro
			case 4: printf("%.2f \t %.2f \t %.2f \t \n", MPU9250.Ax, MPU9250.Ay, MPU9250.Az); break; //Accel
			case 5: printf("%.2f \t %.2f \t %.2f \t \n", MPU9250.Mx, MPU9250.My, MPU9250.Mz); break; //Mag
			case 6: printf("%f \t %f \t %f \t \n", MPU9250.Mx_Offset, MPU9250.My_Offset, MPU9250.Mz_Offset); break; //Mag_Offset
			case 11: printf("%d %d %d %d %d %d %d %d %d %d \n", iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwB, iBus.VrA, iBus.VrB, iBus.SwC, iBus.SwD); break; //Mag_Offset
			default: break;
			}
		}

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00B03FDB;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 39999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W25qxx_CS_GPIO_Port, W25qxx_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W25qxx_CS_Pin */
  GPIO_InitStruct.Pin = W25qxx_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W25qxx_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Compass_Calibration(uint8_t mag_calibration_enable)
{
	if(mag_calibration_enable == 1)
	{
		for(int i =0;i<5;i++)
		{
			MPU9250_Read_All(&hi2c1);
			HAL_Delay(1);
			MPU9250_Parsing_NoOffset(&MPU9250);
		}
		MPU9250.Mx_Max = MPU9250.Mx;
		MPU9250.Mx_Min = MPU9250.Mx;
		MPU9250.My_Max = MPU9250.My;
		MPU9250.My_Min = MPU9250.My;
		MPU9250.Mz_Max = MPU9250.Mz;
		MPU9250.Mz_Min = MPU9250.Mz;

		while(mag_calibration_enable != 0)
		{
			Receive_Command();
			MPU9250_Read_All(&hi2c1);
			HAL_Delay(1);
			MPU9250_Parsing_NoOffset(&MPU9250);
			if(MPU9250.Mx > MPU9250.Mx_Max) MPU9250.Mx_Max = MPU9250.Mx;
			if(MPU9250.Mx < MPU9250.Mx_Min) MPU9250.Mx_Min = MPU9250.Mx;

			if(MPU9250.My > MPU9250.My_Max) MPU9250.My_Max = MPU9250.My;
			if(MPU9250.My < MPU9250.My_Min) MPU9250.My_Min = MPU9250.My;

			if(MPU9250.Mz > MPU9250.Mz_Max) MPU9250.Mz_Max = MPU9250.Mz;
			if(MPU9250.Mz < MPU9250.Mz_Min) MPU9250.Mz_Min = MPU9250.Mz;
		}
		MPU9250.Mx_Offset = (MPU9250.Mx_Max + MPU9250.Mx_Min) / 2;
		MPU9250.My_Offset = (MPU9250.My_Max + MPU9250.My_Min) / 2;
		MPU9250.Mz_Offset = (MPU9250.Mz_Max + MPU9250.Mz_Min) / 2;

		*(float*)&Mag_Calib[0] = MPU9250.Mx_Offset;
		*(float*)&Mag_Calib[4] = MPU9250.My_Offset;
		*(float*)&Mag_Calib[8] = MPU9250.Mz_Offset;

		W25qxx_EraseSector(0);
		W25qxx_WriteSector(Mag_Calib, 0, 0, 12);
	}
}

void ESC_Calibration(void)
{
	TIM3->CCR1 = 20000;
	TIM3->CCR2 = 20000;
	TIM3->CCR3 = 20000;
	TIM3->CCR4 = 20000;
	HAL_Delay(7000);
	TIM3->CCR1 = 10000;
	TIM3->CCR2 = 10000;
	TIM3->CCR3 = 10000;
	TIM3->CCR4 = 10000;
	HAL_Delay(8000);
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

