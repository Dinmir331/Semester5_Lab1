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

#include "disp1color.h"
#include "font.h"

#include <stdio.h>
#define WIDTH 32 //Ширина светодиодной панели
#define HEIGHT 16 //Высота светодиодной панели

//difficulty very easy = 1000, easy = 300, medium = 100, hard = 30, very hard=10, terror = 1
uint16_t difficulty = 100; // Шаг по времени для движения мяча

volatile uint32_t inter_time = 0; // Таймер миллисекунд
uint32_t time_last = 0; // Переменная для указания шага обновления координаты шарика

uint8_t ballX = WIDTH / 2;  // Координата мяча по оси X, установка мяча в центре
uint8_t ballY = HEIGHT / 2;  // Координата мяча по оси Y, установка мяча в центре
uint8_t ballSpeedX = 1; // Скорость мяча по оси X
uint8_t ballSpeedY = 1; // Скорость мяча по оси Y

//Potentiometers
float adc_value1 = 0; // Значение с потенциометра 1
float adc_value2 = 0; // Значение с потенциометра 2
float adc_filt_val1 = 0; //Фильтрованное значение потенциометра 1
float adc_filt_val2 = 0; // Фильтрованное значение потенциометра 2
uint8_t value1, value2; // Пропорциональное значение потенциометра 1 и 2
float k = 0.1; // Коэффициент фильтрации\

uint8_t score1_left, score2_right; // Счет левого и правого игрока
uint8_t plat_lenght = 5; // Длина платформы игрока

uint8_t flag_score = 0; // Флаг для показа счета
uint8_t shift1 = 0; //Сдвиг числа счета игрока 1 при 2 символах
uint8_t shift2 = 0; //Сдвиг числа счета игрока 2 при 2 символах

extern uint8_t data1[16], data2[16], data3[16], data4[16]; //Буфер для заполнения матрицы лед-панели

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t player1_left() { // функция для определения положения платформы игрока 1
	uint8_t y = 1 + value1; // Положение платформы игрока 1
	disp1color_DrawLine(0, y, 0, y + plat_lenght); // Рисование платформы 1
	return y; // возвращение положения платформы 1
}

uint8_t player2_right() { // функция для определения положения платформы игрока 2
	uint8_t y = 1 + value2; //Положение платформы игрока 2
	disp1color_DrawLine(31, y, 31, y + plat_lenght);//Рисование платформы 2
	return y; // возвращение положения платформы 2
}

void game() { // функция для реализации игрового процесса
	uint8_t player1, player2; // Реальное положение платформы игрока 1 и 2
	player1 = player1_left();
	player2 = player2_right();
	if (ballX == 0 || ballX == 31) { // находится ли мяч у левой и правой границы
		if (ballX == 0) { // если мяч находится у левой границы
			if (ballY < player1 || ballY > (player1 + plat_lenght)) { //если мяч находится вне координат платформы
				score1_left++; // увеличение счета 2 игрока
				ballX = WIDTH / 2; // возращение мяча в начальное положение по X
				ballY = HEIGHT / 2; // возращение мяча в начальное положение по Y
				time_last = inter_time + difficulty * 20; //изменение времени ожидания, перед началом нвого раунда
				flag_score = 1; // поднятие флага для вывода счета
			} else {
				BallPosition(); // функция для изменения положения мяча
			}
		} else {
			if (ballY < player2 || ballY > (player2 + plat_lenght)) {
				score2_right++; // увеличение счета 2 игрока
				ballX = WIDTH / 2; // возращение мяча в начальное положение по X
				ballY = HEIGHT / 2; // возращение мяча в начальное положение по Y
				time_last = inter_time + difficulty * 20; //изменение времени ожидания, перед началом нвого раунда
				flag_score = 1; // поднятие флага для вывода счета
			} else {
				BallPosition(); // функция для изменения положения мяча
			}
		}
	} else {
		BallPosition(); // функция для изменения положения мяча
	}

	if ((10 * difficulty + inter_time) <= time_last) { // условие для временного вывода счета перед началом нового раунда
		flag_score = 1;
	} else {
		flag_score = 0;
	}
}

void scorer() { // функция для вывода счета игроков
	char buffer1[3]; //Буфер для вывода счета в строку 2
	char buffer2[3]; //Буфер для вывода счета в строку 1

	sprintf(buffer1, "%d", score1_left); // преобразование числа в строку
	sprintf(buffer2, "%d", score2_right);

	if (score1_left == 10) { // при счете игрока 2 == 10 сдвигаем число для лучшего отображения
		shift1 = 3; //Сдвиг увеличивается с 0 до 3 пикселей
	}
	if (score2_right == 10) { // при счете игрока 2 == 10 сдвигаем число для лучшего отображения
		shift2 = 3; //Сдвиг увеличивается с 0 до 3 пикселей
	}

	if (flag_score) { // вывод счета на экран
		disp1color_printf(6 - shift1, 4, FONTID_6X8M, buffer2);
		disp1color_printf(21 - shift2, 4, FONTID_6X8M, buffer1);
	}
}


void BallPosition() { // функция для изменения положения мяча
	uint8_t player1, player2; // // Реальное положение платформы игрока 1 и 2
	player1 = player1_left();
	player2 = player2_right();
	// условие движения мяча по полю
	if (inter_time >= time_last) { // когда время на таймере больше заданного значения, изменяем положение мяча
		ballX = ballX + ballSpeedX; // смещение координаты мяча по оси Х
		ballY = ballY + ballSpeedY; // смещение координаты мяча по оси Х
		if (ballX == 0 || ballX == 31) { // находится ли мяч у левой и правой границы
			ballSpeedX = -ballSpeedX; //изменение смешения шарика на обратное
		} else if (ballX == 1 && (ballY >= player1 && ballY <= (player1 + plat_lenght))) { //касается ли мяч платформы игрока 1
			ballSpeedX = -ballSpeedX; //изменение смешения шарика на обратное
		} else if (ballX == 30 && (ballY >= player2 && ballY <= (player2 + plat_lenght))){ //касается ли мяч платформы игрока 2
			ballSpeedX = -ballSpeedX; //изменение смешения шарика на обратное
		}
		if (ballY == 1 || ballY == 14) { // находится ли мяч у нижней и верхней стенки
			ballSpeedY = -ballSpeedY; //изменение смешения шарика на обратное
		}
		//добавление шага по времени к последнему времени таймера, для задания частоты изменения положения мяча
		time_last = inter_time + difficulty;
	}
}

float expFilter1(float inputValue) { //экспоненциальный фильтр для потенциометра 1
	static float FiltVal = 0; //переменная запоминающая прошлое значения с потенциометра
	FiltVal += (inputValue - FiltVal) * k; //изменение нового значения потенциометра по коэффициенту
	return FiltVal; //возращение нового значения с потенциометра
}

float expFilter2(float inputValue) { //экспоненциальный фильтр для потенциометра 2
	static float FiltVal = 0; //переменная запоминающая прошлое значения с потенциометра
	FiltVal += (inputValue - FiltVal) * k; //изменение нового значения потенциометра по коэффициенту
	return FiltVal; //возращение нового значения с потенциометра
}

void Potentiometers() { // функция управления потенциометрами
	HAL_ADC_Start(&hadc1); //включение АЦП для оцифровки аналогого сигнала 1
	HAL_ADC_Start(&hadc2); //включение АЦП для оцифровки аналогого сигнала 2

	HAL_ADC_PollForConversion(&hadc1, 5); //ожидание окончания преобразования АЦП по истечению 5 тактов
	HAL_ADC_PollForConversion(&hadc2, 5); //ожидание окончания преобразования АЦП по истечению 5 тактов

	adc_value1 = HAL_ADC_GetValue(&hadc1); //получение значения с потенциометра 1
	adc_value2 = HAL_ADC_GetValue(&hadc2); //получение значения с потенциометра 2

	adc_filt_val1 = expFilter1(adc_value1); //получение фильтрованного значения с потенциометра 1
	adc_filt_val2 = expFilter2(adc_value2); //получение фильтрованного значения с потенциометра 1


	HAL_ADC_Stop(&hadc1); //остановка АЦП преобразований
	HAL_ADC_Stop(&hadc2); //остановка АЦП преобразований

	//пропорциональное изменение значения с потенциометра в значения от 0 до 9
	value1 = adc_filt_val1 * (14 - plat_lenght) / 4096;
	value2 = adc_filt_val2 * (14 - plat_lenght) / 4096;
}

void disp_row(uint8_t row) { //заполнение буфера матрицы

	if (row == 0) {

		for (uint8_t i = 0; i < 6; i++) {
			HAL_SPI_Transmit(&hspi1,(uint8_t*) &data1, 16, 10);
		}

		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	}
	if (row == 1) {

		for (uint8_t i = 0; i < 6; i++) {
			HAL_SPI_Transmit(&hspi1,(uint8_t*) &data2, 16, 10);
		}

		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	}

	if (row == 2) {

		for (uint8_t i = 0; i < 6; i++) {
			HAL_SPI_Transmit(&hspi1,(uint8_t*) &data3, 16, 10);
		}

		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	}

	if (row == 3) {

		for (uint8_t i = 0; i < 6; i++) {
			HAL_SPI_Transmit(&hspi1,(uint8_t*) &data4, 16, 10);
		}

		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_SET);
	for (uint16_t x = 0; x <= 300; x++) {
	};
	HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_RESET);
}

void show_screen() { //отрисовка экрана игры: мяча и стенок
	disp1color_DrawPixel(ballX, ballY, 1); //отсовка мяча
	disp1color_DrawLine(0, 0, 31, 0); //отрисовка верхней стены
	disp1color_DrawLine(0, 15, 31, 15); // отрисовка нижней стены

	disp1color_UpdateFromBuff(); //обновленеи буфера
	prepare_data(); //подготовка данных для заполнения
	for (uint8_t i = 0; i < 20; i++) { //заполнение каждого пикселя светодиодной панели
		disp_row(0);
		disp_row(1);
		disp_row(2);
		disp_row(3);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1); //включение  таймера

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		disp1color_FillScreenbuff(0); //стирание прошлых данных матрицы

		Potentiometers(); // функция управления потенциометрами

		game(); // функция для реализации игрового процесса

		scorer(); // функция для вывода счета игроков

		show_screen(); //отрисовка экрана игры: мяча и стенок
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nOE_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, B_Pin|A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nOE_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = nOE_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Pin A_Pin */
  GPIO_InitStruct.Pin = B_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
