/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "sensors.h"
#include <stdio.h>
#include <string.h>
#include "black_eyes.h"
#include "rgb_led.h"
#include "buzzer.h"
#include "servo.h"
#include "mode_switch.h"
#include "state_estimation.h"
#include "state.h"
#include "batt_sense.h"
#include "telemetry.h"
#include "flash.h"
#include "control.h"
#include "packets.h"
#include "btn.h"
#include "timing.h"
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
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

OSPI_HandleTypeDef hospi1;
MDMA_HandleTypeDef hmdma_octospi1_fifo_th;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
// Custom driver contexts
rgb_led_t led0 = {
    .handle = &htim3,
    .channel_r = TIM_CHANNEL_1,
    .channel_g = TIM_CHANNEL_2,
    .channel_b = TIM_CHANNEL_3,
	.brightness_shift = 2
};
rgb_led_t led1 = {
    .handle = &htim4,
    .channel_r = TIM_CHANNEL_1,
    .channel_g = TIM_CHANNEL_2,
    .channel_b = TIM_CHANNEL_3,
	.brightness_shift = 2
};
buzzer_t buzzer = {
    .handle = &htim16,
    .tim_freq = 64000000,
    .seq = 0,
    .seq_len = 0,
    .seq_idx = 0,
    .beep_start = 0,
    .seq_playing = 0
};

const buzzer_beep_t seq_startup_3[] = {
	{4186, 250},
	{4435, 250},
	{4699, 500}
};

const buzzer_beep_t seq_mode_1_1[] = {
		{4186, 250}
};

const buzzer_beep_t seq_mode_2_3[] = {
		{4186, 250},
		{0, 250},
		{4186, 250}
};

const buzzer_beep_t seq_mode_3_5[] = {
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250}
};

const buzzer_beep_t seq_mode_4_7[] = {
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250}
};

const buzzer_beep_t seq_mode_5_9[] = {
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250}
};

const buzzer_beep_t seq_mode_6_11[] = {
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250}
};

const buzzer_beep_t seq_mode_7_13[] = {
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250},
		{0, 250},
		{4186, 250}
};

const buzzer_beep_t seq_launch_detect_beep_5[] = {
		{4186, 100},
		{0, 50},
		{4186, 100},
		{0, 50},
		{4186, 100}
};

const buzzer_beep_t seq_apogee_detect_2[] = {
	{4186, 200},
//	{4435, 200},
	{4699, 200}
};


__attribute__((section(".bdma_buf"))) volatile uint16_t servo_dma_buf[1];
servo_t servo = {
	.tim_handle = &htim17,
	.en_gpio_port = SERVO_EN_GPIO_Port,
	.en_pin = SERVO_EN_Pin,
	.adc_handle = &hadc3,
	.dma_buf = servo_dma_buf,
//	.duty_retracted = 1000,
//	.duty_extended = 2370
	.duty_retracted = 620,
	.duty_extended = 1800
};
uint8_t endpoint_selected = 0;

batt_sense_t batt_sense = {
	.adc_handle = &hadc1,
	.dma_buf = {0, 0}
};

telemetry_t telemetry = {
    .handle = &huart4
};

flash_t flash = {
    .hospi = &hospi1,
};

btn_t btns[4]; // BTN0 to BTN4 drivers
mode_switch_t mode_switch;

extern uint8_t baro_ready;
extern uint8_t mag_ready;
extern uint8_t imu_ready;

// Control system ticks
uint8_t tick_100Hz = 0; // bool
uint8_t tick_500Hz = 0; // bool

// Mode selection
enum Mode {
	START = -1,
	IDLE = 0,
	TEST_UI = 1,
	TEST_SIMULINK = 2,
	TEST_SERVO = 3,
	TEST_SENSORS = 4,
	TEST_FLASH = 5,
	TEST_CONTROL = 6,
	LAUNCH_DETECT = 7
};
enum Mode mode = -1;
enum Mode mode_prev = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_BDMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// printf UART
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t millisec);

// State transitions
void mode_transition_handler(enum Mode prev, enum Mode curr);
void mode_current_handler(enum Mode curr);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_BDMA_Init();
  MX_MDMA_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_OCTOSPI1_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //printf("System reset\r\n");
    telemetry_init(&telemetry);
    telemetry_log(LOG_LVL_DEBUG, "System reset\r\n");

    black_eye_set(0, 0);
    black_eye_set(1, 0);

    rgb_led_init(&led0);
    rgb_led_init(&led1);
    rgb_led_set(&led0, 0x006000);
    rgb_led_set(&led1, 0x0000A0);

    btn_init(&btns[0], BTN_0_GPIO_Port, BTN_0_Pin);
	btn_init(&btns[1], BTN_1_GPIO_Port, BTN_1_Pin);
	btn_init(&btns[2], BTN_2_GPIO_Port, BTN_2_Pin);
	btn_init(&btns[3], BTN_3_GPIO_Port, BTN_3_Pin);

	mode_switch_init(&mode_switch);
	mode = mode_switch_get(&mode_switch);

    batt_sense_init(&batt_sense);

    buzzer_init(&buzzer);

    servo_init(&servo);
    servo_set_duty(&servo, 500);
    servo_enable(&servo, 0);

    sensors_init();

    flash_init(&flash);

  	HAL_TIM_Base_Start_IT(&htim7); // start 100 Hz
    HAL_TIM_Base_Start_IT(&htim6); // start 500 Hz
    HAL_TIM_Base_Start(&htim2); // start microsecond counter

	telemetry_log(LOG_LVL_DEBUG, "Init done\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  telemetry_rx_poll(&global_state);

	  flash_process();

	  if (tick_100Hz) {
		  tick_100Hz = 0;

		  batt_sense_get(&batt_sense, &(global_state.batt_v), &(global_state.batt_i)); // Read power info

		  servo_get_angle(&servo); // Servo feedback

		  // Button driver state updates for current state and edge detection
		  btn_update(&btns[0]);
		  btn_update(&btns[1]);
		  btn_update(&btns[2]);
		  btn_update(&btns[3]);

		  buzzer_update(&buzzer); // buzzer

		  global_state.t = HAL_GetTick(); // t in ms seems reasonable

		  mode_switch_update(&mode_switch); // driver state updates

		  // Mode selection
		  enum Mode mode_switch_val = (enum Mode)mode_switch_get(&mode_switch); // mode selection from physical switch
		  enum Mode mode_selected = STATE_FLAG_GET(FLAG_MODE_OVERRIDE_EN) ? (enum Mode)global_state.mode_override : mode_switch_val; // choose between control panel override and switch

		  if (mode_selected != mode_prev) {
			  mode_transition_handler(mode_prev, mode_selected);
			  mode_prev = mode_selected;
		  }
		  mode = mode_selected;
		  mode_current_handler(mode);

		  if (mode != TEST_FLASH) {
			  telemetry_packet(&global_state); // sending telemetry last in case a log needs to be sent before (we will drop a frame ig)
		  }
	  }

	  if (tick_500Hz) {
		  tick_500Hz = 0;

		  global_state.t = HAL_GetTick(); // [ms]
		  global_state.elapsed_t = global_state.t - global_state.launch_t;

		  if (mode != TEST_SIMULINK) {
			  state_estimation();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 64;
  RCC_OscInitStruct.PLL.PLLR = 16;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV8;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 32;
  PeriphClkInitStruct.PLL2.PLL2Q = 32;
  PeriphClkInitStruct.PLL2.PLL2R = 8;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = ADC3_OVERSAMPLING_RATIO_2;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_1LINE;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 639;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 639;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  htim7.Init.Prescaler = 63;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 16000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 63;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 3002;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 2000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EYE_0_Pin|EYE_1_Pin|SERVO_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BARO_NSS_GPIO_Port, BARO_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAG_NSS_GPIO_Port, MAG_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_NSS_GPIO_Port, IMU_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MODE_C_GPIO_Port, MODE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EYE_0_Pin EYE_1_Pin MAG_NSS_Pin */
  GPIO_InitStruct.Pin = EYE_0_Pin|EYE_1_Pin|MAG_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BARO_NSS_Pin */
  GPIO_InitStruct.Pin = BARO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BARO_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BARO_INT_Pin */
  GPIO_InitStruct.Pin = BARO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BARO_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG_INT_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MAG_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_NSS_Pin */
  GPIO_InitStruct.Pin = IMU_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_INT1_Pin IMU_INT2_Pin BTN_0_Pin BTN_1_Pin
                           BTN_2_Pin BTN_3_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin|IMU_INT2_Pin|BTN_0_Pin|BTN_1_Pin
                          |BTN_2_Pin|BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_1_Pin MODE_2_Pin MODE_4_Pin */
  GPIO_InitStruct.Pin = MODE_1_Pin|MODE_2_Pin|MODE_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_C_Pin */
  GPIO_InitStruct.Pin = MODE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODE_C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SERVO_EN_Pin */
  GPIO_InitStruct.Pin = SERVO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(BTN_0_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BTN_0_EXTI_IRQn);

  HAL_NVIC_SetPriority(BTN_1_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BTN_1_EXTI_IRQn);

  HAL_NVIC_SetPriority(BTN_2_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BTN_2_EXTI_IRQn);

  HAL_NVIC_SetPriority(BTN_3_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BTN_3_EXTI_IRQn);

  HAL_NVIC_SetPriority(BARO_INT_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BARO_INT_EXTI_IRQn);

  HAL_NVIC_SetPriority(IMU_INT1_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(IMU_INT1_EXTI_IRQn);

  HAL_NVIC_SetPriority(MAG_INT_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MAG_INT_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	printf("EXTI:%u\r\n", GPIO_Pin);
	if (GPIO_Pin == BARO_INT_Pin) {
		baro_int_drdy_handler();
	} else if(GPIO_Pin == IMU_INT1_Pin || GPIO_Pin == IMU_INT2_Pin) {
		imu_int_drdy_handler();
	} else if(GPIO_Pin == MAG_INT_Pin) {
		mag_int_drdy_handler();
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {
		baro_spi_callback();
	} else if (hspi->Instance == SPI2) {
		imu_spi_callback();
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI4) {
		mag_spi_callback();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) { // 100 Hz
		tick_100Hz = 1;
	}
	if (htim->Instance == TIM6) { // 500 Hz (state estimation)
		tick_500Hz = 1;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
        telemetry_log(LOG_LVL_ERROR, "HAL_UART_ErrorCallback UART4: %lu", huart->ErrorCode);
    }
}

void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi) {
	if (hospi->Instance == OCTOSPI1) {
		telemetry_log(LOG_LVL_ERROR, "HAL_OSPI_ErrorCallback OCTOSPI1: %lu", hospi->ErrorCode);
	}
}

// State
void mode_transition_handler(enum Mode prev, enum Mode curr) {
//	printf("Mode transition from %u to %u\r\n", prev, curr);
	telemetry_log(LOG_LVL_INFO, "Mode transition from %d to %d\r\n", prev, curr);

	// Handle exit from previous mode
	switch (prev) {
		case TEST_SERVO:
			servo_enable(&servo, 0); // Disable servo power
			break;
		case TEST_UI:
			buzzer_set(&buzzer, 0);  // Ensure buzzer is off on exit
			break;
//		case TEST_SIMULINK:
//			HAL_NVIC_EnableIRQ(BARO_INT_EXTI_IRQn);
//			HAL_NVIC_EnableIRQ(IMU_INT1_EXTI_IRQn); // IMU both accel and omega
//			HAL_NVIC_EnableIRQ(MAG_INT_EXTI_IRQn);
//			servo_enable(&servo, 0); // Disable servo power
//			break;
		default:
			break;
	}

	// Play (or dont) buzzer for mode transition
#define BUZZER_ON_TRANSITION
#ifdef BUZZER_ON_TRANSITION
	switch (curr) {
		case 0:
			buzzer_play_sequence(&buzzer, seq_startup_3, 3);
			break;
		case 1:
			buzzer_play_sequence(&buzzer, seq_mode_1_1, 1);
			break;
		case 2:
			buzzer_play_sequence(&buzzer, seq_mode_2_3, 3);
			break;
		case 3:
			buzzer_play_sequence(&buzzer, seq_mode_3_5, 5);
			break;
		case 4:
			buzzer_play_sequence(&buzzer, seq_mode_4_7, 7);
			break;
		case 5:
			buzzer_play_sequence(&buzzer, seq_mode_5_9, 9);
			break;
		case 6:
			buzzer_play_sequence(&buzzer, seq_mode_6_11, 11);
			break;
		case 7:
			buzzer_play_sequence(&buzzer, seq_mode_7_13, 13);
			break;
		default:
			break;
	}
#endif

	// Handle entry to new mode
	switch (curr) {
		case IDLE:
			//buzzer_play_sequence(&buzzer, seq_startup_3, 3);
			telemetry_log(LOG_LVL_DEBUG, "Mode 0 IDLE\r\n");
			break;
		case TEST_UI:
			//buzzer_play_sequence(&buzzer, seq_mode_1_1, 1);
			telemetry_log(LOG_LVL_DEBUG, "Mode 1 TEST_UI\r\n");
			break;
//		case TEST_SIMULINK:
//			buzzer_play_sequence(&buzzer, seq_mode_2_3, 3);
// 			telemetry_log(LOG_LVL_DEBUG, "Mode 2 TEST_SIMULINK\r\n");
//			HAL_NVIC_DisableIRQ(BARO_INT_EXTI_IRQn);
//			HAL_NVIC_DisableIRQ(IMU_INT1_EXTI_IRQn); // IMU both accel and omega
//			HAL_NVIC_DisableIRQ(MAG_INT_EXTI_IRQn);
//			servo_enable(&servo, 1); // Enable servo power
//			break;
		case TEST_SERVO:
			//buzzer_play_sequence(&buzzer, seq_mode_3_5, 5);
			telemetry_log(LOG_LVL_DEBUG, "Mode 3 TEST_SERVO\r\n");
			servo_enable(&servo, 1); // Enable servo power
			servo_set_deployment(&servo, 0);
			break;
		case TEST_SENSORS:
			//buzzer_play_sequence(&buzzer, seq_mode_4_7, 7);
			telemetry_log(LOG_LVL_DEBUG, "Mode 4 TEST_SENSORS\r\n");
			break;
		case TEST_FLASH:
			//buzzer_play_sequence(&buzzer, seq_mode_5_9, 9);
			telemetry_log(LOG_LVL_DEBUG, "Mode 5 TEST_FLASH\r\n");

//			uint32_t flash_id = flash_read_jedec_id(&flash);
//			if (flash_id == W25Q128JV_JEDEC_ID) {
//				printf("JEDEC ID good: 0x%06lX\r\n", flash_id);
//				rgb_led_set(&led0, 0x00FF00); // status LED green
//			} else {
//				printf("JEDEC ID error: 0x%06lX (Expected 0x%06X)\r\n", flash_id, W25Q128JV_JEDEC_ID);
//				rgb_led_set(&led0, 0xFF0000); // status LED red
//			}
//
//			// Test flash erased
//			if (flash_check_erased(&flash)) { // if erased
//				printf("Flash is blank. Ready for flight!\r\n");
//				rgb_led_set(&led0, 0x00FF00); // status LED green
//			} else { // else not erased
//				printf("Flash is NOT erased! Must do so before flight.\r\n");
//				rgb_led_set(&led0, 0xFF0000); // status LED red to indicate needs erasing
//			}

			break;
		case TEST_CONTROL:
			//buzzer_play_sequence(&buzzer, seq_mode_6_11, 11);
			telemetry_log(LOG_LVL_DEBUG, "Mode 6 TEST_CONTROL\r\n");
			break;
		case LAUNCH_DETECT:
			//buzzer_play_sequence(&buzzer, seq_mode_7_13, 13);
			telemetry_log(LOG_LVL_DEBUG, "Mode 7 LAUNCH_DETECT\r\n");
			servo_enable(&servo, 1); // Enable servo power
			servo_set_deployment(&servo, 0);
			lockouts_init(); // TODO maybe also when launch is detected
			break;
		default:
			break;
	}
}

void mode_current_handler(enum Mode curr) {
	switch (curr) {
		case IDLE: // 0
			break;
		case TEST_UI: // 1

			// buzzer test
			if (btn_get_edge(&btns[0]) == 1)
			{
				buzzer_play_sequence(&buzzer, seq_startup_3, 3);
			}

			// btn driver test
			if (btn_get_edge(&btns[1]) == 1)
			{
				telemetry_log(LOG_LVL_DEBUG, "BTN1 pressed");
			}
			if (btn_get_edge(&btns[1]) == -1)
			{
				telemetry_log(LOG_LVL_DEBUG, "BTN1 released");
			}
			break;

//		case TEST_SIMULINK: // 2
//			if (cobs_uart.rx_ready) {
//				if (hil_parse_rx(&cobs_uart, &global_state)) { // Parse and map data
//					imu_ready = 1;
//					mag_ready = 1;
//					baro_ready = 1;
//
//					state_estimation(0.01f); // update alt_agl vel_z and accel_b
//					control_update(0.01f); // 100Hz dt
//					servo_set_deployment(&servo, global_state.output);
//
//					hil_send(&cobs_uart, &global_state); // Encode and return to simulink
//				}
//
//				// Reset RX state for the next packet
//				cobs_uart.rx_idx = 0;
//				cobs_uart.rx_ready = 0;
//			}
//			break;

		case TEST_SERVO: // 3

			if (STATE_FLAG_GET(FLAG_SERVO_OVERRIDE_EN)) { // control panel override
				servo_set_angle(&servo, global_state.servo_cmd_override);
			}
			else if (btn_get_edge(&btns[0]) == 1) // BTN0 goes to retracted endpoint
			{
				servo_set_duty(&servo, servo.duty_retracted);
				endpoint_selected = 0;
				telemetry_log(LOG_LVL_INFO, "Retracted");
			}
			else if (btn_get_edge(&btns[1]) == 1) // BTN1 goes to extended endpoint
			{
				servo_set_duty(&servo, servo.duty_extended);
				endpoint_selected = 1;
				telemetry_log(LOG_LVL_INFO, "Extended");
			}
			else if (btn_get(&btns[2])) // BTN2 decreases deployment of selected endpoint
			{
				if (endpoint_selected == 0) { // retracted
					servo.duty_retracted -= 1;
					servo_set_duty(&servo, servo.duty_retracted);
//					printf("retracted:%lu\r\n", servo.duty_retracted);
					telemetry_log(LOG_LVL_INFO, "retracted:%lu", servo.duty_retracted);
				} else if (endpoint_selected == 1) { // extended
					servo.duty_extended -= 1;
					servo_set_duty(&servo, servo.duty_extended);
//					printf("extended:%lu\r\n", servo.duty_extended);
					telemetry_log(LOG_LVL_INFO, "extended:%lu", servo.duty_extended);
				}
			}
			else if (btn_get(&btns[3])) // BTN3 increases deployment of selected endpoint
			{
				if (endpoint_selected == 0) { // retracted
					servo.duty_retracted += 1;
					servo_set_duty(&servo, servo.duty_retracted);
//					printf("retracted:%lu\r\n", servo.duty_retracted);
					telemetry_log(LOG_LVL_INFO, "retracted:%lu", servo.duty_retracted);
				} else if (endpoint_selected == 1) { // extended
					servo.duty_extended += 1;
					servo_set_duty(&servo, servo.duty_extended);
//					printf("extended:%lu\r\n", servo.duty_extended);
					telemetry_log(LOG_LVL_INFO, "extended:%lu", servo.duty_extended);
				}
			}

			break;

		case TEST_SENSORS: // 4

			printf("mag_x:%f,mag_y:%f,mag_z:%f\r\n",
								global_state.mag_mgauss[0], global_state.mag_mgauss[1], global_state.mag_mgauss[2]);
			break;

		case TEST_FLASH: // 5

			if (HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin)) // if BTN0 pressed print CSV over UART
			{
				uint32_t read_address = 0;
				flash_packet_t packet_r;

				// Print CSV Header
				printf("t,elapsed_t,flags,batt_v,batt_i,accel_ms2[0],accel_ms2[1],accel_ms2[2],omega_rads[0],omega_rads[1],omega_rads[2],mag_mgauss[0],mag_mgauss[1],mag_mgauss[2],pres_pa,accel_b[0],accel_b[1],accel_b[2],omega_b[0],omega_b[1],omega_b[2],mag_b[0],mag_b[1],mag_b[2],quat[0],quat[1],quat[2],quat[3],accel_e[0],accel_e[1],accel_e[2],p_ground,alt_agl,vel_z,predicted,output,p_contrib,i_contrib,servo_cmd,servo_fdbk\r\n");

				// read until end or when timestamp in packet is unwritten
				while (read_address < W25Q128JV_SIZE) {
					flash_read_blocking(read_address, (uint8_t*)&packet_r, sizeof(flash_packet_t));

					if (packet_r.t == 0xFFFFFFFF) { // timestamp won't be 0xFFFFFFFF until 50 days so this indicates end of flash writing
						printf("Timestamp 0xFFFFFFFF\r\n");
						// break;
					}

					// Print as CSV row
//							t   el  fla batv bati ax    ay   az   wx   wy   wz  magx magy magz pres abx  aby   abz  wbx  wby wbz  mbx  mby  mbz  q0   q1   q2   q3   aex  aey  aez  pg    alt velz pred out  p    i    cmd  fdbk
					printf("%lu,%lu,%lx,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
							packet_r.t,
							packet_r.elapsed_t,

							packet_r.flags,

							packet_r.batt_v,
							packet_r.batt_i,

							packet_r.accel_ms2[0], packet_r.accel_ms2[1], packet_r.accel_ms2[2],
							packet_r.omega_rads[0], packet_r.omega_rads[1], packet_r.omega_rads[2],
							packet_r.mag_mgauss[0], packet_r.mag_mgauss[1], packet_r.mag_mgauss[2],
							packet_r.pres_pa,

							packet_r.accel_b[0], packet_r.accel_b[1], packet_r.accel_b[2],
							packet_r.omega_b[0], packet_r.omega_b[1], packet_r.omega_b[2],
							packet_r.mag_b[0], packet_r.mag_b[1], packet_r.mag_b[2],

							packet_r.quat[0], packet_r.quat[1], packet_r.quat[2], packet_r.quat[3],
							packet_r.accel_e[0], packet_r.accel_e[1], packet_r.accel_e[2],

							packet_r.p_ground,
							packet_r.alt_agl,
							packet_r.vel_z,

							packet_r.predicted,
							packet_r.output,
							packet_r.p_contrib,
							packet_r.i_contrib,

							packet_r.servo_cmd,
							packet_r.servo_fdbk);

					read_address += W25Q_PAGE_SIZE;

					HAL_Delay(1);
				}
				printf("end of flash\r\n");
			}

			else if (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_SET) { // if BTN1 pressed test R/W
				// printf("Flash RW test\r\n");
				telemetry_log(LOG_LVL_INFO, "Flash RW test\r\n");

				rgb_led_set(&led1, 0x0000FF); // operating LED blue

				uint32_t test_address = 0x000000; // Sector 0

				flash_packet_t dummy_packet_w = {0}; // test packet to write
				flash_packet_t dummy_packet_r = {0}; // test packet to read back

				dummy_packet_w.t = 1234; // update write state time (first in struct)
				dummy_packet_w.servo_fdbk = 42.0f; // update write state servo fdbk (last in struct)

				// printf("Erase sector 0\r\n");
				telemetry_log(LOG_LVL_DEBUG, "Erase sector 0\r\n");
				flash_erase_sector(test_address);

				// printf("Write %u bytes\r\n", sizeof(flash_packet_t));
				telemetry_log(LOG_LVL_DEBUG, "Write %u bytes\r\n", sizeof(flash_packet_t));
				flash_write_blocking(test_address, (uint8_t*)&dummy_packet_w, sizeof(flash_packet_t));

				printf("Read back\r\n");
				telemetry_log(LOG_LVL_DEBUG, "Read back\r\n");
				memset(&dummy_packet_r, 0, sizeof(flash_packet_t));
				flash_read_blocking(test_address, (uint8_t*)&dummy_packet_r, sizeof(flash_packet_t));

				if (dummy_packet_r.t == dummy_packet_w.t && dummy_packet_r.servo_fdbk == dummy_packet_w.servo_fdbk) {
//					printf("RW test passed with t=%lu and servo_fdbk=%f\r\n", dummy_packet_r.t, dummy_packet_r.servo_fdbk);
					telemetry_log(LOG_LVL_INFO, "RW test passed with t=%lu and servo_fdbk=%f\r\n", dummy_packet_r.t, dummy_packet_r.servo_fdbk);
					rgb_led_set(&led1, 0x00FF00);
				} else {
//					printf("RW test failed with t=%lu and servo_fdbk=%f\r\n", dummy_packet_r.t, dummy_packet_r.servo_fdbk);
					telemetry_log(LOG_LVL_ERROR, "RW test failed with t=%lu and servo_fdbk=%f\r\n", dummy_packet_r.t, dummy_packet_r.servo_fdbk);
					rgb_led_set(&led1, 0xFF0000);
				}

				HAL_Delay(1000);
				rgb_led_set(&led1, 0x000000);
			}

			else if (HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_SET) { // if BTN2 pressed check if erased
				// Test flash erased
				// Turn off both LEDS
				rgb_led_set(&led0, 0x000000); // status LED
				rgb_led_set(&led1, 0x000000); // operating LED

				rgb_led_set(&led1, 0x0000FF); // Turn on LED1 blue (operating LED)
				if (flash_check_erased(&flash)) { // if erased
					printf("Flash is blank. Ready for flight!\r\n");
					telemetry_log(LOG_LVL_INFO, "Flash is blank. Ready for flight!\r\n");
					rgb_led_set(&led0, 0x00FF00); // status LED green
				} else { // else not erased
//					printf("Flash is NOT erased! Must do so before flight.\r\n");
					telemetry_log(LOG_LVL_WARNING, "Flash is NOT erased! Must do so before flight.\r\n");
					rgb_led_set(&led0, 0xFF0000); // status LED red to indicate needs erasing
				}
				rgb_led_set(&led1, 0x000000); // Turn off operating LED
				HAL_Delay(1000); // basically debounce
			}

			else { // erase flash if hold BTN3
				static uint16_t hold_cnt = 0; // long press hold counter [0.01s]

				if (HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) == GPIO_PIN_SET) { // BTN3 to erase
					hold_cnt++;

					if (hold_cnt == 1) {
//						printf("Hold BTN 3 for 2 seconds to wipe chip...\r\n");
						telemetry_log(LOG_LVL_INFO, "Hold BTN3 for 2 seconds to erase flash\r\n");
						rgb_led_set(&led1, 0xFF0000); // Red warning
					}

					if (hold_cnt >= 200) { // 200 = 2 second hold
//						printf("Erasing flash (this will take a minute)\r\n");
						telemetry_log(LOG_LVL_INFO, "Erasing flash (this will take a minute)\r\n");
						rgb_led_set(&led1, 0x0000FF);

						flash_erase_chip(&flash);

//						printf("Erase complete\r\n");
						telemetry_log(LOG_LVL_INFO, "Flash erased\r\n");
						rgb_led_set(&led0, 0x00FF00); // status LED green
						rgb_led_set(&led1, 0x00FF00); // op LED green
						HAL_Delay(1000);
						rgb_led_set(&led1, 0x000000); // op LED off
						hold_cnt = 0; // reset counter for next time
					}
				} else {
					if (hold_cnt > 0 && hold_cnt < 200) {
//						printf("Erase cancelled\r\n");
						telemetry_log(LOG_LVL_WARNING, "Erase cancelled\r\n");
						rgb_led_set(&led0, 0xFF0000); // status LED red
						rgb_led_set(&led1, 0x000000);
					}
					hold_cnt = 0;
				}
			}
			break;
		case TEST_CONTROL: // 6

			if (HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin))
			{
				STATE_FLAG_SET(FLAG_LAUNCHED);
			}

// 			Print IMU and body acceleration
//			printf("ax:%f,ay:%f,az:%f,ax_b:%f,ay_b:%f,az_b:%f\r\n",
//								state.accel_ms2[0], state.accel_ms2[1], state.accel_ms2[2],
//								state.accel_b[0], state.accel_b[1], state.accel_b[2]);

//			Print orientation only
//			printf("roll:%f,pitch:%f,yaw:%f\r\n", state.roll, state.pitch, state.yaw);

			// Print quaternion
//			printf("q:[%f,%f,%f,%f]\r\n", state.quat[0], state.quat[1], state.quat[2], state.quat[3]);

//			printf("accel_b:[%.3f,%.3f,%.3f]\r\n", state.accel_e[0], state.accel_e[1], state.accel_e[2]);

			break;
		case LAUNCH_DETECT: // 7
			// Control
			if (STATE_FLAG_GET(FLAG_LAUNCHED)) {
				control_update(); // 100Hz dt
				servo_set_deployment(&servo, global_state.output);
			}

			// Flash
			if (flash_should_add()) {
				flash_pkt_buf_add();
			}

			// Buzzer
			static uint32_t last_beep_t = 0;
			if (global_state.t - last_beep_t >= 10000) {
				if (!STATE_FLAG_GET(FLAG_LAUNCHED)){
					buzzer_play_sequence(&buzzer, seq_launch_detect_beep_5, 5);
				} else if (STATE_FLAG_GET(FLAG_APOGEE)) {
					buzzer_play_sequence(&buzzer, seq_apogee_detect_2, 2);
				}
				last_beep_t = global_state.t;
			}
			break;
		default:
			break;
	}
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	telemetry_log(LOG_LVL_ERROR, "Reached Error_Handler");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
