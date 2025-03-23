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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#include <string.h>
#include <stdio.h>
#include "lps22hh_reg.h"
#include "hts221_reg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define    BOOT_TIME        5 //ms
#define TX_BUF_DIM          1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// HTS221 (temp + hum)
static stmdev_ctx_t hts221_dev_ctx;

static int16_t data_raw_humidity;
static int16_t data_raw_temperature;
static float_t humidity_perc;
static float_t temperature_degC;
static uint8_t whoamI_hts221;
static uint8_t tx_buffer[1000];
// LPS22HH (press)
static stmdev_ctx_t lps22hh_dev_ctx;
static lps22hh_reg_t lps22hh_reg;
lps22hh_status_t status;

static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float_t pressure_hPa;
static float_t temperature_degC;
static uint8_t whoamI_lps22hh, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

typedef struct {
	  float x0;
	  float y0;
	  float x1;
	  float y1;
	} lin_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write_lps22hh(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read_lps22hh(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static int32_t platform_write_hts221(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static float linear_interpolation(lin_t *lin, int16_t x);
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
  SystemClock_Config();
  MX_USART2_UART_Init(); // Ensure this is called
  MX_I2C1_Init();

  /* USER CODE BEGIN Init */

  lps22hh_dev_ctx.write_reg = platform_write_lps22hh;
  lps22hh_dev_ctx.read_reg = platform_read_lps22hh;
  lps22hh_dev_ctx.mdelay = HAL_Delay;
  lps22hh_dev_ctx.handle = &hi2c1;

  //LPS22HH
  whoamI_lps22hh = 0;
  lps22hh_device_id_get(&lps22hh_dev_ctx, &whoamI_lps22hh);

  printf("%d\r\n", whoamI_lps22hh);
  if ( whoamI_lps22hh != LPS22HH_ID )
    while (1); /*manage here device not found */

  /* Restore default configuration */
  lps22hh_reset_set(&lps22hh_dev_ctx, PROPERTY_ENABLE);

  do {
    lps22hh_reset_get(&lps22hh_dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lps22hh_block_data_update_set(&lps22hh_dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lps22hh_data_rate_set(&lps22hh_dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

  // HTS221
  hts221_dev_ctx.write_reg = platform_write_hts221;
  hts221_dev_ctx.read_reg = platform_read_hts221;
  hts221_dev_ctx.mdelay = HAL_Delay;
  hts221_dev_ctx.handle = &hi2c1;
  whoamI_hts221 = 0;
  hts221_device_id_get(&hts221_dev_ctx, &whoamI_hts221);

  if ( whoamI_hts221 != HTS221_ID )
    while (1); /*manage here device not found */

  /* Read humidity calibration coefficient */
  lin_t lin_hum;
  hts221_hum_adc_point_0_get(&hts221_dev_ctx, &lin_hum.x0);
  hts221_hum_rh_point_0_get(&hts221_dev_ctx, &lin_hum.y0);
  hts221_hum_adc_point_1_get(&hts221_dev_ctx, &lin_hum.x1);
  hts221_hum_rh_point_1_get(&hts221_dev_ctx, &lin_hum.y1);
  /* Read temperature calibration coefficient */
  lin_t lin_temp;
  hts221_temp_adc_point_0_get(&hts221_dev_ctx, &lin_temp.x0);
  hts221_temp_deg_point_0_get(&hts221_dev_ctx, &lin_temp.y0);
  hts221_temp_adc_point_1_get(&hts221_dev_ctx, &lin_temp.x1);
  hts221_temp_deg_point_1_get(&hts221_dev_ctx, &lin_temp.y1);
  /* Enable Block Data Update */
  hts221_block_data_update_set(&hts221_dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  hts221_data_rate_set(&hts221_dev_ctx, HTS221_ODR_1Hz);
  /* Device power on */
  hts221_power_on_set(&hts221_dev_ctx, PROPERTY_ENABLE);


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// LPSS22HH
	/* Read output only if new value is available */
	lps22hh_read_reg(&lps22hh_dev_ctx, LPS22HH_STATUS, (uint8_t *)&lps22hh_reg, 1);

	if (status.p_da) {
	  memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
	  lps22hh_pressure_raw_get(&lps22hh_dev_ctx, &data_raw_pressure);
	  pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_pressure);
	  printf((char *)tx_buffer, sizeof(tx_buffer), "pressure [hPa]:%6.2f\r\n", pressure_hPa);
	  printf((char *)tx_buffer, strlen( (char const *)tx_buffer ) );
	}

	if (status.t_da) {
	  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&lps22hh_dev_ctx, &data_raw_temperature);
	  temperature_degC = lps22hh_from_lsb_to_celsius(
	                        data_raw_temperature );
	  printf((char *)tx_buffer, sizeof(tx_buffer), "temperature [degC]:%6.2f\r\n",
	          temperature_degC );
	  printf((char *)tx_buffer, strlen( (char const *)tx_buffer ) );
	}
	// HTS221
	hts221_status_reg_t status;
	    hts221_status_get(&hts221_dev_ctx, &status);

	    if (status.h_da) {
	      /* Read humidity data */
	      memset(&data_raw_humidity, 0x00, sizeof(int16_t));
	      hts221_humidity_raw_get(&hts221_dev_ctx, &data_raw_humidity);
	      humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity);

	      if (humidity_perc < 0) {
	        humidity_perc = 0;
	      }

	      if (humidity_perc > 100) {
	        humidity_perc = 100;
	      }

	      printf((char *)tx_buffer, sizeof(tx_buffer), "Humidity [%%]:%3.2f\r\n", humidity_perc);
	      printf((char *)tx_buffer);
	    }

	    if (status.t_da) {
	      /* Read temperature data */
	      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	      hts221_temperature_raw_get(&hts221_dev_ctx, &data_raw_temperature);
	      temperature_degC = linear_interpolation(&lin_temp,
	                                              data_raw_temperature);
	      printf((char *)tx_buffer, sizeof(tx_buffer), "Temperature [degC]:%6.2f\r\n",
	              temperature_degC );
	      printf((char *)tx_buffer);
	    }

	    HAL_Delay(1000); // 1s for the debug
	    //HAL_Delay(10000); // ms -> 10s
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end
	of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
	return ch;
}

// LPS22HH
static int32_t platform_write_lps22hh(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len){
  HAL_I2C_Mem_Write(handle, LPS22HH_ID, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}
static int32_t platform_read_lps22hh(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len){
  HAL_I2C_Mem_Read(handle, LPS22HH_ID, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}
// HTS221
static int32_t platform_write_hts221(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len){
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, HTS221_ID, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len){
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, HTS221_ID, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

static float linear_interpolation(lin_t *lin, int16_t x)
{
  return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
                                     (lin->x0 * lin->y1)))
         / (lin->x1 - lin->x0);
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
