/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "mavlink_out.h"
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_LOOP_MS   5U    /* periode baca sensor: 5 ms = 200 Hz */
#define I2C_MAX_FAIL      10U   /* gagal baca berturut-turut sebelum I2C di-reset */
#define REINIT_PERIOD_MS  500U  /* jeda antar percobaan Init kalau sensor tidak terdeteksi */
#define BARO_LOOP_MS      50U   /* periode baca barometer: 50 ms = 20 Hz */
#define BARO_MAX_FAIL     20U   /* gagal baca berturut-turut sebelum BMP280 di-init ulang */
#define BARO_CAL_SAMPLES  20U   /* jumlah sampel rata-rata untuk tekanan permukaan (P0) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
MPU6050_t         MPU6050;
HAL_StatusTypeDef MPU6050_status = HAL_ERROR;
uint32_t          i2c_error_count = 0;   /* total error sejak boot */
static uint8_t    i2c_fail_streak = 0;   /* error berturut-turut */
static uint32_t   t_loop  = 0;
static uint32_t   t_retry = 0;

BMP280_t          BMP;
HAL_StatusTypeDef BMP_status = HAL_ERROR;
static uint8_t    baro_fail_streak = 0;
static uint32_t   t_baro  = 0;
static uint32_t   t_baro_retry = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();

  /* --- DISABLED FOR ST-LINK ONLY TESTING --- */
  // MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  /* Sensor harus DIAM dan datar sekitar 0,2 detik saat Init, karena sudut awal
   * filter diambil dari accelerometer. */
  MPU6050_status = MPU6050_Init(&hi2c1, &MPU6050);

  /* Barometer: init, lalu rata-rata beberapa sampel sebagai tekanan permukaan (P0).
   * Ketinggian BMP.altitude_f dihitung relatif terhadap P0 ini. Sekitar 0,6 detik. */
  BMP_status = BMP280_Init(&hi2c1, &BMP);
  if (BMP_status == HAL_OK)
  {
    BMP_status = BMP280_CalibrateGround(&hi2c1, &BMP, BARO_CAL_SAMPLES);
  }

  MavOut_Init();
  t_loop       = HAL_GetTick();
  t_retry      = HAL_GetTick();
  t_baro       = HAL_GetTick();
  t_baro_retry = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* --- INDICATOR LED (PC13) ---
     * Berkedip cepat jika MPU6050 OK, berkedip lambat jika MPU6050 gagal.
     * Agar kita tahu board tidak hang saat ST-Link dicabut/pasang. */
    static uint32_t t_led = 0;
    if (MPU6050_status == HAL_OK) {
        if ((now - t_led) > 100) { t_led = now; HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); }
    } else {
        if ((now - t_led) > 500) { t_led = now; HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); }
    }

    /* ---- Barometer (20 Hz) ---- */
    if ((now - t_baro) >= BARO_LOOP_MS)
    {
      t_baro = now;

      if (BMP_status == HAL_OK)
      {
        HAL_StatusTypeDef r = BMP280_Read(&hi2c1, &BMP);
        if (r == HAL_OK || r == HAL_BUSY)   /* HAL_BUSY = sampel belum siap, bukan error */
        {
          baro_fail_streak = 0;
        }
        else if (++baro_fail_streak >= BARO_MAX_FAIL)
        {
          baro_fail_streak = 0;
          BMP_status = HAL_ERROR;           /* paksa init ulang di bawah */
        }
      }
      else if ((now - t_baro_retry) >= REINIT_PERIOD_MS)
      {
        t_baro_retry = now;
        BMP_status = BMP280_Init(&hi2c1, &BMP);
        if (BMP_status == HAL_OK)
        {
          BMP_status = BMP280_CalibrateGround(&hi2c1, &BMP, BARO_CAL_SAMPLES);
        }
      }
    }

    /* ---- Sensor belum terdeteksi: coba Init ulang secara berkala ---- */
    if (MPU6050_status != HAL_OK)
    {
      if ((now - t_retry) >= REINIT_PERIOD_MS)
      {
        t_retry = now;
        MPU6050_status = MPU6050_Init(&hi2c1, &MPU6050);
        t_loop = HAL_GetTick();
      }
      continue;
    }

    /* ---- Baca sensor + update filter ---- */
    if ((now - t_loop) >= MPU6050_LOOP_MS)
    {
      t_loop = now;

      if (MPU6050_Read_All(&hi2c1, &MPU6050) == HAL_OK)
      {
        MPU6050_Update_Angle(&MPU6050);
        i2c_fail_streak = 0;
      }
      else
      {
        i2c_error_count++;
        if (++i2c_fail_streak >= I2C_MAX_FAIL)
        {
          /* Bus I2C kemungkinan nyangkut (flag BUSY): reset peripheral,
           * lalu paksa jalur Init ulang di atas. */
          HAL_I2C_DeInit(&hi2c1);
          MX_I2C1_Init();
          i2c_fail_streak = 0;
          MPU6050_status = HAL_ERROR;
        }
      }
    }

    /* ---- Kirim MAVLink lewat USB CDC (tidak blocking) ---- */
    /* --- DISABLED UNTUK ST-LINK ONLY TESTING --- */
    // MavOut_Service(&MPU6050, &BMP);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
