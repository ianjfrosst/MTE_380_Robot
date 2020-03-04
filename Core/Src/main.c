/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);

}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t NewDatReady = 0;
  uint32_t LoopNb;

  // Wait until it finished
  // use timeout to avoid deadlock
  if (Status == VL53L0X_ERROR_NONE) {
    LoopNb = 0;
    do {
      Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
      if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
        break;
      }
      LoopNb = LoopNb + 1;
      VL53L0X_PollingDelay(Dev);
    } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

    if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
      Status = VL53L0X_ERROR_TIME_OUT;
    }
  }

  return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint32_t StopCompleted = 0;
  uint32_t LoopNb;

  // Wait until it finished
  // use timeout to avoid deadlock
  if (Status == VL53L0X_ERROR_NONE) {
    LoopNb = 0;
    do {
      Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
      if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
        break;
      }
      LoopNb = LoopNb + 1;
      VL53L0X_PollingDelay(Dev);
    } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

    if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
      Status = VL53L0X_ERROR_TIME_OUT;
    }
  }

  return Status;
}

VL53L0X_Error rangingTest(VL53L0X_Dev_t* pMyDevice) {
  VL53L0X_RangingMeasurementData_t RangingMeasurementData;
  VL53L0X_RangingMeasurementData_t* pRangingMeasurementData =
      &RangingMeasurementData;
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Call of VL53L0X_StaticInit\n");
    Status = VL53L0X_StaticInit(pMyDevice);  // Device Initialization
    // StaticInit will set interrupt by default
    print_pal_error(Status);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Call of VL53L0X_PerformRefCalibration\n");
    Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings,
                                           &PhaseCal);  // Device Initialization
    print_pal_error(Status);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Call of VL53L0X_PerformRefSpadManagement\n");
    Status = VL53L0X_PerformRefSpadManagement(
        pMyDevice, &refSpadCount, &isApertureSpads);  // Device Initialization
    print_pal_error(Status);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Call of VL53L0X_SetDeviceMode\n");
    Status = VL53L0X_SetDeviceMode(
        pMyDevice,
        VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);  // Setup in single ranging mode
    print_pal_error(Status);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Call of VL53L0X_StartMeasurement\n");
    Status = VL53L0X_StartMeasurement(pMyDevice);
    print_pal_error(Status);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    uint32_t measurement;
    uint32_t no_of_measurements = 256;

    // uint16_t* pResults =
    //     (uint16_t*)malloc(sizeof(uint16_t) * no_of_measurements);

    uint16_t range;

    // for (measurement = 0; measurement < no_of_measurements; measurement++) {
    do {
      Status = WaitMeasurementDataReady(pMyDevice);

      if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetRangingMeasurementData(pMyDevice,
                                                   pRangingMeasurementData);

        // *(pResults + measurement) = pRangingMeasurementData->RangeMilliMeter;
        range = pRangingMeasurementData->RangeMilliMeter;
        printf("In loop measurement %lu: %d\n", measurement,
               pRangingMeasurementData->RangeMilliMeter);

        // Clear the interrupt
        VL53L0X_ClearInterruptMask(
            pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        VL53L0X_PollingDelay(pMyDevice);
      } else {
        break;
      }
    } while (range > 200);

    // if (Status == VL53L0X_ERROR_NONE) {
    //   for (measurement = 0; measurement < no_of_measurements; measurement++) {
    //     printf("measurement %lu: %d\n", measurement, *(pResults + measurement));
    //   }
    // }

    // free(pResults);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Call of VL53L0X_StopMeasurement\n");
    Status = VL53L0X_StopMeasurement(pMyDevice);
  }

  if (Status == VL53L0X_ERROR_NONE) {
    printf("Wait Stop to be competed\n");
    Status = WaitStopCompleted(pMyDevice);
  }

  if (Status == VL53L0X_ERROR_NONE)
    Status = VL53L0X_ClearInterruptMask(
        pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

  return Status;
}

void drive_dir(uint16_t pwm, GPIO_PinState dir1, GPIO_PinState dir2) {
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, dir1);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, dir1);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, dir2);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, dir2);
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


  /* Enable I-Cache---------------------------------------------------------*/
  // SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  // SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_Dev_t MyDevice;
  VL53L0X_Dev_t* pMyDevice = &MyDevice;

  pMyDevice->I2cDevAddr      = 0x52;
  pMyDevice->comms_type      =  1;
  pMyDevice->comms_speed_khz =  400;

  if (Status == VL53L0X_ERROR_NONE) {
    Status = VL53L0X_DataInit(&MyDevice);  // Data initialization
    print_pal_error(Status);
  }

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  drive_dir(16000, GPIO_PIN_RESET, GPIO_PIN_RESET);

  Status = rangingTest(pMyDevice);

  drive_dir(0, GPIO_PIN_RESET, GPIO_PIN_RESET);

  HAL_Delay(200);

  drive_dir(16000, GPIO_PIN_SET, GPIO_PIN_SET);

  HAL_Delay(1000);

  drive_dir(0, GPIO_PIN_RESET, GPIO_PIN_SET);

  HAL_Delay(200);

  drive_dir(16000, GPIO_PIN_RESET, GPIO_PIN_SET);

  HAL_Delay(1000);

  drive_dir(0, GPIO_PIN_SET, GPIO_PIN_SET);

  while(1);

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
  printf("Assertion failed at %s:%ld\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
