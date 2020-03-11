/* USER CODE BEGIN Header */
/**
 * @file main.c
 * @author Ian Frosst
 * @brief Main program body
 * @version 0.1
 * @date 2020-03-06
 *
 * @copyright Copyright (c) 2020
 *
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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

void print_pal_error(VL53L0X_Error Status) {
  char buf[VL53L0X_MAX_STRING_LENGTH];
  VL53L0X_GetPalErrorString(Status, buf);
  printf("API Status: %i : %s\n", Status, buf);
}

void print_range_status(
    VL53L0X_RangingMeasurementData_t* pRangingMeasurementData) {
  char buf[VL53L0X_MAX_STRING_LENGTH];
  uint8_t RangeStatus;

  /*
   * New Range Status: data is valid when pRangingMeasurementData->RangeStatus =
   * 0
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

  printf("Call of VL53L0X_DataInit\n");
  Status = VL53L0X_DataInit(pMyDevice);  // Data initialization
  print_pal_error(Status);

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
    uint16_t range;

    do {
      Status = WaitMeasurementDataReady(pMyDevice);

      if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetRangingMeasurementData(pMyDevice,
                                                   pRangingMeasurementData);

        range = pRangingMeasurementData->RangeMilliMeter;
        printf("data: %d\r\n", range);

        // Clear the interrupt
        VL53L0X_ClearInterruptMask(
            pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        VL53L0X_PollingDelay(pMyDevice);
      } else {
        break;
      }
    } while (true);
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

void drive_dir(uint16_t pwm, bool left, bool right) {
  // Front left
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, !left);

  // Back left
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, !left);

  // Front right
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, right);

  // Back right
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, right);
}

#define DO_SETUP
#define VL53L0X_ADDRESS (0x52)
#define VL53L0X_NEW_ADDRESS (0x54)

VL53L0X_Dev_t side_range = {
    .I2cDevAddr = VL53L0X_ADDRESS,
};

VL53L0X_Dev_t front_range = {
#ifdef DO_SETUP
    .I2cDevAddr = VL53L0X_ADDRESS,
#else
    .I2cDevAddr = VL53L0X_NEW_ADDRESS,
#endif
};

void range_init(void) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;

  // Shutdown the side sensor
  HAL_GPIO_WritePin(SIDE_XSHUT_GPIO_Port, SIDE_XSHUT_Pin, GPIO_PIN_RESET);

#ifdef DO_SETUP
  // This will fail if the address has already been changed, but that's ok
  VL53L0X_SetDeviceAddress(&front_range, VL53L0X_NEW_ADDRESS);
  front_range.I2cDevAddr = VL53L0X_NEW_ADDRESS;
#endif

  // Reenable side sensor
  HAL_GPIO_WritePin(SIDE_XSHUT_GPIO_Port, SIDE_XSHUT_Pin, GPIO_PIN_SET);

  Status = VL53L0X_DataInit(&front_range);
  if (Status != VL53L0X_ERROR_NONE) {
    printf("front DataInit\r\n");
    print_pal_error(Status);
  }

  Status = VL53L0X_StaticInit(&front_range);
  if (Status != VL53L0X_ERROR_NONE) {
    printf("front StaticInit\r\n");
    print_pal_error(Status);
  }

  Status = VL53L0X_DataInit(&side_range);
  if (Status != VL53L0X_ERROR_NONE) {
    printf("side DataInit\r\n");
    print_pal_error(Status);
  }

  Status = VL53L0X_StaticInit(&side_range);
  if (Status != VL53L0X_ERROR_NONE) {
    printf("side StaticInit\r\n");
    print_pal_error(Status);
  }
}

void range_calibrate(void) {
  uint8_t VhvSettings = 0;
  uint8_t PhaseCal = 0;
  uint32_t refSpadCount = 0;
  uint8_t isApertureSpads = 0;

  VL53L0X_PerformRefCalibration(&front_range, &VhvSettings, &PhaseCal);
  VL53L0X_PerformRefSpadManagement(&front_range, &refSpadCount,
                                   &isApertureSpads);

  VL53L0X_PerformRefCalibration(&side_range, &VhvSettings, &PhaseCal);
  VL53L0X_PerformRefSpadManagement(&side_range, &refSpadCount,
                                   &isApertureSpads);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  printf("\r\n\r\nStartup!\r\n");

#if 1
  range_init();
  range_calibrate();

  VL53L0X_SetDeviceMode(&front_range, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  VL53L0X_StartMeasurement(&front_range);

  VL53L0X_SetDeviceMode(&side_range, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  VL53L0X_StartMeasurement(&side_range);

  VL53L0X_RangingMeasurementData_t data = {0};

  while (1) {
    HAL_Delay(50);
    VL53L0X_GetRangingMeasurementData(&front_range, &data);
    printf("front: %d\r\n", data.RangeMilliMeter);
    VL53L0X_ClearInterruptMask(&front_range, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    HAL_Delay(50);
    VL53L0X_GetRangingMeasurementData(&side_range, &data);
    printf("side:  %d\r\n", data.RangeMilliMeter);
    VL53L0X_ClearInterruptMask(&side_range, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
  }

#elif 0
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  uint16_t pwm = 0xFFFF * 0.4f;

  drive_dir(pwm, true, true);

  HAL_Delay(2000);

  drive_dir(0, true, true);

  HAL_Delay(200);

  drive_dir(pwm, false, false);

  HAL_Delay(2000);

  drive_dir(0, true, true);

  HAL_Delay(200);

  drive_dir(pwm, true, false);

  HAL_Delay(2000);

  drive_dir(0, true, true);

  HAL_Delay(200);

  drive_dir(pwm, false, true);

  HAL_Delay(2000);

  drive_dir(0, true, true);

  while (1)
    ;
#endif

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
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  printf("Assertion failed at %s:%ld\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
