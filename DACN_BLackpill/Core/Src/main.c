/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 10-bit RAW ADC
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// ✅ Raw ADC values (10-bit: 0-1023)
volatile uint16_t adc_values[NUM_SENSORS];

// ✅ RAW Min/Max arrays
uint16_t sensor_min[NUM_SENSORS];
uint16_t sensor_max[NUM_SENSORS];

// ✅ Y values
float sensor_y_j0[NUM_SENSORS];
float y_min_avg = 0;
float y_max_avg = 0;
float line_position_X = 0.0;

// ✅ State machine
typedef enum {
    CALIB_IDLE,
    CALIB_TRACKING_RAW,
    CALIB_COMPUTE_X
} CalibState_t;

CalibState_t calib_state = CALIB_IDLE;

char usb_buffer[200];
uint8_t tracking_done = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void USB_Print(char* msg);
void Init_RAW_Tracking(void);
void Track_RAW_For_10s(void);
void Compute_YJ0_RAW(void);
void Compute_X_RAW(void);
void Print_X_Position(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void USB_Print(char* msg) {
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(10);
}

// ✅ Khởi tạo tracking RAW
void Init_RAW_Tracking(void) {
    for(int i = 0; i < NUM_SENSORS; i++) {
        sensor_min[i] = 1023;  // Max 10-bit value
        sensor_max[i] = 0;
    }
    tracking_done = 0;
}

// ✅ Track RAW MIN/MAX trong 10 giây
void Track_RAW_For_10s(void) {
    sprintf(usb_buffer, "\r\n>>> Tracking RAW MIN/MAX for 10 seconds...\r\n");
    USB_Print(usb_buffer);
    sprintf(usb_buffer, ">>> Move sensors over BLACK and WHITE surfaces!\r\n\r\n");
    USB_Print(usb_buffer);

    // Reset tracking
    Init_RAW_Tracking();

    uint32_t start_time = HAL_GetTick();
    uint32_t duration = 10000;  // 10 seconds

    while((HAL_GetTick() - start_time) < duration) {
        // Track RAW min/max
        for(int i = 0; i < NUM_SENSORS; i++) {
            if(adc_values[i] < sensor_min[i]) {
                sensor_min[i] = adc_values[i];
            }
            if(adc_values[i] > sensor_max[i]) {
                sensor_max[i] = adc_values[i];
            }
        }

        // Print countdown mỗi 1 giây
        static uint32_t last_print = 0;
        if((HAL_GetTick() - last_print) >= 1000) {
            uint32_t remaining = (duration - (HAL_GetTick() - start_time)) / 1000;
            sprintf(usb_buffer, ">>> Time remaining: %lu seconds...\r\n", remaining);
            USB_Print(usb_buffer);
            last_print = HAL_GetTick();
        }

        HAL_Delay(50);
    }

    tracking_done = 1;

    sprintf(usb_buffer, "\r\n>>> RAW tracking complete!\r\n");
    USB_Print(usb_buffer);

    // Hiển thị kết quả
    sprintf(usb_buffer, "RAW MIN: ");
    USB_Print(usb_buffer);
    for(int i = 0; i < NUM_SENSORS; i++) {
        sprintf(usb_buffer, "%4d ", sensor_min[i]);
        USB_Print(usb_buffer);
    }
    sprintf(usb_buffer, "\r\n");
    USB_Print(usb_buffer);

    sprintf(usb_buffer, "RAW MAX: ");
    USB_Print(usb_buffer);
    for(int i = 0; i < NUM_SENSORS; i++) {
        sprintf(usb_buffer, "%4d ", sensor_max[i]);
        USB_Print(usb_buffer);
    }
    sprintf(usb_buffer, "\r\n\r\n");
    USB_Print(usb_buffer);
}

// ✅ Tính Y_j0 từ RAW
void Compute_YJ0_RAW(void) {
    if(!tracking_done) {
        sprintf(usb_buffer, "\r\n>>> ERROR: Please complete tracking first!\r\n\r\n");
        USB_Print(usb_buffer);
        return;
    }

    // Tính y_min_avg và y_max_avg từ RAW
    uint32_t sum_min = 0;
    uint32_t sum_max = 0;

    for(int i = 0; i < NUM_SENSORS; i++) {
        sum_min += sensor_min[i];
        sum_max += sensor_max[i];
    }

    y_min_avg = (float)sum_min / NUM_SENSORS;
    y_max_avg = (float)sum_max / NUM_SENSORS;

    // Tính y_j0 cho từng cảm biến
    for(int i = 0; i < NUM_SENSORS; i++) {
        float x_max_i = (float)sensor_max[i];  // RAW max
        float x_min_i = (float)sensor_min[i];  // RAW min
        float x_ij = (float)adc_values[i];     // RAW hiện tại

        // Công thức: y_j0 = y_min + [(y_max - y_min) / (x_max,i - x_min,i)] × (x_ij - x_min,i)
        if((x_max_i - x_min_i) > 0) {
            sensor_y_j0[i] = y_min_avg +
                            ((y_max_avg - y_min_avg) / (x_max_i - x_min_i)) *
                            (x_ij - x_min_i);
        } else {
            sensor_y_j0[i] = y_min_avg;
        }
    }
}

// ✅ Tính X position từ Y_j0
void Compute_X_RAW(void) {
    if(!tracking_done) {
        sprintf(usb_buffer, "\r\n>>> ERROR: Please complete tracking first!\r\n\r\n");
        USB_Print(usb_buffer);
        return;
    }

    const float L_cb = 12.0;

    float y1 = sensor_y_j0[0];
    float y2 = sensor_y_j0[1];
    float y3 = sensor_y_j0[2];
    float y4 = sensor_y_j0[3];
    float y5 = sensor_y_j0[4];

    float sum_y = y1 + y2 + y3 + y4 + y5;

    if(sum_y < 0.01) {
        sprintf(usb_buffer, "\r\n>>> ERROR: Sum of y values too small!\r\n\r\n");
        USB_Print(usb_buffer);
        line_position_X = 0.0;
        return;
    }

    // X = L_cb × [2(y_5 - y_1) + (y_4 - y_2)] / Σy_i
    float numerator = 2.0 * (y5 - y1) + (y4 - y2);
    line_position_X = L_cb * (numerator / sum_y);
}

// ✅ In X position
void Print_Y(void){
	sprintf(usb_buffer, "Y_1: %.2f mm\r\n", sensor_y_j0[0]);
	        USB_Print(usb_buffer);
	    	sprintf(usb_buffer, "Y_2: %.2f mm\r\n", sensor_y_j0[1]);
	    	        USB_Print(usb_buffer);
	    	    	sprintf(usb_buffer, "Y_3: %.2f mm\r\n", sensor_y_j0[2]);
	    	    	        USB_Print(usb_buffer);
	    	    	    	sprintf(usb_buffer, "Y_4: %.2f mm\r\n", sensor_y_j0[3]);
	    	    	    	        USB_Print(usb_buffer);
	    	    	    	    	sprintf(usb_buffer, "Y_5: %.2f mm\r\n", sensor_y_j0[4]);
	    	    	    	    	        USB_Print(usb_buffer);
}
void Print_X_Position(void) {
    sprintf(usb_buffer, "\r\n========== LINE POSITION ==========\r\n");
    USB_Print(usb_buffer);

    sprintf(usb_buffer, "X = %.2f mm\r\n", line_position_X);
    USB_Print(usb_buffer);

    sprintf(usb_buffer, "===================================\r\n\r\n");
    USB_Print(usb_buffer);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(2000);  // Wait for USB

  // Initialize
  Init_RAW_Tracking();

  // Start ADC with DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, NUM_SENSORS);
  HAL_Delay(100);

  // Welcome message
  sprintf(usb_buffer, "\r\n=== 10-bit RAW ADC Line Follower ===\r\n");
  USB_Print(usb_buffer);

  sprintf(usb_buffer, "Commands:\r\n");
  USB_Print(usb_buffer);
  sprintf(usb_buffer, "  '1' - Start 10s tracking (move over black/white)\r\n");
  USB_Print(usb_buffer);
  sprintf(usb_buffer, "  '3' - Show current RAW values\r\n\r\n");
  USB_Print(usb_buffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    switch(calib_state) {
      case CALIB_IDLE:
        // Idle - waiting for command
        HAL_Delay(100);
        break;

      case CALIB_TRACKING_RAW:
        // Track 10s để lưu MIN/MAX
        Track_RAW_For_10s();

        sprintf(usb_buffer, "\r\n>>> Starting continuous X computation...\r\n\r\n");
        USB_Print(usb_buffer);

        // ✅ Chuyển sang COMPUTE_X và KHÔNG BAO GIỜ QUAY LẠI
        calib_state = CALIB_COMPUTE_X;
        break;

      case CALIB_COMPUTE_X:
        // ✅ VÒNG LẶP VÔ HẠN - tính X liên tục

        // Tính Y_j0 từ RAW hiện tại (dùng min/max đã lưu)
        Compute_YJ0_RAW();

        // Tính X
        Compute_X_RAW();

        // In X
        Print_Y();
        Print_X_Position();

        // ✅ Delay ngắn để không spam quá nhanh
        HAL_Delay(100);  // 100ms = 10 lần/giây

        // ✅ KHÔNG BREAK - vẫn ở state CALIB_COMPUTE_X
        break;
    }

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;  // ✅ 10-bit
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 0 - PA0
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 1 - PA1
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 2 - PA2
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 3 - PA3
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 4 - PA4
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure PA0-PA4 as Analog
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                        GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
