/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * FINAL VERSION FOR PC MASTER TESTING:
  *
  * 1. PROTOCOL (RX): [0xAA] [CMD] [3B_Data] [CS]
  * - CMD 0x1X: Velocity Control (PID). Input = RPM * 10.
  * - CMD 0x2X: Position Control (PID). Input = Ticks.
  * - X determines direction/sign.
  *
  * 2. DEBUG:
  * - Use STM32CubeIDE "Live Expressions" to view variables.
  * - UART TX is disabled (or used for feedback 0x2X only) to avoid conflict.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_IDLE = 0,
    MODE_VELOCITY_PID,
    MODE_DIRECT_PWM,
    MODE_POSITION
} ControlMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 6
#define TX_BUFFER_SIZE 6
#define START_BYTE 0xAA

#define CMD_VELOCITY_BASE 0x10
#define CMD_PWM_BASE      0x20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- Constants ---
const int32_t pwm_resolution = 1000;
const float sample_time_s = 0.01f;
const float integral_max_A = 1000.0f;
const float integral_max_B = 1000.0f;
const float PULSES_PER_REVOLUTION = 937.2f;
const int32_t MAX_PWM_POSITION = 400;

// --- Control State ---
volatile ControlMode_t control_mode = MODE_IDLE;

// --- Motor A Variables ---
volatile float target_A_val = 0.0f;
volatile float motor_A_speed_rpm = 0.0f;
volatile float actual_vel_A = 0;
volatile int32_t target_pos_A = 0;
volatile int32_t actual_pos_A = 0;
volatile uint16_t last_enc_A_raw = 0;

// --- Motor B Variables ---
volatile float target_B_val = 0.0f;
volatile float motor_B_speed_rpm = 0.0f;
volatile float actual_vel_B = 0;
volatile int32_t target_pos_B = 0;
volatile int32_t actual_pos_B = 0;
volatile uint16_t last_enc_B_raw = 0;

// --- PID Variables ---
float Kp_Vel_A = 0.5977f, Ki_Vel_A = 31.2f;
float Kp_Vel_B = 0.5302f, Ki_Vel_B = 33.11f;
volatile float err_vel_A = 0, int_vel_A = 0;
volatile float err_vel_B = 0, int_vel_B = 0;
volatile float pid_output_A = 0;
volatile float pid_output_B = 0;

float Kp_Pos = 1.5f, Kd_Pos = 0.5f;
volatile float err_pos_A = 0, prev_err_pos_A = 0;
volatile float err_pos_B = 0, prev_err_pos_B = 0;
// Khai báo đầy đủ để tránh lỗi biên dịch
volatile float int_pos_A = 0;
volatile float int_pos_B = 0;
volatile float error_A = 0, integral_A = 0; // Alias
volatile float error_B = 0, integral_B = 0; // Alias

/* Buffers */
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t print_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void set_motor_A_speed(int32_t speed);
void set_motor_B_speed(int32_t speed);
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Motor A
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Motor B
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Encoder A
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Encoder B
  HAL_TIM_Base_Start_IT(&htim4); // Loop Interrupt

  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

// ... (Giữ nguyên SystemClock_Config và các hàm MX_..._Init) ...
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim1);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim1);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

static void MX_TIM3_Init(void) {
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}

static void MX_TIM4_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim4);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

/* USER CODE BEGIN 4 */

/**
  * @brief Callback for UART Receive
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        if (Size == RX_BUFFER_SIZE && rx_buffer[0] == START_BYTE)
        {
            uint8_t cmd = rx_buffer[1];
            uint8_t cmd_base = cmd & 0xF0;
            uint8_t calc_cs = rx_buffer[1] ^ rx_buffer[2] ^ rx_buffer[3] ^ rx_buffer[4];
            if (calc_cs == rx_buffer[5])
            {
                uint8_t mode = cmd & 0x0F;
                uint16_t val_m1 = ((uint16_t)rx_buffer[2] << 4) | (rx_buffer[3] >> 4);
                uint16_t val_m2 = ((uint16_t)(rx_buffer[3] & 0x0F) << 8) | rx_buffer[4];
                float raw_A = (float)val_m1;
                float raw_B = (float)val_m2;
                float sign_A = 1.0f, sign_B = 1.0f;
                switch (mode) {
                    case 0: sign_A = 1; sign_B = 1; break;
                    case 1: sign_A = -1; sign_B = -1; break;
                    case 2: sign_A = 1; sign_B = -1; break;
                    case 3: sign_A = -1; sign_B = 1; break;
                }

                if (cmd_base == CMD_VELOCITY_BASE)
                {
                    control_mode = MODE_VELOCITY_PID;
                    target_A_val = (raw_A / 10.0f) * sign_A;
                    target_B_val = (raw_B / 10.0f) * sign_B;

//                    int_vel_A = 0; int_vel_B = 0;
                    int_pos_A = 0; int_pos_B = 0;
                }
                else if (cmd_base == CMD_PWM_BASE) // Actually POSITION Mode (0x2X)
                {
                    control_mode = MODE_POSITION;
                    int32_t delta_A = (int32_t)(raw_A * sign_A);
                    int32_t delta_B = (int32_t)(raw_B * sign_B);

                    target_pos_A = actual_pos_A + delta_A;
                    target_pos_B = actual_pos_B + delta_B;

                    int_vel_A = 0; int_vel_B = 0;
//                    int_pos_A = 0; int_pos_B = 0;
                }
            }
        }
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
    }
}

/**
  * @brief TIM4 Interrupt (10ms Loop)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        // --- 1. CẬP NHẬT TRẠNG THÁI ENCODER ---
        // Motor A
        uint16_t curr_A_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t delta_A = (int16_t)(curr_A_raw - last_enc_A_raw);
        last_enc_A_raw = curr_A_raw;
        actual_pos_A += delta_A;
        actual_vel_A = (float)delta_A * 100.0f / PULSES_PER_REVOLUTION * 60.0f;
        motor_A_speed_rpm = actual_vel_A; // Update global var for debug

        // Motor B
        uint16_t curr_B_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t delta_B = (int16_t)(curr_B_raw - last_enc_B_raw);
        last_enc_B_raw = curr_B_raw;
        actual_pos_B += delta_B;
        actual_vel_B = (float)delta_B * 100.0f / PULSES_PER_REVOLUTION * 60.0f;
        motor_B_speed_rpm = actual_vel_B; // Update global var for debug

        float output_pwm_A = 0.0f;
        float output_pwm_B = 0.0f;

        // --- 2. TÍNH TOÁN ĐIỀU KHIỂN ---
        if (control_mode == MODE_VELOCITY_PID)
        {
            // PID VẬN TỐC
            err_vel_A = target_A_val - actual_vel_A;
            if (target_A_val != 0) {
                int_vel_A += err_vel_A * sample_time_s;
                if (int_vel_A > integral_max_A) int_vel_A = integral_max_A;
                else if (int_vel_A < -integral_max_A) int_vel_A = -integral_max_A;
            } else int_vel_A = 0;
            output_pwm_A = (Kp_Vel_A * err_vel_A) + (Ki_Vel_A * int_vel_A);

            err_vel_B = target_B_val - actual_vel_B;
            if (target_B_val != 0) {
                int_vel_B += err_vel_B * sample_time_s;
                if (int_vel_B > integral_max_B) int_vel_B = integral_max_B;
                else if (int_vel_B < -integral_max_B) int_vel_B = -integral_max_B;
            } else int_vel_B = 0;
            output_pwm_B = (Kp_Vel_B * err_vel_B) + (Ki_Vel_B * int_vel_B);
        }
        else if (control_mode == MODE_POSITION)
        {
            // PID VỊ TRÍ
            err_pos_A = (float)(target_pos_A - actual_pos_A);
            float d_pos_A = (err_pos_A - prev_err_pos_A) / sample_time_s;
            output_pwm_A = (Kp_Pos * err_pos_A) + (Kd_Pos * d_pos_A);
            prev_err_pos_A = err_pos_A;

            if (output_pwm_A > MAX_PWM_POSITION) output_pwm_A = MAX_PWM_POSITION;
            if (output_pwm_A < -MAX_PWM_POSITION) output_pwm_A = -MAX_PWM_POSITION;
            if (abs(target_pos_A - actual_pos_A) < 10) output_pwm_A = 0;

            err_pos_B = (float)(target_pos_B - actual_pos_B);
            float d_pos_B = (err_pos_B - prev_err_pos_B) / sample_time_s;
            output_pwm_B = (Kp_Pos * err_pos_B) + (Kd_Pos * d_pos_B);
            prev_err_pos_B = err_pos_B;

            if (output_pwm_B > MAX_PWM_POSITION) output_pwm_B = MAX_PWM_POSITION;
            if (output_pwm_B < -MAX_PWM_POSITION) output_pwm_B = -MAX_PWM_POSITION;
            if (abs(target_pos_B - actual_pos_B) < 10) output_pwm_B = 0;
        }

        // --- 3. XUẤT PWM ---
        set_motor_A_speed((int32_t)output_pwm_A);
        set_motor_B_speed((int32_t)output_pwm_B);
    }
}

void set_motor_A_speed(int32_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    if (speed > 5) {
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)speed);
    } else if (speed < -5) {
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(-speed));
    } else {
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void set_motor_B_speed(int32_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    if (speed > 5) {
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)speed);
    } else if (speed < -5) {
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(-speed));
    } else {
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}
void Error_Handler(void) {
  __disable_irq();
  while (1) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(100);
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
/* USER CODE END 4 */
