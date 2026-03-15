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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    STATE_IDLE = 0,
    STATE_A_TO_B,       // direction locked: vehicle going A → B, rfidB ON
    STATE_B_TO_A,       // direction locked: vehicle going B → A, rfidA ON
} system_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void rfidA_on();
void rfidA_off();
void rfidB_on();
void rfidB_off();
void sensorA_event(uint8_t blocked);
void sensorB_event(uint8_t blocked);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile system_state_t system_state = STATE_IDLE;
uint32_t state_timer = 0;
uint32_t clear_timer = 0;
uint32_t wait_clear_timer = 0;
uint8_t sensorA_blocked = 0;
uint8_t sensorB_blocked = 0;
uint8_t rfid_cleared = 0;
uint8_t debounce_A = 0;
uint8_t debounce_B = 0;
uint8_t wait_clear = 0;       // 1 = waiting for old vehicle to clear after timeout
uint8_t ignore_sensor = 0;   // 'A' or 'B' - which sensor to ignore during wait_clear
uint8_t active_rfid = 0;     // 'A' or 'B' - which rfid is still on after timeout
#define DEBOUNCE_COUNT 3   // 3 x 10ms = 30ms debounce
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void sensorA_event(uint8_t blocked);
void sensorB_event(uint8_t blocked);
void process_sensors(void);
void clear_to_idle_task(void);
void system_timeout_task(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void rfidA_on()
{
    HAL_GPIO_WritePin(RFID_A_PIN_GPIO_Port, RFID_A_PIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, GPIO_PIN_SET);
}

void rfidA_off()
{
    HAL_GPIO_WritePin(RFID_A_PIN_GPIO_Port, RFID_A_PIN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, GPIO_PIN_RESET);
}

void rfidB_on()
{
    HAL_GPIO_WritePin(RFID_B_PIN_GPIO_Port, RFID_B_PIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
}

void rfidB_off()
{
    HAL_GPIO_WritePin(RFID_B_PIN_GPIO_Port, RFID_B_PIN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
}

void sensorA_event(uint8_t blocked)
{
    sensorA_blocked = blocked;

    switch(system_state)
    {
        case STATE_IDLE:
            if(blocked)
            {
                // If waiting for old vehicle to clear, handle exit sensor
                if(wait_clear && ignore_sensor == 'A')
                {
                    // Old vehicle reached exit sensor -> turn off its rfid
                    if(active_rfid == 'A') rfidA_off();
                    else if(active_rfid == 'B') rfidB_off();
                    active_rfid = 0;
                    rfid_cleared = 1;
                    break;
                }

                // Vehicle enters from A side -> turn on rfidB
                rfidB_on();
                rfidA_off();
                wait_clear = 0;
                active_rfid = 0;
                system_state = STATE_A_TO_B;
                state_timer = HAL_GetTick();
            }
        break;

        case STATE_A_TO_B:
            // Ignore all sensorA events - direction is locked, waiting for sensorB
        break;

        case STATE_B_TO_A:
            if(blocked)
            {
                // Vehicle reached A side -> turn off rfidA
                rfidA_off();
                rfid_cleared = 1;
            }
        break;

        default:
        break;
    }
}

void sensorB_event(uint8_t blocked)
{
    sensorB_blocked = blocked;

    switch(system_state)
    {
        case STATE_IDLE:
            if(blocked)
            {
                // If waiting for old vehicle to clear, handle exit sensor
                if(wait_clear && ignore_sensor == 'B')
                {
                    // Old vehicle reached exit sensor -> turn off its rfid
                    if(active_rfid == 'A') rfidA_off();
                    else if(active_rfid == 'B') rfidB_off();
                    active_rfid = 0;
                    rfid_cleared = 1;
                    break;
                }

                // Vehicle enters from B side -> turn on rfidA
                rfidA_on();
                rfidB_off();
                wait_clear = 0;
                active_rfid = 0;
                system_state = STATE_B_TO_A;
                state_timer = HAL_GetTick();
            }
        break;

        case STATE_B_TO_A:
            // Ignore all sensorB events - direction is locked, waiting for sensorA
        break;

        case STATE_A_TO_B:
            if(blocked)
            {
                // Vehicle reached B side -> turn off rfidB
                rfidB_off();
                rfid_cleared = 1;
            }
        break;

        default:
        break;
    }
}

void clear_to_idle_task()
{
    if(system_state != STATE_IDLE && rfid_cleared)
    {
        if(!sensorA_blocked && !sensorB_blocked)
        {
            if(HAL_GetTick() - clear_timer > 500)
            {
                rfidA_off();
                rfidB_off();
                rfid_cleared = 0;
                system_state = STATE_IDLE;
            }
        }
        else
        {
            clear_timer = HAL_GetTick();
        }
    }

    // Clear wait_clear flag once both sensors are unblocked for 500ms
    if(wait_clear && system_state == STATE_IDLE)
    {
        if(!sensorA_blocked && !sensorB_blocked && rfid_cleared)
        {
            if(HAL_GetTick() - wait_clear_timer > 500)
            {
                // Vehicle has fully cleared, turn off any remaining rfid
                if(active_rfid == 'A') rfidA_off();
                else if(active_rfid == 'B') rfidB_off();
                active_rfid = 0;
                wait_clear = 0;
                rfid_cleared = 0;
            }
        }
        else
        {
            wait_clear_timer = HAL_GetTick();
        }
    }
}

void system_timeout_task()
{
    if(system_state != STATE_IDLE)
    {
        if(HAL_GetTick() - state_timer > 300000)  // 5 minutes safety timeout
        {
            // Remember direction info but KEEP RFID ON
            if(system_state == STATE_A_TO_B)
            {
                ignore_sensor = 'B';   // vehicle going A->B, exit is sensorB
                active_rfid = 'B';     // rfidB is still on, keep it on
            }
            else
            {
                ignore_sensor = 'A';   // vehicle going B->A, exit is sensorA
                active_rfid = 'A';     // rfidA is still on, keep it on
            }

            // DON'T turn off rfid - vehicle still on scale
            rfid_cleared = 0;
            wait_clear = 1;
            wait_clear_timer = HAL_GetTick();
            system_state = STATE_IDLE;
        }
    }
}

void process_sensors()
{
    // Read GPIO directly every 10ms - no EXTI dependency
    uint8_t a_now = (HAL_GPIO_ReadPin(SENSOR_A_PIN_GPIO_Port, SENSOR_A_PIN_Pin) == GPIO_PIN_RESET);
    uint8_t b_now = (HAL_GPIO_ReadPin(SENSOR_B_PIN_GPIO_Port, SENSOR_B_PIN_Pin) == GPIO_PIN_RESET);

    // Debounce sensorA: must see new state for DEBOUNCE_COUNT consecutive reads
    if(a_now != sensorA_blocked)
    {
        debounce_A++;
        if(debounce_A >= DEBOUNCE_COUNT)
        {
            sensorA_event(a_now);
            debounce_A = 0;
        }
    }
    else
    {
        debounce_A = 0;
    }

    // Debounce sensorB
    if(b_now != sensorB_blocked)
    {
        debounce_B++;
        if(debounce_B >= DEBOUNCE_COUNT)
        {
            sensorB_event(b_now);
            debounce_B = 0;
        }
    }
    else
    {
        debounce_B = 0;
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
  /* USER CODE BEGIN 2 */
  __enable_irq();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    system_timeout_task();
    process_sensors();
    clear_to_idle_task();
    HAL_Delay(10);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RFID_A_PIN_Pin|RFID_B_PIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LEDA_Pin|LEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDA_Pin|LEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR_A_PIN_Pin SENSOR_B_PIN_Pin */
  GPIO_InitStruct.Pin = SENSOR_A_PIN_Pin|SENSOR_B_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RFID_A_PIN_Pin RFID_B_PIN_Pin */
  GPIO_InitStruct.Pin = RFID_A_PIN_Pin|RFID_B_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDA_Pin LEDB_Pin */
  GPIO_InitStruct.Pin = LEDA_Pin|LEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Intentionally empty - sensors are polled in main loop
    // EXTI kept active to wake from sleep if needed
    (void)GPIO_Pin;
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


