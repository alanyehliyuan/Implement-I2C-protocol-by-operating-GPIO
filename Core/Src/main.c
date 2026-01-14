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
#include "cmsis_os2.h"
#include "FreeRTOS.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
#include "soft_i2c.h"
// 定義 Semaphore 的 Handle
osSemaphoreId_t myBinarySem01Handle;

// 定義它的屬性 (名稱)
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};

// 接收緩衝區 (Slave 用)
uint8_t slave_rx_buffer[1];
volatile uint8_t button_pressed = 0;
// extern I2C_HandleTypeDef hi2c1; // 硬體 I2C Handle

// 定義 Mutex ID 與屬性
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
osMutexId_t MutexAHandle;
const osMutexAttr_t MutexA_attributes = {.name = "MutexA"};

osMutexId_t MutexBHandle;
const osMutexAttr_t MutexB_attributes = {.name = "MutexB"};

// 定義兩個任務的屬性
osThreadId_t Task1Handle;
const osThreadAttr_t task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Task2Handle;
const osThreadAttr_t task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartTask1(void *argument);
void StartTask2(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdlib.h> // 為了 malloc

// 讓 printf 能夠透過 UART1 輸出的關鍵函式
int _write(int file, char *ptr, int len) {
    // 確保這裡是用 huart1 (因為你的板子 VCP 是接在 UART1)
    extern UART_HandleTypeDef huart1;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    return len;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);
  MutexAHandle = osMutexNew(&MutexA_attributes);
  MutexBHandle = osMutexNew(&MutexB_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  // 建立 Binary Semaphore
  // 參數 1: max_count (最大計數，Binary 就是 1)
  // 參數 2: initial_count (初始值，通常設 0 代表一開始是空的，要等 ISR 給信號)
  myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  Task1Handle = osThreadNew(StartTask1, NULL, &task1_attributes);
  Task2Handle = osThreadNew(StartTask2, NULL, &task2_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 100;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SOFT_SCL_Pin|SOFT_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SOFT_SCL_Pin SOFT_SDA_Pin */
  GPIO_InitStruct.Pin = SOFT_SCL_Pin|SOFT_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 當硬體 I2C (Slave) 收到完整的資料後，HAL 庫會呼叫這個函式
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // 判斷是不是 I2C1 叫的
  if (hi2c->Instance == I2C1) {
    // 1. 發送信號給 Task 2 (Printer)
    osSemaphoreRelease(myBinarySem01Handle);

    // 2. 重要！重新啟動接收功能，準備接下一筆
    // 如果不加這行，Slave 收完一次就會「收工」，下次 Master 再送它就不理了
    HAL_I2C_Slave_Receive_IT(&hi2c1, slave_rx_buffer, 1);
  }
}

volatile int32_t shared_counter = 0;
int32_t temp;

void mutex_test_task1() {
  osMutexAcquire(MutexAHandle, osWaitForever);
  printf("[Task 1] Got Mutex A! Waiting for Mutex B...\r\n");
  osDelay(100);
  osMutexAcquire(MutexBHandle, osWaitForever);
  printf("[Task 1] Got Both Mutexes! Working...\r\n");
  // 1. 讀取 (Read)
  temp = shared_counter;
  
  // 2. 故意拖台錢 (讓 Context Switch 更有機會發生在這裡)
  // 就像你剛把存款領出來，正在數錢，結果被叫去接電話
  for(int i=0; i<100; i++) { __NOP(); } 
  
  // 3. 修改 (Modify)
  temp = temp + 1;
  
  // 4. 寫回 (Write)
  shared_counter = temp;
  osMutexRelease(MutexBHandle);
  osMutexRelease(MutexAHandle);
  osDelay(1);
  // 注意：這裡完全拿掉 osDelay，讓它全速搶 CPU
}

// 定義要發送的資料 (例如從 'A' 開始)
uint8_t data_to_send = 'A'; 

// Slave 地址 (假設 CubeMX 設 0x32) -> 寫入地址是 0x64
uint8_t slave_addr = (0x32 << 1) | 0;
void soft_i2c_test() {
// --- 開始 I2C 傳輸 ---
    SoftI2C_Start();
    
    // 1. 送地址
    if (SoftI2C_WriteByte(slave_addr)) {
        // 收到 ACK，代表 Slave 在線上
        
        // 2. 送資料
        SoftI2C_WriteByte(data_to_send);
        
        // 準備下一個字元 ('A' -> 'B' -> 'C'...)
        data_to_send++;
        if (data_to_send > 'Z') data_to_send = 'A';
        
    } else {
        // 沒收到 ACK (可能線沒接好)
        printf("[Master] NACK! Check wiring.\r\n");
    }
    
    SoftI2C_Stop();
    // --- 結束 I2C 傳輸 ---

    // 每 1 秒送一次
    osDelay(1000);
}

// 任務 1：負責 +1
void StartTask1(void *argument) {
  // 1. 初始化軟體 GPIO
  SoftI2C_Init();
  
  for(;;) {
    // mutex_test();
    soft_i2c_test();
  }
}

void mutex_test_task2() {
  static int32_t temp_b = 0;
  osMutexAcquire(MutexAHandle, osWaitForever);
  printf("[Task 2] Got Mutex A! Waiting for Mutex B...\r\n");
  osDelay(100);

  osMutexAcquire(MutexBHandle, osWaitForever);
  printf("[Task 2] Got Both Mutexes! Working... \r\n");

  // 1. 讀取
  temp_b = shared_counter;
  
  // 2. 故意拖台錢
  for(int i=0; i<100; i++) { __NOP(); } 
  
  // 3. 修改
  temp_b = temp_b - 1; // 這裡是減！
  
  // 4. 寫回
  shared_counter = temp_b;
  osMutexRelease(MutexBHandle);
  osMutexRelease(MutexAHandle);
  osDelay(1);
}

// 任務 2：負責 -1
void StartTask2(void *argument) {
  HAL_I2C_Slave_Receive_IT(&hi2c1, slave_rx_buffer, 1);
  for(;;) {
    // mutex_test_task2();
    // 1. 死守 Semaphore (等待 ISR 通知)
    // 當 Callback 呼叫 osSemaphoreRelease 時，這裡會醒來
    if (osSemaphoreAcquire(myBinarySem01Handle, osWaitForever) == osOK) {
        
        // 2. 醒來後，把 Buffer 裡的資料印出來
        printf("[Slave] Received: %c (Hex: 0x%02X)\r\n", 
               slave_rx_buffer[0], slave_rx_buffer[0]);
    }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  printf("Wait for button SW1...\r\n");

  while(button_pressed == 0) {

  }

  printf("Button Detected! Program Continue... \r\n");
  // 稍微延遲一下，讓 VCP 連線穩定
  HAL_Delay(500);

  printf("\r\n\r\n");
  printf("========================================\r\n");
  printf("   STM32WB55 Physical Memory Layout     \r\n");
  printf("========================================\r\n");

  // --- 變數宣告 ---
  static int static_var = 10;       // 預期：SRAM (.data)
  static int static_bss;            // 預期：SRAM (.bss)
  int stack_var = 20;               // 預期：SRAM (Stack 區)
  int *heap_var = (int*)malloc(4);  // 預期：SRAM (Heap 區)
  *heap_var = 30;

  // --- 印出地址 ---
  // 注意：在 MCU 上，我們看到的是真實的物理地址 (Physical Address)
  printf("[Code/Flash] .text address:  0x0800xxxx (Base)\r\n");
  printf("[Data/SRAM ] .data address:  %p\r\n", &static_var);
  printf("[BSS /SRAM ] .bss  address:  %p\r\n", &static_bss);
  printf("[Heap/SRAM ] Heap  address:  %p\r\n", heap_var);
  printf("[Stack/SRAM] Stack address:  %p\r\n", &stack_var);
  
  printf("========================================\r\n");
  
  free(heap_var);
  /* Infinite loop */
  for(;;)
  {
    // printf("Counter: %ld\r\n", shared_counter);
    osDelay(500); // 每 0.5 秒回報一次
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
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
