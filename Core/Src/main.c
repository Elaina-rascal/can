/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "maincpp.h"
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
void CANFilterConfig_AnyId(void);
void test();
void can_filter_init(void);
void Configure_Filter(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  // can_filter_init();
  // CANFilterConfig_AnyId();
  // HAL_CAN_Start(&hcan1);
  // HAL_CAN_Start(&hcan2);
  // can_filter_init();
  Configure_Filter();
  main_cpp();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    test();
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
  RCC_OscInitStruct.PLL.PLLM = 6;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void test()
{

  // �?????0个是电调1的高8位，�?????1个是�????? �?????1的低8位，依次类推
  uint8_t _common_buffer[8] = {0};
  _common_buffer[0] = 0x01;
  _common_buffer[1] = 0x00;
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef Can_Tx;
  Can_Tx.DLC = 0x08;
  Can_Tx.ExtId = 0x0000;

  // 大疆的电机协议，1-4�?????0x200,5-8�?????0x1FF

  Can_Tx.StdId = 0x200;

  Can_Tx.IDE = CAN_ID_STD;
  Can_Tx.RTR = CAN_RTR_DATA;
  Can_Tx.TransmitGlobalTime = DISABLE; // 不传输时间戳
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
    /* code */
  }
  HAL_CAN_AddTxMessage(&hcan1, &Can_Tx, _common_buffer, &TxMailbox);

  HAL_Delay(1);
}
void Configure_Filter(void)
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 把接收到的报文放入到FIFO0
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterActivation = ENABLE;
  // sFilterConfig.SlaveStartFilterBank = 14;//为从CAN实例选择启动筛�?�器组�?�对于单个CAN实例，此参数没有意义。对于双CAN实例，所有具有较低索引的过滤器组都被分配给主CAN实例，�?�所有具有较大索引的过滤器组都被分配给从CAN实例。该参数必须为Min_Data = O和Max_Data =27之间的一个数�??.

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) // creat CanFilter
  {
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
    Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) // initialize can
  {
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
    Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
  }
  // 当FIFO0中有消息的时候进入中
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // The FIFO0 receive interrupt function was enabled
  {
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
    Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
  }
}
void can_filter_init(void)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;           // �??启过滤器
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  // 过滤器模式，此处设置为掩码模�??
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器大小设�??
  can_filter_st.FilterIdHigh = 0x0000;               // 校验码高八位
  can_filter_st.FilterIdLow = 0x0000;                // 校验码低八位
  can_filter_st.FilterMaskIdHigh = 0x0000;           // 掩码高八�??
  can_filter_st.FilterMaskIdLow = 0x0000;            // 掩码低八�??
  can_filter_st.FilterBank = 0;                      // 过滤器编号，CAN1�??0~13
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; // 设置FIFO
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);      // 设置过滤�??
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // �??启中断接�??
}
void CANFilterConfig_AnyId(void)
{
  // CAN_FilterConfTypeDef  sFilterConfig;
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterIdHigh = 0x0000; // 32 �???????? ID		  不使用过滤器
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000; // 32 �???????? MASK
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterBank = 0;                          // 过滤�????????0
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 过滤�????????0关联到FIFO0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE; // �????????活过滤器0
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  // sFilterConfig.SFilterBank = 14;
  // if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  // {
  // Error_Handler();
  // }
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN接收中断
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  // HAL_Delay(1);
  switch (rx_header.StdId)
  {

  default:
  {
    break;
  }
  }
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM13 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13)
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
