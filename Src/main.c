#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "event_groups.h"

#include "string.h"
#include "stdio.h"

UART_HandleTypeDef huart3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);

TaskHandle_t HPTHandler;
void Sender_HPT_TASK(void *pvParameter);

TaskHandle_t LPTHandler;
void Sender_MPT_TASK(void *pvParameter);

TaskHandle_t LPTHandler;
void Receiver_LPT_TASK(void *pvParameter);

uint8_t rx_data = 0;
uint16_t indx1 = 1, indx2 = 2;
int SendFromISR = 0;

QueueHandle_t structQueue;

typedef struct
{
	char *str;
	int counter;
	uint16_t large_value;
}my_struct;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();
	
	structQueue = xQueueCreate(2, sizeof(my_struct));
	
	if(structQueue == NULL)
		HAL_UART_Transmit(&huart3, (uint8_t *) "Khong the tao Queue\r\r", 21, 100);
	else
		HAL_UART_Transmit(&huart3, (uint8_t *) "Tao Queue thanh cong\r\r", 21, 100);
	
	xTaskCreate(Sender_HPT_TASK, "Sender HPT Task", 128, NULL, 2, &HPTHandler);
	xTaskCreate(Sender_MPT_TASK, "Sender MPT Task", 128, NULL, 1, &LPTHandler);
	xTaskCreate(Receiver_LPT_TASK, "Receiver LPT Task", 128, NULL, 0, &LPTHandler);
	
	vTaskStartScheduler();
}

void Sender_HPT_TASK(void *pvParameter)
{
	my_struct *pSendStruct;
	
	while(1)
	{
		pSendStruct = pvPortMalloc(sizeof(my_struct));
		
		char str[50] = "Dang nhap TASK 1\rChuan bi gui du lieu\r";
		HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY);
		
		pSendStruct->counter = indx1;
		pSendStruct->large_value = indx1*3;
		pSendStruct->str = "DU LIEU TU TASK 1";
		
		if(xQueueSend(structQueue, pSendStruct, portMAX_DELAY) == pdPASS)
		{	
			char str[50] = "Hoan thanh gui Task 1\rThoat khoi TASK 1\r";
			HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY); 
		}
		else
		{
			char str[50] = "Loi khi gui queue\r";
			HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY); 
		}
		indx1 += 2;
		vPortFree(pSendStruct);
		vTaskDelay(1000);
	}
}

void Sender_MPT_TASK(void *pvParameter)
{
	my_struct *pSendStruct;

	while(1)
	{
		pSendStruct = pvPortMalloc(sizeof(my_struct));
		
		char str[50] = "Dang nhap TASK 2\rChuan bi gui du lieu\r";
		HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY);
		
		pSendStruct->counter = indx2;
		pSendStruct->large_value = indx2*2;
		pSendStruct->str = "DU LIEU TU TASK 2";
		
		if(xQueueSend(structQueue, pSendStruct, portMAX_DELAY) == pdPASS)
		{	
			char str[50] = "Hoan thanh gui Task 2\rThoat khoi TASK 2\r";
			HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY); 
		}
		else
		{
			char str[50] = "Loi khi gui queue\r";
			HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY); 
		}
		indx2 += 2;
		vPortFree(pSendStruct);
		vTaskDelay(1000);
	}
}

void Receiver_LPT_TASK(void *pvParameter)
{
  my_struct *pReceiveStruct;
	char *ptr;
	
	while(1)
	{
		pReceiveStruct = pvPortMalloc(2*sizeof(my_struct));
		
		char str[50] = "Dang nhap TASK 3\rChuan bi nhan du lieu\r";
		HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY); 
		
		if(xQueueReceive(structQueue, pReceiveStruct, portMAX_DELAY) == pdPASS)
		{
			char str[100] = "Nhan queue thanh cong\r ";
			ptr = pvPortMalloc(100 * sizeof (char));
			sprintf (ptr, "Nhan tu Queue:\rCOUNTER = %d\rLARGE VALUE = %u\rSTRING = %s\r\r\r",pReceiveStruct->counter,pReceiveStruct->large_value, pReceiveStruct->str);
			HAL_UART_Transmit(&huart3, (uint8_t *) ptr, strlen(ptr), HAL_MAX_DELAY); 
			vPortFree(ptr);
		}
		else
			{
				char str[20] = "Loi khi nhan queue\r";
				HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), HAL_MAX_DELAY); 
			}
			vPortFree(pReceiveStruct);
			vTaskDelay(1000);
		}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
	}

}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void Error_Handler(void)
{
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
