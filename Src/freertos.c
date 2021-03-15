/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vTaskEth1 */
osThreadId_t vTaskEth1Handle;
const osThreadAttr_t vTaskEth1_attributes = {
  .name = "vTaskEth1",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for vTaskEth2 */
osThreadId_t vTaskEth2Handle;
const osThreadAttr_t vTaskEth2_attributes = {
  .name = "vTaskEth2",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void vTaskEth1St(void *argument);
void vTaskEth2St(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
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

  /* creation of vTaskEth1 */
  vTaskEth1Handle = osThreadNew(vTaskEth1St, NULL, &vTaskEth1_attributes);

  /* creation of vTaskEth2 */
  vTaskEth2Handle = osThreadNew(vTaskEth2St, NULL, &vTaskEth2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		//BME680_Read();
		Time();
		ssd1306_UpdateScreen();
		   osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_vTaskEth1St */
/**
* @brief Function implementing the vTaskEth1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskEth1St */
void vTaskEth1St(void *argument)
{
  /* USER CODE BEGIN vTaskEth1St */
  /* Infinite loop */

	  uint32_t i;
	          uint8_t delay_arp = 0;

	          for (;;) {
	                  vTaskDelay(configTICK_RATE_HZ/2); // полсекунды
	                  delay_arp++;
	                  for (i = 0; i < UIP_CONNS; i++) {
	                          uip_periodic(i);
	                          if (uip_len > 0) {
	                                  uip_arp_out();
	                                  enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
	                          }
	                  }

	  #if UIP_UDP
	                  for(i = 0; i < UIP_UDP_CONNS; i++) {
	                          uip_udp_periodic(i);
	                          if(uip_len > 0) {
	                                  uip_arp_out();
	                                  network_send();
	                          }
	                  }
	  #endif

	                  if (delay_arp >= 50) { // один раз за 50 проходов цикла, около 10 сек.
	                          delay_arp = 0;
	                          uip_arp_timer();
	                  }
	          }
  /* USER CODE END vTaskEth1St */
}

/* USER CODE BEGIN Header_vTaskEth2St */
/**
* @brief Function implementing the vTaskEth2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskEth2St */
void vTaskEth2St(void *argument)
{
  /* USER CODE BEGIN vTaskEth2St */
  /* Infinite loop */
  for(;;)
  {
      uip_len = enc28j60_recv_packet((uint8_t *) uip_buf, UIP_BUFSIZE);

      if (uip_len > 0) {
              if (BUF->type == htons(UIP_ETHTYPE_IP)) {
                      uip_arp_ipin();
                      uip_input();
                      if (uip_len > 0) {
                              uip_arp_out();
                              enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
                      }
              } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
                      uip_arp_arpin();
                      if (uip_len > 0) {
                              enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
                      }
              }
      }
      taskYIELD();


  }
  /* USER CODE END vTaskEth2St */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
