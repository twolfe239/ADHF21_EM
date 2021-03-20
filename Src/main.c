/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"



#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>


#include <dhcpc.h>
#include <umqtt.h>
#include <enc28j60.h>
#include <clock.h>
#include <uip.h>
#include <uip_arp.h>
#include <nic.h>
#include <main.h>
#include <net_config.h>
#include <enc28j60def.h>

#if UIP_SPLIT_HACK
#include "uip-split.h"
#elif UIP_EMPTY_PACKET_HACK
#include "uip-empty-packet.h"
#endif




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RTC_TimeTypeDef sTime;

struct timer periodic_timer;
struct timer arp_timer;
struct timer mqtt_kalive_timer;
struct timer mqtt_conn_timer;
struct timer mqtt_pub_timer;

static clock_time_t timerCounter = 0;

static struct uip_eth_addr uNet_eth_address;

static uint8_t mqtt_txbuff[200];
static uint8_t mqtt_rxbuff[150];

static void handle_message(struct umqtt_connection *conn, char *topic, char *data);

static struct umqtt_connection mqtt =
{
  .txbuff =
  {
    .start = mqtt_txbuff,
    .length = sizeof(mqtt_txbuff),
  },
  .rxbuff =
  {
    .start = mqtt_rxbuff,
    .length = sizeof(mqtt_rxbuff),
  },
  .message_callback = handle_message,
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

#define MQTT_KEEP_ALIVE			30
#define MQTT_CLIENT_ID			"enc28j60_stm32_client"
#define MQTT_TOPIC_ALL			"/#"
#define MQTT_IP0                        192
#define MQTT_IP1                        168
#define MQTT_IP2                        0
#define MQTT_IP3                        46//46

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile char bufbme[20];
volatile char bufuart[50];
volatile uint32_t ii = 0;
volatile uint8_t flPin = 239;
volatile uint8_t flIntrpt = 0;








uint8_t nic_send_timer;
uint16_t nic_txreset_num = 0;

// TCP/IP parameters in data memory
uint8_t _eth_addr[6] = {ETHADDR0, ETHADDR1, ETHADDR2, ETHADDR3, ETHADDR4, ETHADDR5};
uint8_t _ip_addr[4] = {IPADDR0, IPADDR1, IPADDR2, IPADDR3};
uint8_t _net_mask[4] = {NETMASK0, NETMASK1, NETMASK2, NETMASK3};
uint8_t _gateway[4] = {DRIPADDR0, DRIPADDR1, DRIPADDR2, DRIPADDR3};





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */







/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */







//==============================================================================
// Функция возвращает счётчик времени
//==============================================================================
clock_time_t clock_time(void)
{
  return timerCounter;
}
//==============================================================================


//==============================================================================
// Обработчик входящих MQTT-сообщений
//==============================================================================
static void handle_message(struct umqtt_connection *conn, char *topic, char *data)
{
  uint8_t TopicMached = umqtt_isTopicMatched(MQTT_TOPIC_ALL, topic);
  if (TopicMached)
  {
#if DEBUG_UMQTT
	  sprintf(bufuart,"%s (%s)\r\n", topic, data);
	  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
#endif
  }
}
//==============================================================================


//==============================================================================
// Обработчик прерывания от таймера (период = 1 мс). Используется всеми программными таймерами
//==============================================================================
void msTick_Handler(void)
{
  timerCounter++;

  if (nic_send_timer)
    nic_send_timer--;
}
//==============================================================================


//==============================================================================
// Процедура инициализирует 1мс таймера (для работы программных таймеров)
//==============================================================================
void clock_init(void)
{
  tmr2_init(CLOCK_SECOND, msTick_Handler);
  timerCounter = 0;
  tmr2_start();
}
//==============================================================================


//==============================================================================
// Процедура обработчик успешного получения параметров IP от DHCP-сервера
//==============================================================================
void dhcpc_configured(const struct dhcpc_state *s)
{
  uip_sethostaddr(&s->ipaddr);
  uip_setnetmask(&s->netmask);
  uip_setdraddr(&s->default_router);
}
//==============================================================================


//==============================================================================
// Процедура установки соединения с MQTT-брокером
//==============================================================================
void nethandler_umqtt_init(struct umqtt_connection *conn)
{
  struct uip_conn *uc;
  uip_ipaddr_t ip;

  uip_ipaddr(&ip, MQTT_IP0, MQTT_IP1, MQTT_IP2, MQTT_IP3);
  uc = uip_connect(&ip, htons(1883));

  if (uc == NULL)
    return;

  conn->kalive = MQTT_KEEP_ALIVE;
  conn->clientid = (char*)MQTT_CLIENT_ID;

  umqtt_init(conn);
  umqtt_circ_init(&conn->txbuff);
  umqtt_circ_init(&conn->rxbuff);

  umqtt_connect(conn);
  uc->appstate.conn = conn;
}
//==============================================================================


//==============================================================================
// Функция записывает форматированную строку в буфер StrBuff
//==============================================================================
int8_t str_printf(char *StrBuff, uint8_t BuffLen, const char *args, ...)
{
  va_list ap;
  va_start(ap, args);
  int8_t len = vsnprintf(StrBuff, BuffLen, args, ap);
  va_end(ap);
  return len;
}
//==============================================================================

















void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	flIntrpt = 1;

	if (GPIO_Pin == GPIO_PIN_0) {
		flPin = intPin0;
	}

	if (GPIO_Pin == GPIO_PIN_12) {
		flPin = intPin12;
	}

	if (GPIO_Pin == GPIO_PIN_13) {
		flPin = intPin13;
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
 // MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  	//------------------------------------------------------------------ Knock-knock-knock
  	SPON();
  	HAL_Delay(50);
  	SPOFF();
  	HAL_Delay(50);
  	SPON();
  	HAL_Delay(45);
  	SPOFF();




    //------------------------------------------------------------------ LCD
  	lcd_Init();
  	lcd_Clear();

    //------------------------------------------------------------------ MQTT

    //SystemInit();

    for (uint8_t  i = 0; i < 6; i++)
      uNet_eth_address.addr[i] = _eth_addr[i];

  #if DEBUG_UIP
    sprintf(bufuart,"MAC address:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
           _eth_addr[0], _eth_addr[1], _eth_addr[2], _eth_addr[3], _eth_addr[4], _eth_addr[5]);
	  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
    sprintf(bufuart,"Init NIC...\r\n");
	  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
  #endif

    // init NIC device driver
    nic_init(SPI1, _eth_addr);
    uip_ipaddr_t ipaddr;
    uip_setethaddr(uNet_eth_address);

  #if DEBUG_UIP
    sprintf(bufuart,"Init uIP...\r\n");
	  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
  #endif

    //init uIP
    uip_init();

  #if DEBUG_UIP
    sprintf(bufuart,"Init ARP...\r\n");
	  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
  #endif

    //init ARP cache
    uip_arp_init();

    // init periodic timer
    clock_init();

  #if (ENABLE_DHCP == 1)
    uip_ipaddr(ipaddr, 0, 0, 0, 0);
    uip_sethostaddr(ipaddr);
    uip_setnetmask(ipaddr);
    uip_setdraddr(ipaddr);

    dhcpc_init(&uNet_eth_address.addr[0], 6);
    dhcpc_request();




  #else
#if DEBUG_UIP
  sprintf(bufuart,"Static IP %d.%d.%d.%d\r\n", _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
  sprintf(bufuart,"NetMask %d.%d.%d.%d\r\n", _net_mask[0], _net_mask[1], _net_mask[2], _net_mask[3]);
  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
  sprintf(bufuart,"Gateway %d.%d.%d.%d\r\n", _gateway[0], _gateway[1], _gateway[2], _gateway[3]);
  HAL_UART_Transmit(&huart1, bufuart, strlen((char *)bufuart), 1000);
#endif
    uip_ipaddr(ipaddr, _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, _net_mask[0], _net_mask[1], _net_mask[2], _net_mask[3]);
    uip_setnetmask(ipaddr);
    uip_ipaddr(ipaddr, _gateway[0], _gateway[1], _gateway[2], _gateway[3]);
    uip_setdraddr(ipaddr);
  #endif

    /// Стартуем программные таймеры
    // Таймер используется uIP для выполнения периодических действий с соединениями
    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    // Таймер используется uIP для обслуживания ARP
    timer_set(&arp_timer, CLOCK_SECOND * 10);
    // Таймер используется для уведомлеия брокера о том, что мы ещё живы
    timer_set(&mqtt_kalive_timer, CLOCK_SECOND * MQTT_KEEP_ALIVE);
    // Таймер используется чтобы инициировать подключение к брокеру если соединение не установлено
    timer_set(&mqtt_conn_timer, CLOCK_SECOND * 3);
    // Таймер используется для периодической отправки сообщения по MQTT
    timer_set(&mqtt_pub_timer, CLOCK_SECOND * 10);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		Time();



	    NetTask();
















		if (flIntrpt) {

			flIntrpt = 0;


			switch (flPin) {
			case intPin0:
				//------------------------------------------------------------------ PWR OFF NRF
				//------------------------------------------------------------------ PWR OFF MEMS
				//------------------------------------------------------------------ PWR OFF LCD
				//------------------------------------------------------------------ StandBy STM
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
				__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
				HAL_PWR_EnterSTANDBYMode();
				break;

			case intPin12:
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
				sTime.Hours = sTime.Hours;
				if (sTime.Minutes == 0x59) {
					sTime.Hours = sTime.Hours--;
				}

				sTime.Minutes = sTime.Minutes++;
				sTime.Seconds = 0x01;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
					Error_Handler();
				}

				break;
			case intPin13:
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
				sTime.Hours = sTime.Hours++;
				sTime.Minutes = sTime.Minutes;
				sTime.Seconds = 0x01;

				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
					Error_Handler();
				}


				break;
			default:
		/*		ssd1306_Clear();
				ssd1306_UpdateScreen();
				ssd1306_SetCursor(0, 32);
				ssd1306_WriteString("ERR INT", Font_16x26);
				ssd1306_UpdateScreen();*/
				HAL_Delay(2000);
				break;
			}
			flPin = intPinDef;
		}


	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//==============================================================================
// Процедура, вызываемая в основном цикле
//==============================================================================
void NetTask(void)
{
  u8_t i;

  uip_len = nic_poll();

  if (uip_len > 0)
  {
    if (BUF->type == htons(UIP_ETHTYPE_IP))
    {
      uip_arp_ipin();
      uip_input();

      /* If the above function invocation resulted in data that	should be
      sent out on the network, the global variable uip_len is set to a value > 0. */
      if (uip_len > 0)
      {
	uip_arp_out();
#if UIP_SPLIT_HACK
	uip_split_output();
#elif UIP_EMPTY_PACKET_HACK
	uip_emtpy_packet_output();
#else
	nic_send();
#endif
      }
    }
    else if (BUF->type == htons(UIP_ETHTYPE_ARP))
    {
      uip_arp_arpin();

      /* If the above function invocation resulted in data that	should be sent
      out on the network, the global variable uip_len is set to a value > 0. */
      if (uip_len > 0)
      {
	nic_send();
      }
    }
  }
  else
  {
    if (timer_expired(&periodic_timer))
    {
      timer_reset(&periodic_timer);

      for (i = 0; i < UIP_CONNS; i++)
      {
        uip_periodic(i);

        /* If the above function invocation resulted in data that	should be sent
        out on the network, the global variable uip_len is set to a value > 0. */
        if (uip_len > 0)
        {
          uip_arp_out();
#if UIP_SPLIT_HACK
          uip_split_output();
#elif UIP_EMPTY_PACKET_HACK
          uip_emtpy_packet_output();
#else
          nic_send();
#endif
        }
      }

#if UIP_UDP
      for (i = 0; i < UIP_UDP_CONNS; i++)
      {
        uip_udp_periodic(i);

        /* If the above function invocation resulted in data that should be sent
        out on the network, the global variable uip_len is set to a value > 0. */
        if (uip_len > 0)
        {
          uip_arp_out();
          nic_send();
        }
      }
#endif /* UIP_UDP */

      /* Call the ARP timer function every 10 seconds. */
      if (timer_expired(&arp_timer))
      {
        timer_reset(&arp_timer);
        uip_arp_timer();
      }
    }

    // Обрабатываем пропадание LINKа
    if (!enc28j60linkup())
    {
      uip_abort();
      //umqtt_disconnected(&mqtt);
    }

    // Отправляем KeepAlive-пакет брокеру если нужно
    if (timer_expired(&mqtt_kalive_timer))
    {
      timer_reset(&mqtt_kalive_timer);
      umqtt_ping(&mqtt);
    }

    // Устанавливаем соединение с брокером если оно не установлено
    if (timer_expired(&mqtt_conn_timer))
    {
      timer_reset(&mqtt_conn_timer);

      if ((mqtt.state == UMQTT_STATE_INIT) ||
          (mqtt.state == UMQTT_STATE_FAILED) ||
          (mqtt.state == UMQTT_STATE_DISCONNECTED))
      {
        nethandler_umqtt_init(&mqtt);
        umqtt_subscribe(&mqtt, MQTT_TOPIC_ALL);
      }
    }

    // Отправляем новое сообщение со своим IP брокеру
    if (timer_expired(&mqtt_pub_timer))
    {
      timer_reset(&mqtt_pub_timer);

      char message[16];
      uip_gethostaddr(_ip_addr);
      int8_t len = str_printf(message, sizeof(message), "%d.%d.%d.%d", _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
      if (len > 0)
      {
        umqtt_publish(&mqtt, "/enc28j60_stm32_client", (uint8_t *) message, strlen(message));
      }
    }
  }

  // Ждём окончания отправки пакета. Если проходит более N мс ожидания - делаем
  // сброс логики передатчика enc28j60
  nic_send_timer = 5;
  while (nic_sending())
  {
    //    if (enc28j60Read(EIR) & EIR_TXERIF)   // Этот способ не работает, т.к. флаг ошибки передачи не устанавливается
    // Истекло время ожидания окончания передачи
    if (!nic_send_timer)
    {
      nic_txreset_num++;
      enc28j60ResetTxLogic();
    }
  }
}
//==============================================================================



/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
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
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
