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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "enc28j60.h"
#include "uip.h"
#include "uip_arp.h"
#include "uip-conf.h"
#include "hello-world.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct bme680_dev gas_sensor;
struct bme680_field_data data;
RTC_TimeTypeDef sTime;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile char bufbme[50];
volatile char bufbme1[50];
volatile uint32_t var1 = 0;
volatile uint32_t var2 = 0;
volatile uint8_t set_required_settings;
volatile int8_t rslt = 0;
volatile uint16_t meas_period;
volatile uint32_t ii = 0;
volatile uint8_t flPin = 239;
volatile uint8_t flIntrpt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);

void BME680_Read(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE BEGIN 2 */
	//------------------------------------------------------------------ OLED
	ssd1306_Init();
	ssd1306_FlipScreenVertically();
	ssd1306_Clear();
	ssd1306_SetColor(White);
	//------------------------------------------------------------------ Knock-knock-knock
	SPON();
	HAL_Delay(50);
	SPOFF();
	HAL_Delay(50);
	SPON();
	HAL_Delay(45);
	SPOFF();
	//------------------------------------------------------------------ BME INI
	gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
	gas_sensor.intf = BME680_I2C_INTF;
	gas_sensor.read = user_i2c_read;
	gas_sensor.write = user_i2c_write;
	gas_sensor.delay_ms = user_delay_ms;
	gas_sensor.amb_temp = 25;

	/* Set the temperature, pressure and humidity settings */
	gas_sensor.tph_sett.os_hum = BME680_OS_16X;
	gas_sensor.tph_sett.os_pres = BME680_OS_16X;
	gas_sensor.tph_sett.os_temp = BME680_OS_16X;
	gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_127;

	/* Set the remaining gas sensor settings and link the heating profile */
	gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	/* Create a ramp heat waveform in 3 steps */
	gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
	gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

	/* Select the power mode */
	/* Must be set before writing the sensor configuration */
	gas_sensor.power_mode = BME680_FORCED_MODE;

	/* Set the required sensor settings needed */
	set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL
			| BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

	rslt = bme680_init(&gas_sensor);

	/* Set the desired sensor configuration */
	rslt = bme680_set_sensor_settings(set_required_settings, &gas_sensor);

	/* Set the power mode */
	rslt = bme680_set_sensor_mode(&gas_sensor);

	bme680_get_profile_dur(&meas_period, &gas_sensor);

	ssd1306_Clear();



	// это будет наш МАС-адрес
	        struct uip_eth_addr mac = {{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x00 }};
	        // проинитим наш  enc28j60
	        enc28j60Init(mac.addr,0);




	        // инициализация стека
	        uip_init();
	        uip_arp_init();

	        // инициализация приложения, потом втулим сюда веб-сервер
	        hello_world_init();
	        // установим наш МАС
	        uip_setethaddr(mac);

	        // установим адрес хоста (не используем dhcp)
	        // наш хост будет доступен по адресу 192.168.2.55

	        uip_ipaddr_t ipaddr;
	        uip_ipaddr(ipaddr, 192, 168, 0, 4);
	        uip_sethostaddr(ipaddr);
	        uip_ipaddr(ipaddr, 192, 168, 0, 1);
	        uip_setdraddr(ipaddr);
	        uip_ipaddr(ipaddr, 255, 255, 255, 0);
	        uip_setnetmask(ipaddr);















	        uint32_t i;
	           uint8_t delay_arp = 0;



  /* USER CODE END 2#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN]) */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


HAL_Delay(500);
           delay_arp++;
           for (i = 0; i < UIP_CONNS; i++) {
                   uip_periodic(i);
                   if (uip_len > 0) {
                           uip_arp_out();
                           enc28j60PacketSend((uint8_t *) uip_buf, uip_len);
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
#endif /* UIP_UDP */

           if (delay_arp >= 50) { // один раз за 50 проходов цикла, около 10 сек.
                   delay_arp = 0;
                   uip_arp_timer();
           }


           uip_len = enc28j60_recv_packet((uint8_t *) uip_buf, UIP_BUFSIZE);

                         if (uip_len > 0) {
                               //  if (BUF->type == htons(UIP_ETHTYPE_IP)) {
                                         uip_arp_ipin();
                                         uip_input();
                                         if (uip_len > 0) {
                                                 uip_arp_out();
                                                 enc28j60PacketSend((uint8_t *) uip_buf, uip_len);
                         //                }
                            /*     } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
                                         uip_arp_arpin();
                                         if (uip_len > 0) {
                                                 enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
                                         }*/
                                 }
                         }




















		if (flIntrpt) {

			flIntrpt = 0;


			switch (flPin) {
			case intPin0:
				//------------------------------------------------------------------ PWR OFF NRF
				//------------------------------------------------------------------ PWR OFF MEMS
				gas_sensor.power_mode = BME680_SLEEP_MODE;
				rslt = bme680_set_sensor_mode(&gas_sensor);
				if (!(rslt)) {
					ssd1306_Clear();
					ssd1306_UpdateScreen();
					ssd1306_SetCursor(0, 32);
					ssd1306_WriteString("BME SLP", Font_16x26);
					ssd1306_UpdateScreen();
					HAL_Delay(2000);

				}
				else {
					ssd1306_Clear();
					ssd1306_UpdateScreen();
					ssd1306_SetCursor(0, 32);
					ssd1306_WriteString("ERR BME", Font_16x26);
					ssd1306_UpdateScreen();
					HAL_Delay(2000);
				}
				//------------------------------------------------------------------ PWR OFF OLED
				ssd1306_Clear();
				ssd1306_UpdateScreen();
				ssd1306_SetCursor(0, 32);
				ssd1306_WriteString("STANDBY", Font_16x26);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				ssd1306_Clear();
				ssd1306_UpdateScreen();
				ssd1306_DisplayOff();
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
				ssd1306_Clear();
				ssd1306_UpdateScreen();
				ssd1306_SetCursor(0, 32);
				ssd1306_WriteString("ERR INT", Font_16x26);
				ssd1306_UpdateScreen();
				HAL_Delay(2000);
				break;
			}
			flPin = intPinDef;
		}

		Time();

		BME680_Read();

		ssd1306_UpdateScreen();

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


void BME680_Read(void) {

	user_delay_ms(meas_period);
	rslt = bme680_get_sensor_data(&data, &gas_sensor);

	sprintf(bufbme1, "T: %.2f degC", data.temperature / 100.0f);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString((char*) bufbme1, Font_7x10);

	sprintf(bufbme1, "P: %.2f hPa", data.pressure / 100.0f);
	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString((char*) bufbme1, Font_7x10);

	sprintf(bufbme1, "H %.2f %%rH ", data.humidity / 1000.0f);
	ssd1306_SetCursor(0, 40);
	ssd1306_WriteString((char*) bufbme1, Font_7x10);

	if (data.status & BME680_GASM_VALID_MSK) {

		sprintf(bufbme1, "G: %.2f Kohms ", data.gas_resistance / 1000.0f);
		ssd1306_SetCursor(0, 50);
		ssd1306_WriteString((char*) bufbme1, Font_7x10);
	}

	/*  Trigger the next measurement if you would like to read data out continuously*/
	if (gas_sensor.power_mode == BME680_FORCED_MODE) {
		rslt = bme680_set_sensor_mode(&gas_sensor);
	}

}

void user_delay_ms(uint32_t period) {
	HAL_Delay(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len) {
	int8_t rslt = 0;
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, dev_id, reg_addr, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) reg_data, len, 0x10000);
	if (status != HAL_OK)
		rslt = -3;
	return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len) {
	int8_t rslt = 0;
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, dev_id, reg_addr, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) reg_data, len, 0x10000);
	if (status != HAL_OK)
		rslt = -3;

	return rslt;

}

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
