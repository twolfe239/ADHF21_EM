/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "main.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
/*	tmr_handler tmr2_handler;



	void tmr2_init(uint32_t Freq, void (*func)(void))
	{
	  // ���������� �������, ������� ����� �������� � ����������� ���������� �������
	  tmr2_handler = func;

	  // �������� ������������ ������� (������� ������������ = AHB/2)
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	  // �������� ������� ������ APB1, �� ������� ����� ������� �������� ��� �������
	  RCC_ClocksTypeDef rcc_clocks;
	  RCC_GetClocksFreq(&rcc_clocks);
	  uint32_t APB1 = rcc_clocks.PCLK1_Frequency;
	  if (RCC->CFGR & RCC_CFGR_PPRE1)
	    APB1 <<= 1;

	  // ������� ����������� �������� ������� � �������� ��� ���������
	  uint32_t Coef = APB1 / Freq;
	  uint32_t OC2_value = 100000;
	  uint16_t Prescaler = 1;

	  // ���������� �������� ������� ������� � �����������
	  for (uint8_t i = 0; i < 16; i++)
	  {
	    OC2_value = Coef / Prescaler;

	    if (OC2_value < 65536)        // �������� ������
	      break;

	    Prescaler <<= 1;            // ��������� ��������
	  }
*/
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
/*  TIM_TimeBaseInitTypeDef TIM2_InitStruct;
  TIM2_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM2_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM2_InitStruct.TIM_Prescaler = Prescaler - 1;
  TIM2_InitStruct.TIM_Period = OC2_value - 1;
  TIM2_InitStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM2_InitStruct);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  // ����������� ���������� �������
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);*/
  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//==============================================================================

/*
//==============================================================================
// ��������� �������� ������
//==============================================================================
void tmr2_start(void)
{
  TIM_Cmd(TIM2, ENABLE);
}
//==============================================================================


//==============================================================================
// ��������� ������������� ������
//==============================================================================
void tmr2_stop(void)
{
  TIM_Cmd(TIM2, DISABLE);
}
//==============================================================================

*/
//==============================================================================

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
