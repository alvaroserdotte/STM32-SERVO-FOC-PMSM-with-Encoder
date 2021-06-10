/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern const uint16_t ENC_INITIAL;
volatile int Encoder_CNT;
volatile int aligned = 0;
extern void Trip(void);
uint8_t SPI_RX;
extern volatile short int Pos_degrees,Pos_elec,Pos_Sin,Pos_Cos;
extern void PMSM_FOC(void);
short int  Z_TABLE[100];
int Z_TABLE_idx;
long int	ADC_DATA1,ADC_DATA2;
int ADC_Status;
int CurrentLoopPer;
int IU_OFFSET,IV_OFFSET;
float IUF,IVF,VBus,VFBK2,TempSTK; 
int IU_INT,IV_INT,IUM,IVM,IU,IV;
extern uint32_t ADC_values[4];
extern nrf24l01 nrf;
extern char Status;
short int cnt1,cnt_antes;
float rpm,RPMf;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	//nrf_irq_handler(&nrf);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	//Z_TABLE_idx++;
	//if (Z_TABLE_idx > 99) Z_TABLE_idx = 0;
	//Z_TABLE[Z_TABLE_idx] = Pos_degrees;
	if (aligned == 0) {
		TIM2->CNT = ENC_INITIAL;
		Encoder_CNT = 0;
		aligned = 1;
		EXTI->IMR &= ~EXTI_IMR_IM4;
	}
	
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	//ADC_DATA1 = ADC1->DR;;
	//ADC_DATA2 = ADC2->DR;
	ADC_Status = ADC1->SR;
	//IU,IV,VBus,VFBK2,TempSTK;
	
	CurrentLoopPer++;

	//IV = (float) (0x0000FFFF & ADC_values[0])-IV_OFFSET;
	//IU = (float) (ADC_values[0]>>16)-IU_OFFSET;

	VBus = ((float) (0x0000FFFF & ADC_values[1])-0.0f)/24.5f;
	//VFBK2 = (float) (ADC_values[1]>>16)-0.0f;
	//TempSTK = 3320.0f - (float) (0x0000FFFF & ADC_values[2]);
	
	 
	
	IU_INT = -((ADC_values[0]>>16)-IU_OFFSET);
	IV_INT = -((0x0000FFFF & ADC_values[0])-IV_OFFSET);
	
	IU = (IU_INT + IU*4)/5;
	IV = (IV_INT + IV*4)/5;
	// 3,3V->4096 = 8.1380208333333333333333333333333e-4 mV/bit resolution
	// ACS712-20A-> 100mV/A ->conversão de 5 para 3,3V-> 66.6667mV/A
	// 82,74747 count/Ampere ou * 39599/32767 para retornar corrente*100
	//Conversão somente no float UVF IUF
	IUF = IU/82.74747f;
	IVF = IV/82.74747f;
	
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//144000 -> 2ms
	//3000 RPM = 50R/s @8000ppr = 50*8000/s= 400000p/segundo 
	// em 2ms => 400000/500Hz=80000 pulsos
	// em 10kHz => 400000/10000=40pulsos
	// = 40 contagens em 3000RPM
		//short int cnt1,cnt_antes;
	//float rpm;
	cnt1 = TIM2->CNT;
	rpm = cnt1 - cnt_antes;
	if ((rpm < 60) & (rpm > -60)) RPMf = ((rpm*75)+RPMf*50)/51;
	cnt_antes = cnt1;
	if (CurrentLoopPer >= 2) {
		
		PMSM_FOC();
		//Periodic();
		CurrentLoopPer = 0;
	}	
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt.
  */
void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */
	TIM1->BDTR &= ~TIM_BDTR_MOE;	//MASTER OUTPUT PWM
	Status = 'S';	//Trip STK
  /* USER CODE END TIM1_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */
	Trip();
	while (1)
  {
  }
  /* USER CODE END TIM1_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	
	//Acumula encoder no contador
	Encoder_CNT += TIM2->CNT - ENC_INITIAL;
	TIM2->CNT = ENC_INITIAL;
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
	//SPI_RX = SPI1->DR;
	//nrf_packet_received_callback(&nrf,&SPI_RX);
  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
