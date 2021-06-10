/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "nrf24l01.h"
#include "arm_math.h"
#include "nrf24.h"
//#include "svpwm.h"
volatile float ABS_Position;
extern volatile int Encoder_CNT;
uint8_t check;
const float SQRT13 =  0.5773502f;   //	1/sqrt(3)
const float SQRT23 =  1.1547005f;   //	2/sqrt(3)
const float SQRT32 = 	0.8660254f;  	//	sqrt(3)/2
const int SQRT32_INT = 28377;
const int SQRT13_INT = 18918;
const int SQRT23_INT = 37836;
const float PIRAD = 3.1415926f;
const float PIRAD_2 = 1.57079f;

//ENCODER CONFIG
const uint16_t ENC_PULSES = 8000;
const uint16_t ENC_INITIAL = ENC_PULSES/2 - 1;
const float ENC_PULSES_TO_DEGREES = 360.0f/ENC_PULSES;

//MOTOR CONFIG
const uint16_t MOTOR_POLES = 4;


//REDUCER GEAR
const float GEAR_RATIO = 1.0f/30.0f;
typedef union {
		float f;
		unsigned char c[4];
}FloatU;
int ix;
typedef struct {
	int P;
	int I;
	int D;
	int P2;
	int SetPoint;
} Control_Parameter;
extern float IUF,IVF,VBus,VFBK2,TempSTK; 
int I_PROT_COUNT = 15, Protect_I_Count = 0;
int I_PROTECTION = 1000;
uint32_t ADC_values[4];
extern int IU_OFFSET,IV_OFFSET;
float IUCALIB,IVCALIB,Data_Position,DataT;
float SP_spd,SP_q,SP_d;
char Status;
int Pos_int;
int POT = 0;
int IQM;
uint16_t Pos_uint;


int delay;
volatile short int Pos_degrees,Pos_elec,Pos_Sin,Pos_Cos;
volatile short int SENO,SENO2;
uint16_t merda;

//float Pos_e;
Control_Parameter CSpd, CPosition, CCur;
int Va,Vb,Vc;
int Va2,Vb2,Vc2,Vm2,VaMAX;
//float ialfa,ibeta;
int Error_d,Iterm_d,Pterm_dTMP,Pterm_d;
int Error_q,Iterm_q,Pterm_q,Pterm_qTMP;
FloatU id,iq;
extern int IU,IV;
int ialfa_INT,ibeta_INT;
int Id,Iq;
int qLimit;
int qLimit2,qLimit3;
volatile int Pos_temp,Pos_temp2,Pos_temp3;
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void nRF24CSN_L(void) {
	HAL_GPIO_WritePin(nRF24_CSN_GPIO_Port,nRF24_CSN_Pin,0);
}
void nRF24CSN_H(void) {
	HAL_GPIO_WritePin(nRF24_CSN_GPIO_Port,nRF24_CSN_Pin,1);
}
void nRF24CE_L(void) {
	HAL_GPIO_WritePin(nRF24_CE_GPIO_Port,nRF24_CE_Pin,0);
}
void nRF24CE_H(void) {
	HAL_GPIO_WritePin(nRF24_CE_GPIO_Port,nRF24_CE_Pin,1);
}

void Trip(void){
	
	TIM1->CCER &= 0xEAAA;						//Disable OUTPUTS 1-6
	TIM1->CR1  &= 0xFFFE;						//STOP Counter
	TIM1->BDTR &= ~TIM_BDTR_MOE;
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);//USART1_IRQHandler	
	while (1)
	{
	  TIM1->BDTR &= ~TIM_BDTR_MOE;	//MASTER OUTPUT PWM
	}
	
}
void PMSM_FOC(void){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,1);
	if((delay > 10000) && (Status <= 'C')) {   
		//delay = 2 = 1ms/ 4000 = 2s
		
		delay++;
		if(delay < 24000) {
			
			if(delay > 10002) {
				TIM1->CCER |= 0x555;										//Enable OUTPUTS 1-6
				TIM1->BDTR |= TIM_BDTR_MOE;
			}
			if(delay > 14000) {
				TIM1->SR &= 0x7F;						//Clear Fault Break Status
				TIM1->BDTR |= 0x1000;				//Habilita Fault Break
				TIM1->DIER |= TIM_DIER_BIE;		//Ativa interrupt de Fault Break
				delay = 25000;
				
				if ((IUCALIB <= 30.0f) && (IVCALIB <= 30.0f)) {	//Ajusta Offset de corrente ACS712
					IU_OFFSET = IU_OFFSET - ((int) IUCALIB);
					IV_OFFSET = IV_OFFSET - ((int) IVCALIB);
					Status = 'B';
				}
				else {
					//Fault_OFFSET_I = 1;
					Status = 'O';		//OFFSET ERROR 
				}
				
			}
			TIM1->CCR1 = 3600/2 + (int) 450;		//Leg U
			TIM1->CCR2 = 3600/2 + (int) 0;		//Leg V
			TIM1->CCR3 = 3600/2 + (int) 0;		//Leg W
			TIM2->CNT = ENC_INITIAL;
		}
		else {
		if(delay > 25000) delay = 25000;
			
		//Proteção
		//IU e IV
		//I_PROTECTION = 1000 = 12.08A
		
		
			
		if ((IU > I_PROTECTION) || (IV > I_PROTECTION) || (IU < -I_PROTECTION) || (IV < -I_PROTECTION)) Protect_I_Count++;
		else 	Protect_I_Count = 0;
		
		if (Protect_I_Count > I_PROT_COUNT) {		
			Status = 'I';	//Overcurrent
			//Fault_Overcurrent	= 1;
			Trip();
			return;
		}

		//Calculo da posição 8000p/Rotation
		//Pos_elec 4 turns/rotation -> Electrical Position -> 4 Pole Pairs / 0 - 360°/ 0 - 32767
		//Pos_degrees mechanical position / 0 - 360°/ 0 - 32767

		Pos_temp3 = (TIM2->CNT - 3999) + Encoder_CNT;
		
		Pos_temp2 = Pos_temp3%2000;
		Pos_elec = (Pos_temp2*163835/10000) & 0x7FFF;				//angulo elétrico motor 4 Par de Polos
		Pos_degrees = (Pos_temp2*40958/10000) & 0x7FFF;				//angulo mecanico
		Pos_Sin = arm_sin_q15(Pos_elec);
		Pos_Cos = arm_cos_q15(Pos_elec);

		//TIPO DE CONTROLE
		
		#define POSITION 	0
		#define SPEED 		0
		#define CURRENT 	1
		
		#define FIELDW 	0
		
				#if (POSITION) 	//Controle de Posição
			SP_pos 	= Sy;//SP_pos + (float)oi3*0.01f;
		#elif (CURRENT) 	//Controle de corrente
			SP_q		=(((float)POT) + SP_q*15)/16;
		#elif (SPEED) 	//Controle de RPM/Corrente
 				SP_spd	=(((float)POT/70) + SP_spd*15)/16;
			
		#endif
		
		//*******   A B C to Alpha Beta
		
		//ialfa = SQRT23*Iam - SQRT23*Ibm/2 - SQRT23*Icm/2; ia + ib + ic = 0 / ic = -ia -ib
		//ibeta = 0 + SQRT2_2*Ibm - SQRT2_2*Icm;

		
		ialfa_INT = IU;
		ibeta_INT = SQRT13_INT*IU/32767 + SQRT23_INT*IV/32767;
		
		//Alpha Beta to DQ
		
		Id = (Pos_Cos*ialfa_INT/32767) + (Pos_Sin*ibeta_INT/32767);
		Iq = -(Pos_Sin*ialfa_INT/32767) + (Pos_Cos*ibeta_INT/32767);
		
		IQM = (Iq*39599)/32767;
		//id.f =  (Pos_Cos*ialfa_INT/32767) + (Pos_Sin*ibeta_INT/32767);
		//iq.f = -(Pos_Sin*ialfa_INT/32767) + (Pos_Cos*ibeta_INT/32767);
		
		//*******   Current control 'D' part
		{
		#if (SPEED &&  FIELDW) 	//Controle de RPM/Corrente
			SP_d = (int)Z	;		//Field Weakening
			SP_d = SP_d/100.0f;
		#endif

		Error_d=SP_d-Id;
		Iterm_d = Iterm_d + Error_d*CCur.I/32767;					//+Iterm2_d;
		if (Iterm_d > 900) Iterm_d = 900;					//anti-windup
		else if (Iterm_d < -900) Iterm_d = -900;
		//Pterm_dTMP = Error_d*CCur.P.f;
		Pterm_d = Error_d*CCur.P/32767 + Iterm_d;				//Resultante Controle Corrente D

		if (Pterm_d > 900) Pterm_d = 900;					//Limitador Saída
		else if (Pterm_d < -900) Pterm_d = -900;
		}	
				
		//*******   Current control 'Q' part
		{
		arm_sqrt_q31(3534400 - (Pterm_d*Pterm_d),&qLimit);				//Limitador geométrico do 'q' baseado no Pterm_D
		qLimit = qLimit/46341;		//SQRT function adequação
		if (qLimit > 1880) qLimit = 1880;

		Error_q=SP_q-Iq;
			
		Iterm_q = Iterm_q + Error_q*CCur.I/32767;

		if (Iterm_q > qLimit) Iterm_q = qLimit;					//anti-windup
		else if (Iterm_q < -(qLimit)) Iterm_q = -qLimit;
		//Pterm_qTMP = Error_q*CCur.P.f;
		Pterm_q = Error_q*CCur.P/32767 + Iterm_q;				//Resultante Controle Corrente Q

		if (Pterm_q > qLimit) Pterm_q = qLimit;					//Limitador Saída
		else if (Pterm_q < -qLimit) Pterm_q = -qLimit;		
		
		//Q positivo Movimento ANTIHORARIO, Angulação aumenta (Positiva) +

		}
		
		ialfa_INT = (Pos_Cos*Pterm_d/32767) - (Pos_Sin*Pterm_q/32767);		
		ibeta_INT = (Pos_Sin*Pterm_d/32767) + (Pos_Cos*Pterm_q/32767);	
		
		Va = ialfa_INT;
		Vb = -ialfa_INT/2 + SQRT32_INT*ibeta_INT/32767;
		Vc = -ialfa_INT/2 - SQRT32_INT*ibeta_INT/32767;
		
		Vm2 = (MAX(Va,Vb,Vc) + MIN(Va,Vb,Vc))/2;
		Va2 = -Vm2 + Va;
		Vb2 = -Vm2 + Vb;
		Vc2 = -Vm2 + Vc;
		
		Status = 'C';	//Control Loop working
	
		if (VaMAX < Va2) {
			VaMAX = Va2;
		}
		
		TIM1->CCR1 = 1800 + (int) Va2;//Va;		//Leg A
		TIM1->CCR2 = 1800 + (int) Vb2;// + (int)Tabela[indiceB]*amp;		//Leg B
		TIM1->CCR3 = 1800 + (int) Vc2;// + (int)Tabela[indiceC]*amp;		//Leg C
				
	}
		
	}
		else {
		IUCALIB = (IU + IUCALIB*2000)/2001;
		IVCALIB = (IV + IVCALIB*2000)/2001;	
		delay++;
		}
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
}
int MAX(int v1,int v2,int v3) {
	if ((v1 >= v2) && (v1 >= v3)) {
		return v1;
	}
	else if ((v2 > v1) && (v2 > v3)) {
		return v2;
	}
	else {
		return v3;
	}
}
int MIN(int v1,int v2,int v3) {
	if ((v1 <= v2) && (v1 <= v3)) {
		return v1;
	}
	else if ((v2 < v1) && (v2 < v3)) {
		return v2;
	}
	else {
		return v3;
	}
}

uint32_t i,j,k;

// Buffer to store a payload of maximum width
uint8_t nRF24_payload[32];

// Pipe number
nRF24_RXResult pipe;

// Length of received payload
uint8_t payload_length;

void nRF24_GPIO_Init(void) {
int merda;
	merda= 1;
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	// Send byte to SPI (TXE cleared)
	SPI1->DR = data;
	//SPI_I2S_SendData(nRF24_SPI_PORT, data);
	// Wait while receive buffer is empty
	while ((SPI1->SR & SPI_SR_RXNE) == 0);

	// Return received byte
	return (uint8_t)SPI1->DR;
}

void INIT_ALL(void) {
	
	//SysTick_Config(1440); ////144000 -> 2ms
	//DMA INIT
			//ADC_DMA

	DMA1_Channel1->CNDTR = 2;															//Tamanho do dado
	DMA1_Channel1->CPAR = (uint32_t)&ADC12_COMMON->DR;
	DMA1_Channel1->CMAR = (uint32_t)&ADC_values;
	//DMA1_Channel1->CCR = 0x3AAB;	//Canal habilitado, FULL INT, HALFWORD	
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	
	//ADC INIT
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);

	
	ADC1->CR1 |= ADC_CR1_EOCIE;
	ADC1->CR2 = 0;
	ADC2->CR2 = 0;
	ADC1->CR2 |= ADC_CR2_DMA;
	ADC2->CR2 |= ADC_CR2_DMA;
	
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_EXTTRIG;
	ADC2->CR2 |= ADC_CR2_ADON | ADC_CR2_EXTTRIG;	
	
	// TIMER 1 PWM
	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
	TIM1->DIER |= TIM_DIER_BIE;	//BREAK INTERRUPT
	TIM1->BDTR |= TIM_BDTR_MOE;	//MASTER OUTPUT PWM
	
	TIM1->CR1 |= TIM_CR1_CEN;		//TIMER EN
	
	//TIMER 2 ENCODER
	TIM2->DIER |= TIM_DIER_CC4IE | TIM_DIER_CC3IE;
	TIM2->CCER |= TIM_CCER_CC4E | TIM_CCER_CC3E;
	TIM2->CCR3 = ENC_INITIAL-2000;
	TIM2->CCR4 = ENC_INITIAL+2000;
	
	TIM2->CNT = ENC_INITIAL;//3999;
	TIM2->ARR = 7999;//
	TIM2->CR1 |= TIM_CR1_CEN;		//TIMER EN
	
	IU_OFFSET = 2045;
	IV_OFFSET = 2035;
	Status = '0';	//Initial State
	
	CCur.P = 0.110f * 32767;		//15			//120				//1000	//300
	CCur.I = 0.0305f * 32767;		//10			//30				//30	//12
	
	//CSpd.P.f = 1.0f;			//0.5			//1
	//CSpd.I.f = 0.002f;		//0.0005		//0.008
	//CSpd.D.f = 0.05f;			//0.05		//0.5
	
	//CPosition.P.f = 0.01f;
	//CPosition.P2.f = 0.03f;
	//CPosition.I.f = 0.0002f;		//I when Locked
	//CPosition.D.f = 0.2f;			//P2 when locked
	SP_d = 0;
	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->CR2 |= SPI_CR2_RXNEIE;
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  INIT_ALL();
	HAL_Delay(2000);
	HAL_GPIO_WritePin(relay2_GPIO_Port,relay2_Pin,1);	
	Status = 'A';	//Initial State + Relayok
	
			//svpwm1.b_Vm = 10; 
		//svpwm1.b_freq = 10;
	//nRF24CE_L();
	//check = nRF24_Check();
	

	 
	//example();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
  {
    /* USER CODE END WHILE */
	
    /* USER CODE BEGIN 3 */
		ABS_Position = ((float)Pos_temp3)*ENC_PULSES_TO_DEGREES;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT_ALTERTRIG;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 3600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 80;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|relay2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nRF24_CSN_GPIO_Port, nRF24_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nRF24_CE_Pin|DBG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin relay2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|relay2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF24_CSN_Pin */
  GPIO_InitStruct.Pin = nRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF24_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF24_IRQ_Pin Encoder_Z_Pin */
  GPIO_InitStruct.Pin = nRF24_IRQ_Pin|Encoder_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF24_CE_Pin */
  GPIO_InitStruct.Pin = nRF24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF24_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DBG_Pin */
  GPIO_InitStruct.Pin = DBG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DBG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INP_4_Pin */
  GPIO_InitStruct.Pin = INP_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INP_4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
