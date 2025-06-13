/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define adcbufflen 1000
#define adcbufffsrlen 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Error */
osThreadId_t ErrorHandle;
const osThreadAttr_t Error_attributes = {
  .name = "Error",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
volatile uint16_t adc_buf[adcbufflen];
volatile uint16_t adc_buf_fsr[adcbufffsrlen];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void alarms(void *argument);

/* USER CODE BEGIN PFP */
float increase_variable(float variable, float fac);
float decrease_variable(float variable, float fac);
float keep_variable_constant(float variable);
float algo(float rate, float process_var, float setpoint);

void IncreaseRate(void);
void DecreaseRate(void);
static void digits(void);
static void start(void);
void stop(void);
void clear(void);
void air(void);
void sens(void);
void stop_process(void);
void start_process(void);
//void detect_peaks_from_pressure_array(void);
//void floatToCharArray(float value, char* array);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Cmd_End[3] = {0xFF,0xFF,0xFF};// command end sequence
char buf[]={"HELO WORLD"};
uint8_t rxbuf[9]; 
void NEXTION_SendVal (char *ID, int string)
{
	char buf[50];
	int len = sprintf (buf, "%s.val=%d", ID, string);
//	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
//	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}
void NEXTION_ChangeScreen (int page)
{
	char buf[50];
	int len = sprintf (buf, "page %d", page);
//	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
//	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

volatile  uint32_t pressure;
float yn,yn1=0;

int pump_stop=0;
float xn[3]={0};
int	y[1000]={0};	
int deflation = 0;
uint32_t pressure_plot;
float beta=0.1;
float peaks[1000]={0};
float peaks_diff[1000]={0};
float peaks2[500]={0};
float peaks_pressure[50]={0};
float pulse_time[50]={0};
int ind,k=0;
int new_val=0;
int map_ind=0;
int fstart=0;
float f1_avg,f2_avg,f3_avg,f4_avg=0;


int avg_pressure[2]={0};
float peak=0;
int tt1,tt2,ttime,rt1,rt2,error_wait1,error_wait2=0;
int map=0;
float map_amp,dia_amp,sys_amp=0;
float dia_peaks[200],sys_peaks[200]={0};
int sys_ind,dia_ind,sys,dia=0;
float peakplot=0;
int mapflag=0;
int pulse_rate=0;
int findSys(float array[], float val);
int findDia(float array[], float val);
float max(float x[],int n);
int error=0;
float peak_sensitivity=0.05;
int p_ind=0;
int temp = 0;
float difft=0;
int graph=0;
char print_pressure[3];
int beat=0;
int p_check=1;
int esnd = 0;
int test=0;


float rate = 0.0f;         // Initial rate value
float process_var = 0.0f;  // Initial process variable value
float setpoint = 70.0f;
float infusion = 0.0f; 
int pre_inf = 0;
int inf = 0;


int bpdiff=0;
int prevsys=0;
float sensitivity=0;
float sensitivityavg=0;
int i=0;
double stepsize=0;
float dose=0;
float prevdose=0;
float dosediff=0;
int count=1;
char doseArray[5] = {0};
int length=0;
int n=0;
int val;
float t;
int bpArray[20];
float doses[20];
float sensitivityArray[20];
float sensitivityavgArray[20];
int p=0;
int  condition;
int pressureArray[1000];
int j=0;
uint8_t buffer[4] = {0x00,0xEB,0x02,0xED};
uint8_t digitbuff[14]={0x00,0xEB,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x6c,0x01,0x01,0xED};
uint8_t clearbuff[14]={0x00,0xEB,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0xED};
int flowrateval;
int volumeval;
uint8_t rxbuff1[5];
uint8_t errorbuffer[4] = {0x00,0xEB,0x11,0xED};
int rf=0;
uint8_t iperror=0;
uint8_t volumebuff[8];
uint8_t volbuff[4]={0x00, 0xEB, 0x12, 0xED};
uint8_t buff[4];
int ab=0;
int cd=0;
float volume=0;
int q=0;

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	NEXTION_ChangeScreen(0);
	HAL_UART_Receive_DMA(&huart1,(uint8_t*) rxbuf, 9);
	osDelay(50);
	HAL_UART_Receive_DMA(&huart2,(uint8_t*) rxbuff1, 5);

	//IncreaseRate();
	//HAL_UART_Transmit(&huart1,(uint8_t*) buffer, 4, 100);
	
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  /* creation of Error */
  ErrorHandle = osThreadNew(alarms, NULL, &Error_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
//	error_wait1=HAL_GetTick();
//	NEXTION_SendVal("sec",60);
//	while (error_wait2-error_wait1<60000)
//	{
//	error_wait2=HAL_GetTick();
//	//osDelay(10);
//	}
//	
//	goto here;
	
	
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  if (p_check==1){

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

 
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	

}
	else {
	
	hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
	
	
	 /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

	}
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pump_Pin|valve_Pin|S2_Pin|B_Pin
                          |Power_Pin|S1_Pin|S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIGNAL_Pin|A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pump_Pin valve_Pin S2_Pin B_Pin
                           Power_Pin S1_Pin S0_Pin */
  GPIO_InitStruct.Pin = pump_Pin|valve_Pin|S2_Pin|B_Pin
                          |Power_Pin|S1_Pin|S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SIGNAL_Pin A_Pin */
  GPIO_InitStruct.Pin = SIGNAL_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
	
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sens(){
	dosediff=dose-prevdose;
					prevdose=dose;
			if ((bpdiff>0)&&(dosediff>0)&&(dosediff<bpdiff)){
				if (count<=4)
				{
					sensitivity=100-((dosediff)/(bpdiff))*100;
					if(count==1){
					sensitivityavg= sensitivity;
					}
					else
					{
						sensitivityavg=(sensitivityavg+sensitivity)/2;
				}
					sensitivityArray[count]=sensitivity;
				sensitivityavgArray[count]=sensitivityavg;
					count++;
			}
			
		}
}
int findSys(float array[], float val){
	
	int p;
	float closest=0;	
	float diff=0;
	diff=	val-array[0];
	closest=diff;
	for (int i=0;i<=map_ind;i++){
	diff=	val-array[i];
	if (diff>0 && diff < closest){
		closest=diff;
		p=i;
		}
	}
	
return p;
}
int findDia(float array[], float val) {
    int di = p_ind / 2;  // start from last peak
    float closest = fabs(array[di] - val);
    float threshold = 0.35;  // or adjust to 0.3 for more sensitivity

    for (int i = map_ind + 1; i < p_ind / 2; i++) {
        // Look for where amplitude falls below threshold * max (val = dia_amp)
        if (array[i] < threshold * val) {
            // Additionally, confirm next few points are also below to avoid noise
            if (array[i + 1] < threshold * val && array[i + 2] < threshold * val) {
                return i;
            }
        }
    }

    // Fallback to previous logic
    for (int i = p_ind / 2; i >= map_ind + 1; i--) {
        float diff = fabs(array[i] - val);
        if (diff < closest) {
            closest = diff;
            di = i;
        }
    }

    return di;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
			if(rf==0){
        iperror = rxbuff1[3];
        // Process the value as needed
			}
			else
			{
			        iperror = rxbuff1[3];
 
			}
			
			}
}
//int findDia(float array[], float val){

//	int di = 0;
//	difft = 0;
//	float closest =	fabs(array[p_ind/2]-val);

//	for (int i=p_ind/2 ; i>=map_ind+1 ; i--)

//	{
//		difft = fabs(array[i]-val);
//		if(difft <= closest) 
//		{
//			di = i;
//			closest = difft;
//		}
//		else closest = closest;
//		difft = 0;
//	}
//	return di;
//	
//}


//float max(float x[],int n)
//{
////	float t;
//	t=x[0];
//	if (map_ind ==0){
//	for(int i=0;i<n;i++)
//        {
//		if(x[i]>t){
//			t=x[i];
//			map_ind=i;
//		}
//	}
//					
//}
//return(t);
//}
float max(float x[],int n)
{
//	float t;
	
	t=x[0];
	for(int i=0;i<n;i++)
        {
		if(x[i]>t&& t!=0&&x[i]!=0){
			t=x[i];		
			map_ind=i;
		}
	}

return(t);
}

void IncreaseRate(void)
{
    if (dose <= 32)
    {
        dose=dose+stepsize;
      
    }
		else{
			dose=dose;
		}
		snprintf(doseArray, 5, "%.1f", dose);
		length=strlen(doseArray);
		stop();
		osDelay(1000);
		clear();
		osDelay(1000);
		digits();
		osDelay(1000);
		start();
		
		//HAL_Delay(5000);
//		if(iperror=='0x07'){
//		air();
//			ab++;
//		}
	
		
			
//		
//		return dose;
//		
}

// Function to decrease infusion rate
void DecreaseRate(void)
{
    if (dose >= 3)
    {
        dose=dose+stepsize;
       
    }
		else{
		dose=dose;
		}
		snprintf(doseArray, 5, "%.1f", dose);
		length=strlen(doseArray);
		stop();
		osDelay(500);
		clear();
		osDelay(500);
		digits();
		osDelay(500);
		start();
}




/*
INFUSION RATE FUNCTIONS
----------------------------------------------------------------
*/void digits(){
flowrateval=dose*10;
					digitbuff[3] = (flowrateval >> 24) & 0xFF; // Most significant byte
					digitbuff[4] = (flowrateval >> 16) & 0xFF;
					digitbuff[5] = (flowrateval >> 8) & 0xFF;
					digitbuff[6] = flowrateval & 0xFF;
	HAL_UART_Transmit(&huart2,(uint8_t *)digitbuff,sizeof(digitbuff),100);
					
}
void air(){
buffer[0]=0x00;
buffer[1]=0xEB;
buffer[2]=0x04;
buffer[3]=0xED;
	HAL_UART_Transmit(&huart2,(uint8_t *)buffer,sizeof(buffer),100);
}
void stop_process(){

}
void start_process(){

}
void start(){
buffer[0]=0x00;
buffer[1]=0xEB;
buffer[2]=0x02;
buffer[3]=0xED;
	HAL_UART_Transmit(&huart2,(uint8_t *)buffer,sizeof(buffer),100);
	rf=1;
	
	memset(rxbuff1,0 , sizeof(rxbuff1));
}
void stop(){
buffer[0]=0x00;
buffer[1]=0xEB;
buffer[2]=0x03;
buffer[3]=0xED;
HAL_UART_Transmit(&huart2,(uint8_t *)buffer,sizeof(buffer),100);
}
	void clear(){
	HAL_UART_Transmit(&huart2,(uint8_t *)clearbuff,sizeof(clearbuff),100);
	}
float increase_variable(float variable, float fac) {
    return variable + fac;
}

float decrease_variable(float variable, float fac) {
    return variable - fac;
}

float keep_variable_constant(float variable) {
    return variable;
}

float algo(float rate, float process_var, float setpoint) {
    // Find error
    float error = process_var - setpoint;

    // Threshold error condition
    if (error < 20) {
        if (-5 < error && error < 5) {  // if bp within +/- 5 mmHg
            rate = keep_variable_constant(rate);
        } else if (error > 5) {
            rate = increase_variable(rate, 2);
        } else if (error < -5) {
            rate = decrease_variable(rate, 2);
        }
    } else {
        rate = increase_variable(rate, 2);
    }

    return rate;
}


/*
----------------------------------------------------------------
*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
if(p_check==1)
{
	float f1,f2,f3,f4 = 0;
	for(int i=0;i<adcbufffsrlen;i=i+4)
	{
		f1 += adc_buf_fsr[i];
		f2 += adc_buf_fsr[i+1];
		f3 += adc_buf_fsr[i+2];
		f4 += adc_buf_fsr[i+3];
	}
		f1_avg = f1/(adcbufffsrlen/4);
		f2_avg = f2/(adcbufffsrlen/4);
		f3_avg = f3/(adcbufffsrlen/4);
		f4_avg = f4/(adcbufffsrlen/4);
	
}	
else
{	
	int sum = 0;
	
	for (int i=0;i<adcbufflen;i++){
      		sum+= adc_buf[i];
		}
	
	 avg_pressure[1] = sum/adcbufflen;
	
 	//if (avg_pressure[1]!=avg_pressure[0]){
//		if (avg_pressure[1]<=264) pressure=(0.0745*avg_pressure[1])-11.7004;
//		else pressure = 164.474 - 1.887*avg_pressure[1] + 0.00598828*pow(avg_pressure[1],2);
////	
		pressure=(0.0745*avg_pressure[1])-11.7004;
	avg_pressure[0]=avg_pressure[1];
	
		yn = ((1-0.01)*pressure)+(0.01*yn1);
		yn1 = yn;


		
	if(deflation == 1)
		{
			int val = (int)(yn*100)%100;
			if(val>0 && val<=25) temp = (((int)yn)*100)+25;
			else if(val>25 && val<=50) temp = (((int)yn)*100)+50;
			else if(val>50 && val<=75) temp = (((int)yn)*100)+75;
			else if(val>75 && val<100) temp = (((int)yn)*100)+100;
			yn = temp;
			
			if(y[ind]!=(int)yn && ind<1000 )
				{
					ind++;
					y[ind] = (int)yn;
					graph=y[ind];
	

					if(ind>4 && ((y[ind-2]>=y[ind-3] && y[ind-2]>y[ind-1]) || (y[ind-2]<y[ind-3] && y[ind-2]<=y[ind-1])))
						{
							peaks[p_ind] = y[ind-2];
							peak=y[ind-2];
							NEXTION_SendVal("beat",1);
							if (p_ind%2 == 1){
								peaks_diff[p_ind/2]=(peaks[p_ind]-peaks[p_ind-1])/100;
								pulse_time[p_ind/2]=HAL_GetTick();
							}
														
							p_ind++;
														
						}
					
//				
//    ind++;
//    y[ind] = (int)yn;
//    graph = y[ind];

//    if (ind >= 4) {
//        int prev2 = y[ind - 3];
//        int prev1 = y[ind - 2];
//        int curr  = y[ind - 1];
//        int next  = y[ind];

//        // Only detect peak if it’s a local maximum AND above noise threshold
//        if (prev1 > prev2 && prev1 > curr && prev1 > 80) {
//            peaks[p_ind] = prev1;
//            peak = prev1;
//            NEXTION_SendVal("beat", 1);

//            if (p_ind % 2 == 1) {
//                peaks_diff[p_ind / 2] = (peaks[p_ind] - peaks[p_ind - 1]) / 100.0;
//                pulse_time[p_ind / 2] = HAL_GetTick();
//            }

//            p_ind++;
//        }
//    }

					
			
//					if (ind > 4)
//{
//    int prev = y[ind - 3];
//    int curr = y[ind - 2];
//    int next = y[ind - 1];

//    // Detect local maxima only (peak)
//    if (curr > prev && curr > next && curr > 100)
//    {
//        peaks[p_ind] = curr;
//        peak = curr;
//        NEXTION_SendVal("beat", 1);

//        if (p_ind % 2 == 1)
//        {
//            peaks_diff[p_ind / 2] = (peaks[p_ind] - peaks[p_ind - 1]) / 100;
//            pulse_time[p_ind / 2] = HAL_GetTick();
//        }

//        p_ind++;
//    }
//}
				}
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
  /* Infinite loop */
  for(;;)
  {
		memset(rxbuff1,0,sizeof(rxbuff1));
		rf=0;
		HAL_UART_Transmit(&huart2,(uint8_t*)errorbuffer, 4,100);
		HAL_Delay(2000);
		if(iperror==0 ||iperror==1){
		mapflag=0;
		sys=0;
		dia=0;
		pressure=0;
		error=0;
		deflation=0;
		p_ind=0;
		pulse_rate=0;
		dia_ind=0;
		map=0;
		map_ind=0;
		ind=0;
		p_check=1;
		condition=0;
		i=0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);
	MX_ADC1_Init();
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf_fsr,adcbufffsrlen);
	osDelay(100);
	
	while(p_check==1)
	{
//		if(f1_avg<600 && f2_avg<550 && f3_avg <410 && f4_avg<300)
		//	if(f1_avg650 && f2_avg<550 && f3_avg <410)
		if(f1_avg<510 && f2_avg<510 && f3_avg<300)
		{
		p_check=0;
		//p_check=1;
			osDelay(100);
		}

			osDelay(100);
		
		if (esnd==0 && p_check==1)                                                    
		{
				NEXTION_ChangeScreen(3);
				error=5;
				osDelay(100);
				NEXTION_SendVal("error",error);
				esnd=1;
			
		}
		
	}
	
	error=0;
	esnd=0;
	HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,valve_Pin, GPIO_PIN_SET);
	rt1=HAL_GetTick();
	NEXTION_ChangeScreen(1);
	
	HAL_ADC_Stop_DMA(&hadc1);
	MX_ADC1_Init();
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,adcbufflen);
	 while (p_check==0)
  {
    
    /* USER CODE BEGIN 3 */
		
				rt2=HAL_GetTick();
				
		if (rt2-rt1>5000 && pressure<20) 
		{
			HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_RESET);
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_GPIO_WritePin(GPIOB,valve_Pin, GPIO_PIN_RESET);
			NEXTION_ChangeScreen(3);
			error=1;
			osDelay(500);
			NEXTION_SendVal("error",error);			
		}
		
		if (error > 0) break;
		
				
		while(deflation==0){		
		if(pressure>140&&deflation==0){
					HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_RESET);
		osDelay(50);
			deflation=1;
		}
		if(deflation==1){
		while(pressure>=135){

		}}
		if(p_ind>=1){
			p_ind=0;
		deflation=0;
			memset(peaks,0,sizeof(peaks));
			memset(peaks_diff,0,sizeof(peaks_diff));
			memset(pulse_time,0,sizeof(pulse_time));
				HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_SET);
			while(pressure<180){
			
			}
				
					HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_RESET);
		osDelay(50);
			deflation=1;
		}
		if(deflation==1){

			while(pressure>=175){

		}}
		if(p_ind>=1){
			p_ind=0;
		deflation=0;
			memset(peaks,0,sizeof(peaks));
			memset(peaks_diff,0,sizeof(peaks_diff));
			memset(pulse_time,0,sizeof(pulse_time));
				HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_SET);
			while(pressure<220){
			
			}
				
					HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_RESET);
		osDelay(50);
			deflation=1;
		}
				if(deflation==1){
		while(pressure>=215){

		}}
		if(p_ind>=1){
			p_ind=0;
		deflation=0;
			memset(peaks,0,sizeof(peaks));
			memset(peaks_diff,0,sizeof(peaks_diff));
			memset(pulse_time,0,sizeof(pulse_time));
				HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_SET);
			while(pressure<260){
			
			}
				
					HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_RESET);
		osDelay(50);
			deflation=1;
		}
		if(deflation==1){
		while(pressure>=255){

		}}
		if(p_ind>=5){
			p_ind=0;
		deflation=0;
			memset(peaks,0,sizeof(peaks));
			memset(peaks_diff,0,sizeof(peaks_diff));
			memset(pulse_time,0,sizeof(pulse_time));
				HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_SET);
			while(pressure<290){
			
			}
				
					HAL_GPIO_WritePin(GPIOB,pump_Pin, GPIO_PIN_RESET);
		osDelay(50);
			deflation=1;
		}
		}


	//	pressure_plot = xn[2];
		
		NEXTION_SendVal("pressure",pressure);
		if (deflation == 1)
		{
			
		if(i<1000){
			pressureArray[i]=pressure;
			}
			i++;
		 
			
			if (pressure<=60 && mapflag == 0) 
			{
				HAL_ADC_Stop_DMA(&hadc1);
				HAL_GPIO_WritePin(GPIOB,valve_Pin, GPIO_PIN_RESET);
		//	detect_peaks_from_pressure_array();  // <
					
				if (p_ind<2)
				{
					NEXTION_ChangeScreen(3);
					error=2;
					osDelay(500);
					NEXTION_SendVal("error",error);
					break;
				}
				if(p_ind<25){
					
			map_amp=max(peaks_diff,10);
			map=peaks[map_ind*2]/100;
				}
			else{
			map_amp=max(peaks_diff,30);
			map=peaks[map_ind*2]/100;
			}																															/*  infusion  */				
			pre_inf = algo(infusion, map, 40);
      inf += pre_inf;
      if (inf < 0){
       inf = 0;
			}
			for(int i=6;i<1000;i++){
			if(pressureArray[i]>pressureArray[i-1]+5){
			condition=1;
			}
			}			
//				if (peaks_diff[(p_ind/2)-1]>=4  ||(p_ind/2)- map_ind <=2 || (p_ind/2)- map_ind >=(p_ind/2)-1 || condition==1 )			
			//	if (peaks_diff[(p_ind/2)-1]>=4  ||(p_ind/2)- map_ind <2 || (p_ind/2)- map_ind >=(p_ind/2)-1 || condition==1 )			
if ( condition==1 )			
						
				{
					NEXTION_ChangeScreen(3);
					error=4;
					osDelay(500);
					NEXTION_SendVal("error",error);					
					break;
				}				
								
			dia_amp=map_amp*0.6; //0.90
			sys_amp=map_amp*0.20; // 0.25
			
			sys_ind=findSys(peaks_diff,sys_amp);
			dia_ind=findDia(peaks_diff,dia_amp);
			
			pulse_rate = (60*((p_ind/2)-1))/((pulse_time[((p_ind/2)-1)]-pulse_time[1])/1000)/2;  // time difference between sys and dia beats//no of beats = dia_ind
//			pulse_rate = (60*((p_ind/2)-1))/((pulse_time[(map_ind)]-pulse_time[1])/1000);
			sys=peaks[3]/100;
			dia=peaks[dia_ind*2]/100;
							//dia=(3*map-(sys))/2;
			if (dia>sys || dia<= 30 || sys>=300 || pulse_rate <=35 || pulse_rate >=160 )
			{
				NEXTION_ChangeScreen(3);
				error=3;
				osDelay(500);
				NEXTION_SendVal("error",error);
				break;
			}					                    
			
			NEXTION_ChangeScreen (1);
			NEXTION_SendVal("sys",sys);
			NEXTION_SendVal("dia",dia);
			NEXTION_SendVal("bpm",pulse_rate);
			q=1;
			HAL_UART_DMAStop(&huart2);

      // Clear receive buffer
      memset(buff, 0, sizeof(buff));

      // Restart UART DMA reception from index 0
      HAL_UART_Receive_DMA(&huart2, buff, sizeof(buff));
			if (j==0)
			{
				
				prevsys=sys;
				stepsize=60;
				IncreaseRate();
				prevdose=dose;
				sensitivity=0;
				sensitivityavg=0;
				j++;
				 							
				osDelay(100000);				
				
			}
			//
			else
			{
					//calculation of sensitivity and sensitivity average
					bpdiff=prevsys-sys;
					prevsys=sys;

				//time and dose increament w.r.t BP
			if (sys>=180)
			{
				stepsize=3;
				IncreaseRate();
				sens();
				osDelay(30000);
			}
			else if (sys>=165 && sys<180)
			{
				stepsize=2.5;
				IncreaseRate();
				sens();
				osDelay(30000);
			}
			else if(sys>=145 && sys<165)
			{
				stepsize=2;
				IncreaseRate();
				sens();
				osDelay(30000);
			}
			else if( sys>=130 && sys<145)
			{
				stepsize=1;
				IncreaseRate();
				sens();
				osDelay(30000);
			}
			else if( sys>=100 && sys<130)
			{
				stepsize=3;
				
				IncreaseRate();
				rf=1;
				sens();	
				osDelay(30000);
			}
			else if (sys>=0 && sys<100)
			{
				if (sensitivityavg>80)
				{
					stepsize=-3;
					DecreaseRate();
			osDelay(30000);
			}
				else if(sensitivityavg>=60 && sensitivityavg<80)
				{
					stepsize=-2;
					DecreaseRate();
				osDelay(30000);
				}
				else if(sensitivityavg >=40 && sensitivityavg<60)
				{
					stepsize=-1;
				DecreaseRate();
			osDelay(30000);
				}
				else
				{
					stepsize=-0.5;
				DecreaseRate();
			osDelay(30000);
			}
				}
			// sensitivity portion
				dosediff=dose-prevdose;
					prevdose=dose;
			if ((bpdiff>0)&&(dosediff>0)&&(dosediff<bpdiff)){
				if (count<=4)
				{
					sensitivity=100-((dosediff)/(bpdiff))*100;
					if(count==1){
					sensitivityavg= sensitivity;
					}
					else
					{
						sensitivityavg=(sensitivityavg+sensitivity)/2;
				}
					sensitivityArray[count]=sensitivity;
				sensitivityavgArray[count]=sensitivityavg;
					count++;
			}
			
		}
	}
		
	if(p<20){
	bpArray[p]=sys;
		doses[p]=dose;
		p++;
	}
			deflation=0;
			mapflag = 1;
			p_check=1;
//			osDelay(30000);
			
			
		}
	}		
		
	osDelay(50);
		
		
  }
}
	osDelay(2500);
}
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE BEGIN Header_alarms */
/**
* @brief Function implementing the Error thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_alarms */
void alarms(void *argument)
{
  /* USER CODE BEGIN alarms */
  /* Infinite loop */
  for(;;)
  {
		if (rf==1){
			
			//memset(rxbuff1,0,sizeof(rxbuff1));
		HAL_UART_Transmit(&huart2,(uint8_t*)errorbuffer,4,100);
			HAL_UART_DMAStop(&huart2);
		
      // Clear receive buffer
      memset(rxbuff1, 0, sizeof(rxbuff1));

      // Restart UART DMA reception from index 0
      HAL_UART_Receive_DMA(&huart2, rxbuff1, sizeof(rxbuff1));
			osDelay(5000);
				if(iperror==7&&cd==0){
					stop();
					HAL_Delay(2000);
			air();
					cd=1;
				ab++;
			}
				HAL_UART_Transmit(&huart2,(uint8_t*)volbuff,4,100);
			HAL_UART_DMAStop(&huart2);

      // Clear receive buffer
      memset(volumebuff, 0, sizeof(volumebuff));

      // Restart UART DMA reception from index 0
      HAL_UART_Receive_DMA(&huart2, volumebuff, sizeof(volumebuff));
			osDelay(500);
			volume = (volumebuff[3] << 24) |
         (volumebuff[4] << 16) |
         (volumebuff[5] << 8)  |
         (volumebuff[6]);

// Divide by 10 to get the actual volume value (since it's multiplied by 10)
						volume = volume / 10;
		}
    osDelay(1);
  }
  /* USER CODE END alarms */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
