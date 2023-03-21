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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CAN_3508Moto1_ID 0x201
#define CAN_3508Moto2_ID 0x202
#define CAN_3508Moto3_ID 0x203
#define CAN_3508Moto4_ID 0x204

typedef struct 
{
  uint16_t theta;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperature;
  int16_t last_theta;
}moto_measure;
void get_moto_measure(moto_measure *ptr,uint8_t *data){
  (ptr)->last_theta = (ptr)->theta;
  (ptr)->theta = (uint16_t)((data)[0]<<8|(data)[1]);
  (ptr)->speed_rpm = (uint16_t)((data)[2]<<8|(data)[3]);
  (ptr)->given_current = (uint16_t)((data)[4]<<8|(data)[5]);
  (ptr)->temperature = (data)[6];
}

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static moto_measure moto_chassis[4];
CAN_TxHeaderTypeDef chassis_tx_message;
uint8_t chassis_can_send_data[8]={0};
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) 
{ 
	uint32_t send_mail_box; 
	
	chassis_tx_message.StdId = 0x200; 
	chassis_tx_message.IDE = CAN_ID_STD; 
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08; 
	
	chassis_can_send_data[0] = motor1 >> 8; 
	chassis_can_send_data[1] = motor1; 
	chassis_can_send_data[2] = motor2 >> 8; 
	chassis_can_send_data[3] = motor2; 
	chassis_can_send_data[4] = motor3 >> 8; 
	chassis_can_send_data[5] = motor3; 
	chassis_can_send_data[6] = motor4 >> 8; 
	chassis_can_send_data[7] = motor4; 
	
	HAL_CAN_AddTxMessage(&hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box); 
}

extern CAN_HandleTypeDef hcan;
//extern CAN_HandleTypeDef hcan2;
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE; //筛�?�器�?�?
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; //标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; //过滤器位宽为单个32�?
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; //FIFO0的中断和FIFO1的中断是不一样的，这里是把接收到的报文放入到FIFO0�?
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

//    can_filter_st.SlaveStartFilterBank = 14; //为从属can选择�?始的过滤库，对于单个CAN实例，这个参数没有意�?
//    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void NumToChar(uint8_t Num,unsigned char *Chars){
  for(int i=0;i<=7;i++){
    Chars[i] = Num%10;
    Num-=Num%10;
    Num/=10;
  }
}
typedef struct{
	double preValue;
	double tarValue;
	double preErr;
	double inValue;
	double deValue;
	double lastErr;
}Pid;
Pid cal;//cuowu
double dPIDcal(const double Kp,const double Ki,const double Kd,double Target,double Prevous){
	double ans=0;
	cal.preValue = Prevous;
	cal.tarValue = Target;
	cal.preErr=cal.tarValue-cal.preValue;
	cal.inValue+=cal.preErr;
	cal.deValue=cal.lastErr-cal.preErr;
	ans = Kp*cal.preErr+Ki*cal.inValue+Kd*cal.deValue;
	if(ans>16384){//��ֹ���Խ��
		ans=16384;
	}
	if(ans<-16384){
		ans=-16384;
	}
	cal.lastErr=cal.preErr;
	return ans;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);
	can_filter_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  CAN_cmd_chassis(dPIDcal(1,0.01,10,5500,moto_chassis[0].speed_rpm),16384,3000,3000);
	//CAN_cmd_chassis(16384,16384,3000,3000);
	  unsigned char re[8]={0};
    NumToChar(moto_chassis->temperature,re);
	if(moto_chassis[0].theta!=0){
	}
	  HAL_UART_Transmit(&huart1,re,8,0xFFFF);
//    HAL_Delay(1000);
//    if(moto_chassis[0].theta==0){
//      CAN_cmd_chassis(0,16384,3000,3000);
//    }
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
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan){
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
  switch(rx_header.StdId){	
    case CAN_3508Moto1_ID://0x201
    case CAN_3508Moto2_ID://0x202
    case CAN_3508Moto3_ID://0x203
    case CAN_3508Moto4_ID://0x204
    {
		static uint8_t i = 0;
		i = rx_header.StdId - CAN_3508Moto1_ID;
		get_moto_measure(&moto_chassis[i], rx_data);
		break;
    }
    
    default:
    {
      break;
    }
  }
}
/* USER CODE END 4 */

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
