/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <wchar.h>
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "SIM868.h"

#define LISENCE_CODE "%E6%B5%99A9D395"				//浙A9D563
#define INNER_CODE	"012345"									//此处最好位数不动
#define PHONE_NUMBER "18610312360"						//此处最好位数不动
#define IMEI_CODE "1234567890"								//此处最好位数不动
#define VECHILE_ODO "10000"
#define TIRE_NUMBER 6													//6轮车辆
#define CAN_TX_LEN 8
#define CAN_FIFO_NUM_SEL 0
#define TX_TIME_INTERVAL (640-40)						//40秒为采集时间，1800是30分钟

//unsigned short *pProvince=L"京";
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t i;
uint32_t timer2_count;
char des[2000];	
uint8_t DY_data_ready=0;
uint8_t TPMS_data_ready=0;
uint8_t ODO_data_ready=0;
uint32_t ODO_value=10000;
uint8_t IMEI_RX_BUF[20];
uint8_t IMEI[16];
uint8_t RxBuf[64]={0};
int16_t PressData[6]={868,868,868,868,868,868};
int16_t TempData[6]={25,25,25,25,25,25};
uint16_t ID[6]={0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
uint16_t  WaringFlag[6]={0};
float accel_x1[3];
float accel_y1[3];
float accel_z1[3]; 
char send_num=0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void	CAN1_Filter_TPMS_Init(void);
static void CAN1_Filter_ODO_Init(void);
static void MX_TIM2_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
static void MX_USART1_UART_Init(void);
static void CAN1_Tx(uint8_t* CanTxBuf);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HTTP_Tx_Data(void);
static void GPRS_POWER_ON(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
float  accel_x,accel_y,accel_z;
/* USER CODE END 0 */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t j=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
	GPRS_POWER_ON();															//GPRS模块上电，如果不加此函数，状态灯显示状态不确定
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  CAN1_Filter_TPMS_Init();
  /* USER CODE BEGIN 2 */
	
	
	for(i=0;i<40;i++)
		HAL_Delay(1000);														//延时30秒，等待天线信号就位
	
	Send_AT_Command(&huart1, "ATE0\n");						//关闭回显
	HAL_UART_Receive(&huart1, RxBuf, 32,2000);
	HAL_Delay(10000);	
	
	Send_AT_Command(&huart1, "AT+GSN\n");
	HAL_UART_Receive(&huart1, IMEI_RX_BUF, 20, 4000);
	HAL_Delay(2000);			
	
	for(i=0;i<15;i++)
	{
	 if(IMEI_RX_BUF[i]=='\n')
		 break;
	}
	for(j=0;j<15;j++)
	{
		IMEI[j]=IMEI_RX_BUF[1+i+j];
	}
	IMEI[15]=0x00;
	timer2_count=0;
	HAL_TIM_Base_MspInit(&htim2);
	MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);
	__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//接收TPMS数据
//		CAN1_Filter_TPMS_Init();										//CAN过滤器初始化
//		TPMS_data_ready=0;
//		__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);		//重新开启FIF00消息挂号中断
		
		//while(TPMS_data_ready!=1)
		{
			//HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);	//执行CAN总线接收TPMS信息
		}
		#ifdef BYD_VECHILE														//如果是BYD车辆，则可以获取里程
		//接收ODO数据
		CAN1_Filter_ODO_Init();
		ODO_data_ready=0;
		while(ODO_data_ready!=1)
		{
			HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
		}
		#endif
		#ifdef BYD_TPMS
		uint8_t waringflag=0;
		for(uint8_t a=0;a<6;a++)
		{
		 if((WaringFlag[i]&0x0400)||(WaringFlag[i]&0x80)||((WaringFlag[i]&0xff)==0x20)||((WaringFlag[i]&0xff)==0x60))
		 {
			 waringflag=1;
		 }
		}
		if(waringflag==1)
		{
			 DY_data_ready=1;//关闭陀螺仪接受
			 waringflag=0;//报警标志清除
			 HAL_TIM_Base_Stop_IT(&htim2);//定时器停止
			 timer2_count=0;//定时器清0
			 sprintf(des,"%s%s%s%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s",\
		   "{","\"imei\":\"",IMEI,"\",\"firsttire\":{\"press\":\"",PressData[0],"\",\"temp\":\"",TempData[0],"\",\"id\":\"",ID[0],"\"},\"secondtire\":{\"press\":\"",PressData[1],"\",\"temp\":\"", TempData[1],\
		   "\",\"id\":\"",ID[1],"\"},\"thirdtire\":{\"press\":\"",PressData[2],"\",\"temp\":\"",TempData[2],"\",\"id\":\"",ID[2],"\"},\"fourtire\":{\"press\":\"",PressData[3],"\",\"temp\":\"", TempData[3],\
		   "\",\"id\":\"",ID[3],"\"},\"fivetire\":{\"press\":\"",PressData[4],"\",\"temp\":\"",TempData[4],"\",\"id\":\"",ID[4],"\"},\"sixtire\":{\"press\":\"",PressData[5],"\",\"temp\":\"", TempData[5],\
		   "\",\"id\":\"",ID[5],"\"},\"acceleration\":{\"x\":\"",accel_x1[0],"\",\"y\":\"",accel_y1[0],"\",\"z\":\"",accel_z1[0],"\"},\"acceleration\":{\"x\":\"",accel_x1[1],"\",\"y\":\"",accel_y1[1],
			 "\",\"z\":\"",accel_z1[1],"\"},\"acceleration\":{\"x\":\"",accel_x1[2],"\",\"y\":\"",accel_y1[2],"\",\"z\":\"",accel_z1[2],"\"}}\r\n");
			 HTTP_Tx_Data();	
			 HAL_TIM_Base_Start_IT(&htim2);//打开定时器
			 DY_data_ready=0;//打开陀螺仪接受
			 HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
		}
		#endif
		#ifdef STD_TPMS
		uint8_t waringflag=0;
		for(uint8_t a=0;a<6;a++)
		{
		 if((WaringFlag[a]&0xff))
		 {
			 waringflag=1;
		 }
		}
		if(waringflag==1)
		{
			 DY_data_ready=1;//关闭陀螺仪接受
			 waringflag=0;//报警标志清除
			 HAL_TIM_Base_Stop_IT(&htim2);//定时器停止
			 timer2_count=0;//定时器清0
			sprintf(des,"%s%s%s%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s",\
		   "{","\"imei\":\"",IMEI,"\",\"firsttire\":{\"press\":\"",PressData[0],"\",\"temp\":\"",TempData[0],"\",\"id\":\"",ID[0],"\"},\"secondtire\":{\"press\":\"",PressData[1],"\",\"temp\":\"", TempData[1],\
		   "\",\"id\":\"",ID[1],"\"},\"thirdtire\":{\"press\":\"",PressData[2],"\",\"temp\":\"",TempData[2],"\",\"id\":\"",ID[2],"\"},\"fourtire\":{\"press\":\"",PressData[3],"\",\"temp\":\"", TempData[3],\
		   "\",\"id\":\"",ID[3],"\"},\"fivetire\":{\"press\":\"",PressData[4],"\",\"temp\":\"",TempData[4],"\",\"id\":\"",ID[4],"\"},\"sixtire\":{\"press\":\"",PressData[5],"\",\"temp\":\"", TempData[5],\
		   "\",\"id\":\"",ID[5],"\"},\"acceleration\":{\"x\":\"",accel_x1[0],"\",\"y\":\"",accel_y1[0],"\",\"z\":\"",accel_z1[0],"\"},\"acceleration\":{\"x\":\"",accel_x1[1],"\",\"y\":\"",accel_y1[1],
			 "\",\"z\":\"",accel_z1[1],"\"},\"acceleration\":{\"x\":\"",accel_x1[2],"\",\"y\":\"",accel_y1[2],"\",\"z\":\"",accel_z1[2],"\"}}\r\n");
																															   
			 HTTP_Tx_Data();	
			 HAL_TIM_Base_Start_IT(&htim2);//打开定时器
			 DY_data_ready=0;//打开陀螺仪接受
			 HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
		}
		#endif
		 if(timer2_count==TX_TIME_INTERVAL)
		 {
			 DY_data_ready=1;//关闭陀螺仪接受
			 sprintf(des,"%s%s%s%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s",\
		   "{","\"imei\":\"",IMEI,"\",\"firsttire\":{\"press\":\"",PressData[0],"\",\"temp\":\"",TempData[0],"\",\"id\":\"",ID[0],"\"},\"secondtire\":{\"press\":\"",PressData[1],"\",\"temp\":\"", TempData[1],\
		   "\",\"id\":\"",ID[1],"\"},\"thirdtire\":{\"press\":\"",PressData[2],"\",\"temp\":\"",TempData[2],"\",\"id\":\"",ID[2],"\"},\"fourtire\":{\"press\":\"",PressData[3],"\",\"temp\":\"", TempData[3],\
		   "\",\"id\":\"",ID[3],"\"},\"fivetire\":{\"press\":\"",PressData[4],"\",\"temp\":\"",TempData[4],"\",\"id\":\"",ID[4],"\"},\"sixtire\":{\"press\":\"",PressData[5],"\",\"temp\":\"", TempData[5],\
		   "\",\"id\":\"",ID[5],"\"},\"acceleration\":{\"x\":\"",accel_x1[0],"\",\"y\":\"",accel_y1[0],"\",\"z\":\"",accel_z1[0],"\"},\"acceleration\":{\"x\":\"",accel_x1[1],"\",\"y\":\"",accel_y1[1],
			 "\",\"z\":\"",accel_z1[1],"\"},\"acceleration\":{\"x\":\"",accel_x1[2],"\",\"y\":\"",accel_y1[2],"\",\"z\":\"",accel_z1[2],"\"}}\r\n");
																															   
		   HTTP_Tx_Data();			//通过HTTP服务向服务器发送数据
			 timer2_count=0;
			 DY_data_ready=0;//打开陀螺仪接受
			 HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
		 }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{
	static CanRxMsgTypeDef RxMessage;      //接收消息
	static CanTxMsgTypeDef TxMessage;			//发送消息
	
	hcan1.pTxMsg = &TxMessage;
  hcan1.pRxMsg = &RxMessage;
	
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_6TQ;
  hcan1.Init.BS2 = CAN_BS2_7TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
	hcan1.pTxMsg->StdId=0x000;
	hcan1.pTxMsg->ExtId=0x18FEF4D7;
	hcan1.pTxMsg->RTR=CAN_RTR_DATA;
	hcan1.pTxMsg->IDE=CAN_ID_EXT;
	hcan1.pTxMsg->DLC=CAN_TX_LEN;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void CAN1_Filter_TPMS_Init(void)
{
    CAN_FilterConfTypeDef  CAN1_FilerConf;
		CAN1_FilerConf.FilterIdHigh=0X07F0;     //32位ID  ID:FEF4
    CAN1_FilerConf.FilterIdLow=0XA000;
		CAN1_FilerConf.FilterMaskIdHigh=0x07F0; //32位MASK  //0X07FF
    CAN1_FilerConf.FilterMaskIdLow=0X0000;              //0XF800
		CAN1_FilerConf.BankNumber=14;
	
    CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
    CAN1_FilerConf.FilterNumber=CAN_FIFO_NUM_SEL;          //过滤器0
    CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器0
    
    if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK) 
		{
			_Error_Handler(__FILE__, __LINE__);
		}
}

static void CAN1_Filter_ODO_Init(void)
{
		CAN_FilterConfTypeDef  CAN1_FilerConf;
	
		CAN1_FilerConf.FilterIdHigh=0X07F6;     //32位ID  ID:FEC1			ODO里程
    CAN1_FilerConf.FilterIdLow=0X0800;
		CAN1_FilerConf.FilterMaskIdHigh=0x07FF; //32位MASK
    CAN1_FilerConf.FilterMaskIdLow=0XF800;  
		CAN1_FilerConf.BankNumber=14;
	
    CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
    CAN1_FilerConf.FilterNumber=CAN_FIFO_NUM_SEL;          //过滤器0
    CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器0
    
    if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK) 
		{
			_Error_Handler(__FILE__, __LINE__);
		}
}

/* CAN RxCpltCallback function */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
		uint8_t i=0;
		static uint8_t TPMS_count=0;
	
		#ifdef BYD_TPMS
		if((TPMS_data_ready==0) && (hcan1.pRxMsg->ExtId==0x18FEF433))		//如果收到的ID是BYD版本TPMS ID
		{
			i=(((hcan1.pRxMsg->Data[0])&0x10)>>4)*2 + (hcan1.pRxMsg->Data[0])&0x0F;			//i是轮位值
			/*PressData[i] = (hcan1.pRxMsg->Data[1])*16;
			TempData[i] = (int16_t)((hcan1.pRxMsg->Data[2] * 255) + (hcan1.pRxMsg->Data[3]))/128 -128;
			*/
			if(((hcan1.pRxMsg->Data[7])&0x04)==0)			//byte8 bit2  为0，需要扩充
			{
				PressData[i]=(hcan1.pRxMsg->Data[1])*4 + 256*4;		//多扩充位相当于增加256	 获取压力值
			}
			else
			{
				PressData[i] = (hcan1.pRxMsg->Data[1])*4;
			}
			TempData[i] = ((int16_t)((hcan1.pRxMsg->Data[3])*255) + (hcan1.pRxMsg->Data[2]))*0.03125-273; //获取温度值
			ID[i]=(hcan1.pRxMsg->Data[5]<<8)|hcan1.pRxMsg->Data[6];
			WaringFlag[i]=(hcan1.pRxMsg->Data[4]<<8)|hcan1.pRxMsg->Data[7];
			TPMS_count++;	
      HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);			//收到轮胎数据+1
		}
		#endif
		
		#ifdef STD_TPMS
		if((TPMS_data_ready==0) && (hcan1.pRxMsg->ExtId==0x18FEF4D8))		//如果收到的ID是BYD版本TPMS ID
		{
			i=(((hcan1.pRxMsg->Data[0])&0x10)>>4)*2 + (hcan1.pRxMsg->Data[0])&0x0F;			//i是轮位值
			PressData[i] = (hcan1.pRxMsg->Data[1])*16;
			TempData[i] = (int16_t)((hcan1.pRxMsg->Data[2] * 255) + (hcan1.pRxMsg->Data[3]))/128 -128;
			ID[i]=(hcan1.pRxMsg->Data[5]<<8)|hcan1.pRxMsg->Data[6];
			WaringFlag[i]=hcan1.pRxMsg->Data[4];
			HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
		}
		#endif
			
		#ifdef BYD_VECHILE
		if((ODO_data_ready==0) && (hcan1.pRxMsg->ExtId==0x18FEC117))
		{
			ODO_value=hcan1.pRxMsg->Data[3];
			ODO_value<<=4;
			ODO_value|=hcan1.pRxMsg->Data[2];
			ODO_value<<=4;
			ODO_value|=hcan1.pRxMsg->Data[1];
			ODO_value<<=4;
			ODO_value|=hcan1.pRxMsg->Data[0];
			ODO_data_ready=1;						//ODO数据齐全
		}
		#endif
		if((hcan1.pRxMsg->ExtId==0x18FEE0D8)&&(DY_data_ready==0))
		{
			accel_x=hcan1.pRxMsg->Data[1]*0.02-2.5;
			accel_y=hcan1.pRxMsg->Data[2]*0.02-2.5;
			accel_z=hcan1.pRxMsg->Data[3]*0.02-2.5;
			accel_x1[send_num]=accel_x;
			accel_y1[send_num]=accel_y;
			accel_z1[send_num]=accel_z;				   
			HAL_CAN_Receive_IT(&hcan1, CAN_FIFO_NUM_SEL);
			//__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);
		}
		//
		
}
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig); 
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	send_num++;
	if(send_num>3)send_num=0;						
  timer2_count++;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
    /* 基本定时器外设时钟使能 */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 外设中断配置 */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}
/* CAN Tx Function */
static void CAN1_Tx(uint8_t* CanTxBuf)
{
		for(i=0;i<CAN_TX_LEN;i++)
    hcan1.pTxMsg->Data[i]=CanTxBuf[i];
    HAL_CAN_Transmit(&hcan1,CAN_TX_LEN);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
    
    /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = GPRS_PWR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPRS_PWR_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HTTP_Tx_Data(void)
{
	//Send_AT_Command(&huart1, "AT+CGATT?\r\n");
	//HAL_UART_Receive(&huart1, RxBuf,32,2000);
	//HAL_Delay(2000);
	//////////////////////////////////////////////////////
	//Send_AT_Command(&huart1,"AT+GSN\r\n");//获取IMEI号
	//HAL_UART_Receive(&huart1, RxBuf,32,2000);
	//for(i=0;i<15;i++) IMEI[i] = RxBuf[i];
	//IMEI[15]=0;				//补上结尾'\0'
	//
	Send_AT_Command(&huart1, "AT+CGATT=1\r\n");//附着网络
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	
	HAL_Delay(2000);
//	Send_AT_Command(&huart1, "AT+CSTT\r\n");//Start Task and Set APN, USER NAME, PASSWORD
//	HAL_UART_Receive(&huart1, RxBuf,32,2000);
//	HAL_Delay(2000);
//	Send_AT_Command(&huart1, "AT+CIICR\r\n");//激活移动场景
//	HAL_UART_Receive(&huart1, RxBuf,32,2000);
//	HAL_Delay(2000);
//	Send_AT_Command(&huart1, "AT+CIFSR\r\n");//获得本地模块内部IP地址
//	HAL_UART_Receive(&huart1, RxBuf,32,2000);
//	HAL_Delay(2000);
	///////////////////////////////////////////////////////
	Send_AT_Command(&huart1, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	Send_AT_Command(&huart1, "AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	Send_AT_Command(&huart1, "AT+SAPBR=1,1\r\n");// open bearer To open a GPRS context
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	////////
	Send_AT_Command(&huart1, "AT+SAPBR=2,1\r\n");// To query the GPRS context
	HAL_UART_Receive(&huart1, RxBuf,64,2000);
	HAL_Delay(2000);
//	Send_AT_Command(&huart1, "AT+SAPBR=0,1\r\n");// To close the GPRS context
//	HAL_UART_Receive(&huart1, RxBuf,32,2000);
//	HAL_Delay(2000);
	///////
	Send_AT_Command(&huart1, "AT+HTTPINIT\r\n");
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	Send_AT_Command(&huart1, "AT+HTTPPARA=\"CID\",1\r\n");
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	//47.100.106.215
	//Send_AT_Command(&huart1, "AT+HTTPPARA=\"URL\",\"http://101.200.237.84/api/tireinfos?carcode=666685&carmille=180&mobile=13800138088&carinnercode=888847&imei=abcd123485&firsttire={\"press\":\"710\",\"tmep\":\"10\"}&sencordtire={\"press\":\"620\",\"tmep\":\"20\"}&thirdtire={\"press\":\"600\",\"tmep\":\"30\"}\"\r\n");
	//Send_AT_Command(&huart1, "AT+HTTPPARA=\"URL\",\"http://101.200.237.84/api/tireinfos?carcode=666685&carmille=180&mobile=13800138088&carinnercode=888847&imei=abcd123485&firsttire={'press':\"710\",'tmep':\"10\"}&sencordtire={'press':\"710\",'tmep':\"10\"}&thirdtire={'press':\"710\",'tmep':\"10\"}\"\r\n");
	//Send_AT_Command(&huart1, "AT+HTTPPARA=\"URL\",\"http://101.200.237.84/api/tireinfos?carcode=Q1888&carmille=180&mobile=13800138088&carinnercode=888847&imei=abcd123485&firsttire=%7B%22press%22:%22710%22,%22tmep%22:%2210%22%7D&sencordtire=%7B%22press%22:%22710%22,%22tmep%22:%2210%22%7D&thirdtire=%7B%22press%22:%22710%22,%22tmep%22:%2210%22%7D\"\r\n");
	Send_AT_Command(&huart1, "AT+HTTPPARA=\"URL\",\"https://data01.fleetsguru.com/api/tms/?appid=e2cbbb386dbe41dbb7ea8ba72617f2d6&app=tms_luonai\"\r\n");
	//Send_AT_Command(&huart1, "AT+HTTPPARA=\"URL\",\"http://console.kshine.com.cn/api.php?op=ceshi\"\r\n");
	
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	//Send_AT_Command(&huart1, "AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"\r\n");//服务器一定是POST
	Send_AT_Command(&huart1, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");//服务器一定是POST
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	char a[200]="&imei=18610312360\r\n";
	char b[100];
	sprintf(b,"%s%d%s","AT+HTTPDATA=",strlen(des),",10000\r\n");
	//Send_AT_Command(&huart1, "AT+HTTPDATA=11,10000\r\n");
	Send_AT_Command(&huart1,b);
	HAL_UART_Receive(&huart1, RxBuf,64,2000);
	HAL_Delay(2000);
	Send_AT_Command(&huart1, des);
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	Send_AT_Command(&huart1, "AT+HTTPSSL=1\r\n");//用https方式
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);
	Send_AT_Command(&huart1, "AT+HTTPACTION=1\r\n");//用POST方式
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	HAL_Delay(2000);

	Send_AT_Command(&huart1, "AT+HTTPTERM\r\n");
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	//////////////
	HAL_Delay(2000);
	Send_AT_Command(&huart1, "AT+SAPBR=0,1\r\n");// To close the GPRS context
	HAL_UART_Receive(&huart1, RxBuf,32,2000);
	/////////////
}

static void GPRS_POWER_ON(void)
{
	HAL_GPIO_WritePin(GPRS_PWR_PORT, GPRS_PWR_PIN, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPRS_PWR_PORT, GPRS_PWR_PIN, GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPRS_PWR_PORT, GPRS_PWR_PIN, GPIO_PIN_SET);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
