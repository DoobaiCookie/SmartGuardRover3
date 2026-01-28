/* USER CODE BEGIN Header */
/**dhsajdhsajdhjksdhakjhjk
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Car + LCD + DHT11 (Safe for Code Gen)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// [온습도 구조체]
typedef struct {
    uint8_t temperature;
    uint8_t humidity;
    uint8_t temp_decimal;
    uint8_t hum_decimal;
    uint8_t checksum;
} DHT11_Data;

// [LCD 매크로]
#define LCD_WIDTH  160
#define LCD_HEIGHT 80
#define X_OFFSET   0
#define Y_OFFSET   26

// Colors (RGB565)
#define BLACK       0x0000
#define WHITE       0xFFFF
#define RED         0xF800
#define GREEN       0x07E0
#define BLUE        0x001F
#define YELLOW      0xFFE0
#define EYE_COLOR   0x07E0
#define EYE_BRIGHT  0xAFE5
#define EYE_DIM     0x0320

// 눈 위치
#define LX          40
#define RX          120
#define CY          40
#define EYE_W       30
#define EYE_H       50
#define EYE_R       10

// ST7735 Commands
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

typedef enum {
    EXPR_NORMAL, EXPR_HAPPY, EXPR_SAD, EXPR_ANGRY,
    EXPR_SURPRISED, EXPR_SLEEPY, EXPR_Love
} Expression_t;

// GPIO 제어 매크로
#define LCD_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_DC_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_DC_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_RES_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define LCD_RES_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_0
#define HIGH 1
#define LOW 0
#define AUTO_STOP_DELAY 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DHT11_Data dht11_data;
char uart_buffer[100];

uint8_t rx_data;
long unsigned int echo_time1, echo_time2, echo_time3, echo_time4;
int dist1, dist2, dist3, dist4;
volatile uint32_t last_cmd_time = 0;

volatile uint8_t is_avoiding = 0;
volatile uint8_t is_emergency = 0;
volatile uint8_t is_fire = 0;

Expression_t current_expr = EXPR_NORMAL;
uint16_t current_bg_color = BLUE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
// [DHT11 Functions]
void DHT11_SetPinOutput(void);
void DHT11_SetPinInput(void);
void DHT11_SetPin(GPIO_PinState state);
GPIO_PinState DHT11_ReadPin(void);
void DHT11_DelayUs(uint32_t us);
uint8_t DHT11_Start(void);
uint8_t DHT11_ReadByte(void);
uint8_t DHT11_ReadData(DHT11_Data *data);

// [Smart Car Functions]
void Avoid_Obstacle_Routine(void);
void trig1(void); long unsigned int echo1(void);
void trig2(void); long unsigned int echo2(void);
void trig3(void); long unsigned int echo3(void);
void trig4(void); long unsigned int echo4(void);
void delay_us(uint16_t us);
int LFRB(); int RFLB(); void ST(); void MB();
void Check_Light(void);
void Check_Tilt_State(void);
void Check_Fire_State(void);

// [LCD Functions]
void LCD_Init(void);
void LCD_Clear(uint16_t color);
void Draw_Expression(Expression_t expr, uint16_t bg_color);
void Update_Face_Logic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}
// ============================================================================
// [DHT11 온습도 센서 함수 구현]
// ============================================================================
void DHT11_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPin(GPIO_PinState state) {
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, state);
}

GPIO_PinState DHT11_ReadPin(void) {
    return HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN);
}

void DHT11_DelayUs(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

uint8_t DHT11_Start(void) {
    uint8_t response = 0;
    DHT11_SetPinOutput();
    DHT11_SetPin(GPIO_PIN_RESET); HAL_Delay(20);
    DHT11_SetPin(GPIO_PIN_SET); DHT11_DelayUs(30);
    DHT11_SetPinInput();
    DHT11_DelayUs(40);
    if (!(DHT11_ReadPin())) {
        DHT11_DelayUs(80);
        if (DHT11_ReadPin()) response = 1; else response = 0;
    }
    while (DHT11_ReadPin());
    return response;
}

uint8_t DHT11_ReadByte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        while (!(DHT11_ReadPin()));
        DHT11_DelayUs(30);
        if (DHT11_ReadPin()) { byte |= (1 << (7 - i)); while (DHT11_ReadPin()); }
    }
    return byte;
}

uint8_t DHT11_ReadData(DHT11_Data *data) {
    if (!DHT11_Start()) return 0;
    data->humidity = DHT11_ReadByte();
    data->hum_decimal = DHT11_ReadByte();
    data->temperature = DHT11_ReadByte();
    data->temp_decimal = DHT11_ReadByte();
    data->checksum = DHT11_ReadByte();
    if ((data->humidity + data->hum_decimal + data->temperature + data->temp_decimal) == data->checksum) return 1;
    return 0;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // [초음파용 타이머 시작]
  HAL_TIM_Base_Start(&htim2);

  // [DHT11용 타이머 시작]
  HAL_TIM_Base_Start(&htim1);

  // [LCD 초기화]
  LCD_Init();
  LCD_Clear(BLUE);
  Draw_Expression(EXPR_HAPPY, BLUE);

  // [부저 초기화]
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_Delay(500);

  is_fire = 0; is_emergency = 0;

  // [UART 수신 인터럽트 시작]
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  printf("\n=== SmartGuardRover With Face LCD ===\n");
  last_cmd_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_time = 0;
  uint32_t last_light_check_time = 0;
  static uint32_t last_dht_time = 0;

  while (1)
  {
      Check_Tilt_State();
      Check_Fire_State();
      Update_Face_Logic();

      // [전복 처리]
      if (is_emergency == 1)
      {
          ST();
          // 복구 확인
          if (HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_RESET) {
              int clear_cnt = 0;
              for(int i=0; i<20; i++) {
                  HAL_Delay(100);
                  if(HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_SET) { clear_cnt = 0; break; }
                  clear_cnt++;
              }
              if(clear_cnt >= 20) {
                  is_emergency = 0; is_avoiding = 0;
                  last_cmd_time = HAL_GetTick();
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0);
                  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0);
                  printf("차체 복구!\r\n");
                  continue;
              }
          }
          continue;
      }

      // [화재 처리]
      if (is_fire == 1)
      {
          ST();
          // 복구 확인
          if (HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_RESET) {
              int clear_cnt = 0;
              for(int i=0; i<20; i++) {
                  HAL_Delay(100);
                  if(HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_SET) { clear_cnt = 0; break; }
                  clear_cnt++;
              }
              if(clear_cnt >= 20) {
                  is_fire = 0;
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0);
                  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0);
                  is_avoiding = 0; last_cmd_time = HAL_GetTick();
                  printf("화재 진압완료!\r\n");
                  continue;
              }
          }
          continue;
      }

      if (HAL_GetTick() - last_light_check_time > 500) { Check_Light(); last_light_check_time = HAL_GetTick(); }
      if (is_avoiding == 1) continue;
      if (HAL_GetTick() - last_cmd_time > AUTO_STOP_DELAY) ST();

      trig1(); echo_time1 = echo1(); dist1 = (echo_time1 > 0 && echo_time1 < 23000) ? (int)(17 * echo_time1 / 100) : 999; delay_us(5000);
      trig2(); echo_time2 = echo2(); dist2 = (echo_time2 > 0 && echo_time2 < 23000) ? (int)(17 * echo_time2 / 100) : 999;

      if ((dist1 < 150 && dist1 > 0) || (dist2 < 150 && dist2 > 0)) Avoid_Obstacle_Routine();
      if (HAL_GetTick() - last_print_time > 200) { printf("D: %d, %d\r\n", dist1, dist2); last_print_time = HAL_GetTick(); }

      // [온습도 측정 - 2초 간격]
      if(HAL_GetTick() - last_dht_time >= 2000)
      {
          if(DHT11_ReadData(&dht11_data)) {
              printf("Temp: %d°C, Hum: %d%%\r\n", dht11_data.temperature, dht11_data.humidity);
          }
          last_dht_time = HAL_GetTick();
      }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_LEFT_Pin|LED_RIGHT_Pin|TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RES_Pin|LCD_DC_Pin|TRIG2_Pin|LBF_Pin
                          |LFB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin
                          |RBF_Pin|RBB_Pin|LCD_CS_Pin|RFF_Pin
                          |RFB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAME_Pin */
  GPIO_InitStruct.Pin = FLAME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FLAME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LEFT_Pin LED_RIGHT_Pin TRIG1_Pin */
  GPIO_InitStruct.Pin = LED_LEFT_Pin|LED_RIGHT_Pin|TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TEMPER_Pin ECHO2_Pin */
  GPIO_InitStruct.Pin = TEMPER_Pin|ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RES_Pin LCD_DC_Pin TRIG2_Pin LBF_Pin
                           LFB_Pin */
  GPIO_InitStruct.Pin = LCD_RES_Pin|LCD_DC_Pin|TRIG2_Pin|LBF_Pin
                          |LFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO1_Pin ECHO3_Pin ECHO4_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO3_Pin|ECHO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LBB_Pin TRIG3_Pin TRIG4_Pin LFF_Pin
                           RBF_Pin RBB_Pin LCD_CS_Pin RFF_Pin
                           RFB_Pin */
  GPIO_InitStruct.Pin = LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin
                          |RBF_Pin|RBB_Pin|LCD_CS_Pin|RFF_Pin
                          |RFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : C6_Pin */
  GPIO_InitStruct.Pin = C6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(C6_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



// ============================================================================
// [LCD 그래픽 함수 구현]
// ============================================================================
static void LCD_Cmd(uint8_t cmd) {
    LCD_DC_LOW(); LCD_CS_LOW(); HAL_SPI_Transmit(&hspi1, &cmd, 1, 10); LCD_CS_HIGH();
}
static void LCD_Data(uint8_t data) {
    LCD_DC_HIGH(); LCD_CS_LOW(); HAL_SPI_Transmit(&hspi1, &data, 1, 10); LCD_CS_HIGH();
}
static void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LCD_Cmd(ST7735_CASET); LCD_Data(0); LCD_Data(x0 + X_OFFSET); LCD_Data(0); LCD_Data(x1 + X_OFFSET);
    LCD_Cmd(ST7735_RASET); LCD_Data(0); LCD_Data(y0 + Y_OFFSET); LCD_Data(0); LCD_Data(y1 + Y_OFFSET);
    LCD_Cmd(ST7735_RAMWR);
}
static void LCD_WriteColorFast(uint16_t color, uint32_t count) {
    uint8_t hi = color >> 8; uint8_t lo = color & 0xFF;
    LCD_DC_HIGH(); LCD_CS_LOW();
    while(count--) { HAL_SPI_Transmit(&hspi1, &hi, 1, 10); HAL_SPI_Transmit(&hspi1, &lo, 1, 10); }
    LCD_CS_HIGH();
}
static void LCD_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if(x >= LCD_WIDTH || y >= LCD_HEIGHT || w <= 0 || h <= 0) return;
    if(x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if(y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    LCD_SetWindow(x, y, x + w - 1, y + h - 1);
    LCD_WriteColorFast(color, (uint32_t)w * h);
}
void LCD_Clear(uint16_t color) { LCD_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, color); }

static void LCD_HLine(int16_t x, int16_t y, int16_t w, uint16_t color) { LCD_FillRect(x, y, w, 1, color); }
static void LCD_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t x = r, y = 0, err = 1 - r;
    while(x >= y) {
        LCD_HLine(x0 - x, y0 + y, x * 2 + 1, color); LCD_HLine(x0 - x, y0 - y, x * 2 + 1, color);
        LCD_HLine(x0 - y, y0 + x, y * 2 + 1, color); LCD_HLine(x0 - y, y0 - x, y * 2 + 1, color);
        y++; if(err < 0) err += 2 * y + 1; else { x--; err += 2 * (y - x + 1); }
    }
}
static void LCD_RoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
    LCD_FillRect(x+r, y, w-2*r, h, color); LCD_FillRect(x, y+r, r, h-2*r, color);
    LCD_FillRect(x+w-r, y+r, r, h-2*r, color);
    LCD_FillCircle(x+r, y+r, r, color); LCD_FillCircle(x+w-r-1, y+r, r, color);
    LCD_FillCircle(x+r, y+h-r-1, r, color); LCD_FillCircle(x+w-r-1, y+h-r-1, r, color);
}
static void LCD_ThickLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t t, uint16_t color) {
    int16_t dx = abs(x1-x0), dy = abs(y1-y0), sx = (x0<x1)?1:-1, sy = (y0<y1)?1:-1, err = dx-dy;
    while(1) {
        LCD_FillRect(x0-t/2, y0-t/2, t, t, color);
        if(x0==x1 && y0==y1) break;
        int16_t e2=2*err; if(e2 > -dy){err-=dy; x0+=sx;} if(e2 < dx){err+=dx; y0+=sy;}
    }
}

// [눈 그리기]
static void Eye_Normal(int16_t cx, int16_t ox, int16_t oy) {
    LCD_RoundRect(cx - EYE_W/2, CY - EYE_H/2, EYE_W, EYE_H, EYE_R, EYE_COLOR);
    LCD_FillCircle(cx - 5 + ox, CY - 8 + oy, 4, EYE_BRIGHT);
}
static void Eye_Happy(int16_t cx) {
    for(int16_t i = -EYE_W/2 + 2; i <= EYE_W/2 - 2; i++) {
        int32_t n = (int32_t)i * i * 100 / ((EYE_W/2) * (EYE_W/2));
        int16_t y = CY + 5 - (12 * (100 - n) / 100);
        LCD_FillRect(cx + i, y - 3, 2, 5, EYE_COLOR);
    }
}
// 악마 눈 (수정본)
static void Eye_Angry(int16_t cx, uint8_t is_left) {
    LCD_RoundRect(cx - EYE_W/2, CY - EYE_H/2 + 10, EYE_W, EYE_H - 14, EYE_R - 3, EYE_COLOR);
    int16_t horn_base_inx = 5; int16_t horn_base_iny = 8;
    int16_t horn_tip_outx = 12; int16_t horn_tip_outy = 12;
    uint16_t horn_color = BLACK;

    if(is_left) {
        LCD_ThickLine(cx - EYE_W/2 + horn_base_inx, CY - EYE_H/2 + horn_base_iny, cx - EYE_W/2 - horn_tip_outx, CY - EYE_H/2 - horn_tip_outy, 6, horn_color);
        LCD_ThickLine(cx - EYE_W/2 - horn_tip_outx + 3, CY - EYE_H/2 - horn_tip_outy + 3, cx - EYE_W/2 - horn_tip_outx, CY - EYE_H/2 - horn_tip_outy, 2, horn_color);
        LCD_FillCircle(cx + 4, CY + 3, 4, BLACK);
    } else {
        LCD_ThickLine(cx + EYE_W/2 - horn_base_inx, CY - EYE_H/2 + horn_base_iny, cx + EYE_W/2 + horn_tip_outx, CY - EYE_H/2 - horn_tip_outy, 6, horn_color);
        LCD_ThickLine(cx + EYE_W/2 + horn_tip_outx - 3, CY - EYE_H/2 - horn_tip_outy + 3, cx + EYE_W/2 + horn_tip_outx, CY - EYE_H/2 - horn_tip_outy, 2, horn_color);
        LCD_FillCircle(cx - 4, CY + 3, 4, BLACK);
    }
    if(is_left) LCD_ThickLine(cx - EYE_W/2 - 2, CY - EYE_H/2 + 12, cx + EYE_W/2 + 2, CY - EYE_H/2 + 2, 4, EYE_COLOR);
    else LCD_ThickLine(cx - EYE_W/2 - 2, CY - EYE_H/2 + 2, cx + EYE_W/2 + 2, CY - EYE_H/2 + 12, 4, EYE_COLOR);
}

void Draw_Expression(Expression_t expr, uint16_t bg_color) {
    LCD_Clear(bg_color);
    switch(expr) {
        case EXPR_NORMAL: Eye_Normal(LX,0,0); Eye_Normal(RX,0,0); break;
        case EXPR_HAPPY:  Eye_Happy(LX); Eye_Happy(RX); break;
        case EXPR_ANGRY:  Eye_Angry(LX,1); Eye_Angry(RX,0); break;
        default: Eye_Normal(LX,0,0); Eye_Normal(RX,0,0); break;
    }
}

void LCD_Init(void) {
    LCD_RES_LOW(); HAL_Delay(50); LCD_RES_HIGH(); HAL_Delay(50);
    LCD_Cmd(ST7735_SWRESET); HAL_Delay(150);
    LCD_Cmd(ST7735_SLPOUT); HAL_Delay(150);
    LCD_Cmd(0xB1); LCD_Data(0x01); LCD_Data(0x2C); LCD_Data(0x2D);
    LCD_Cmd(0xB4); LCD_Data(0x07);
    LCD_Cmd(0xC0); LCD_Data(0xA2); LCD_Data(0x02); LCD_Data(0x84);
    LCD_Cmd(ST7735_MADCTL); LCD_Data(0x60);
    LCD_Cmd(ST7735_COLMOD); LCD_Data(0x05);
    LCD_Cmd(ST7735_NORON); HAL_Delay(10);
    LCD_Cmd(ST7735_DISPON); HAL_Delay(100);
}

void Update_Face_Logic(void) {
    Expression_t target_expr = EXPR_HAPPY;
    uint16_t target_bg = BLUE;

    if (is_emergency == 1 || is_fire == 1) {
        target_expr = EXPR_ANGRY;
        target_bg = RED;
    } else {
        target_expr = EXPR_HAPPY;
        target_bg = BLUE;
    }

    if (target_expr != current_expr || target_bg != current_bg_color) {
        current_expr = target_expr;
        current_bg_color = target_bg;
        Draw_Expression(current_expr, current_bg_color);
    }
}

// ============================================================================
// [스마트카 모터 및 센서 로직]
// ============================================================================
int LF(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); break; case 0: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1); break; case 2: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); break; } return 0; }
int LB(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); break; case 0: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1); break; case 2: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); break; } return 0; }
int RF(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); break; case 0: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1); break; case 2: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); break; } return 0; }
int RB(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); break; case 0: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1); break; case 2: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); break; } return 0; }
void MF() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
void MB() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1); }
void ML() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
void MR() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
void ST() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
int TLF(){ RF(1); RB(1); LF(2); LB(2); return 0; } int TRF(){ RF(2); RB(2); LF(1); LB(1); return 0; }
int TLB(){ RF(0); RB(0); LF(2); LB(2); return 0; } int TRB(){ RF(2); RB(2); LF(0); LB(0); return 0; }
int LFRB(){ RF(0); RB(0); LF(1); LB(1); return 0; } int RFLB(){ RF(1); RB(1); LF(0); LB(0); return 0; }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        HAL_UART_Transmit(&huart2, &rx_data, 1, 10);
        if (is_avoiding == 0 && is_emergency == 0 && is_fire == 0) {
            last_cmd_time = HAL_GetTick();
            switch (rx_data) {
                case 'W': case 'w': MF(); break; case 'A': case 'a': ML(); break;
                case 'D': case 'd': MR(); break; case 'S': case 's': MB(); break;
                case 'Q': case 'q': TLF(); break; case 'E': case 'e': TRF(); break;
                case 'Z': case 'z': TLB(); break; case 'C': case 'c': TRB(); break;
                case 'R': case 'r': LFRB(); break; case 'T': case 't': RFLB(); break;
                default: ST(); break;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

void Check_Tilt_State(void) {
    static uint32_t tilt_start_time = 0;
    if (HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_SET) {
        if (tilt_start_time == 0) tilt_start_time = HAL_GetTick();
        if (HAL_GetTick() - tilt_start_time > 2000) {
            ST(); is_emergency = 1;
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
            printf("⚠️차량 전복됨!!\r\n");
            tilt_start_time = 0;
        }
    } else { tilt_start_time = 0; }
}

void Check_Fire_State(void) {
    if (HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_SET) {
        if (is_fire == 0) {
            ST(); is_fire = 1;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
            printf("🔥 화재 발생!\r\n");
        }
    }
}

void Avoid_Obstacle_Routine(void) {
    is_avoiding = 1; ST(); HAL_Delay(500);
    uint32_t last_avoid_light_check = 0;
    trig3(); echo_time3 = echo3(); dist3 = (echo_time3 > 0 && echo_time3 < 23000) ? (int)(17 * echo_time3 / 100) : 999; delay_us(15000);
    trig4(); echo_time4 = echo4(); dist4 = (echo_time4 > 0 && echo_time4 < 23000) ? (int)(17 * echo_time4 / 100) : 999;
    int escape_plan = 0;
    if (dist3 <= 150 && dist4 > 150) escape_plan = 1; else if (dist4 <= 150 && dist3 > 150) escape_plan = 2; else if (dist3 > 150 && dist4 > 150) escape_plan = 3; else escape_plan = 4;
    printf("회피기동모드: %d\r\n", escape_plan);
    while (1) {
        Check_Tilt_State(); Check_Fire_State(); Update_Face_Logic();
        if (is_emergency == 1 || is_fire == 1) { ST(); is_avoiding = 0; break; }
        if (HAL_GetTick() - last_avoid_light_check > 500) { Check_Light(); last_avoid_light_check = HAL_GetTick(); }
        trig1(); echo_time1 = echo1(); dist1 = (echo_time1 > 0 && echo_time1 < 23000) ? (int)(17 * echo_time1 / 100) : 999; delay_us(2000);
        trig2(); echo_time2 = echo2(); dist2 = (echo_time2 > 0 && echo_time2 < 23000) ? (int)(17 * echo_time2 / 100) : 999;
        if (dist1 > 350 && dist2 > 350) { ST(); is_avoiding = 0; last_cmd_time = HAL_GetTick(); break; }
        switch (escape_plan) { case 1: LFRB(); break; case 2: RFLB(); break; case 3: MB(); HAL_Delay(500); escape_plan = 2; break; case 4: MB(); break; }
        HAL_Delay(100);
    }
}

void timer_start(void) { HAL_TIM_Base_Start(&htim2); }
void delay_us(uint16_t us) { __HAL_TIM_SET_COUNTER(&htim2, 0); while((__HAL_TIM_GET_COUNTER(&htim2)) < us); }
void trig1(void) { HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, LOW); }
long unsigned int echo1(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == HIGH) { timeout++; if(timeout > 20000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
void trig2(void) { HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, LOW); }
long unsigned int echo2(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == HIGH) { timeout++; if(timeout > 20000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
void trig3(void) { HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, LOW); }
long unsigned int echo3(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == HIGH) { timeout++; if(timeout > 40000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
void trig4(void) { HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, LOW); }
long unsigned int echo4(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == HIGH) { timeout++; if(timeout > 40000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }

void Check_Light(void)
{
    uint32_t adc_value = 0; float voltage = 0.0f;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1); voltage = (adc_value * 3.3f) / 4095.0f;
        if (voltage < 2.0f && is_fire == 0) { HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1); HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1); }
        else if (is_fire == 0) { HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0); HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0); }
    }
    HAL_ADC_Stop(&hadc1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
