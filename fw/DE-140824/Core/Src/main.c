/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_TRIAC 16
#define SNAKE_LENGTH 4  // Dužina zmije
#define SNAKE_SPEED 20   // Brzina zmije (veća vrednost = sporije)
#define MENU_TIMEOUT    10000 // 10 sekundi timeout
#define BUTTON_PAUSE    200 // dodatni debouncing
#define TOGGLE_INTERVAL 50 // Period treptanja LED
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
GPIO_TypeDef *TRIAC_PORT[NUMBER_OF_TRIAC] = {GPIOB, GPIOB, GPIOB, GPIOA, GPIOF, GPIOF, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t GATE_PIN[NUMBER_OF_TRIAC] = {GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_15, GPIO_PIN_7, GPIO_PIN_6 ,GPIO_PIN_12 ,GPIO_PIN_11, GPIO_PIN_8, \
                        GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_2};

static uint8_t snake_head = 0;  // Trenutna glava zmije
static uint8_t snake_counter = 0;  // Brojač za usporavanje

// Stanje LED dioda (simulacija konstante, treba postavljati u aplikaciji)
volatile uint8_t led_states[NUMBER_OF_TRIAC] = {0};
volatile uint8_t triac_states[NUMBER_OF_TRIAC] = {0};  // Trenutnostanje releja

// Trenutno aktivni red
static uint8_t active_row = 0;

volatile uint32_t last_press_time = 0;
volatile uint32_t menu_timer = 0;
volatile uint32_t last_toggle_time = 0;
volatile uint8_t toggle_state = 0;
volatile uint8_t menu_activ = 0;
volatile uint8_t selected_channel = NUMBER_OF_TRIAC - 1;
uint8_t button_select_prev = 0, button_onoff_prev = 0;
static uint8_t my_address = 0;
static uint8_t first_toggle = 1;
static uint16_t triac_start = 0;
static uint16_t triac_end = 0;
bool init_tf = false;
bool setup_mod = false;
uint8_t rec;
TinyFrame tfapp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void RS485_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Bus address update
  * @retval None
  */
void get_address(void)
{
    my_address = 0;
    if ((HAL_GPIO_ReadPin(ADDRESS0_GPIO_Port, ADDRESS0_Pin) == GPIO_PIN_RESET))   my_address |= 0x20;
    if ((HAL_GPIO_ReadPin(ADDRESS1_GPIO_Port, ADDRESS1_Pin) == GPIO_PIN_RESET))   my_address |= 0x10;
    if ((HAL_GPIO_ReadPin(ADDRESS2_GPIO_Port, ADDRESS2_Pin) == GPIO_PIN_RESET))   my_address |= 0x08;
    if ((HAL_GPIO_ReadPin(ADDRESS3_GPIO_Port, ADDRESS3_Pin) == GPIO_PIN_RESET))   my_address |= 0x04;
    if ((HAL_GPIO_ReadPin(ADDRESS4_GPIO_Port, ADDRESS4_Pin) == GPIO_PIN_RESET))   my_address |= 0x02;
    if ((HAL_GPIO_ReadPin(ADDRESS5_GPIO_Port, ADDRESS5_Pin) == GPIO_PIN_RESET))   my_address |= 0x01;
    if(my_address)
    {
        triac_start = (NUMBER_OF_TRIAC*(my_address-1))+1;
        triac_end = triac_start + NUMBER_OF_TRIAC - 1;
    }
}
/**
  * @brief LED efect for settings menu
  * @retval None
  */
void update_snake(void) 
{
    if (++snake_counter < SNAKE_SPEED) {
        return;  // Čekamo dok ne prođe dovoljno ciklusa
    }
    snake_counter = 0;  // Reset brojača

    // Očisti prethodno stanje LED-ova
    for (uint8_t i = 0; i < 16; i++) {
        led_states[i] = 0;  // Privremeno uklanjamo const
    }

    // Podesi svetleće LED-ove
    for (uint8_t i = 0; i < SNAKE_LENGTH; i++) {
        int8_t pos = snake_head - i;
        if (pos < 0) pos += 16;  // Omogućava kružno kretanje po matrici
        led_states[pos] = 1;  // Upali LED
    }

    // Pomeraj zmiju napred
    snake_head = (snake_head + 1) % 16;
}
/**
  * @brief Refresh triac gate outputs
  * @retval None
  */
void set_triac(void)
{
    // Samo proleti kroz sve izlaze i postavi stanje prema baferu
    for (uint8_t i = 0; i < NUMBER_OF_TRIAC; i++)
    {
        HAL_GPIO_WritePin(TRIAC_PORT[i], GATE_PIN[i], (triac_states[i] > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
/**
  * @brief Copy triac states to LED
  * @retval None
  */
void refresh_led(void)
{
    // Samo kopiraj izlaze na LED
    for (uint8_t i = 0; i < NUMBER_OF_TRIAC; i++)
    {
        led_states[i] = triac_states[i];
    }
}
/**
  * @brief Funkcija za osvežavanje LED matrice
  * @retval None
  */
void refresh_led_matrix(void) 
{
    // Isključi trenutno aktivni red
    HAL_GPIO_WritePin(LED_R0_GPIO_Port, LED_R0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R1_GPIO_Port, LED_R1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R2_GPIO_Port, LED_R2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R3_GPIO_Port, LED_R3_Pin, GPIO_PIN_RESET);

    // Očisti kolone
    HAL_GPIO_WritePin(LED_C0_GPIO_Port, LED_C0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_C1_GPIO_Port, LED_C1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_C2_GPIO_Port, LED_C2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_C3_GPIO_Port, LED_C3_Pin, GPIO_PIN_SET);

    // Aktiviraj sledeći red
    switch (active_row) {
    case 0:
        HAL_GPIO_WritePin(LED_R0_GPIO_Port, LED_R0_Pin, GPIO_PIN_SET);
        break;
    case 1:
        HAL_GPIO_WritePin(LED_R1_GPIO_Port, LED_R1_Pin, GPIO_PIN_SET);
        break;
    case 2:
        HAL_GPIO_WritePin(LED_R2_GPIO_Port, LED_R2_Pin, GPIO_PIN_SET);
        break;
    case 3:
        HAL_GPIO_WritePin(LED_R3_GPIO_Port, LED_R3_Pin, GPIO_PIN_SET);
        break;
    }

    // Postavi kolone na osnovu led_state[] tako da se kolone i redovi zamjene
    for (uint8_t row = 0; row < 4; row++) {
        uint8_t index = row * 4 + active_row;  // Zamjeni red i kolonu
        if (led_states[index]) {
            switch (row) {
            case 0:
                HAL_GPIO_WritePin(LED_C0_GPIO_Port, LED_C0_Pin, GPIO_PIN_RESET);
                break;
            case 1:
                HAL_GPIO_WritePin(LED_C1_GPIO_Port, LED_C1_Pin, GPIO_PIN_RESET);
                break;
            case 2:
                HAL_GPIO_WritePin(LED_C2_GPIO_Port, LED_C2_Pin, GPIO_PIN_RESET);
                break;
            case 3:
                HAL_GPIO_WritePin(LED_C3_GPIO_Port, LED_C3_Pin, GPIO_PIN_RESET);
                break;
            }
        }
    }

    // Prebaci na sledeći red
    active_row = (active_row + 1) % 4;
}
/**
  * @brief Front panel taster button processing function
  * @retval None
  */
void handle_buttons(void)
{
    uint32_t current_time = HAL_GetTick();
    uint8_t button_select = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14);
    uint8_t button_onoff = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13);

    if (button_select == GPIO_PIN_RESET) {
        if (!button_select_prev) {
            button_select_prev = 1; // lock press time load
            menu_activ = 1;
            first_toggle = 1;
            if (++selected_channel >= NUMBER_OF_TRIAC) selected_channel = 0;
            toggle_state = triac_states[selected_channel]; // Resetuj toggle_state po novom kanalu
            menu_timer = current_time + MENU_TIMEOUT;
            last_press_time = current_time; // load first press time
            refresh_led();
            HAL_Delay(BUTTON_PAUSE);
        }
    }
    else if ((button_select == GPIO_PIN_SET) && button_select_prev)
    {
        button_select_prev = 0;
        HAL_Delay(BUTTON_PAUSE);
        refresh_led();
    }

    if (menu_activ && (button_onoff == GPIO_PIN_RESET)) {
        if (!button_onoff_prev) {
            button_onoff_prev = 1; // lock press time load
            last_press_time = current_time; // load first press time
            triac_states[selected_channel] ^= 1; // Toggle stanja triaka

            // Resetujemo first_toggle i toggle_state kako bi ispravno započelo treperenje
            first_toggle = 1;
            toggle_state = triac_states[selected_channel]; 
            HAL_Delay(BUTTON_PAUSE);
            refresh_led();
        }
        menu_timer = current_time + MENU_TIMEOUT;
    }
    else if ((button_onoff == GPIO_PIN_SET) && button_onoff_prev)
    {
        button_onoff_prev = 0; // release button press
        HAL_Delay(BUTTON_PAUSE);
    }
}
/**
  * @brief Jednostavan meni za podešavanje stanja izlaza sa različitim odnosom pauze i signala
  *         odabrane LED prema stanju izlaza kraći signal duža pauza za isključen izlaz    
  * @retval None
  */
void menu_logic(void)
{
    uint32_t current_time = HAL_GetTick();

    // Resetujemo meni ako istekne vrijeme
    if (current_time > menu_timer) {
        selected_channel = NUMBER_OF_TRIAC - 1;
        menu_activ = 0;
        toggle_state = 0;
        refresh_led();
        first_toggle = 1;  // Resetuj first_toggle ako se resetuje meni
    }

    // Definiši vremenske intervale zavisno od triac_states
    uint32_t on_time, off_time;
    if (triac_states[selected_channel] == 1) {
        on_time = TOGGLE_INTERVAL * 5;  // LED ON duže
        off_time = TOGGLE_INTERVAL;     // LED OFF kraće
    } else {
        on_time = TOGGLE_INTERVAL;      // LED ON kraće
        off_time = TOGGLE_INTERVAL * 5; // LED OFF duže
    }

    // Pri prvom ulasku postavi početno stanje
    if (first_toggle) {
        toggle_state = triac_states[selected_channel]; // Ako je triac_state 1, LED počinje upaljena
        last_toggle_time = current_time;  // Resetujemo timer
        first_toggle = 0; // Obilježimo da više nije prvi ulazak
    }

    // Odredi trenutno trajanje na osnovu toggle_state
    uint32_t toggle_period = toggle_state ? on_time : off_time;

    // Ako je prošao period, promjeni stanje
    if (current_time - last_toggle_time >= toggle_period) {
        led_states[selected_channel] ^= 1;  // Toggler LED stanje
        toggle_state = !toggle_state;       // Promjena stanja za sljedeći ciklus
        last_toggle_time = current_time;    // Ažuriraj vrijeme zadnjeg toggle-a
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
    MX_ADC_Init();
    MX_I2C1_Init();
    MX_IWDG_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();
    MX_TIM14_Init();
    /* USER CODE BEGIN 2 */
    RS485_Init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        get_address();
        handle_buttons();
        menu_logic();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        set_triac();
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                                       |RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_4;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0010020A;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

    /* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
    /* USER CODE END IWDG_Init 0 */

    /* USER CODE BEGIN IWDG_Init 1 */

    /* USER CODE END IWDG_Init 1 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN IWDG_Init 2 */
#endif
    /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

    /* USER CODE BEGIN TIM14_Init 0 */

    /* USER CODE END TIM14_Init 0 */

    /* USER CODE BEGIN TIM14_Init 1 */

    /* USER CODE END TIM14_Init 1 */
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 48000 - 1;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 5 - 1;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */
    HAL_TIM_Base_Start_IT(&htim14);
    /* USER CODE END TIM14_Init 2 */

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
    if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

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
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED_R3_Pin|LED_R2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, LED_R1_Pin|LED_R0_Pin|THY_GATE5_Pin|THY_GATE4_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, THY_GATE15_Pin|THY_GATE14_Pin|THY_GATE13_Pin|THY_GATE12_Pin
                      |THY_GATE11_Pin|THY_GATE10_Pin|THY_GATE9_Pin|THY_GATE2_Pin
                      |THY_GATE1_Pin|THY_GATE0_Pin|LED_C3_Pin|LED_C2_Pin
                      |LED_C1_Pin|LED_C0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, THY_GATE8_Pin|THY_GATE7_Pin|THY_GATE6_Pin|THY_GATE3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED_R3_Pin LED_R2_Pin */
    GPIO_InitStruct.Pin = LED_R3_Pin|LED_R2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : ADDRESS5_Pin */
    GPIO_InitStruct.Pin = ADDRESS5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADDRESS5_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_R1_Pin LED_R0_Pin THY_GATE5_Pin THY_GATE4_Pin */
    GPIO_InitStruct.Pin = LED_R1_Pin|LED_R0_Pin|THY_GATE5_Pin|THY_GATE4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pins : ADDRESS4_Pin ADDRESS3_Pin ADDRESS2_Pin */
    GPIO_InitStruct.Pin = ADDRESS4_Pin|ADDRESS3_Pin|ADDRESS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : ADDRESS1_Pin ADDRESS0_Pin */
    GPIO_InitStruct.Pin = ADDRESS1_Pin|ADDRESS0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : THY_GATE15_Pin THY_GATE14_Pin THY_GATE13_Pin THY_GATE12_Pin
                             THY_GATE11_Pin THY_GATE10_Pin THY_GATE9_Pin THY_GATE2_Pin
                             THY_GATE1_Pin THY_GATE0_Pin LED_C3_Pin LED_C2_Pin
                             LED_C1_Pin LED_C0_Pin */
    GPIO_InitStruct.Pin = THY_GATE15_Pin|THY_GATE14_Pin|THY_GATE13_Pin|THY_GATE12_Pin
                          |THY_GATE11_Pin|THY_GATE10_Pin|THY_GATE9_Pin|THY_GATE2_Pin
                          |THY_GATE1_Pin|THY_GATE0_Pin|LED_C3_Pin|LED_C2_Pin
                          |LED_C1_Pin|LED_C0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : THY_GATE8_Pin THY_GATE7_Pin THY_GATE6_Pin THY_GATE3_Pin */
    GPIO_InitStruct.Pin = THY_GATE8_Pin|THY_GATE7_Pin|THY_GATE6_Pin|THY_GATE3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
#ifndef USE_DEBUGGER
    /*Configure GPIO pins : ADDRESS4_Pin ADDRESS3_Pin ADDRESS2_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Timer interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {
        if(setup_mod == true) update_snake();  // Ažuriraj stanje zmije ako je mod podešavanja aktivan
        refresh_led_matrix();
    }
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result BINARY_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t idx = 0;
    uint8_t res = 0;
    uint16_t adr = ((msg->data[0] << 8) | msg->data[1]); // Kombinacija dva bajta u adresu
    uint8_t ret[1]; // Niz za vraćanje stanja

    // Proveravamo da li je adresa unutar opsega rel_start i rel_end
    if (adr >= triac_start && adr <= triac_end) {
        idx = adr - triac_start; // Izračunavanje indeksa releja
        if (msg->data[2] == 0x01) {
            triac_states[idx] = 1;  // Uključivanje releja
        } else if (msg->data[2] == 0x02) {
            triac_states[idx] = 0;  // Isključivanje releja
        }
        refresh_led();
        res++;  // Povećavamo broj uspešnih operacija
    }

    // Ako je došlo do promene stanja, vraćamo novo stanje
    if (res == 1) {
        if (triac_states[idx] == 1) {
            ret[0] = 1;  // Stanje 1 (ON)
        } else {
            ret[0] = 2;  // Stanje 2 (OFF)
        }

        msg->data = ret;  // Postavljamo novi niz za odgovor
        msg->len = 1;  // Dužina odgovora je 1
        TF_Respond(tf, msg);  // Odgovaramo sa novim stanjem
    }

    return TF_STAY;  // Održavanje trenutnog stanja
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint16_t adr = ((msg->data[0] << 8) | msg->data[1]);  // Kombinacija dva bajta u adresu
    uint8_t ret[1]; // Niz za vraćanje stanja

    // Proveravamo da li je adresa unutar opsega rel_start i rel_end
    if (adr >= triac_start && adr <= triac_end) {
        uint8_t idx = adr - triac_start;  // Izračunavanje indeksa releja
        // Vraćamo stanje releja na toj adresi
        if (triac_states[idx] == 1) {
            ret[0] = 1;  // Stanje 1 (ON)
        } else {
            ret[0] = 2;  // Stanje 2 (OFF)
        }

        msg->data = ret;  // Postavljamo novi niz sa stanjem releja
        msg->len = 1;  // Dužina odgovora je 1 (samo jedan bajt)
        TF_Respond(tf, msg);  // Odgovaramo sa stanjem releja
    }

    return TF_STAY;  // Održavanje trenutnog stanja
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Init(void)
{
    if(!init_tf) {
        init_tf = TF_InitStatic(&tfapp, TF_SLAVE);
        TF_AddTypeListener(&tfapp, V_STATUS, STATUS_Listener);
        TF_AddTypeListener(&tfapp, S_BINARY, BINARY_Listener);
    }
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Tick(void)
{
    if (init_tf == true) {
        TF_Tick(&tfapp);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    HAL_UART_Transmit(&huart1,(uint8_t*)buff, len, RESP_TOUT);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    TF_AcceptChar(&tfapp, rec);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
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
