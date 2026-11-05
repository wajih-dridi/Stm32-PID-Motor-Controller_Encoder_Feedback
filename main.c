/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"


/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "pid.h"
#include "motor_control.h"

#include <stdio.h>
#include <string.h>

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

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;


/* USER CODE BEGIN PV */

/*
 * Main motor controller object.
 */
MotorController_t motor;


/*
 * Flag used to tell the main loop that
 * a new control calculation has occurred.
 *
 * This variable is modified inside the TIM6
 * interrupt and read inside the main loop.
 */
volatile uint8_t control_update_flag = 0;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

void MX_USB_HOST_Process(void);


/* USER CODE BEGIN PFP */

/*
 * Set TIM3 PWM duty cycle.
 */
static void Motor_SetPWM(float percentage);


/*
 * Send a string through USART2.
 */
static void UART_SendString(const char *message);

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

    /*
     * Reset of all peripherals,
     * Initializes the Flash interface and the Systick.
     */
    HAL_Init();


    /* USER CODE BEGIN Init */

    /* USER CODE END Init */


    /*
     * Configure the system clock.
     */
    SystemClock_Config();


    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */


    /*
     * Initialize all configured peripherals.
     */
    MX_GPIO_Init();

    MX_I2S3_Init();

    MX_USB_HOST_Init();

    MX_TIM3_Init();

    MX_TIM4_Init();

    MX_TIM6_Init();

    MX_USART2_UART_Init();


    /* USER CODE BEGIN 2 */


    /*
     * Initialize the motor controller.
     */
    MotorController_Init(&motor);


    /*
     * Set desired motor speed.
     *
     * Target = 1500 RPM
     */
    MotorController_SetTarget(&motor, 1500.0f);


    /*
     * Start TIM3 PWM.
     *
     * TIM3 CH1 -> PA6
     */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Start TIM4 encoder interface.
     *
     * TIM4 CH1 -> PB6
     * TIM4 CH2 -> PB7
     */
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Reset encoder counter to zero.
     */
    __HAL_TIM_SET_COUNTER(&htim4, 0);


    /*
     * Start TIM6 in interrupt mode.
     *
     * TIM6:
     *
     * 100 Hz
     * 10 ms period
     */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Start with 0% PWM.
     */
    Motor_SetPWM(0.0f);


    /* USER CODE END 2 */


    /* Infinite loop */

    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* USER CODE BEGIN 3 */


        /*
         * Keep USB Host processing because this project
         * was originally generated with the Discovery
         * USB Host middleware.
         */
        MX_USB_HOST_Process();


        /*
         * Check whether the TIM6 control loop has
         * completed a new calculation.
         */
        if (control_update_flag)
        {
            char message[120];


            /*
             * Clear the flag.
             */
            control_update_flag = 0;


            /*
             * Format motor information.
             */
            sprintf(message,
                    "Target: %.1f RPM | Actual: %.1f RPM | PWM: %.1f %%\r\n",
                    motor.target_rpm,
                    motor.actual_rpm,
                    motor.pwm_percent);


            /*
             * Send information to PC through USART2.
             */
            UART_SendString(message);
        }


        /* USER CODE END 3 */
    }

    /* USER CODE END WHILE */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /*
     * Configure the main internal regulator output voltage.
     */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


    /*
     * Configure HSE and PLL.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState = RCC_HSE_ON;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 8;

    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ = 7;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Configure CPU, AHB and APB buses clocks.
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;


    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */

static void MX_I2S3_Init(void)
{
    /* USER CODE BEGIN I2S3_Init 0 */

    /* USER CODE END I2S3_Init 0 */


    /* USER CODE BEGIN I2S3_Init 1 */

    /* USER CODE END I2S3_Init 1 */


    hi2s3.Instance = SPI3;

    hi2s3.Init.Mode = I2S_MODE_MASTER_TX;

    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;

    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;

    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;

    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;

    hi2s3.Init.CPOL = I2S_CPOL_LOW;

    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;

    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;


    if (HAL_I2S_Init(&hi2s3) != HAL_OK)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN I2S3_Init 2 */

    /* USER CODE END I2S3_Init 2 */
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


    TIM_MasterConfigTypeDef sMasterConfig = {0};

    TIM_OC_InitTypeDef sConfigOC = {0};


    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */


    /*
     * TIM3 configuration.
     *
     * Timer clock = 84 MHz
     *
     * Prescaler = 83
     *
     * 84 MHz / (83 + 1)
     * = 1 MHz
     *
     * Period = 99
     *
     * 1 MHz / (99 + 1)
     * = 10 kHz PWM
     */
    htim3.Instance = TIM3;

    htim3.Init.Prescaler = 83;

    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;

    htim3.Init.Period = 99;

    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    /*
     * IMPORTANT:
     *
     * TIM3 is PWM.
     *
     * Do NOT use HAL_TIM_OC_Init().
     */
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Master configuration.
     */
    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim3,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * TIM3 Channel 1 PWM configuration.
     */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 0;

    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;


    if (HAL_TIM_PWM_ConfigChannel(
            &htim3,
            &sConfigOC,
            TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Configure TIM3 GPIO.
     *
     * PA6 -> TIM3_CH1
     */
    HAL_TIM_MspPostInit(&htim3);


    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}


/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */


    TIM_Encoder_InitTypeDef sConfig = {0};

    TIM_MasterConfigTypeDef sMasterConfig = {0};


    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */


    /*
     * TIM4 Encoder configuration.
     *
     * TIM4:
     *
     * CH1 -> PB6
     * CH2 -> PB7
     */
    htim4.Instance = TIM4;

    htim4.Init.Prescaler = 0;

    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;

    htim4.Init.Period = 65535;

    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    htim4.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    /*
     * Quadrature encoder mode.
     */
    sConfig.EncoderMode =
        TIM_ENCODERMODE_TI12;


    /*
     * Encoder Channel 1.
     */
    sConfig.IC1Polarity =
        TIM_ICPOLARITY_RISING;

    sConfig.IC1Selection =
        TIM_ICSELECTION_DIRECTTI;

    sConfig.IC1Prescaler =
        TIM_ICPSC_DIV1;

    sConfig.IC1Filter = 0;


    /*
     * Encoder Channel 2.
     */
    sConfig.IC2Polarity =
        TIM_ICPOLARITY_RISING;

    sConfig.IC2Selection =
        TIM_ICSELECTION_DIRECTTI;

    sConfig.IC2Prescaler =
        TIM_ICPSC_DIV1;

    sConfig.IC2Filter = 0;


    /*
     * Initialize encoder interface.
     */
    if (HAL_TIM_Encoder_Init(&htim4,
                             &sConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Master configuration.
     */
    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim4,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}


/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM6_Init(void)
{
    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */


    TIM_MasterConfigTypeDef sMasterConfig = {0};


    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */


    /*
     * TIM6 configuration.
     *
     * Timer clock = 84 MHz
     *
     * Prescaler = 8399
     *
     * 84 MHz / 8400
     * = 10 kHz
     *
     * Period = 99
     *
     * 10 kHz / 100
     * = 100 Hz
     *
     * Period = 10 ms
     */
    htim6.Instance = TIM6;

    htim6.Init.Prescaler = 8399;

    htim6.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim6.Init.Period = 99;

    htim6.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    /*
     * Initialize TIM6 base timer.
     */
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Master configuration.
     */
    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim6,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */
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

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;


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

    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_GPIOH_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();


    /* Configure GPIO pin Output Level */

    HAL_GPIO_WritePin(
        CS_I2C_SPI_GPIO_Port,
        CS_I2C_SPI_Pin,
        GPIO_PIN_RESET);


    /* Configure GPIO pin Output Level */

    HAL_GPIO_WritePin(
        OTG_FS_PowerSwitchOn_GPIO_Port,
        OTG_FS_PowerSwitchOn_Pin,
        GPIO_PIN_SET);


    /* Configure GPIO pin Output Level */

    HAL_GPIO_WritePin(
        GPIOD,
        LD4_Pin |
        LD3_Pin |
        LD5_Pin |
        LD6_Pin |
        Audio_RST_Pin,
        GPIO_PIN_RESET);


    /* Configure GPIO pin : CS_I2C_SPI_Pin */

    GPIO_InitStruct.Pin =
        CS_I2C_SPI_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        CS_I2C_SPI_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */

    GPIO_InitStruct.Pin =
        OTG_FS_PowerSwitchOn_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        OTG_FS_PowerSwitchOn_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : PDM_OUT_Pin */

    GPIO_InitStruct.Pin =
        PDM_OUT_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Alternate =
        GPIO_AF5_SPI2;

    HAL_GPIO_Init(
        PDM_OUT_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : B1_Pin */

    GPIO_InitStruct.Pin =
        B1_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_EVT_RISING;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        B1_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : SPI1_SCK_Pin */

    GPIO_InitStruct.Pin =
        SPI1_SCK_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Alternate =
        GPIO_AF5_SPI1;

    HAL_GPIO_Init(
        SPI1_SCK_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : BOOT1_Pin */

    GPIO_InitStruct.Pin =
        BOOT1_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        BOOT1_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : CLK_IN_Pin */

    GPIO_InitStruct.Pin =
        CLK_IN_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Alternate =
        GPIO_AF5_SPI2;

    HAL_GPIO_Init(
        CLK_IN_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pins :
       LD4_Pin
       LD3_Pin
       LD5_Pin
       LD6_Pin
       Audio_RST_Pin
    */

    GPIO_InitStruct.Pin =
        LD4_Pin |
        LD3_Pin |
        LD5_Pin |
        LD6_Pin |
        Audio_RST_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOD,
        &GPIO_InitStruct);


    /* Configure GPIO pin : OTG_FS_OverCurrent_Pin */

    GPIO_InitStruct.Pin =
        OTG_FS_OverCurrent_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        OTG_FS_OverCurrent_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : Audio_SDA_Pin */

    GPIO_InitStruct.Pin =
        Audio_SDA_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_OD;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Alternate =
        GPIO_AF4_I2C1;

    HAL_GPIO_Init(
        Audio_SDA_GPIO_Port,
        &GPIO_InitStruct);


    /* Configure GPIO pin : MEMS_INT2_Pin */

    GPIO_InitStruct.Pin =
        MEMS_INT2_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_EVT_RISING;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        MEMS_INT2_GPIO_Port,
        &GPIO_InitStruct);


    /*
     * IMPORTANT:
     *
     * TIM3/TIM4/USART2 pins are configured by
     * their HAL MSP initialization functions:
     *
     * PA6 -> TIM3_CH1
     * PB6 -> TIM4_CH1
     * PB7 -> TIM4_CH2
     * PA2 -> USART2_TX
     * PA3 -> USART2_RX
     *
     * These should be generated by CubeMX.
     */


    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */


/**
  * @brief Set TIM3 PWM duty cycle.
  *
  * @param percentage PWM duty cycle from 0 to 100.
  */

static void Motor_SetPWM(float percentage)
{
    uint32_t arr;

    uint32_t compare;


    /*
     * Limit PWM to 0...100%.
     */
    if (percentage < 0.0f)
    {
        percentage = 0.0f;
    }


    if (percentage > 100.0f)
    {
        percentage = 100.0f;
    }


    /*
     * Read TIM3 Auto Reload Register.
     *
     * Our configuration:
     *
     * ARR = 99
     */
    arr =
        __HAL_TIM_GET_AUTORELOAD(&htim3);


    /*
     * Convert percentage to compare value.
     */
    compare =
        (uint32_t)(
            (percentage / 100.0f) *
            (float)arr
        );


    /*
     * Update TIM3 Channel 1.
     *
     * TIM3 CH1 -> PA6
     */
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        compare
    );
}


/**
  * @brief Send string through USART2.
  */

static void UART_SendString(const char *message)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)message,
        strlen(message),
        100
    );
}


/**
  * @brief Timer Period Elapsed Callback.
  *
  * TIM6 calls this function every 10 ms.
  */

void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim)
{
    /*
     * Make sure the interrupt came from TIM6.
     */
    if (htim->Instance == TIM6)
    {
        uint16_t encoder;


        /*
         * Read the TIM4 encoder counter.
         *
         * TIM4 = ENCODER
         *
         * TIM3 = PWM
         */
        encoder =
            (uint16_t)__HAL_TIM_GET_COUNTER(
                &htim4
            );


        /*
         * Update motor controller.
         *
         * dt = 0.01 second
         *
         * because TIM6 period = 10 ms.
         */
        MotorController_Update(
            &motor,
            encoder,
            0.01f
        );


        /*
         * Apply PID output to TIM3 PWM.
         */
        Motor_SetPWM(
            motor.pwm_percent
        );


        /*
         * Tell main loop that new data
         * is available.
         */
        control_update_flag = 1;
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

    __disable_irq();

    while (1)
    {
    }

    /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

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

    /*
     * User can add implementation here to report
     * the file name and line number.
     */

    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
