/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

#include "lsm6dsv_reg.h"

#include <string.h>
#include "utils.h"
#include "comdef.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_ADDR 0x6
#define UART_TX_SIZE 12
#define UART_RX_SIZE 3

// #define SENSOR_BUS hi2c1
#define SENSOR_BUS hspi1

#define BOOT_TIME 10
#define FIFO_WATERMARK 32

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static lsm6dsv_filt_settling_mask_t filt_settling_mask;


static uint8_t whoamI;
// static uint8_t tx_buffer[1000];
static lsm6dsv_fifo_sflp_raw_t fifo_sflp;

static HAL_StatusTypeDef spi_status;

static uint16_t uart_cmd[3] = {0};
static uint8_t uart_TX[UART_TX_SIZE] = {0};
static uint8_t uart_RX[UART_RX_SIZE] = {0};
static uint8_t uart_received_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, 0);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1); // CS high also means IMU deselected from SPI and I2c activated

    HAL_MultiProcessor_Init(&huart2, UART_ADDR, UART_WAKEUPMETHOD_ADDRESSMARK);
    HAL_MultiProcessorEx_AddressLength_Set(&huart2, UART_ADDRESS_DETECT_4B);

    HAL_MultiProcessor_EnableMuteMode(&huart2);
    HAL_MultiProcessor_EnterMuteMode(&huart2);

    HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_RX, UART_RX_SIZE);

    lsm6dsv_all_sources_t all_sources;
    lsm6dsv_reset_t rst;
    stmdev_ctx_t dev_ctx;
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &SENSOR_BUS;
    /* Init test platform */
    platform_init();
    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);
    /* Check device ID */

    lsm6dsv_device_id_get(&dev_ctx, &whoamI);
    while (whoamI != LSM6DSV_ID) {
        lsm6dsv_device_id_get(&dev_ctx, &whoamI);
        HAL_Delay(10);
    }

    /* Restore default configuration */
    lsm6dsv_reset_set(&dev_ctx, LSM6DSV_RESTORE_CTRL_REGS);
    do {
        lsm6dsv_reset_get(&dev_ctx, &rst);
    } while (rst != LSM6DSV_READY);

    /* Enable Block Data Update */
    lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate.
     * Selected data rate have to be equal or greater with respect
     * with MLC data rate.
     */
//    lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_7Hz5);
    lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_120Hz);
//    lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_15Hz);
    lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_120Hz);
    /* Set full scale */
    lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_2g);
    lsm6dsv_gy_full_scale_set(&dev_ctx, LSM6DSV_2000dps);
    /* Configure filtering chain */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    lsm6dsv_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV_GY_STRONG);
//    lsm6dsv_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV_GY_ULTRA_LIGHT);
    lsm6dsv_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV_XL_STRONG);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* Read output only if new xl value is available */
        lsm6dsv_all_sources_get(&dev_ctx, &all_sources);

        if (all_sources.drdy_xl) {
            /* Read acceleration field data */
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, data_raw_acceleration[0]);
        }

        if (all_sources.drdy_gy) {
            /* Read angular rate field data */
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            lsm6dsv_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, data_raw_angular_rate[0]);
        }

        uart_TX[0] = (uint8_t)(data_raw_acceleration[0] >> (7)) & 0b01111111;
		 uart_TX[1] = (uint8_t)(data_raw_acceleration[0] >> (0)) & 0b01111111;
		 uart_TX[2] = (uint8_t)(data_raw_acceleration[1] >> (7)) & 0b01111111;
		 uart_TX[3] = (uint8_t)(data_raw_acceleration[1] >> (0)) & 0b01111111;
		 uart_TX[4] = (uint8_t)(data_raw_acceleration[2] >> (7)) & 0b01111111;
		 uart_TX[5] = (uint8_t)(data_raw_acceleration[2] >> (0)) & 0b01111111;
		 uart_TX[6] = (uint8_t)(data_raw_angular_rate[0] >> (7)) & 0b01111111;
		 uart_TX[7] = (uint8_t)(data_raw_angular_rate[0] >> (0)) & 0b01111111;
		 uart_TX[8] = (uint8_t)(data_raw_angular_rate[1] >> (7)) & 0b01111111;
		 uart_TX[9] = (uint8_t)(data_raw_angular_rate[1] >> (0)) & 0b01111111;
		 uart_TX[10] = (uint8_t)(data_raw_angular_rate[2] >> (7)) & 0b01111111;
		 uart_TX[11] = (uint8_t)(data_raw_angular_rate[2] >> (0)) & 0b01111111;





    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 1000000;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel2_3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : UART_DE_Pin */
    GPIO_InitStruct.Pin = UART_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(UART_DE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : INT1_Pin INT2_Pin */
    GPIO_InitStruct.Pin = INT1_Pin | INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : SPI_CS_Pin */
    GPIO_InitStruct.Pin = SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    // I2C version
    // HAL_I2C_Mem_Write(handle, LSM6DSV_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);

    // SPI version
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);

    spi_status = HAL_SPI_Transmit(handle, &reg, 1, 1000);

    spi_status = HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 1000);

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    // I2C version
    // HAL_I2C_Mem_Read(handle, LSM6DSV_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

    // SPI version
    reg |= 0x80;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    spi_status = HAL_SPI_Transmit(handle, &reg, 1, 1000);

    spi_status = HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len) {
    // TODO
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) {
    HAL_Delay(ms);
}


static void platform_init() {
    HAL_Delay(100);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    uart_received_flag = 1;

    if (uart_received_flag) { //was in main loop before
		uart_received_flag = 0;

		uart_cmd[0] = uart_RX[0] & CMD_MASK;
		uart_cmd[1] = (uart_RX[1] << 7) | (uart_RX[2]);
		uart_cmd[1] = pad14(uart_cmd[1]);

		if (uart_cmd[0] == CMD_GET_POSITION) {

			HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, 1); // enable TX
			HAL_UART_Transmit_DMA(&huart2, uart_TX, 12);
		}else{
			HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, 0); // set RX
			HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_RX, UART_RX_SIZE);
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, 0); // set RX
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_RX, UART_RX_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { // shouldn't happen but just in case of overrun, restart RX
    // clear the uart buffer
    uint8_t temp_buffer[20];
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        HAL_UART_Receive(&huart2, temp_buffer, 1, 1);
    }

    HAL_GPIO_WritePin(UART_DE_GPIO_Port, UART_DE_Pin, 0); // set RX
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart_RX, UART_RX_SIZE);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
