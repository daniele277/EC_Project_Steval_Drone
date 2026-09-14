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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsr.h"
#include "lps22hh.h"
#include <math.h>
#include "sensor_fusion.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TIMEOUT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Variabili globali per salvare le letture dei sensori
LSM6DSR_AxesRaw_t accel_data;
LSM6DSR_AxesRaw_t gyro_data;
float pressure_data;
uint8_t who_am_i_response_imu = 0;
uint8_t who_am_i_response_bar = 0;
uint8_t status_reg;
uint8_t tx_buffer_dma[13];
uint8_t rx_buffer_dma[13];
uint8_t rawPressure_DMA[3];
float target_pitch = 0.0f;
float target_roll = 0.0f;
float target_yaw = 0.0f;
float throttle = 0.0f;

float m1 = 0.0f;
float m2 = 0.0f;
float m3 = 0.0f;
float m4 = 0.0f;

// Flag dati pront
volatile uint8_t data_ready = 0;

// Variabili globali per calibrare i sensori
int32_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
int32_t acc_offset_x = 0,  acc_offset_y = 0,  acc_offset_z = 0;
float pressure_zero = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM10) {
		// Il primo byte da trasmettere è l'indirizzo del registro di partenza
		// I restanti 12 sono byte dummy inviati dal master per generare il segnale di clock
		// Ricordiamo infatti che SPI è un protocollo full-duplex!
	    tx_buffer_dma[0] = LSM6DSR_OUTX_L_G | 0x80;
	    HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, GPIO_PIN_RESET);
	    // Il DMA gestisce l'intera comunicazione master-slave in modo asincrono rispeto al processore
	    HAL_SPI_TransmitReceive_DMA(&hspi2, tx_buffer_dma, rx_buffer_dma, 13);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SPI2) {
		HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, GPIO_PIN_SET);

		// Scartiamo il primo byte perché si riferisce all'invio dell'indirizzo

		gyro_data.x = (int16_t)((rx_buffer_dma[2] << 8) | rx_buffer_dma[1]) - gyro_offset_x;
		gyro_data.y = (int16_t)((rx_buffer_dma[4] << 8) | rx_buffer_dma[3]) - gyro_offset_y;
		gyro_data.z = (int16_t)((rx_buffer_dma[6] << 8) | rx_buffer_dma[5]) - gyro_offset_z;

		accel_data.x = (int16_t)((rx_buffer_dma[8] << 8) | rx_buffer_dma[7]) - acc_offset_x;
		accel_data.y = (int16_t)((rx_buffer_dma[10] << 8) | rx_buffer_dma[9]) - acc_offset_y;
		accel_data.z = (int16_t)((rx_buffer_dma[12] << 8)| rx_buffer_dma[11]) - acc_offset_z;

        data_ready = 1;
	}
}

HAL_StatusTypeDef readSPI(GPIO_TypeDef* CS_Port, uint16_t CS_Pin, uint8_t reg_addr,  uint8_t *pData, uint16_t size) {

    HAL_StatusTypeDef status;

    // Aggiunge bit di lettura (MSB a 1)
    uint8_t tx_addr = reg_addr | 0x80;

    // Seleziona l'IMU (CS Low)
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

    // Invia l'indirizzo del registro da cui voglio leggere
    status = HAL_SPI_Transmit(&hspi2, &tx_addr, 1, SPI_TIMEOUT);

    if (status == HAL_OK) {
        // Riceve i dati dal sensore
        status = HAL_SPI_Receive(&hspi2, pData, size, SPI_TIMEOUT);
    }

    // Deseleziona il sensore usando i pin specifici dell'LSM6DSR
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef writeSPI(GPIO_TypeDef* CS_Port, uint16_t CS_Pin, uint8_t reg_addr, uint8_t value) {
    HAL_StatusTypeDef status;

    // Aggiunge bit di scrittura (MSB a 0)
    uint8_t tx_addr = reg_addr & 0x7F;

    // Seleziona l'IMU (CS Low)
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

    // Invia l'indirizzo del registro
    status = HAL_SPI_Transmit(&hspi2, &tx_addr, 1, SPI_TIMEOUT);

    if (status == HAL_OK) {
        // Invia il valore da scrivere nel registro
        status = HAL_SPI_Transmit(&hspi2, &value, 1, SPI_TIMEOUT);
    }

    // Deseleziona il sensore (CS High)
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    return status;
}

void Calibrate_Sensors(void) {

    // Variabili per sommare i campioni
    int32_t sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    int32_t sum_acc_x = 0,  sum_acc_y = 0,  sum_acc_z = 0;
    float sum_pressure = 0.0f;

    uint8_t rawIMU[6];
    uint8_t rawPressure[3];
    uint16_t num_samples = 500;

    // Ciclo di campionamento: acquisisce 500 campioni
    for (uint16_t i = 0; i < num_samples; i++) {

    	// Polling bloccante sullo STATUS_REG (indirizzo 0x1E) dell'IMU LSM6DSR
    	// Attende che sia l'accelerometro (XLDA, bit 0) che il giroscopio (GDA, bit 1) abbiano nuovi dati
        do {
        	readSPI(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, LSM6DSR_STATUS_REG, &status_reg, 1);
        } while ((status_reg & 0x03) != 0x03);

        // Lettura Accelerometro
        if (readSPI(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, LSM6DSR_OUTX_L_A, rawIMU, 6) == HAL_OK) {
            sum_acc_x += (int16_t)((rawIMU[1] << 8) | rawIMU[0]);
            sum_acc_y += (int16_t)((rawIMU[3] << 8) | rawIMU[2]);
            sum_acc_z += (int16_t)((rawIMU[5] << 8) | rawIMU[4]);
        }

        // Lettura Giroscopio
        if (readSPI(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, LSM6DSR_OUTX_L_G, rawIMU, 6) == HAL_OK) {
            sum_gyro_x += (int16_t)((rawIMU[1] << 8) | rawIMU[0]);
            sum_gyro_y += (int16_t)((rawIMU[3] << 8) | rawIMU[2]);
            sum_gyro_z += (int16_t)((rawIMU[5] << 8) | rawIMU[4]);
        }

        // Lettura Barometro
        // N.B: LPS22HH è impostato a 100 Hz, un polling specifico sul suo STATUS_REG sarebbe ottimale,
        // ma essendo la frequenza quasi coincidente all'IMU, la lettura consecutiva è accettabile in questa fase.
        if (readSPI(LPS22HH_CS_GPIO_Port, LPS22HH_CS_Pin, LPS22HH_PRESS_OUT_XL, rawPressure, 3) == HAL_OK) {
            int32_t press_raw = (int32_t)((rawPressure[2] << 16) | (rawPressure[1] << 8) | rawPressure[0]);
            // Conversione in hPa
            sum_pressure += (float)press_raw / 4096.0f;
        }
    }

    // -- CALCOLO DELLE MEDIE (OFFSET) --

    // Giroscopio
    gyro_offset_x = sum_gyro_x / num_samples;
    gyro_offset_y = sum_gyro_y / num_samples;
    gyro_offset_z = sum_gyro_z / num_samples;

    // Accelerometro X e Y
    acc_offset_x = sum_acc_x / num_samples;
    acc_offset_y = sum_acc_y / num_samples;

    // Accelerometro Z
    // Sensibilità nominale per FS +/-4g = 0.122 mg/LSB. 1 g = 1000 / 0.122 = 8196.7 LSB
    int32_t avg_acc_z = sum_acc_z / num_samples;
    acc_offset_z = avg_acc_z - 8197;

    // Barometro
    pressure_zero = sum_pressure / num_samples;
}

void Test_Motors(void)
{
    // Parametri operativi in termini di valori di registro CCR
    const uint32_t PWM_OFF = 0;       // Duty Cycle 0%
    const uint32_t PWM_TEST = 1400;    // Duty Cycle 25% (500/(1+55999)) per limitare la coppia a vuoto
    const uint32_t TIME_TEST_MS = 1000; // Durata dell'azionamento (1 secondo)
    const uint32_t TIME_PAUSE_MS = 500; // Pausa tra i motori

    // Avvio della generazione del PWM sui 4 canali del Timer 4
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // Condizione di sicurezza iniziale: tutti i duty cycle a 0%
    // La macro __HAL_TIM_SET_COMPARE modifica il valore del CCR a runtime
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_OFF);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_OFF);
    // Si forza l'aggiornamento del duty cycle senza aspettare il reset successivo del timer
    htim4.Instance->EGR = TIM_EGR_UG;

    // Canali del timer
    uint32_t tim_channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

    // Attivazione sequenziale
    for(uint8_t i = 0; i < 4; i++) {
        // Applica tensione al motore i-esimo
        __HAL_TIM_SET_COMPARE(&htim4, tim_channels[i], PWM_TEST);
        htim4.Instance->EGR = TIM_EGR_UG;
        HAL_Delay(TIME_TEST_MS);
        // Disabilita tensione
        __HAL_TIM_SET_COMPARE(&htim4, tim_channels[i], PWM_OFF);
        htim4.Instance->EGR = TIM_EGR_UG;
        HAL_Delay(TIME_PAUSE_MS);
    }
}

void Motor_Mixer(float throttle, float pid_pitch, float pid_roll, float pid_yaw) {
    // Matrice di motor mixing
    m1 = throttle + pid_pitch - pid_roll + pid_yaw; // posteriore dx
    m2 = throttle - pid_pitch - pid_roll - pid_yaw; // anteriore dx
    m3 = throttle - pid_pitch + pid_roll + pid_yaw; // anteriore sx
    m4 = throttle + pid_pitch + pid_roll - pid_yaw; // posteriore sx

    // Definizione dei limiti PWM
    // Questo è da vedere, perché dipende dall'ARR
    float pwm_min = 2800.0f;  // Minimo per evitare lo spegnimento in volo (5% del valore massimo)
    float pwm_max = 55999.0f;  // Duty cycle massimo al 100%

    // Sicurezza: motori spenti se il gas è a zero
    if (throttle < 10.0f) {
        m1 = 0; m2 = 0; m3 = 0; m4 = 0;
    } else {
        // Saturazione sicura dei valori
        if (m1 > pwm_max) m1 = pwm_max; else if (m1 < pwm_min) m1 = pwm_min;
        if (m2 > pwm_max) m2 = pwm_max; else if (m2 < pwm_min) m2 = pwm_min;
        if (m3 > pwm_max) m3 = pwm_max; else if (m3 < pwm_min) m3 = pwm_min;
        if (m4 > pwm_max) m4 = pwm_max; else if (m4 < pwm_min) m4 = pwm_min;
    }

    // Scrittura nei registri del timer utilizzando la macro HAL
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)m1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)m2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)m3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint32_t)m4);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  // Disabilita inizialmente i sensori assicurandoti che i pin CS siano HIGH
  HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LPS22HH_CS_GPIO_Port, LPS22HH_CS_Pin, GPIO_PIN_SET);

  // Test di comunicazione: Lettura del WHO_AM_I
  readSPI(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, LSM6DSR_WHO_AM_I, &who_am_i_response_imu, 1);
  readSPI(LPS22HH_CS_GPIO_Port, LPS22HH_CS_Pin, LPS22HH_WHO_AM_I, &who_am_i_response_bar, 1);

  // -- CONFIGURAZIONE ACCELEROMETRO SCRIVENDO IL REGISTRO CTRL1_XL --
  // Valore 0x68 (0b0110 1000)
  // 0110 = ODR a 416 Hz
  // 10 = Full Scale a +-4g
  // 00 = Filtri di default
  writeSPI(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, LSM6DSR_CTRL1_XL, 0x68);

  // -- CONFIGURAZIONE GIROSCOPIO SCRIVENDO IL REGISTRO CTRL2_G --
  // Valore 0x6C in binario (0b0110 1100)
  // 0110 = ODR a 416 Hz
  // 11 = Full Scale a 2000 dps
  // 00 = Default
  writeSPI(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, LSM6DSR_CTRL2_G, 0x6C);

  // -- CONFIGURAZIONE BAROMETRO SCRIVENDO IL REGISTRO CTRL_REG1
  // Valore 0x06 in binario (0b0000 0110)
  // 0110 = ODR a 100 Hz
  writeSPI(LPS22HH_CS_GPIO_Port, LPS22HH_CS_Pin, LPS22HH_CTRL_REG1, 0x06);

  Calibrate_Sensors();

  //Test_Motors();

  // Avvio della generazione del PWM sui 4 canali del Timer 4
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // Condizione di sicurezza iniziale: tutti i duty cycle a 0%
  // La macro __HAL_TIM_SET_COMPARE modifica il valore del CCR a runtime
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  // Si forza l'aggiornamento del duty cycle senza aspettare il reset successivo del timer
  htim4.Instance->EGR = TIM_EGR_UG;

  // Avvio timer in modalità interrupt
  HAL_TIM_Base_Start_IT(&htim10);

  // Inizializzazione dei PID
  PID pid_pitch, pid_roll, pid_yaw;
  PID_Init(&pid_pitch, 100.0f, 10.0f, 0.0f, 400.0f, -400.0f);
  PID_Init(&pid_roll,  100.0f, 10.0f, 0.0f, 400.0f, -400.0f);
  PID_Init(&pid_yaw,   150.0f, 0.0f,  0.0f, 400.0f, -400.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // TBD: La lettura dalla IMU sarà fatta in DMA
  while (1)
  {
	  if(data_ready == 1) {
		  // Stima degli angoli
		  Sensor_Fusion_Update(&accel_data, &gyro_data);

		  // Calcolo azione di controllo
		  float out_pitch = PID_Compute(&pid_pitch, target_pitch, pitch_est, DT);
		  float out_roll  = PID_Compute(&pid_roll, target_roll, roll_est, DT);
          float out_yaw   = PID_Compute(&pid_yaw, target_yaw, yaw_est, DT);

          // Resetta la somma integrale dei PID quando il drone è fermo
          if(throttle < 10) {
        	  PID_Reset(&pid_pitch);
        	  PID_Reset(&pid_roll);
        	  PID_Reset(&pid_yaw);
          }

          // Motor mixer
          Motor_Mixer(throttle, out_pitch, out_roll, out_yaw);

		  data_ready = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32767;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 55999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 51;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8076;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 24;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LPS22HH_CS_GPIO_Port, LPS22HH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLE_RSTN_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LPS22HH_CS_Pin */
  GPIO_InitStruct.Pin = LPS22HH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LPS22HH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_Monitor_Pin */
  GPIO_InitStruct.Pin = USB_Monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_Monitor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_IRQ_Pin */
  GPIO_InitStruct.Pin = BLE_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_RSTN_Pin */
  GPIO_InitStruct.Pin = BLE_RSTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_RSTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSR_CS_Pin */
  GPIO_InitStruct.Pin = LSM6DSR_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LSM6DSR_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
