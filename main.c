/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ======================= SETTINGS (tune these) ======================= */
#define TEMP_LIMIT_X10        500    /* fault above 50.0 C                          */
#define CURRENT_LIMIT_MA      1000   /* fault above 1.0 A                           */
#define CUR_HITS_REQUIRED     3      /* readings in a row (50 ms each) above limit  */
#define CUR_DEADBAND_MA       50     /* readings below this are shown as 0          */

#define VIB_ACTIVE_LEVEL      1      /* 1 = DO goes HIGH on vibration. If the fault
                                        triggers by itself, change this to 0        */
#define VIB_EDGES_REQUIRED    2      /* vibration pulses needed ...                 */
#define VIB_WINDOW_MS         300    /* ... inside this time window                 */
#define MOTOR_SETTLE_MS       1000   /* ignore vibration/current just after start   */

/* ACS712: 185 = 5 A module, 100 = 20 A module, 66 = 30 A module */
#define ACS_SENS_MV_PER_A_5V  185
#define ACS_VCC_MV            3300   /* voltage the ACS712 is powered from          */
#define ACS_DIVIDER_PCT       100    /* 100 = no divider, 67 = 10k/20k divider      */

/* ============================== PINS ================================= */
/* PA0 = DS18B20 (set in ds18b20.h), PB6/PB7 = LCD I2C, PA2 = ACS712 ADC  */
#define VIB_PORT  GPIOA
#define VIB_PIN   GPIO_PIN_1     /* SW-420 DO                */
#define BUZ_PORT  GPIOA
#define BUZ_PIN   GPIO_PIN_8     /* buzzer +                 */
#define BTN_PORT  GPIOA
#define BTN_PIN   GPIO_PIN_9     /* push button to GND       */
#define MOT_PORT  GPIOB
#define MOT_IN1   GPIO_PIN_0     /* L298N IN1                */
#define MOT_IN2   GPIO_PIN_1     /* L298N IN2                */
#define MOT_ENA   GPIO_PIN_10    /* L298N ENA                */
#define LED_PORT  GPIOC
#define LED_PIN   GPIO_PIN_13    /* Blue Pill on-board LED   */

/* ACS712 sensitivity as seen at the ADC pin (mV per amp) */
#define ACS_SENS_MV  ((ACS_SENS_MV_PER_A_5V * ACS_VCC_MV / 5000) * ACS_DIVIDER_PCT / 100)

/* fault bits */
#define F_TEMP  0x01U
#define F_CUR   0x02U
#define F_VIB   0x04U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static uint8_t  faults = 0;
static uint8_t  motor_on = 0;
static uint32_t motor_start_ms = 0;

static uint8_t  lcd_ok = 0;
static uint8_t  lcd_addr = 0x27;
static uint32_t lcd_next_ms = 0;

static int16_t  temp_x10 = 0;
static uint8_t  temp_valid = 0;
static uint8_t  temp_err = 0;
static uint8_t  temp_started = 0;
static uint32_t temp_t0 = 0;

static uint32_t cur_mA = 0;
static uint32_t cur_t0 = 0;
static uint8_t  cur_hits = 0;
static uint32_t adc_zero = 2048;

static uint8_t  vib_init = 0;
static uint8_t  vib_prev = 0;
static uint8_t  vib_edges = 0;
static uint32_t vib_win_start = 0;

static uint8_t  btn_state = 1;
static uint32_t btn_t0 = 0;
static uint8_t  btn_pressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int time_reached(uint32_t now, uint32_t t)
{
    return ((int32_t)(now - t) >= 0);
}

/* ------------------------- temperature ------------------------- */
static void temp_task(uint32_t now)
{
    int16_t t;

    if ((now - temp_t0) < 1000U) return;
    temp_t0 = now;

    if (temp_started)
    {
        if (DS18B20_ReadTemp(&t))
        {
            temp_x10 = t;
            temp_valid = 1;
            temp_err = 0;
            if (t > TEMP_LIMIT_X10) { faults |= F_TEMP; }
        }
        else
        {
            if (temp_err < 3U) { temp_err++; }
            if (temp_err >= 3U) { temp_valid = 0; }
        }
    }
    temp_started = DS18B20_StartConversion();
}

/* --------------------- current (ACS712 on PA2) ------------------ */
static uint32_t read_adc_avg(uint8_t n)
{
    uint32_t sum = 0;
    uint8_t good = 0, i;

    for (i = 0; i < n; i++)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
            good++;
        }
        HAL_ADC_Stop(&hadc1);
    }
    if (good == 0U) return adc_zero;
    return sum / good;
}

static uint32_t counts_to_mA(uint32_t counts)
{
    uint32_t delta = (counts > adc_zero) ? (counts - adc_zero) : (adc_zero - counts);
    uint32_t delta_mv = (delta * 3300U) / 4095U;      /* assumes ADC reference = 3.3 V */
    uint32_t ma = (delta_mv * 1000U) / (uint32_t)ACS_SENS_MV;
    if (ma < CUR_DEADBAND_MA) { ma = 0; }
    return ma;
}

static void current_task(uint32_t now)
{
    if ((now - cur_t0) < 50U) return;
    cur_t0 = now;

    cur_mA = counts_to_mA(read_adc_avg(16));

    if (motor_on && (now - motor_start_ms) > MOTOR_SETTLE_MS)
    {
        if (cur_mA > CURRENT_LIMIT_MA)
        {
            if (cur_hits < 255U) { cur_hits++; }
            if (cur_hits >= CUR_HITS_REQUIRED) { faults |= F_CUR; }
        }
        else
        {
            cur_hits = 0;
        }
    }
    else
    {
        cur_hits = 0;
    }
}

/* ------------------- vibration (SW-420) and button ------------------- */
static void poll_vibration(uint32_t now)
{
    uint8_t v = (HAL_GPIO_ReadPin(VIB_PORT, VIB_PIN) == (VIB_ACTIVE_LEVEL ? GPIO_PIN_SET : GPIO_PIN_RESET)) ? 1U : 0U;

    if (!vib_init) { vib_prev = v; vib_init = 1; return; }

    if (motor_on && (now - motor_start_ms) < MOTOR_SETTLE_MS)
    {
        vib_edges = 0;
        vib_prev = v;
        return;
    }

    if (v && !vib_prev)   /* new vibration pulse */
    {
        if (vib_edges == 0U || (now - vib_win_start) > VIB_WINDOW_MS)
        {
            vib_win_start = now;
            vib_edges = 1;
        }
        else if (vib_edges < 255U)
        {
            vib_edges++;
        }
        if (vib_edges >= VIB_EDGES_REQUIRED) { faults |= F_VIB; }
    }
    vib_prev = v;
}

static void poll_button(uint32_t now)
{
    uint8_t s = (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_SET) ? 1U : 0U;

    if (s != btn_state && (now - btn_t0) >= 50U)
    {
        btn_t0 = now;
        btn_state = s;
        if (s == 0U) { btn_pressed = 1; }   /* pressed (pulled to GND) */
    }
}

/* ----------------------------- motor ----------------------------- */
static void motor_run(void)
{
    HAL_GPIO_WritePin(MOT_PORT, MOT_IN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOT_PORT, MOT_IN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_PORT, MOT_ENA, GPIO_PIN_SET);
    motor_start_ms = HAL_GetTick();
    motor_on = 1;
}

static void motor_stop(void)
{
    HAL_GPIO_WritePin(MOT_PORT, MOT_ENA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_PORT, MOT_IN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_PORT, MOT_IN2, GPIO_PIN_RESET);
    motor_on = 0;
}

/* ------------------------------ LCD ------------------------------ */
static void lcd_refresh_now(void)
{
    lcd_next_ms = HAL_GetTick();
}

/* line 1: "T: 25.3C I:0.35A" (always 16 chars) */
static void build_line1(char *s)
{
    char *p = s;
    uint32_t a;

    *p++ = 'T';
    *p++ = ':';

    if (!temp_valid)
    {
        *p++ = ' '; *p++ = '-'; *p++ = '-'; *p++ = '.'; *p++ = '-';
    }
    else
    {
        int32_t t = temp_x10;
        uint32_t whole, frac;
        char neg = 0;

        if (t < 0) { neg = 1; t = -t; }
        whole = (uint32_t)t / 10U;
        frac  = (uint32_t)t % 10U;

        if (whole >= 100U)
        {
            *p++ = (char)('0' + (whole / 100U) % 10U);
            *p++ = (char)('0' + (whole / 10U) % 10U);
            *p++ = (char)('0' + whole % 10U);
        }
        else if (whole >= 10U)
        {
            *p++ = neg ? '-' : ' ';
            *p++ = (char)('0' + whole / 10U);
            *p++ = (char)('0' + whole % 10U);
        }
        else
        {
            *p++ = ' ';
            *p++ = neg ? '-' : ' ';
            *p++ = (char)('0' + whole);
        }
        *p++ = '.';
        *p++ = (char)('0' + frac);
    }
    *p++ = 'C';
    *p++ = ' ';
    *p++ = 'I';
    *p++ = ':';

    a = cur_mA / 10U;                 /* hundredths of an amp */
    if (a > 999U) { a = 999U; }
    *p++ = (char)('0' + a / 100U);
    *p++ = '.';
    *p++ = (char)('0' + (a % 100U) / 10U);
    *p++ = (char)('0' + a % 10U);
    *p++ = 'A';
    *p = '\0';
}

static void build_line2(char *s, uint32_t now)
{
    const char *txt;

    if (faults)
    {
        if ((now / 2000U) & 1U)       txt = "PRESS BTN: RESET";
        else if (faults & F_TEMP)     txt = "FAULT: OVER TEMP";
        else if (faults & F_CUR)      txt = "FAULT: OVERCURR";
        else                          txt = "FAULT: VIBRATION";
    }
    else if (!temp_valid)
    {
        txt = "TEMP SENSOR ERR";
    }
    else
    {
        txt = "STATUS: OK";
    }

    while (*txt) { *s++ = *txt++; }
    *s = '\0';
}

static void lcd_task(uint32_t now)
{
    char l1[20];
    char l2[20];

    if (!lcd_ok) return;
    if (!time_reached(now, lcd_next_ms)) return;

    if (LCD_Failed())
    {
        LCD_Init(&hi2c1, lcd_addr);   /* try to recover after an I2C error */
    }

    build_line1(l1);
    build_line2(l2, now);
    LCD_PrintLine(0, l1);
    LCD_PrintLine(1, l2);

    lcd_next_ms = HAL_GetTick() + (LCD_Failed() ? 2000U : 500U);
}

/* ------------------------ pins used by the app ------------------------ */
static void app_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* outputs start LOW (motor off, buzzer off) */
    HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOT_PORT, MOT_IN1 | MOT_IN2 | MOT_ENA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    /* vibration sensor input */
    g.Pin = VIB_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VIB_PORT, &g);

    /* push button input with pull-up */
    g.Pin = BTN_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_PORT, &g);

    /* buzzer output */
    g.Pin = BUZ_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZ_PORT, &g);

    /* motor driver outputs */
    g.Pin = MOT_IN1 | MOT_IN2 | MOT_ENA;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOT_PORT, &g);

    /* on-board LED */
    g.Pin = LED_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
}

/* ---------------------------- start / loop ---------------------------- */
static void app_init(void)
{
    ADC_ChannelConfTypeDef c = {0};

    app_gpio_init();
    DS18B20_Init();

    /* ADC: calibrate, then channel 2 (PA2) with a long sample time */
    HAL_ADCEx_Calibration_Start(&hadc1);
    c.Channel = ADC_CHANNEL_2;
    c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &c);

    /* find the LCD (0x27 or 0x3F) */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x27U << 1), 2, 50) == HAL_OK)
    {
        lcd_addr = 0x27; lcd_ok = 1;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x3FU << 1), 2, 50) == HAL_OK)
    {
        lcd_addr = 0x3F; lcd_ok = 1;
    }

    if (lcd_ok)
    {
        LCD_Init(&hi2c1, lcd_addr);
        LCD_PrintLine(0, "FAULT DETECTOR");
        LCD_PrintLine(1, "STARTING...");
    }

    /* start-up beep (also proves the buzzer works) */
    HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET);
    HAL_Delay(500);

    /* measure the ACS712 "zero current" level while the motor is still off */
    adc_zero = read_adc_avg(64);

    motor_run();
    temp_t0 = HAL_GetTick();
    lcd_next_ms = HAL_GetTick() + 1000U;
}

static void app_loop(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t period;

    poll_vibration(now);
    poll_button(now);
    temp_task(now);
    current_task(now);

    /* a fault stops the motor */
    if (faults != 0U && motor_on)
    {
        motor_stop();
        lcd_refresh_now();
    }

    /* button clears the fault and restarts the motor */
    if (btn_pressed)
    {
        btn_pressed = 0;
        if (faults != 0U)
        {
            faults = 0;
            cur_hits = 0;
            vib_edges = 0;
            motor_run();
            lcd_refresh_now();
        }
    }

    /* buzzer: beeps while a fault is active */
    if (faults != 0U && (((now / 150U) & 1U) == 0U))
    {
        HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET);
    }

    lcd_task(now);

    /* on-board LED: slow blink = running, fast blink = fault */
    period = (faults != 0U) ? 100U : 500U;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, (((now / period) & 1U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  app_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    app_loop();
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

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
