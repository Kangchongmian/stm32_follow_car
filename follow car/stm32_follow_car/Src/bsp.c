/*
 * bsp.c - 板级支持包: 时钟, GPIO, 定时器, 串口, I2C初始化
 * MCU: STM32F407VGT6
 * 基于接线表 v3
 */
#include "bsp.h"

TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
I2C_HandleTypeDef  hi2c1;

/* ========== 系统时钟: HSE 8MHz -> 168MHz ========== */
void BSP_ClockConfig(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;   /* SYSCLK = 168MHz */
    osc.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* HCLK  = 168MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;      /* APB1  = 42MHz, Timer = 84MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;      /* APB2  = 84MHz, Timer = 168MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ========== GPIO初始化 ========== */
void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* ----- LED输出: PD4(蓝), PD7(红), PE0(绿), PE1(黄) ----- */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = LED_BLUE_PIN | LED_RED_PIN | TURN_LEFT_PIN | TURN_RIGHT_PIN;
    HAL_GPIO_Init(GPIOD, &g);

    g.Pin = LED_GREEN_PIN | LED_YELLOW_PIN;
    HAL_GPIO_Init(GPIOE, &g);

    LED_AllOff();

    /* ----- 输入: PC13(急停), PE3(模式), PE4(上电反馈) ----- */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;

    g.Pin = ESTOP_PIN;
    HAL_GPIO_Init(ESTOP_PORT, &g);

    g.Pin = MODE_SW_PIN | POWER_FB_PIN;
    HAL_GPIO_Init(GPIOE, &g);
}

/* ========== TIM1 PWM - 后轮电机 ========== */
/*
 * TIM1 在 APB2, Timer Clock = 168MHz
 * PWM频率 = 168MHz / (PSC+1) / (ARR+1) = 168MHz / 8 / 1000 = 21kHz
 * 21kHz 超出人耳范围, 消除电机嗡嗡声
 * 占空比范围: 0 - 999
 *
 * CH1 = PE9  -> 左后轮 R_PWM (反转)
 * CH2 = PE11 -> 左后轮 L_PWM (正转)
 * CH3 = PE13 -> 右后轮 R_PWM (反转)
 * CH4 = PE14 -> 右后轮 L_PWM (正转)
 */
void BSP_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    g.Pin       = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &g);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 7;         /* 168MHz / 8 = 21MHz */
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;       /* 21MHz / 1000 = 21kHz PWM */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/* ========== USART1 - JDY-31蓝牙 (PB7 TX, PB6 RX) ========== */
/*
 * JDY-31 默认波特率 9600bps
 * USART1 在 APB2 总线
 * PB7 = TX (AF7)
 * PB6 = RX (AF7)
 */
void BSP_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    g.Pin       = GPIO_PIN_7 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/* ========== USART2 - UWB主站 (PD5 TX, PD6 RX) ========== */
void BSP_USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    g.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &g);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/* ========== USART3 - RS485超声波 (PD8 TX, PD9 RX) ========== */
void BSP_USART3_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOD, &g);

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/* ========== UART4 - LD06雷达 (PA0 TX, PA1 RX) ========== */
void BSP_UART4_Init(void)
{
    __HAL_RCC_UART4_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF8_UART4;
    g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &g);

    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = 230400;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = UART_STOPBITS_1;
    huart4.Init.Parity       = UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart4);

    HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
}

/* ========== I2C1 - OLED (PB8 SCL, PB9 SDA) ========== */
void BSP_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* ========== LED控制 (低电平点亮) ========== */
void LED_Blue(bool on)   { HAL_GPIO_WritePin(LED_BLUE_PORT,   LED_BLUE_PIN,   on ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void LED_Red(bool on)    { HAL_GPIO_WritePin(LED_RED_PORT,    LED_RED_PIN,    on ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void LED_Green(bool on)  { HAL_GPIO_WritePin(LED_GREEN_PORT,  LED_GREEN_PIN,  on ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void LED_Yellow(bool on) { HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void LED_AllOff(void)    { LED_Blue(false); LED_Red(false); LED_Green(false); LED_Yellow(false); }

/* ========== 输入读取 ========== */
/* 急停按钮: 信号断开 = 紧急停止 (常闭触点, 低电平=正常, 高电平/断开=急停) */
bool IsEstopPressed(void) { return HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_SET; }
/* 模式开关: 接通=跟随模式 */
bool IsFollowMode(void)   { return HAL_GPIO_ReadPin(MODE_SW_PORT, MODE_SW_PIN) == GPIO_PIN_RESET; }
/* 上电反馈: 接通=电源正常 */
bool IsPowerOn(void)       { return HAL_GPIO_ReadPin(POWER_FB_PORT, POWER_FB_PIN) == GPIO_PIN_RESET; }

/* ========== 转向灯控制 ========== */
static uint8_t  turn_dir      = TURN_NONE;  /* 当前方向 */
static uint32_t turn_last_tick = 0;
static bool     turn_on       = false;
#define TURN_BLINK_MS  200  /* 闪烁间隔 200ms ≈ 2.5Hz */

void TurnSignal_Set(uint8_t dir)
{
    if (dir == turn_dir) return;
    turn_dir = dir;
    turn_on  = false;
    /* 切换方向时先灭掉两个灯 (高电平灭) */
    HAL_GPIO_WritePin(TURN_LEFT_PORT,  TURN_LEFT_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(TURN_RIGHT_PORT, TURN_RIGHT_PIN, GPIO_PIN_SET);
}

void TurnSignal_Tick(void)
{
    if (turn_dir == TURN_NONE) {
        /* 直行, 两灯灭 (高电平灭) */
        HAL_GPIO_WritePin(TURN_LEFT_PORT,  TURN_LEFT_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(TURN_RIGHT_PORT, TURN_RIGHT_PIN, GPIO_PIN_SET);
        return;
    }

    uint32_t now = HAL_GetTick();
    if (now - turn_last_tick < TURN_BLINK_MS) return;
    turn_last_tick = now;
    turn_on = !turn_on;

    if (turn_dir == TURN_LEFT) {
        HAL_GPIO_WritePin(TURN_LEFT_PORT,  TURN_LEFT_PIN,  turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(TURN_RIGHT_PORT, TURN_RIGHT_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(TURN_RIGHT_PORT, TURN_RIGHT_PIN, turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(TURN_LEFT_PORT,  TURN_LEFT_PIN,  GPIO_PIN_SET);
    }
}
