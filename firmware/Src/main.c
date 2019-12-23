
/*
 * Otter-Iron  -  Stm32f072 based soldering iron.
 * Copyright (C) 2019 Jan Henrik Hemsing
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"
#include "font.h"

#define FILT(a, b, c) ((a) * (c) + (b) * ((1.0f) - (c)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#define TTIP_AVG_FILTER 0.8f
#define MIN_DUTY 0
#define MAX_DUTY 4050

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

PCD_HandleTypeDef hpcd_USB_FS;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USB_PCD_Init(void);

void reg(void);
void disp_init(void);
void refresh(void);
void clear_screen(void);
void set_screen(void);
void write_pixel(int16_t x, int16_t y, uint8_t color);
void draw_char(unsigned char  c, uint8_t x, uint8_t y, uint8_t brightness);
void draw_string(const unsigned char * str, uint8_t x, uint8_t y, uint8_t brightness);


struct status_t{
  float ttip;
  float ttipavg;
  float uin;
  float iin;
  float tref;
  uint8_t button[2];
}s;

struct reg_t{
  float target;
  float error;
  float errorprior;
  float ierror;
  float imax;
  float derror;
  int16_t duty;
  float cycletime;
  float Kp;
  float Ki;
  float Kd;
}r = {.Kp = 40.0f,.Ki = 6.0f,.Kd = 12.0f,.cycletime = 0.1f,.imax=200.0f,.target=220.0f};

struct tipcal_t{
  float offset;
  float coefficient;
} tipcal = {.offset = 120, .coefficient = 92};

static uint16_t ADC_raw[4];

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USB_PCD_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

  HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADC_raw, 4);

  HAL_Delay(20);
  disp_init();
  HAL_Delay(60);
  clear_screen();
  draw_string("Otter-Iron", 15, 1 ,1);
  draw_string("by Jan Henrik", 10, 9 ,1);
  refresh();
  HAL_Delay(1000);

  while (1)
  {
    HAL_Delay(50);

    //UI
    s.button[0] = HAL_GPIO_ReadPin(GPIOA,B1_Pin);
    s.button[1] = HAL_GPIO_ReadPin(GPIOA,B2_Pin);

    if(s.button[0] == 1){
      r.target -= 5;
      HAL_Delay(50);
    }

    if(s.button[1] == 1){
      r.target += 5;
      HAL_Delay(50);
    }

    r.target = CLAMP(r.target, 20, 400);

    //super shitty display code
    char str1[10] = "          ";
    char str2[10] = "          ";
    char str3[10] = "          ";
    sprintf(str1, "%d C", (uint16_t)r.target);
    sprintf(str2, "%d.%d C", (uint16_t)s.ttipavg,(uint16_t)((s.ttipavg-(uint16_t)s.ttipavg)*10.0f));
    sprintf(str3, "%d.%d V", (uint16_t)s.uin,(uint16_t)((s.uin-(uint16_t)s.uin)*10.0f));

    clear_screen();
    draw_string(str1, 10, 1 ,1);
    draw_string(str2, 10, 9 ,1);
    draw_string(str3, 60, 1 ,1);
    if(r.error > 3){
      draw_string("*", 60, 9 ,1);
    } else {
      draw_string(" ", 60, 9 ,1);
    }
    refresh();
  }
}

// Main PID controller and ADC readout
void reg(void) {

  s.tref = ((((float)ADC_raw[3]/4095.0)*3.3)-0.5)/0.01;
  s.ttip = ((ADC_raw[1]-tipcal.offset)*tipcal.coefficient)/1000+s.tref;
  s.uin = ((ADC_raw[2]/4095.0)*3.3)*6.6;
  s.iin = ((ADC_raw[0]/4095.0)*3.3);

  s.ttipavg = FILT(s.ttipavg, s.ttip, TTIP_AVG_FILTER);

  r.error = r.target - s.ttipavg;
  r.ierror = r.ierror + (r.error*r.cycletime);
  r.ierror = CLAMP(r.ierror,-r.imax,r.imax);
  r.derror = (r.error - r.errorprior)/r.cycletime;
  r.duty = r.Kp*r.error + r.Ki*r.ierror + r.Kd*r.derror;
  r.errorprior = r.error;

  r.duty = CLAMP(r.duty, MIN_DUTY, MAX_DUTY); // Clamp to duty cycle

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, r.duty);
}

// init code sequence by Ralim, thanks alot!
#define DEVICEADDR_OLED   (0x3c<<1)
#define OLED_WIDTH        96
#define FRAMEBUFFER_START 17
uint8_t screenBuffer[16 + (OLED_WIDTH * 2) + 10];  // The data buffer
const uint8_t REFRESH_COMMANDS[17] = { 0x80, 0xAF, 0x80, 0x21, 0x80, 0x20, 0x80, 0x7F, 0x80, 0xC0, 0x80, 0x22, 0x80, 0x00, 0x80, 0x01, 0x40 };
uint8_t OLED_Setup_Array[] = {
0x80, 0xAE, /*Display off*/
0x80, 0xD5, /*Set display clock divide ratio / osc freq*/
0x80, 0x52, /*Divide ratios*/
0x80, 0xA8, /*Set Multiplex Ratio*/
0x80, 0x0F, /*16 == max brightness,39==dimmest*/
0x80, 0xC0, /*Set COM Scan direction*/
0x80, 0xD3, /*Set vertical Display offset*/
0x80, 0x00, /*0 Offset*/
0x80, 0x40, /*Set Display start line to 0*/
0x80, 0xA0, /*Set Segment remap to normal*/
0x80, 0x8D, /*Charge Pump*/
0x80, 0x14, /*Charge Pump settings*/
0x80, 0xDA, /*Set VCOM Pins hardware config*/
0x80, 0x02, /*Combination 2*/
0x80, 0x81, /*Contrast*/
0x80, 0x33, /*^51*/
0x80, 0xD9, /*Set pre-charge period*/
0x80, 0xF1, /*Pre charge period*/
0x80, 0xDB, /*Adjust VCOMH regulator ouput*/
0x80, 0x30, /*VCOM level*/
0x80, 0xA4, /*Enable the display GDDR*/
0x80, 0XA6, /*Normal display*/
0x80, 0x20, /*Memory Mode*/
0x80, 0x00, /*Wrap memory*/
0x80, 0xAF /*Display on*/
};
//not Ralim anymore
void disp_init(void) {
  memcpy(&screenBuffer[0], &REFRESH_COMMANDS[0], sizeof(REFRESH_COMMANDS));
  uint16_t _cnt = 0;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) if (_cnt++ > 10000) break;
  HAL_I2C_Master_Transmit(&hi2c1,DEVICEADDR_OLED, &OLED_Setup_Array[0],sizeof(OLED_Setup_Array),1000);
}

void refresh(void) {
  uint16_t _cnt = 0;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) if (_cnt++ > 10000) break;
  HAL_I2C_Master_Transmit(&hi2c1,DEVICEADDR_OLED, screenBuffer,FRAMEBUFFER_START + (OLED_WIDTH * 2),1000);
}

void clear_screen(void) {
  memset(&screenBuffer[FRAMEBUFFER_START], 0, OLED_WIDTH * 2);
}
void set_screen(void) {
  memset(&screenBuffer[FRAMEBUFFER_START], 255, OLED_WIDTH * 2);
}

void write_pixel(int16_t x, int16_t y, uint8_t color){
  if(color == 1){
    screenBuffer[FRAMEBUFFER_START + (x + ((y/8)*96))] |=  (1 << y % 8);
  } else if (color == 0){
    screenBuffer[FRAMEBUFFER_START + (x + ((y/8)*96))] &= ~(1 << y % 8);
  }
}

#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8
void draw_char(unsigned char c, uint8_t x, uint8_t y, uint8_t brightness) {
    c = c & 0x7F;
    if (c < ' ') {
        c = 0;
    } else {
        c -= ' ';
    }
    uint8_t * chr = font[c];
    for (uint8_t j=0; j<CHAR_WIDTH; j++) {
        for (uint8_t i=0; i<CHAR_HEIGHT; i++) {
            if (chr[j] & (1<<i)) {
                write_pixel(x+j, y+i, 1);
            } else {
                write_pixel(x+j, y+i, 0);
            }
        }
    }
}

void draw_string(const unsigned char* str, uint8_t x, uint8_t y, uint8_t brightness) {
    while (*str) {
        draw_char(*str++, x, y, brightness);
        x += CHAR_WIDTH;
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

static void MX_ADC_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC4;//ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  sConfig.Channel = ADC_CHANNEL_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  HAL_ADC_MspInit(&hadc);
}

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);

  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

static void MX_TIM1_Init(void)
{

  __HAL_RCC_TIM1_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2048;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4096;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_OC_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 4090;
  HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  HAL_TIM_MspPostInit(&htim1);
}
/*
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2096;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4096;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim2);

}
*/
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  HAL_PCD_Init(&hpcd_USB_FS);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /*
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
  */

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = INT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_N_GPIO_Port, &GPIO_InitStruct);

}

void Error_Handler(void)
{

}
