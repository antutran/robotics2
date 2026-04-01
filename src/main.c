#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "oled.h"
#include "mpu6050.h"
#include "motor.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

/* 
 * ===== BATTERY TUNING =====
 */
#define BAT_ADC_SAMPLES          30
#define BAT_FILTER_ALPHA         0.05f
#define BAT_UPDATE_INTERVAL_MS   1000
#define BAT_PERCENT_HYSTERESIS   2
#define BAT_MIN_VOLTAGE          9.6f
#define BAT_MAX_VOLTAGE          12.4f
#define BAT_DIVIDER_RATIO        11.0f

/* --- SYSTEM CONFIG --- */
#define ENCODER_CPR         1320.0f
#define WHEEL_DIAMETER_M    0.065f
#define TURN_TOLERANCE_DEG  2.0f
#define TURN_SLOW_ZONE_DEG  15.0f
#define TURN_PWM_FAST       85
#define TURN_PWM_SLOW       55
#define JETSON_TIMEOUT_MS   2000

/* --- State Machine Enums --- */
typedef enum { SCREEN_MAIN_MENU=0, SCREEN_STATE, SCREEN_MOVE_MENU, SCREEN_SPEED, SCREEN_MPU } ScreenState;
typedef enum { MOVE_AUTO=0, MOVE_LEFT, MOVE_RIGHT, MOVE_STRAIGHT, MOVE_BACK, MOVE_STOP, MOVE_PARKING } MoveMode;

typedef enum {
    PARK_IDLE, PARK_FORWARD_1, PARK_TURN_R, PARK_FORWARD_2, PARK_TURN_L, PARK_DONE
} ParkingState;

/* --- Peripheral Handles --- */
TIM_HandleTypeDef htim1, htim2, htim3;
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
USBD_HandleTypeDef hUsbDeviceFS;

/* --- Global variables --- */
static ScreenState  currentScreen = SCREEN_MAIN_MENU;
static MoveMode     currentMove   = MOVE_STOP;
static ParkingState parkState     = PARK_IDLE;
static int8_t       menuIndex     = 0;
static uint8_t      speedValue    = 55;
static uint8_t      needRedraw    = 1;

/* Encoder & Speed Data */
static int32_t encL = 0, encR = 0;
static float   speedAvg = 0;

/* Turn Controller Data */
static float targetYaw = 0.0f;
static uint8_t isTurning = 0;

/* Communication & Battery Data */
static uint32_t lastJetsonRxTick = 0;
static uint8_t  jetsonConnected  = 0;
static float    filteredBatVoltage = 0.0f;
static uint8_t  displayedBatPercent = 0;
static uint32_t lastBatUpdateTick = 0;

typedef struct { uint8_t last, pressed; uint16_t holdCounter; } Button_t;
static Button_t btnUp, btnDown, btnSelect, btnBack;

/* Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void readButtons(void);
static void updateUI(void);
static void applyMoveMode(void);
static void updateOdometry(float dt);
static uint8_t updateTurnController(void);
static void updateParkingMode(void);
static void updateJetsonStatus(void);
static void updateBattery(void);
static const char* getMoveStateText(MoveMode mode);

#define MOVE_MENU_COUNT 7
static const char *moveNames[] = {"AUTO", "LEFT", "RIGHT", "STRAIGHT", "BACK", "STOP", "PARKING"};

#define LED_OFF()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, 0)
#define LED_RED()    do { LED_OFF(); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); } while(0)
#define LED_YELLOW() do { LED_OFF(); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); } while(0)
#define LED_GREEN()  do { LED_OFF(); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); } while(0)

/* ----------------------------------------------------------------------- */

static void updateBattery(void) {
    uint32_t sum = 0;
    for (int i = 0; i < BAT_ADC_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    float rawV = ((float)sum / BAT_ADC_SAMPLES / 4095.0f) * 3.3f * BAT_DIVIDER_RATIO;
    if (filteredBatVoltage < 1.0f) filteredBatVoltage = rawV; 
    else filteredBatVoltage = (BAT_FILTER_ALPHA * rawV) + ((1.0f - BAT_FILTER_ALPHA) * filteredBatVoltage);

    if (HAL_GetTick() - lastBatUpdateTick > BAT_UPDATE_INTERVAL_MS) {
        lastBatUpdateTick = HAL_GetTick();
        uint8_t newPercent;
        if (filteredBatVoltage <= BAT_MIN_VOLTAGE) newPercent = 0;
        else if (filteredBatVoltage >= BAT_MAX_VOLTAGE) newPercent = 100;
        else newPercent = (uint8_t)((filteredBatVoltage - BAT_MIN_VOLTAGE) / (BAT_MAX_VOLTAGE - BAT_MIN_VOLTAGE) * 100.0f);
        if (abs((int)newPercent - (int)displayedBatPercent) >= BAT_PERCENT_HYSTERESIS) displayedBatPercent = newPercent;
        int iv = (int)filteredBatVoltage; int fv = (int)((filteredBatVoltage - iv) * 100);
        printf("BATTERY, filtered=%d.%02dV, percent=%d%%\r\n", iv, fv, displayedBatPercent);
    }
}

static void updateJetsonStatus(void) { jetsonConnected = (HAL_GetTick() - lastJetsonRxTick < JETSON_TIMEOUT_MS); }
void Jetson_Heartbeat_Callback(void) { lastJetsonRxTick = HAL_GetTick(); }

static const char* getMoveStateText(MoveMode mode) {
    switch (mode) {
        case MOVE_STRAIGHT: return "STRAIGHT"; case MOVE_LEFT: return "LEFT";
        case MOVE_RIGHT: return "RIGHT";       case MOVE_BACK: return "BACK";
        case MOVE_STOP: return "STOP";         case MOVE_PARKING: return "PARKING";
        case MOVE_AUTO: return "AUTO";         default: return "UNKNOWN";
    }
}

static void applyMoveMode(void) {
    if (currentMove == MOVE_PARKING) { parkState = PARK_FORWARD_1; LED_YELLOW(); }
    else {
        parkState = PARK_IDLE; isTurning = 0;
        switch (currentMove) {
            case MOVE_LEFT: Motor_TurnLeft(speedValue); break; case MOVE_RIGHT: Motor_TurnRight(speedValue); break;
            case MOVE_STRAIGHT: Motor_Forward(speedValue); break; case MOVE_BACK: Motor_Backward(speedValue); break;
            case MOVE_STOP: Motor_Stop(); break; default: break;
        }
    }
}

static void startTurnRelative(float delta) { targetYaw = MPU6050_GetData()->yaw + delta; isTurning = 1; }

static uint8_t updateTurnController(void) {
    float error = targetYaw - MPU6050_GetData()->yaw;
    if (fabsf(error) < TURN_TOLERANCE_DEG) { Motor_Stop(); isTurning = 0; return 1; }
    uint8_t pwm = (fabsf(error) > TURN_SLOW_ZONE_DEG) ? TURN_PWM_FAST : TURN_PWM_SLOW;
    if (error > 0) Motor_TurnRight(pwm); else Motor_TurnLeft(pwm);
    return 0;
}

static uint32_t parkStartTime = 0;
static void updateParkingMode(void) {
    if (parkState == PARK_IDLE) return;
    switch (parkState) {
        case PARK_FORWARD_1:
            Motor_Forward(speedValue); if (parkStartTime == 0) parkStartTime = HAL_GetTick();
            if (HAL_GetTick() - parkStartTime > 2000) { startTurnRelative(-90.0f); parkState = PARK_TURN_R; }
            break;
        case PARK_TURN_R: if (updateTurnController()) { parkStartTime = HAL_GetTick(); parkState = PARK_FORWARD_2; } break;
        case PARK_FORWARD_2:
            Motor_Forward(speedValue); if (HAL_GetTick() - parkStartTime > 2000) { startTurnRelative(90.0f); parkState = PARK_TURN_L; }
            break;
        case PARK_TURN_L: if (updateTurnController()) parkState = PARK_DONE; break;
        case PARK_DONE: Motor_Stop(); currentMove = MOVE_STOP; parkState = PARK_IDLE; parkStartTime = 0; LED_OFF(); needRedraw = 1; break;
        default: break;
    }
}

static void updateOdometry(float dt) {
    int16_t rl = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t rr = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim2, 0); __HAL_TIM_SET_COUNTER(&htim3, 0);
    encL += rl; encR += rr; float dpt = (M_PI * WHEEL_DIAMETER_M) / ENCODER_CPR;
    speedAvg = ((rl + rr) / 2.0f * dpt) / dt;
}

static void fmt_float1(char* b, float v) {
    int s = (v < 0); if (s) v = -v; int i = (int)v; int f = (int)((v - i) * 10 + 0.5f);
    if (f >= 10) { i++; f -= 10; } sprintf(b, "%s%d.%d", s ? "-" : "", i, f);
}

static void updateUI(void) {
  if (!needRedraw && currentScreen != SCREEN_MPU && currentScreen != SCREEN_STATE) return;
  needRedraw = 0; SSD1306_Clear(); char buf[32], f[16]; const MPU6050_Data_t *mpu = MPU6050_GetData();
  switch (currentScreen) {
    case SCREEN_MAIN_MENU:
      SSD1306_WriteStringInverted(0, 0, "   MAIN MENU    ");
      const char *items[] = {"STATE", "MOVE", "SPEED", "MPU"};
      for (int i=0; i<4; i++) {
        snprintf(buf, sizeof(buf), "%s %s", (i==menuIndex)?">":" ", items[i]);
        if(i==menuIndex) SSD1306_WriteStringInverted(0, (i+2), buf); else SSD1306_WriteString(0, (i+2), buf);
      }
      break;
    case SCREEN_STATE:
      SSD1306_WriteStringInverted(0, 0, "     STATE      ");
      snprintf(buf, sizeof(buf), "Move : %s", getMoveStateText(currentMove)); SSD1306_WriteString(0, 2, buf);
      snprintf(buf, sizeof(buf), "Speed: %d", (int)speedValue); SSD1306_WriteString(0, 3, buf);
      fmt_float1(f, mpu->yaw); snprintf(buf, sizeof(buf), "Angle: %s deg", f); SSD1306_WriteString(0, 4, buf);
      snprintf(buf, sizeof(buf), "JETSON: %s", jetsonConnected ? "OK" : "ERROR"); SSD1306_WriteString(0, 5, buf);
      int iv = (int)filteredBatVoltage; int fv = (int)((filteredBatVoltage - iv)*10);
      snprintf(buf, sizeof(buf), "BAT  : %d.%dV (%d%%)", iv, fv, displayedBatPercent); SSD1306_WriteString(0, 7, buf);
      break;
    case SCREEN_MOVE_MENU:
      SSD1306_WriteStringInverted(0, 0, "      MOVE      ");
      for (int i=0; i<MOVE_MENU_COUNT; i++) {
        snprintf(buf, sizeof(buf), "%s %-10s %s", (i==menuIndex)?">":" ", moveNames[i], (i==(int)currentMove)?"*":"");
        if(i==menuIndex) SSD1306_WriteStringInverted(0, (uint8_t)(i+1), buf); else SSD1306_WriteString(0, (uint8_t)(i+1), buf);
      }
      break;
    case SCREEN_MPU:
      SSD1306_WriteStringInverted(0, 0, "    MPU MODE    ");
      fmt_float1(f, mpu->yaw); snprintf(buf, sizeof(buf), "Yaw  : %s deg", f); SSD1306_WriteString(0, 2, buf);
      snprintf(buf, sizeof(buf), "Rate : %d dps", (int)mpu->gz); SSD1306_WriteString(0, 4, buf);
      snprintf(buf, sizeof(buf), "Speed: %d mm/s", (int)(speedAvg * 1000)); SSD1306_WriteString(0, 5, buf);
      snprintf(buf, sizeof(buf), "Enc  : %ld/%ld", (long)encL, (long)encR); SSD1306_WriteString(0, 7, buf);
      break;
    case SCREEN_SPEED:
      SSD1306_WriteStringInverted(0, 0, "   SPEED MODE   ");
      snprintf(buf, sizeof(buf), " Speed: %d", (int)speedValue); SSD1306_WriteString(0, 3, buf);
      break;
    default: break;
  }
  SSD1306_UpdateScreen();
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  
  /* 1. KHỞI TẠO USB SERIAL DÙNG THƯ VIỆN Middlewares (Sửa lại hằng số và descriptor) */
  USBD_Init(&hUsbDeviceFS, &FS_Desc, 0); /* DEVICE_FS thường là 0 */
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
  USBD_Start(&hUsbDeviceFS);
  
  MX_GPIO_Init(); LED_OFF(); MX_TIM1_Init(); MX_TIM2_Init(); MX_TIM3_Init(); MX_I2C1_Init(); MX_ADC1_Init();
  Motor_Module_Init(&htim1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); Motor_Stop();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  OLED_Module_Init(&hi2c1); SSD1306_Init(); MPU6050_Module_Init(&hi2c1); MPU6050_Init();

  uint32_t lastTick = HAL_GetTick();
  while (1) {
    if (HAL_GetTick() - lastTick >= 50) {
        float dt = (HAL_GetTick() - lastTick) / 1000.0f; lastTick = HAL_GetTick();
        readButtons(); MPU6050_Read_All(); MPU6050_UpdateYaw(dt); updateOdometry(dt); updateJetsonStatus(); updateBattery();
        if (currentScreen == SCREEN_MPU && btnSelect.pressed) MPU6050_ResetYaw();
        
        /* FIXED Indentation and Logic */
        if (isTurning) {
            updateTurnController();
        }
        updateParkingMode();

        switch (currentScreen) {
          case SCREEN_MAIN_MENU: 
            if (btnUp.pressed) { menuIndex = (menuIndex <= 0) ? 3 : menuIndex - 1; needRedraw = 1; }
            if (btnDown.pressed) { menuIndex = (menuIndex >= 3) ? 0 : menuIndex + 1; needRedraw = 1; }
            if (btnSelect.pressed) { currentScreen = (ScreenState)(menuIndex==0?1:(menuIndex==1?2:(menuIndex==2?3:4))); menuIndex=0; needRedraw=1; }
            break;
          case SCREEN_MOVE_MENU:
            if (btnUp.pressed) { menuIndex = (menuIndex <= 0) ? 6 : menuIndex - 1; needRedraw = 1; }
            if (btnDown.pressed) { menuIndex = (menuIndex >= 6) ? 0 : menuIndex + 1; needRedraw = 1; }
            if (btnSelect.pressed) { currentMove = (MoveMode)menuIndex; applyMoveMode(); currentScreen = SCREEN_STATE; needRedraw = 1; }
            if (btnBack.pressed) { currentScreen = SCREEN_MAIN_MENU; LED_OFF(); needRedraw = 1; }
            break;
          case SCREEN_SPEED:
            if (btnUp.pressed) { if(speedValue<250) speedValue+=5; needRedraw=1; applyMoveMode(); }
            if (btnDown.pressed) { if(speedValue>10) speedValue-=5; needRedraw=1; applyMoveMode(); }
            if (btnBack.pressed) { currentScreen = SCREEN_MAIN_MENU; needRedraw = 1; }
            break;
          default: if (btnBack.pressed) { currentScreen = SCREEN_MAIN_MENU; needRedraw = 1; } break;
        }
        updateUI();
    }
  }
}

static void MX_ADC1_Init(void) {
  __HAL_RCC_ADC1_CLK_ENABLE(); hadc1.Instance = ADC1; hadc1.Init.Resolution = ADC_RESOLUTION_12B; 
  hadc1.Init.ScanConvMode = DISABLE; hadc1.Init.ContinuousConvMode = DISABLE; HAL_ADC_Init(&hadc1);
  ADC_ChannelConfTypeDef sC = {0}; sC.Channel = ADC_CHANNEL_8; sC.Rank = 1; sC.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sC); __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef G = {0}; G.Pin = GPIO_PIN_0; G.Mode = GPIO_MODE_ANALOG; HAL_GPIO_Init(GPIOB, &G);
}
static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef G = {0}; G.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13;
  G.Mode = GPIO_MODE_OUTPUT_PP; G.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOB, &G);
  G.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10; G.Mode = GPIO_MODE_INPUT; G.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOA, &G);
}
static void MX_TIM1_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE(); htim1.Instance = TIM1; htim1.Init.Period = 65535; HAL_TIM_PWM_Init(&htim1);
  TIM_OC_InitTypeDef S = {0}; S.OCMode = TIM_OCMODE_PWM1; S.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim1, &S, TIM_CHANNEL_1); HAL_TIM_PWM_ConfigChannel(&htim1, &S, TIM_CHANNEL_2);
  GPIO_InitTypeDef G = {0}; G.Pin = GPIO_PIN_8|GPIO_PIN_9; G.Mode = GPIO_MODE_AF_PP; G.Alternate = GPIO_AF1_TIM1; HAL_GPIO_Init(GPIOA, &G);
}
static void MX_TIM2_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE(); htim2.Instance = TIM2; htim2.Init.Period = 0xFFFF;
  TIM_Encoder_InitTypeDef s = {0}; s.EncoderMode = TIM_ENCODERMODE_TI12; HAL_TIM_Encoder_Init(&htim2, &s);
  GPIO_InitTypeDef G = {0}; G.Pin = GPIO_PIN_0|GPIO_PIN_1; G.Mode = GPIO_MODE_AF_PP; G.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA, &G);
}
static void MX_TIM3_Init(void) {
  __HAL_RCC_TIM3_CLK_ENABLE(); htim3.Instance = TIM3; htim3.Init.Period = 0xFFFF;
  TIM_Encoder_InitTypeDef s = {0}; s.EncoderMode = TIM_ENCODERMODE_TI12; HAL_TIM_Encoder_Init(&htim3, &s);
  GPIO_InitTypeDef G = {0}; G.Pin = GPIO_PIN_6|GPIO_PIN_7; G.Mode = GPIO_MODE_AF_PP; G.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOA, &G);
}
static void MX_I2C1_Init(void) {
  __HAL_RCC_I2C1_CLK_ENABLE(); hi2c1.Instance = I2C1; hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; HAL_I2C_Init(&hi2c1);
  GPIO_InitTypeDef G = {0}; G.Pin = GPIO_PIN_6|GPIO_PIN_7; G.Mode = GPIO_MODE_AF_OD; G.Pull = GPIO_PULLUP; G.Speed = GPIO_SPEED_FREQ_VERY_HIGH; G.Alternate = GPIO_AF4_I2C1; HAL_GPIO_Init(GPIOB, &G);
}
static void updateBtn(GPIO_TypeDef* port, uint16_t pin, Button_t* b) {
  uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
  if (raw) { b->holdCounter++; if(b->holdCounter==1 || (b->holdCounter>=10 && b->holdCounter%2==0)) b->pressed=1; else b->pressed=0; }
  else { b->holdCounter=0; b->pressed=0; } b->last = raw;
}
static void readButtons(void) { updateBtn(GPIOA, GPIO_PIN_4, &btnUp); updateBtn(GPIOA, GPIO_PIN_5, &btnDown); updateBtn(GPIOA, GPIO_PIN_2, &btnSelect); updateBtn(GPIOA, GPIO_PIN_10, &btnBack); }
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; RCC_OscInitStruct.PLL.PLLM = 8; RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; RCC_OscInitStruct.PLL.PLLQ = 4; HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}
void Error_Handler(void) { while(1); }