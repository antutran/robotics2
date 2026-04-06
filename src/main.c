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
typedef enum { SCREEN_MAIN_MENU=0, SCREEN_STATE, SCREEN_MOVE_MENU, SCREEN_SPEED, SCREEN_MPU, SCREEN_MPU_DATA, SCREEN_AUTO_MENU, SCREEN_AUTO_STATE, SCREEN_MANUAL_STATE } ScreenState;
typedef enum { MOVE_AUTO=0, MOVE_LEFT, MOVE_RIGHT, MOVE_STRAIGHT, MOVE_BACK, MOVE_STOP, MOVE_PARKING, MOVE_MANUAL } MoveMode;

typedef enum {
    MANUAL_DIR_STOP = 0,
    MANUAL_DIR_FWD,
    MANUAL_DIR_REV,
    MANUAL_DIR_LEFT,
    MANUAL_DIR_RIGHT
} ManualDirection;

static ManualDirection currentManualDir = MANUAL_DIR_STOP;
static ManualDirection targetManualDir = MANUAL_DIR_STOP;
static float currentManualPwm = 0.0f;
static uint32_t manualLastTick = 0;
static uint8_t remoteKeyUp=0, remoteKeyDown=0, remoteKeyLeft=0, remoteKeyRight=0;

typedef enum {
    PARK_IDLE, PARK_FORWARD_1, PARK_TURN_R, PARK_FORWARD_2, PARK_TURN_L, PARK_DONE
} ParkingState;

typedef enum { WHEEL_CENTER=0, WHEEL_LEFT, WHEEL_RIGHT } WheelState;
typedef enum { 
    DET_NONE=0, DET_ONE_WAY, DET_PARK, DET_TURN_RIGHT, DET_SLOW_DOWN, 
    DET_GREEN_LIGHT, DET_RED_LIGHT, DET_YELLOW_LIGHT, DET_CROSSWALK, DET_OBSTACLE,
    DET_TURN_RIGHT_MARKING 
} DetectState;

typedef enum { 
    AUTO_STATE_DRIVE, 
    AUTO_STATE_TURN_PENDING, AUTO_STATE_XWALK_WAIT, AUTO_STATE_TURNING,
    AUTO_STATE_PARK_PENDING, AUTO_STATE_PARK_TURN_R, AUTO_STATE_PARK_FORWARD, AUTO_STATE_PARK_TURN_L,
    AUTO_STATE_TURN_MARK_WAIT, AUTO_STATE_TURN_MARKING,
    AUTO_STATE_BOUNDARY_WAIT, AUTO_STATE_BOUNDARY_TURN  /* [NEW] Boundary detection states */
} AutoDrivingState;

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
static uint8_t      speedValue    = 80;
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

/* Boundary Detection & AI Data */
static int   left_pwm = 0, right_pwm = 0;
static uint8_t boundary_detected = 0;       /* [NEW] From Jetson: yellow crossed scan line */
static uint32_t boundary_wait_tick = 0;      /* [NEW] Timer for 2s straight after boundary */
static uint32_t boundary_cooldown_tick = 0;  /* [NEW] 3s cooldown after boundary turn */

static uint8_t auto_run_enabled  = 0;
static uint8_t auto_ai_enabled   = 1;
static uint8_t auto_lane_enabled = 1;
static WheelState  wheel_state   = WHEEL_CENTER;
static DetectState detect_state  = DET_NONE;
static AutoDrivingState auto_driving_state = AUTO_STATE_DRIVE;
static uint32_t xwalk_exit_tick = 0; // Timer for crosswalk clearance
static uint32_t oneway_lock_tick = 0; // Timer for one-way sign lockout (5s)
static uint32_t turn_mark_exit_tick = 0; // Timer for turn marking disappearance (3s)

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

/* Lane Centering Functions */
uint8_t is_auto_mode_enabled(void);
void drive_straight(int *l_pwm, int *r_pwm);
void send_motor_command(int l_pwm, int r_pwm);
void stop_motor(void);
void process_lane_centering(void);
void Process_Serial_Data(char *data, uint32_t len);

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
    }
}

static void updateJetsonStatus(void) { jetsonConnected = (HAL_GetTick() - lastJetsonRxTick < JETSON_TIMEOUT_MS); }
void Jetson_Heartbeat_Callback(void) { lastJetsonRxTick = HAL_GetTick(); }

static const char* getMoveStateText(MoveMode mode) {
    switch (mode) {
        case MOVE_STRAIGHT: return "STRAIGHT"; case MOVE_LEFT: return "LEFT";
        case MOVE_RIGHT: return "RIGHT";       case MOVE_BACK: return "BACK";
        case MOVE_STOP: return "STOP";         case MOVE_PARKING: return "PARKING";
        case MOVE_AUTO: return "AUTO";         case MOVE_MANUAL: return "MANUAL";
        default: return "UNKNOWN";
    }
}

static const char* getDetectText(DetectState det) {
    const char* names[] = {"NONE", "ONEWAY", "PARK", "TURN_R", "SLOW", "GREEN", "RED", "YELLOW", "CROSSWK", "OBSTAC", "T_MARK"};
    if ((int)det >= 0 && (int)det < 11) return names[(int)det];
    return "???";
}

static void applyMoveMode(void) {
    if (currentMove == MOVE_PARKING) { parkState = PARK_FORWARD_1; LED_YELLOW(); }
    else if (currentMove == MOVE_AUTO) {
        parkState = PARK_IDLE; isTurning = 0;
        boundary_detected = 0; boundary_wait_tick = 0; boundary_cooldown_tick = 0;
        LED_GREEN(); // Auto mode indicator
        currentScreen = SCREEN_AUTO_MENU; // Change to submenu config
        menuIndex = 0; needRedraw = 1;
    }
    else if (currentMove == MOVE_MANUAL) {
        parkState = PARK_IDLE; isTurning = 0;
        LED_YELLOW();
        currentScreen = SCREEN_MANUAL_STATE;
        currentManualDir = MANUAL_DIR_STOP;
        targetManualDir = MANUAL_DIR_STOP;
        currentManualPwm = 0.0f;
        manualLastTick = HAL_GetTick();
        Motor_Stop();
        needRedraw = 1;
    }
    else {
        parkState = PARK_IDLE; isTurning = 0;
        LED_OFF();
        currentScreen = SCREEN_STATE; // Ensure manual modes show state screen
        switch (currentMove) {
            case MOVE_LEFT: Motor_TurnLeft(speedValue); break; case MOVE_RIGHT: Motor_TurnRight(speedValue); break;
            case MOVE_STRAIGHT: Motor_Forward(speedValue); break; case MOVE_BACK: Motor_Backward(speedValue); break;
            case MOVE_STOP: Motor_Stop(); break; default: break;
        }
    }
}

static void updateManualControl(void) {
    // Exit manual mode when BACK button is pressed (from STM32 buttons)
    if (btnBack.pressed) {
        Motor_Stop();
        currentManualPwm = 0.0f;
        currentManualDir = MANUAL_DIR_STOP;
        remoteKeyUp=0; remoteKeyDown=0; remoteKeyLeft=0; remoteKeyRight=0;
        currentMove = MOVE_STOP;
        currentScreen = SCREEN_MAIN_MENU;
        needRedraw = 1;
        return;
    }

    targetManualDir = MANUAL_DIR_STOP;
    if (remoteKeyUp)      targetManualDir = MANUAL_DIR_FWD;
    else if (remoteKeyDown)  targetManualDir = MANUAL_DIR_REV;
    else if (remoteKeyLeft)  targetManualDir = MANUAL_DIR_LEFT;
    else if (remoteKeyRight) targetManualDir = MANUAL_DIR_RIGHT;
    
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - manualLastTick;
    manualLastTick = now;
    if (dt_ms > 100) dt_ms = 100;
    
    // Smooth speed ramp params
    float ramp_step = (255.0f / 150.0f) * dt_ms; // Acceleration: 150ms from 0 to 255 (faster)
    float down_step = (255.0f / 100.0f) * dt_ms; // Deceleration: 100ms from 255 to 0 (faster)

    if (targetManualDir != currentManualDir && currentManualDir != MANUAL_DIR_STOP) {
        // Must stop or reduce to 0 first!
        currentManualPwm -= down_step;
        if (currentManualPwm <= 0) {
            currentManualPwm = 0;
            currentManualDir = MANUAL_DIR_STOP;
            Motor_Stop();
        }
    } else {
        // Change to target direction and ramp up
        currentManualDir = targetManualDir;
        if (currentManualDir == MANUAL_DIR_STOP) {
            currentManualPwm -= down_step;
            if (currentManualPwm <= 0) {
                currentManualPwm = 0;
                Motor_Stop();
            }
        } else {
            // Accelerate
            currentManualPwm += ramp_step;
            if (currentManualPwm > speedValue) currentManualPwm = speedValue;
        }
    }

    // Apply motor PWM
    if (currentManualDir != MANUAL_DIR_STOP && currentManualPwm > 0) {
        uint8_t pwm = (uint8_t)currentManualPwm;
        switch (currentManualDir) {
            case MANUAL_DIR_FWD: Motor_Forward(pwm); break;
            case MANUAL_DIR_REV: Motor_Backward(pwm); break;
            case MANUAL_DIR_LEFT: Motor_TurnLeft(pwm); break;
            case MANUAL_DIR_RIGHT: Motor_TurnRight(pwm); break;
            default: Motor_Stop(); break;
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
  if (!needRedraw && 
      currentScreen != SCREEN_MPU && 
      currentScreen != SCREEN_MPU_DATA && 
      currentScreen != SCREEN_STATE &&
      currentScreen != SCREEN_AUTO_MENU &&
      currentScreen != SCREEN_AUTO_STATE &&
      currentScreen != SCREEN_MANUAL_STATE) return;
  needRedraw = 0; SSD1306_Clear(); char buf[32], f[16]; const MPU6050_Data_t *mpu = MPU6050_GetData();
  switch (currentScreen) {
    case SCREEN_MAIN_MENU:
      SSD1306_WriteStringInverted(0, 0, "   MAIN MENU    ");
      const char *items[] = {"STATE", "MANUAL", "MOVE", "SPEED", "MPU"};
      for (int i=0; i<5; i++) {
        snprintf(buf, sizeof(buf), "%s %s", (i==menuIndex)?">":" ", items[i]);
        if(i==menuIndex) SSD1306_WriteStringInverted(0, (i+1), buf); else SSD1306_WriteString(0, (i+1), buf);
      }
      break;
    case SCREEN_STATE:
      SSD1306_WriteStringInverted(0, 0, "     STATE      ");
      snprintf(buf, sizeof(buf), "Move : %-10s", getMoveStateText(currentMove)); SSD1306_WriteString(0, 2, buf);
      if (currentMove == MOVE_AUTO) {
          snprintf(buf, sizeof(buf), "BOUND: %-8s", boundary_detected ? "DETECT" : "CLEAR"); SSD1306_WriteString(0, 4, buf);
          snprintf(buf, sizeof(buf), "PWM: L%d R%d", left_pwm, right_pwm); SSD1306_WriteString(0, 5, buf);
          snprintf(buf, sizeof(buf), "JETSON: %-5s", jetsonConnected ? "OK" : "MISS"); SSD1306_WriteString(0, 7, buf);
      } else {
          snprintf(buf, sizeof(buf), "Speed: %d", (int)speedValue); SSD1306_WriteString(0, 3, buf);
          fmt_float1(f, mpu->yaw); snprintf(buf, sizeof(buf), "Angle: %s deg", f); SSD1306_WriteString(0, 4, buf);
          snprintf(buf, sizeof(buf), "JETSON: %s", jetsonConnected ? "OK" : "ERROR"); SSD1306_WriteString(0, 6, buf);
      }
      break;
    case SCREEN_MOVE_MENU:
      SSD1306_WriteStringInverted(0, 0, "      MOVE      ");
      for (int i=0; i<MOVE_MENU_COUNT; i++) {
        snprintf(buf, sizeof(buf), "%s %-10s %s", (i==menuIndex)?">":" ", moveNames[i], (i==(int)currentMove)?"*":"");
        if(i==menuIndex) SSD1306_WriteStringInverted(0, (uint8_t)(i+1), buf); else SSD1306_WriteString(0, (uint8_t)(i+1), buf);
      }
      break;
    case SCREEN_MPU:
      SSD1306_WriteStringInverted(0, 0, "    MPU MENU    ");
      snprintf(buf, sizeof(buf), "%s DISPLAY DATA", (menuIndex==0)?">":" ");
      if(menuIndex==0) SSD1306_WriteStringInverted(0, 2, buf); else SSD1306_WriteString(0, 2, buf);
      snprintf(buf, sizeof(buf), "%s RESET YAW", (menuIndex==1)?">":" ");
      if(menuIndex==1) SSD1306_WriteStringInverted(0, 4, buf); else SSD1306_WriteString(0, 4, buf);
      snprintf(buf, sizeof(buf), "%s CALIBRATE", (menuIndex==2)?">":" ");
      if(menuIndex==2) SSD1306_WriteStringInverted(0, 6, buf); else SSD1306_WriteString(0, 6, buf);
      break;
    case SCREEN_MPU_DATA:
      SSD1306_WriteStringInverted(0, 0, "    MPU DATA    ");
      fmt_float1(f, mpu->yaw); snprintf(buf, sizeof(buf), "Yaw  : %s deg", f); SSD1306_WriteString(0, 2, buf);
      snprintf(buf, sizeof(buf), "Rate : %d dps", (int)mpu->gz); SSD1306_WriteString(0, 4, buf);
      snprintf(buf, sizeof(buf), "Speed: %d mm/s", (int)(speedAvg * 1000)); SSD1306_WriteString(0, 5, buf);
      snprintf(buf, sizeof(buf), "Enc  : %ld/%ld", (long)encL, (long)encR); SSD1306_WriteString(0, 7, buf);
      break;
    case SCREEN_SPEED:
      SSD1306_WriteStringInverted(0, 0, "   SPEED MODE   ");
      snprintf(buf, sizeof(buf), " Base: %d", (int)speedValue); SSD1306_WriteString(0, 3, buf);
      break;
    case SCREEN_AUTO_MENU:
      SSD1306_WriteStringInverted(0, 0, "   AUTO CONFIG  ");
      snprintf(buf, sizeof(buf), "%s RUN: %s", (menuIndex==0)?">":" ", auto_run_enabled?"ON ":"OFF");
      SSD1306_WriteString(0, 2, buf);
      snprintf(buf, sizeof(buf), "%s AI : %s", (menuIndex==1)?">":" ", auto_ai_enabled?"ON ":"OFF");
      SSD1306_WriteString(0, 3, buf);
      snprintf(buf, sizeof(buf), "%s LANE: %s", (menuIndex==2)?">":" ", auto_lane_enabled?"ON ":"OFF");
      SSD1306_WriteString(0, 4, buf);
      SSD1306_WriteString(0, 6, (menuIndex==3)?"> NEXT SCREEN":"  NEXT SCREEN");
      break;
    case SCREEN_AUTO_STATE:
      SSD1306_WriteStringInverted(0, 0, "   AUTO STATE   ");
      snprintf(buf, sizeof(buf), "Run:%s AI:%s L:%s", auto_run_enabled?"ON":"OF", auto_ai_enabled?"ON":"OF", auto_lane_enabled?"ON":"OF"); 
      SSD1306_WriteString(0, 1, buf);
      snprintf(buf, sizeof(buf), "BOUND: %s", boundary_detected ? "DETECT" : "CLEAR"); SSD1306_WriteString(0, 3, buf);
      snprintf(buf, sizeof(buf), "DET  : %s", getDetectText(detect_state)); SSD1306_WriteString(0, 5, buf);
      snprintf(buf, sizeof(buf), "PWM  : L%d R%d", left_pwm, right_pwm); SSD1306_WriteString(0, 7, buf);
      break;
    case SCREEN_MANUAL_STATE:
      SSD1306_WriteStringInverted(0, 0, "  MANUAL MODE   ");
      SSD1306_WriteString(0, 1, "Src : SERIAL KB");
      
      snprintf(buf, sizeof(buf), "Cmd : %-5s", 
               targetManualDir == MANUAL_DIR_FWD ? "UP" : 
               targetManualDir == MANUAL_DIR_REV ? "DOWN" : 
               targetManualDir == MANUAL_DIR_LEFT ? "LEFT" : 
               targetManualDir == MANUAL_DIR_RIGHT ? "RIGHT" : "STOP");
      SSD1306_WriteString(0, 2, buf);
      
      snprintf(buf, sizeof(buf), "Dir : %-5s", 
               currentManualDir == MANUAL_DIR_FWD ? "FWD" : 
               currentManualDir == MANUAL_DIR_REV ? "REV" : 
               currentManualDir == MANUAL_DIR_LEFT ? "LEFT" : 
               currentManualDir == MANUAL_DIR_RIGHT ? "RIGHT" : "STOP");
      SSD1306_WriteString(0, 3, buf);
      
      snprintf(buf, sizeof(buf), "PWM : %3d /%3d", (int)currentManualPwm, speedValue);
      SSD1306_WriteString(0, 5, buf);
      
      SSD1306_WriteStringInverted(0, 7, "BTN BACK to MENU");
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
        // --- Handled in switch below ---
        
        // --- Outbound Status Telemetry ---
        static uint32_t lastLogTick = 0;
        if (HAL_GetTick() - lastLogTick > 100) {
            lastLogTick = HAL_GetTick();
            // Essential for Python script to know the current mode (MOVE_MANUAL is 7)
            printf("S:%d,%d,%d,%d\n", (int)currentMove, (int)auto_run_enabled, (int)auto_ai_enabled, (int)auto_lane_enabled);
        }

        /* Lane Centering / Auto Mode */
        process_lane_centering();

        if (isTurning) {
            updateTurnController();
        }
        updateParkingMode();

        if (currentScreen == SCREEN_MANUAL_STATE) {
            updateManualControl();
        }

        switch (currentScreen) {
          case SCREEN_MAIN_MENU: 
            if (btnUp.pressed) { menuIndex = (menuIndex <= 0) ? 4 : menuIndex - 1; needRedraw = 1; }
            if (btnDown.pressed) { menuIndex = (menuIndex >= 4) ? 0 : menuIndex + 1; needRedraw = 1; }
            if (btnSelect.pressed) { 
                if (menuIndex == 0) currentScreen = SCREEN_STATE;
                else if (menuIndex == 1) { currentMove = MOVE_MANUAL; applyMoveMode(); }
                else if (menuIndex == 2) currentScreen = SCREEN_MOVE_MENU;
                else if (menuIndex == 3) currentScreen = SCREEN_SPEED;
                else if (menuIndex == 4) currentScreen = SCREEN_MPU;
                menuIndex = 0; needRedraw = 1; 
            }
            break;
          case SCREEN_MOVE_MENU:
            if (btnUp.pressed) { menuIndex = (menuIndex <= 0) ? 6 : menuIndex - 1; needRedraw = 1; }
            if (btnDown.pressed) { menuIndex = (menuIndex >= 6) ? 0 : menuIndex + 1; needRedraw = 1; }
            if (btnSelect.pressed) { 
                currentMove = (MoveMode)menuIndex; 
                applyMoveMode(); 
                needRedraw = 1; 
            }
            if (btnBack.pressed) { currentScreen = SCREEN_MAIN_MENU; LED_OFF(); needRedraw = 1; }
            break;
          case SCREEN_SPEED:
            if (btnUp.pressed) { if(speedValue<250) speedValue+=5; needRedraw=1; }
            if (btnDown.pressed) { if(speedValue>10) speedValue-=5; needRedraw=1; }
            if (btnBack.pressed) { currentScreen = SCREEN_MAIN_MENU; needRedraw = 1; }
            break;
          case SCREEN_AUTO_MENU:
            if (btnUp.pressed) { menuIndex = (menuIndex <= 0) ? 3 : menuIndex - 1; needRedraw = 1; }
            if (btnDown.pressed) { menuIndex = (menuIndex >= 3) ? 0 : menuIndex + 1; needRedraw = 1; }
            if (btnSelect.pressed) {
                if (menuIndex == 0) auto_run_enabled = !auto_run_enabled;
                else if (menuIndex == 1) auto_ai_enabled = !auto_ai_enabled;
                else if (menuIndex == 2) auto_lane_enabled = !auto_lane_enabled;
                else if (menuIndex == 3) { currentScreen = SCREEN_AUTO_STATE; menuIndex = 0; }
                needRedraw = 1;
            }
            if (btnBack.pressed) { currentScreen = SCREEN_MOVE_MENU; needRedraw = 1; }
            break;
          case SCREEN_AUTO_STATE:
            if (btnBack.pressed) { currentScreen = SCREEN_AUTO_MENU; needRedraw = 1; }
            break;
          case SCREEN_MPU:
            if (btnUp.pressed) { menuIndex = (menuIndex <= 0) ? 2 : menuIndex - 1; needRedraw = 1; }
            if (btnDown.pressed) { menuIndex = (menuIndex >= 2) ? 0 : menuIndex + 1; needRedraw = 1; }
            if (btnSelect.pressed) {
                if (menuIndex == 0) { currentScreen = SCREEN_MPU_DATA; }
                else if (menuIndex == 1) { MPU6050_ResetYaw(); }
                else if (menuIndex == 2) { 
                    SSD1306_Clear();
                    SSD1306_WriteString(0, 3, " CALIBRATING...");
                    SSD1306_UpdateScreen();
                    MPU6050_Calibrate(500); 
                }
                needRedraw = 1;
            }
            if (btnBack.pressed) { currentScreen = SCREEN_MAIN_MENU; menuIndex = 0; needRedraw = 1; }
            break;
          case SCREEN_MPU_DATA:
            if (btnBack.pressed) { currentScreen = SCREEN_MPU; needRedraw = 1; }
            break;
          case SCREEN_MANUAL_STATE:
            // Do not react to normal btn.pressed events to avoid exiting unintentionally
            // Exiting is handled by holding UP+DOWN in updateManualControl
            btnBack.pressed = 0; btnUp.pressed = 0; btnDown.pressed = 0; btnSelect.pressed = 0;
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

/* -----------------------------------------------------------------------
 * LANE CENTERING & AUTO MODE LOGIC
 * ----------------------------------------------------------------------- */

uint8_t is_auto_mode_enabled(void) {
    return (currentMove == MOVE_AUTO);
}

void stop_motor(void) {
    Motor_Stop();
    left_pwm = 0; right_pwm = 0;
}

void send_motor_command(int l_pwm, int r_pwm) {
    // Clamp PWM values to 0-255
    if (l_pwm > 255) l_pwm = 255;
    if (l_pwm < 0)   l_pwm = 0;
    if (r_pwm > 255) r_pwm = 255;
    if (r_pwm < 0)   r_pwm = 0;
    
    Motor_Set_Speed((int16_t)l_pwm, (int16_t)r_pwm);
}

/* [SIMPLIFIED] No lane centering – car drives straight by default */
void drive_straight(int *l_pwm, int *r_pwm) {
    float base_pwm = (float)speedValue;
    if (detect_state == DET_SLOW_DOWN || detect_state == DET_YELLOW_LIGHT) {
        base_pwm *= 0.5f; // Reduce speed to 50%
    }
    *l_pwm = (int)base_pwm;
    *r_pwm = (int)base_pwm;
    wheel_state = WHEEL_CENTER;
}

void process_lane_centering(void) {
    if (!is_auto_mode_enabled()) return;
    
    // 1. If Run is OFF, stop everything
    if (!auto_run_enabled) {
        stop_motor();
        auto_driving_state = AUTO_STATE_DRIVE; // Reset state
        return;
    }

    // 2. Safety: If Jetson is not connected, stop
    if (!jetsonConnected) {
        stop_motor();
        return;
    }

    // 3. Auto Mode State Machine for Mission (Turning Logic)
    if (auto_ai_enabled) {
        // 1. One-Way Lockout check (5 seconds)
        if (detect_state == DET_ONE_WAY) {
            oneway_lock_tick = HAL_GetTick(); // Start/Reset 5s timer
        }

        uint8_t in_oneway_lock = (oneway_lock_tick != 0 && (HAL_GetTick() - oneway_lock_tick < 5000));
        
        if (in_oneway_lock) {
            // During 5s lock, stay in DRIVE mode even if other signs are seen
            // But still respect critical safety (Obstable/Red Light)
            if (detect_state == DET_RED_LIGHT || detect_state == DET_OBSTACLE) {
                stop_motor();
                return;
            }
            auto_driving_state = AUTO_STATE_DRIVE;
            // Proceed to motor computation (lane centering still active)
        } else {
            // Global detection overrides (only if not in oneway lock)
            if (detect_state == DET_RED_LIGHT || detect_state == DET_OBSTACLE) {
                stop_motor();
                return;
            }
        }

        switch (auto_driving_state) {
            case AUTO_STATE_DRIVE:
                if (in_oneway_lock) {
                     // Stay here
                }
                else if (detect_state == DET_TURN_RIGHT) {
                    auto_driving_state = AUTO_STATE_TURN_PENDING;
                    xwalk_exit_tick = HAL_GetTick(); // Start 2s countdown
                }
                else if (detect_state == DET_TURN_RIGHT_MARKING) {
                    auto_driving_state = AUTO_STATE_TURN_MARK_WAIT;
                }
                else if (detect_state == DET_PARK) {
                    auto_driving_state = AUTO_STATE_PARK_PENDING;
                }
                break;

            case AUTO_STATE_TURN_PENDING:
                if (in_oneway_lock) {
                    auto_driving_state = AUTO_STATE_DRIVE; // Abort turn if one-way sign appears
                }
                // Intersection triggered. Wait until no more intersection/crosswalk seen.
                else if (detect_state == DET_CROSSWALK || detect_state == DET_TURN_RIGHT) {
                    auto_driving_state = AUTO_STATE_XWALK_WAIT;
                    xwalk_exit_tick = HAL_GetTick(); // Keep resetting
                } 
                else if (HAL_GetTick() - xwalk_exit_tick > 3000) {
                    // This case is if we triggered TURN_RIGHT but never saw CROSSWALK 
                    // and then nothing happens for 2s. 
                    startTurnRelative(90.0f);
                    auto_driving_state = AUTO_STATE_TURNING;
                    return;
                }
                break;

            case AUTO_STATE_XWALK_WAIT:
                if (in_oneway_lock) {
                    auto_driving_state = AUTO_STATE_DRIVE; // Abort turn if one-way sign appears
                }
                else if (detect_state == DET_CROSSWALK || detect_state == DET_TURN_RIGHT) {
                    xwalk_exit_tick = HAL_GetTick(); // Still in crossing/intersection zone
                } 
                else if (HAL_GetTick() - xwalk_exit_tick > 3000) {
                    // All clear for 3 seconds
                    startTurnRelative(90.0f);
                    auto_driving_state = AUTO_STATE_TURNING;
                    return;
                }
                break;

            case AUTO_STATE_TURNING:
                if (updateTurnController()) {
                    auto_driving_state = AUTO_STATE_DRIVE; // Turn finished
                }
                return; // Prevent lane centering from fighting the turn

            case AUTO_STATE_PARK_PENDING:
                // We saw the sign. Stay in this state while still seeing it.
                if (detect_state != DET_PARK) {
                    // Sign disappeared! Start parking sequence.
                    startTurnRelative(90.0f);
                    auto_driving_state = AUTO_STATE_PARK_TURN_R;
                    return;
                }
                break;

            case AUTO_STATE_PARK_TURN_R:
                if (updateTurnController()) {
                    parkStartTime = HAL_GetTick();
                    auto_driving_state = AUTO_STATE_PARK_FORWARD;
                }
                return;

            case AUTO_STATE_PARK_FORWARD:
                Motor_Forward(speedValue);
                if (HAL_GetTick() - parkStartTime > 3000) {
                    startTurnRelative(-90.0f);
                    auto_driving_state = AUTO_STATE_PARK_TURN_L;
                }
                return;

            case AUTO_STATE_PARK_TURN_L:
                if (updateTurnController()) {
                    auto_run_enabled = 0; // Mission complete
                    stop_motor();
                    auto_driving_state = AUTO_STATE_DRIVE;
                    needRedraw = 1;
                }
                return;

            case AUTO_STATE_TURN_MARK_WAIT:
                if (in_oneway_lock) {
                    auto_driving_state = AUTO_STATE_DRIVE;
                }
                else if (detect_state == DET_TURN_RIGHT_MARKING) {
                    // Still seeing it, keep moving/reset exit timer
                    turn_mark_exit_tick = 0; 
                } else {
                    // Sign disappeared! Start 3s countdown
                    if (turn_mark_exit_tick == 0) turn_mark_exit_tick = HAL_GetTick();
                    if (HAL_GetTick() - turn_mark_exit_tick > 3000) {
                        startTurnRelative(90.0f);
                        auto_driving_state = AUTO_STATE_TURN_MARKING;
                        return;
                    }
                }
                break;

            case AUTO_STATE_TURN_MARKING:
                if (updateTurnController()) {
                    auto_driving_state = AUTO_STATE_DRIVE;
                }
                return;

            default: break;
        }
    } else {
        auto_driving_state = AUTO_STATE_DRIVE;
    }

    // 4. Boundary Detection – replaces lane centering
    //    When yellow border crosses scan line → straight 2s → turn right → straight
    if (auto_lane_enabled && auto_driving_state == AUTO_STATE_DRIVE) {
        uint8_t cooled = (boundary_cooldown_tick == 0 || 
                          (HAL_GetTick() - boundary_cooldown_tick > 3000));
        if (boundary_detected && cooled) {
            auto_driving_state = AUTO_STATE_BOUNDARY_WAIT;
            boundary_wait_tick = HAL_GetTick();
        }
    }

    if (auto_driving_state == AUTO_STATE_BOUNDARY_WAIT) {
        // Continue straight for 2 seconds after boundary detected
        if (HAL_GetTick() - boundary_wait_tick > 3000) {
            startTurnRelative(90.0f); // Turn right 90 degrees
            auto_driving_state = AUTO_STATE_BOUNDARY_TURN;
            return;
        }
        // Fall through to drive straight below
    }
    else if (auto_driving_state == AUTO_STATE_BOUNDARY_TURN) {
        if (updateTurnController()) {
            auto_driving_state = AUTO_STATE_DRIVE;
            boundary_cooldown_tick = HAL_GetTick(); // 3s cooldown to prevent re-trigger
        }
        return; // Turn controller handles motors
    }

    // 5. Default: drive straight
    drive_straight(&left_pwm, &right_pwm);
    send_motor_command(left_pwm, right_pwm);
}

/**
 * Parse serial data from Jetson.
 * Expected formats: "B:0" or "B:1" (boundary), "D:idx" (detection)
 */
void Process_Serial_Data(char *data, uint32_t len) {
    char *d_ptr = strstr(data, "D:");
    char *b_ptr = strstr(data, "B:");
    
    if (b_ptr) {
        boundary_detected = (uint8_t)atoi(b_ptr + 2);
        lastJetsonRxTick = HAL_GetTick();
    }
    
    if (d_ptr) {
        detect_state = (DetectState)atoi(d_ptr + 2);
        lastJetsonRxTick = HAL_GetTick();
    }
    
    char *k_ptr = strstr(data, "K:");
    if (k_ptr) {
        char cmd = k_ptr[2];
        if (cmd == 'U') remoteKeyUp = 1; else if (cmd == 'u') remoteKeyUp = 0;
        if (cmd == 'D') remoteKeyDown = 1; else if (cmd == 'd') remoteKeyDown = 0;
        if (cmd == 'L') remoteKeyLeft = 1; else if (cmd == 'l') remoteKeyLeft = 0;
        if (cmd == 'R') remoteKeyRight = 1; else if (cmd == 'r') remoteKeyRight = 0;
        if (cmd == 'S') { remoteKeyUp=0; remoteKeyDown=0; remoteKeyLeft=0; remoteKeyRight=0; }
        lastJetsonRxTick = HAL_GetTick();
    }
}

/* ----------------------------------------------------------------------- */

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