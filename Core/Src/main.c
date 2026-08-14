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
  * If no LICENSE file comes \with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    volatile float speed;
    float p,i,d;
    volatile float speed_target; 
    volatile float now_speed_target;
    volatile float different_sum;
    volatile float low_pass_different_sum;
    volatile float last_difference;
    volatile float last_input;
    volatile float low_pass_derivative;
    volatile float last_last_difference;
    volatile float current_d;
    volatile float current_p;
    volatile float current_d_target;
    volatile float current_p_target;
    volatile float current_d_different_sum;
    volatile float current_d_low_pass_different_sum;
    volatile float current_d_last_difference;      //currentは電流といういみ　現在ではない
    volatile float current_p_different_sum;
    volatile float current_p_low_pass_different_sum;
    volatile float current_p_last_difference; 
    volatile float current_d_pgain;
    volatile float current_d_igain;
    volatile float current_d_dgain;
    volatile float current_p_pgain;
    volatile float current_p_igain;
    volatile float current_p_dgain;
}pid_items;

pid_items motor[3];

volatile int cutoff;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c3;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define STSPIN_I2C_ADDR   (0x47 << 1)   // I2Cのスレーブアドレスには7bitアドレスと8bitアドレスがあり、アドレスの後ろに読み込みか書き込みかを示す1bit（R/Wビット）がくっついて送信される。

static HAL_StatusTypeDef stspin_write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c3, STSPIN_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef stspin_read_reg(uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(&hi2c3, STSPIN_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

// STATUSレジスタ(0x80)を読み、フォルトがあればCLEAR(0x09)に0xFFを書く
static uint8_t stspin_clear_faults(void)
{
    uint8_t status = 0;
    stspin_read_reg(0x80, &status);
    if (status & 0x0F) {          // もしstatusの下位4bitの中に1があり、エラーがある場合はifが通る
        stspin_write_reg(0x09, 0xFF);  // CLEARレジスタ
    }
    return status;
}

#define PWM_PERIOD 3999      //カウンターがどこまで数えたら 0 に戻るかを決める天井の数値       
#define POLE_PAIRS 7         //モーターの外側についている磁石の数
#define clock_time 0.0002    //time一回当たりの周期
#define resistance_for_current 0.001   //電流計測に用いる抵抗値
#define opamp_gain 16.0  //cudemxで設定したPGAgainの値
#define drive_voltage 3.3  //マイコンの駆動電圧
static inline float fast_sin(float x) { return sinf(x); }
static inline float fast_cos(float x) { return cosf(x); }

volatile int pid_mode[3] = {0, 0, 0};   //0ならlocate_pid 1ならspeed_pid
volatile int control_motor_mode[3] = {0, 0, 0};       //モーターの制御モード

volatile uint32_t rx_ok_count = 0;
static volatile uint32_t tim2_cnt = 0;

/*電流値　単位はアンペア*/
volatile float current_u = 0.0f;
volatile float current_v = 0.0f;
volatile float current_w = 0.0f;

//現在の速度を計算する？ための関数
volatile float speed_now = 0; //

/*実際にADCで読み取った値*/
uint32_t offset_u = 2048;
uint32_t offset_v = 2048;
uint32_t offset_w = 2048;

#define CAN_MOTOR_CMD_BASE_ID  0x100u

typedef struct __attribute__((packed)) {
    float   speed_target;         /* byte0-3 */
    uint8_t pid_mode;             /* byte4: 0=locate_pid, 1=speed_pid */
    uint8_t control_motor_mode;   /* byte5: 0=電圧制御, 1=電流制御 */
    uint8_t reserved[2];          /* byte6-7 */
} can_motor_cmd_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
float locate_pid(volatile float output, float target, float p, float i, float d,volatile float *different_sum, volatile float *low_pass_different_sum,volatile float *last_difference, int cutoff);
float speed_pid(volatile float output, float target, float p, float i, float d, volatile float *low_pass_different_sum, volatile float *last_difference, volatile float *last_last_difference, volatile float last_input, volatile float *low_pass_derivative, int cutoff);
void measure_current(void);
static void FDCAN1_ConfigFilterAndStart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void omni_to_wheels(float vx, float vy, float omega, float R,
                          float *va, float *vb, float *vc)
{
    *va = -0.86602540378f * vx - 0.5f * vy - omega * R;
    *vb =  1.0f * vy - omega * R;
    *vc =  0.86602540378f * vx - 0.5f * vy - omega * R;
}

static void FDCAN1_ConfigFilterAndStart(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /* 0x100〜0x102 の標準IDだけをRXFIFO0に通す */
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = CAN_MOTOR_CMD_BASE_ID;
    sFilterConfig.FilterID2    = CAN_MOTOR_CMD_BASE_ID ;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* フィルタに合致しないフレームは破棄、リモートフレームも破棄 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    if (RxHeader.IdType != FDCAN_STANDARD_ID) return;
    if (RxHeader.DataLength != FDCAN_DLC_BYTES_8) return;
    if (RxHeader.Identifier != MOTOR_CAN_ID) return;   // 自分宛てのIDでなければ無視

    can_motor_cmd_t cmd;
    memcpy(&cmd, RxData, sizeof(cmd));

    if (cmd.speed_target > 5000.0f)  cmd.speed_target = 5000.0f;
    if (cmd.speed_target < -5000.0f) cmd.speed_target = -5000.0f;

    motor[0].speed_target   = cmd.speed_target;
    pid_mode[0]             = cmd.pid_mode ? 1 : 0;
    control_motor_mode[0]   = cmd.control_motor_mode ? 1 : 0;

    rx_ok_count++;
}

static uint16_t spi_transfer_16(uint16_t tx)
{
    uint16_t rx = 0;

    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);// 通信開始
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx, (uint8_t*)&rx, 1, HAL_MAX_DELAY);// 16ビット送受信
    HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET); // 通信終了
    
    return rx;
}

uint16_t as5047p_read_angle(void)
{

    spi_transfer_16(0xFFFF);   // 1回目に返ってくるデータはゴミ      
    uint16_t raw = spi_transfer_16(0xC000); // 2回目には何もしない空データを送り、その隙に1回目で要求したデータを受け取る
    return raw & 0x3FFF;    // 余計なビットを消して、純粋な14ビットデータにする    

}

int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

void set_pwm(float ua, float ub, float uc)
{

    /*各相の電圧指令値は0から1であるので不正な値を消去*/
    if (ua < 0.0f) ua = 0.0f;
    if (ua > 1.0f) ua = 1.0f;
    if (ub < 0.0f) ub = 0.0f;
    if (ub > 1.0f) ub = 1.0f;
    if (uc < 0.0f) uc = 0.0f;
    if (uc > 1.0f) uc = 1.0f;

    /*CCRがCNT(0からPWM_PERIODまでの高速カウント)より大きい時low、それ以外はhighとなる*/
    TIM1->CCR1 = (uint32_t)(ua * PWM_PERIOD);
    TIM1->CCR2 = (uint32_t)(ub * PWM_PERIOD);
    TIM1->CCR3 = (uint32_t)(uc * PWM_PERIOD);
}

static float electrical_direction = 0.0f;   
static float step_move = 0.01f;      //使っていないが残している
static float amp = 0.05f;           // 出力の大きさを調整できる
static float zero_offset_rad = 0.0f;  //初期位置のずれを確認する

/*while表示用*/
uint16_t diag;
float angle_deg;
uint16_t angle_raw;
float speed_rpm;
int32_t speed_rpm_int;

void update_openloop(float voltage)
{
    static uint16_t prev_angle_raw = 0;
    static float filtered_speed = 0.0;  //cutoffはd項のfilter これは全体の出力filter
    angle_raw = as5047p_read_angle();
    
    /*一回転した時の処理*/
    int16_t diff = (int16_t)angle_raw - (int16_t)prev_angle_raw;
    if (diff > 8192)  diff -= 16384;
    if (diff < -8192) diff += 16384;
    prev_angle_raw = angle_raw;

    /*速度関連*/
    speed_now = ((float)diff / 16384.0f) / clock_time * 60.0f; //rpmの計算
    filtered_speed = filtered_speed * 0.7 + speed_now * 0.3;
    motor[0].speed = filtered_speed;

    float angle_mech = ((float)angle_raw / 16384.0f) * 2.0f * M_PI;  //360度の角度に変更
    //electrical_direction = (angle_mech - zero_offset_rad) * POLE_PAIRS;
    static float motor_direction = 1.0f; 
    
    electrical_direction = (angle_mech - zero_offset_rad) * POLE_PAIRS * motor_direction;
    
    float vd;
    float vq;
    /*速度制御*/
    if (control_motor_mode[0] == 0) {

      /*座標返還*/
      vd = 0.0f;
      vq = voltage;
                              
    }

    /*電流制御*/
    if (control_motor_mode[0] == 1){
        vd = locate_pid(motor[0].current_d, motor[0].current_d_target, motor[0].current_d_pgain, motor[0].current_d_igain, motor[0].current_d_dgain, &motor[0].current_d_different_sum, &motor[0].current_d_low_pass_different_sum, &motor[0].current_d_last_difference, cutoff);
        vq = locate_pid(motor[0].current_p, motor[0].current_p_target, motor[0].current_p_pgain, motor[0].current_p_igain, motor[0].current_p_dgain, &motor[0].current_p_different_sum, &motor[0].current_p_low_pass_different_sum, &motor[0].current_p_last_difference, cutoff);

        if (vd > 0.3f)  vd = 0.3f;
        if (vd < -0.3f) vd = -0.3f;
        if (vq > 0.3f)  vq = 0.3f;
        if (vq < -0.3f) vq = -0.3f;

    }
    
    float va = vd * fast_cos(electrical_direction) - vq * fast_sin(electrical_direction);
    float vb = vd * fast_sin(electrical_direction) + vq * fast_cos(electrical_direction);

    float u = va;
    float v = -0.5f * va + 0.866f * vb;
    float w = -0.5f * va - 0.866f * vb;

    float offset = 0.5f;
    set_pwm(u + offset, v + offset, w + offset);
    
    /*意味ない*/
    electrical_direction += step_move;
    if (electrical_direction > 2.0f * M_PI) electrical_direction -= 2.0f * M_PI;
    /*ここまで*/

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    
    if (htim->Instance == TIM2)
    {
        tim2_cnt++;
        /*意味ないここから(この部分があるからとはいえプログラムに変化はない)*/
        static uint32_t cnt = 0;
        
        if (++cnt >= 5000) {
          cnt = 0;
        }
       
        if (step_move < 0.010f) step_move +=  0.00001f;  // 速度
        if (amp < 0.05f) amp += 0.0000001f;               // トルク
        /*ここまで*/

        /*最初電流差でがたがたいうので速度目標を少しずつ上げることで回避したい*/
        if (motor[0].now_speed_target < motor[0].speed_target) {

          motor[0].now_speed_target += 0.05f;

          if (motor[0].now_speed_target > motor[0].speed_target) {
            motor[0].now_speed_target = motor[0].speed_target;
          }

        }
        else if (motor[0].now_speed_target > motor[0].speed_target) {
          motor[0].now_speed_target -= 0.05f;
          if (motor[0].now_speed_target < motor[0].speed_target) {
            motor[0].now_speed_target = motor[0].speed_target;
          }
        }
        
        float voltage_out = 0.0f;  // ← スコープを外に出す

        if (pid_mode[0] == 0) {
          voltage_out = locate_pid(motor[0].speed, motor[0].now_speed_target,
          motor[0].p, motor[0].i, motor[0].d,
          &motor[0].different_sum, &motor[0].low_pass_different_sum,
          &motor[0].last_difference, cutoff);
        }

        if (pid_mode[0] == 1) {
          voltage_out = speed_pid(
            motor[0].speed,
            motor[0].now_speed_target,
            motor[0].p,
            motor[0].i,
            motor[0].d,
            &motor[0].low_pass_different_sum,
            &motor[0].last_difference,
            &motor[0].last_last_difference,
            motor[0].last_input,
            &motor[0].low_pass_derivative,
            cutoff
    );
    motor[0].last_input = voltage_out;
        }

        // locate_pidとspeed_pidの出力は-16384〜16384なので0〜0.3にスケール変換
        float voltage = voltage_out / 16384.0f * 0.3f;
        

        update_openloop(voltage);  

        //measure_current();   //電流測定
    }
}

/*pid(速度型ではない方)*/
float locate_pid(volatile float output, float target, float p, float i, float d, volatile float *different_sum, volatile float *low_pass_different_sum, volatile float *last_difference, int cutoff) {

  float input;
  float difference;
  float derivarate;

  difference = target - output;

  *different_sum += difference * clock_time;
  if (*different_sum > 1000) {
    *different_sum = 1000;
  }
  if (*different_sum < -1000) {
    *different_sum = -1000;
  }

  derivarate = difference - *last_difference;
  *low_pass_different_sum += (derivarate - *low_pass_different_sum) / (float)cutoff;  //ローパス
  
  if (*low_pass_different_sum > 1000.0) {
    *low_pass_different_sum = 1000;
  }
  if (*low_pass_different_sum < -1000.0) {
    *low_pass_different_sum = -1000.0;
  }

  input = p * difference + i * (*different_sum) + d * (*low_pass_different_sum);

  *last_difference = difference;

  if (input >= 16384) {
    input = 16384;
  }
  if (input <= -16384) {
    input = -16384;
  }

  return input;
  }

/*pid速度型*/
float speed_pid(volatile float output, float target, float p, float i, float d, volatile float *low_pass_different_sum, volatile float *last_difference, volatile float *last_last_difference, volatile float last_input, volatile float *low_pass_derivative, int cutoff) {
  float input;
  float difference;
  float derivarate;

  difference = target - output;
  derivarate = difference - 2 * *last_difference + *last_last_difference;
  *low_pass_derivative += (derivarate - *low_pass_derivative) / (float)cutoff;  //ローパス

  if (*low_pass_derivative > 1000.0) {
    *low_pass_derivative = 1000;
  }
  if (*low_pass_derivative < -1000.0) {
    *low_pass_derivative = -1000.0;
  }

  input = p * (difference - *last_difference) + i * (difference * clock_time) + d * *low_pass_derivative / clock_time;
  input += last_input;

  if (input >= 16384) {
    input = 16384;
  }
  if (input <= -16384) {
    input = -16384;
  }

  *last_last_difference = *last_difference;
  *last_difference = difference;

  return input;

}

void measure_current(void) {

  // ADCのInjected変換結果  生データ: 0〜4095を取得
  uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  //uint32_t raw_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  uint32_t raw_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

  float magnification_conversion = (drive_voltage / 4096.0f) / (opamp_gain * resistance_for_current);    //生データを実際の電流値にする変数と計算

  current_u = ((float)raw_u - (float)offset_u) * magnification_conversion;
  current_w = ((float)raw_w - (float)offset_w) * magnification_conversion;
  current_v = -(current_u + current_w);   //キルヒホッフの法則

  // Clarke変換 (u,v,w → alpha,beta)
  float i_alpha = current_u;
  float i_beta  = (current_u + 2.0f * current_v) * 0.57735f; // 1/sqrt(3)

  float id =  i_alpha * fast_cos(electrical_direction) + i_beta * fast_sin(electrical_direction);
  float iq = -i_alpha * fast_sin(electrical_direction) + i_beta * fast_cos(electrical_direction);

  motor[0].current_d = id;
  motor[0].current_p = iq; 
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {  //PWMのスイッチングタイミングとADCサンプリングが同期させてノイズを減らす? よくわからない
        measure_current();
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  printf("step1: peripherals init done\r\n");
  for(int i=0;i<3;i++){
    motor[i].speed=0.0;
    motor[i].speed_target=0.0;
    if (pid_mode[i] == 0) {
      //n2830
      //motor[i].p=37.0;
      //motor[i].i=2.0;
      //motor[i].d=1.4;
      //n5065
      if(control_motor_mode[i] == 0){
        motor[i].p=30;
        motor[i].i=1.7;
        motor[i].d=1.2;
      }
      if(control_motor_mode[i] == 1){
        motor[i].current_d_pgain = 0.0;
        motor[i].current_d_igain = 0.0;
        motor[i].current_d_dgain = 0.0;
        motor[i].current_p_pgain = 0.0;
        motor[i].current_p_igain = 0.0;
        motor[i].current_p_dgain = 0.0;
      }
    }
    if (pid_mode[i] == 1) {
      if(control_motor_mode[i] == 0){
      motor[i].p=1.0;
      motor[i].i=0.05;
      motor[i].d=0.0;
      }
      if(control_motor_mode[i] == 1){
        motor[i].current_d_pgain = 0.0;
        motor[i].current_d_igain = 0.0;
        motor[i].current_d_dgain = 0.0;
        motor[i].current_p_pgain = 0.0;
        motor[i].current_p_igain = 0.0;
        motor[i].current_p_dgain = 0.0;
      }
    }
    motor[i].now_speed_target = 0.0;
    motor[i].different_sum = 0.0;
    motor[i].low_pass_different_sum = 0.0;
    motor[i].last_difference = 0.0;
    motor[i].last_input = 0.0;
    motor[i].low_pass_derivative = 0.0;
    motor[i].last_last_difference = 0.0;
    motor[i].current_d = 0.0;
    motor[i].current_p = 0.0;
    motor[i].current_d_target = 0.0;
    motor[i].current_p_target = 0.0;
    motor[i].current_d_different_sum = 0.0;
    motor[i].current_d_low_pass_different_sum = 0.0;
    motor[i].current_d_last_difference = 0.0;      //currentは電流といういみ　現在ではない
    motor[i].current_p_different_sum = 0.0;
    motor[i].current_p_low_pass_different_sum = 0.0;
    motor[i].current_p_last_difference = 0.0; 
  }  
  cutoff = 8;

  printf("step2: motor struct init done\r\n");


  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
  //エンコーダ―起動
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // WAKEピンでドライバ起動
  HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  printf("step3: encoder + WAKE done\r\n");

  HAL_Delay(5);
  stspin_clear_faults();
  HAL_Delay(5);
  stspin_clear_faults();   // 念のため2回

  printf("step4: stspin faults cleared\r\n");

  // PWM出力開始
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  printf("step5: PWM started\r\n");

  //ADCキャリブレーション＆スタート
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  printf("step6: ADC calibrated\r\n");

  // OPAMPスタート
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  HAL_Delay(2);

  printf("step7: OPAMP started\r\n");

  //ADCキャリブレーション＆スタート
  // まずITなしでオフセット計測 オフセットを正確に行うため
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  printf("step8: injected ADC started\r\n");

  /*offsetを計算するための変数*/
  uint32_t sum_u = 0.0;
  uint32_t sum_v = 0.0;
  uint32_t sum_w = 0.0;

  for (int i = 0; i < 100; i++) {
    HAL_Delay(1); // 少し待ってからADCのInjected変換結果を取得
    sum_u += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    sum_v += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    sum_w += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  }

  offset_u = sum_u / 100;
  offset_v = sum_v / 100;
  offset_w = sum_w / 100;

  printf("Offset U:%lu, V:%lu, W:%lu\r\n", offset_u, offset_v, offset_w);
  printf("step9: offset measured U:%lu V:%lu W:%lu\r\n", offset_u, offset_v, offset_w);

  // オフセット計測後にIT版に切り替え ここから割り込みを行うことでoffsetにノイズがのることを防ぐ
  HAL_ADCEx_InjectedStop(&hadc1);
  HAL_ADCEx_InjectedStop(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  printf("step10: injected IT started\r\n");

  //FCO処理
  set_pwm(0.60f, 0.45f, 0.45f); // U相に電圧をかけてモータを「0度」に強制ロック
  HAL_Delay(500);             // 1秒待って完全に静止させる
  // その位置を「ゼロ点ズレ」として記憶
  zero_offset_rad = ((float)as5047p_read_angle() / 16384.0f) * 2.0f * M_PI; 
  set_pwm(0.5f, 0.5f, 0.5f);   // ロック解除

  printf("step11: FCO lock done, zero_offset_rad=%d\r\n", (int)(zero_offset_rad*1000));

  // TIM2でコントロールループ開始（割り込み）
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);

  HAL_TIM_Base_Start_IT(&htim2);
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("boot\r\n");
  printf("boot: control loop started\r\n");
  //uint16_t prev_angle_raw = as5047p_read_angle();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t fault_check_cnt = 0;
    if (++fault_check_cnt >= 5) {   // 200ms×5=1秒ごとくらいでもOK。もっと頻繁でも良い
        fault_check_cnt = 0;
        uint8_t st = stspin_clear_faults();
        if (st & 0x04) {
            printf("VDS protection tripped! cleared.\r\n");
        }
        if (st & 0x08) {
            printf("Device RESET detected! cleared.\r\n");
        }
    }

    
    fflush(stdout);
    HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //printf("speed=%d rpm, target=%d rpm\r\n",(int32_t)motor[0].speed,(int32_t)motor[0].speed_target);
    //uint32_t display_deg = (uint32_t)((float)angle_raw * 360.0f / 16384.0f);
    //HAL_Delay(200);

    // spi_transfer_16(0xFFFC | 0x4000);   //1回目のごみ
    // diag = spi_transfer_16(0xC000);  //0xc000でエラー確認
    // printf("DIAG=0x%04X ", diag);
    // angle_raw = as5047p_read_angle();   //角度取得
    // angle_deg = angle_raw * 360.0f / 16384.0f; 
    // printf("RAW=%u (%.1f deg) ", angle_raw, angle_deg);
    // int16_t diff = (int16_t)angle_raw - (int16_t)prev_angle_raw;
    // if (diff > 8192)  diff -= 16384; // 逆回転時の跨ぎ補正
    // if (diff < -8192) diff += 16384; // 正回転時の跨ぎ補正
    // prev_angle_raw = angle_raw;
    // float speed_rpm = ((float)diff / 16384.0f) / 0.2f * 60.0f;
    // printf("speed=%d rpm\r\n", (int32_t)speed_rpm);
    
    // fflush(stdout);
    // HAL_Delay(200);
    
    // すべて整数(%u や %d)で安全に出力
    //printf("RAW=%u (%u deg)\r\n", angle_raw, display_deg);
    // printf("spd=%d I_U=%d I_V=%d I_W=%d\n", 
    // (int)(motor[0].speed), 
    // (int)(current_u * 1000000),  // μA単位
    // (int)(current_v * 1000000), 
    // (int)(current_w * 1000000));
    // fflush(stdout);
    // HAL_Delay(200);
    if (++fault_check_cnt >= 5) {
        fault_check_cnt = 0;
        uint8_t st = stspin_clear_faults();
        if (st & 0x04) printf("VDS protection tripped! cleared.\r\n");
        if (st & 0x08) printf("Device RESET detected! cleared.\r\n");
    }

    /* ★ 追加: CAN受信状況を表示 */
    FDCAN_ProtocolStatusTypeDef pstatus;
    FDCAN_ErrorCountersTypeDef  ecounters;
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &pstatus);
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ecounters);
    printf("rx_ok=%lu BusOff=%d ErrPassive=%d TEC=%lu REC=%lu target=%d tim2_cnt=%d pid_mode=%d ctrl_mode=%d speed=%d angle=%u\r\n",
       rx_ok_count, pstatus.BusOff, pstatus.ErrorPassive,
       (unsigned long)ecounters.TxErrorCnt, (unsigned long)ecounters.RxErrorCnt,
       (int)motor[0].speed_target, tim2_cnt,
       pid_mode[0], control_motor_mode[0],
       (int)motor[0].speed, angle_raw);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN1_ConfigFilterAndStart();
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30D29DE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 64;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  
  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 99;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 15999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 159;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWM_LED_GPIO_Port, PWM_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_SW_Pin */
  GPIO_InitStruct.Pin = USER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : k_Pin */
  GPIO_InitStruct.Pin = k_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(k_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WAKE_Pin */
  GPIO_InitStruct.Pin = WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWM_LED_Pin */
  GPIO_InitStruct.Pin = PWM_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWM_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
