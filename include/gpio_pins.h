// --- 相機腳位定義 (ESP32-S3 Board) ---
// 這些腳位通常是某些 S3-CAM 模組的標準配置，請檢查您的模組是否匹配
#define CAM_PIN_SIOD  4
#define CAM_PIN_SIOC  5
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF  7
#define CAM_PIN_PCLK  13
#define CAM_PIN_XCLK  15
#define CAM_PIN_D7    16
#define CAM_PIN_D6    17
#define CAM_PIN_D5    18
#define CAM_PIN_D4    12
#define CAM_PIN_D3    10
#define CAM_PIN_D2    8
#define CAM_PIN_D1    9
#define CAM_PIN_D0    11
#define CAM_PIN_RESET -1 // 復位腳位，若無連接請設為 -1
#define CAM_PIN_PWDN  -1 // 待機腳位，若無連接請設為 -1

// --- 馬達驅動晶片 GPIO 設定 (Motor H-Bridge DRV8833) ---
const int motorA_pwm_fwd = 44;
const int motorA_pwm_rev = 43;
const int motorB_pwm_left  = 42;
const int motorB_pwm_right = 41;
const int motor_stby = 40; // 馬達啟用/禁用

