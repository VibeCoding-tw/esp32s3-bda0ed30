#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <esp_ota_ops.h>      // For OTA partition functions
#include <esp_partition.h>    // For finding partitions
#include <esp_heap_caps.h>    // For PSRAM heap functions
#include <esp_camera.h>       // For Camera support
#include "gpio_pins.h"        // 引入腳位定義檔 (motorA_pwm_fwd, motor_stby, CAM_PIN_*)

// === 全域設定與連線狀態 ===
// OTA & Web Services
WebSocketsServer webSocket(81);
AsyncWebServer server(80);

// --- 馬達驅動晶片 GPIO 設定 ---
// 腳位由 gpio_pins.h 提供: motorA_pwm_fwd, motorA_pwm_rev, motorB_pwm_left, motorB_pwm_right, motor_stby

// LEDC Channel for PWM (與相機 XCLK 分開使用)
const int CH_A_FWD = 0;
const int CH_A_REV = 1;
const int CH_B_LEFT = 2;
const int CH_B_RIGHT = 3;
const int CAMERA_XCLK_CHANNEL = 4; // 相機 XCLK 建議使用獨立通道
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES = 8;      // 8-bit, 0-255 duty cycle

// 馬達控制變數
const int MAX_DUTY = 200; // 255 (Max) * 0.78 = 200, 限制最大速度以保護馬達
volatile int targetA = 0; // Motor A (前後)
volatile int targetB = 0; // Motor B (左右/轉向)
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 300; // 300ms 沒收到命令則停止

// === 應用程式模式 ===
enum DriveMode { AUTO, MANUAL };
DriveMode currentMode = MANUAL;

// ----------------------------------------------------------------------
// I. 遠端日誌 (Remote Logging)
// ----------------------------------------------------------------------

// 提供統一的日誌輸出通道 (Serial & WebSocket)
void sendLogMessage(const String& message) {
  Serial.println(message);
  // 將日誌訊息廣播給所有已連線的瀏覽器客戶端
  webSocket.broadcastTXT(message.c_str(), message.length());
}

// ----------------------------------------------------------------------
// II. 連線失敗機制 (Connection Fallback)
// ----------------------------------------------------------------------

// 連線超時時，跳轉回 Factory 分區的 Launcher App
void jumpToFactory() {
  sendLogMessage("--- WiFi connection failed. JUMPING TO FACTORY PARTITION (Launcher App) ---");

  // 1. 尋找 Factory 分區
  const esp_partition_t* factory = esp_partition_find_first(
      ESP_PARTITION_TYPE_APP,
      ESP_PARTITION_SUBTYPE_APP_FACTORY,
      NULL);
  
  if (factory != NULL) {
      // 2. 設定 Factory 分區為下一次啟動的目標
      esp_err_t err = esp_ota_set_boot_partition(factory);
      if (err == ESP_OK) {
          sendLogMessage("Successfully set Factory partition as next boot target. Rebooting...");
          delay(500); 
          ESP.restart(); // 重新啟動
      } else {
          sendLogMessage("Error setting boot partition! (" + String(esp_err_to_name(err)) + ") Rebooting anyway...");
          delay(2000);
          ESP.restart();
      }
  } else {
      sendLogMessage("FATAL: Factory partition not found! Rebooting...");
      delay(2000);
      ESP.restart();
  }
}

// ----------------------------------------------------------------------
// III. 網路連線 (Network Connection)
// ----------------------------------------------------------------------

// 嘗試連線到由 Launcher App 儲存的 Wi-Fi 網路
void connectToWiFi() {
  const unsigned long CONNECT_TIMEOUT_MS = 15000; // 15秒超時
  unsigned long connectStart = millis();

  sendLogMessage("Setting WiFi mode to Station and connecting with stored credentials...");
  WiFi.mode(WIFI_STA);
  // WiFi.begin() 會使用 NVS 中儲存的憑證 (由 Launcher App 配網成功後儲存)
  WiFi.begin();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    
    if (millis() - connectStart > CONNECT_TIMEOUT_MS) {
      sendLogMessage("WiFi connection timed out.");
      jumpToFactory(); // 超時，回退到 Launcher App
      return; 
    }
    
    // 輸出等待日誌
    if (WiFi.status() == WL_DISCONNECTED) {
        Serial.println("...Waiting for WiFi connection (Status: Disconnected)");
    } else {
        Serial.println("...Waiting for WiFi connection (Status: " + String(WiFi.status()) + ")");
    }
  }
  
  // 連線成功
  sendLogMessage("WiFi Connected! IP Address: " + WiFi.localIP().toString());
}

// ----------------------------------------------------------------------
// IV. 相機初始化與串流 (Camera Init & Stream)
// ----------------------------------------------------------------------

/**
 * @brief 嘗試以指定配置初始化相機。
 * @param config 相機配置結構
 * @return true: 初始化成功, false: 初始化失敗
 */
bool attemptCameraInit(camera_config_t& config) {
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
        sendLogMessage("Camera Init SUCCESS! Using: " + String(config.fb_count) + " buffers, Size: " + String(config.frame_size));
        sensor_t * s = esp_camera_sensor_get();
        if (s) {
            // 預設設定：可以根據需要調整
            s->set_vflip(s, 1);       // 垂直翻轉 (多數相機模組需要)
            s->set_hmirror(s, 1);    // 水平鏡像
            s->set_framesize(s, config.frame_size); // 確保設定生效
        }
        return true;
    } else {
        sendLogMessage("Camera Init FAILED (0x" + String(err, HEX) + ") with buffers=" + String(config.fb_count) + ", size=" + String(config.frame_size));
        // 如果失敗，釋放資源 (雖然 esp_camera_init 失敗時不一定會分配資源，但這是一個良好的習慣)
        esp_camera_deinit();
        return false;
    }
}

/**
 * @brief 初始化並設定相機模組，採用安全級聯降級邏輯 (移植自 main.c 的 init_camera_safe)。
 * @return true: 初始化成功, false: 初始化失敗
 */
bool initCamera() {
    // 相機配置結構：作為嘗試的基礎
    camera_config_t config;
    
    // 腳位配置 (與舊版本相同，使用 gpio_pins.h 的定義)
    config.ledc_channel = (ledc_channel_t)CAMERA_XCLK_CHANNEL; 
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK; config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC; config.pin_href = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD; config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN; config.pin_reset = CAM_PIN_RESET;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.jpeg_quality = 12; // 預設品質
    
    // ====================================================================
    // 關鍵修訂：相機硬體重置與電源啟用序列 (增加穩定性)
    // ====================================================================

    // 1. 處理 PWDN (Power Down) 腳位：確保相機處於電源開啟狀態
    if (config.pin_pwdn != -1) {
        // PWDN 腳位通常需要設為 LOW (開啟) 才能啟動相機
        pinMode(config.pin_pwdn, OUTPUT);
        digitalWrite(config.pin_pwdn, LOW);
        delay(10); // 等待電源穩定
        sendLogMessage("PWDN pin set LOW (Power ON).");
    }

    // 2. 處理 RESET (重置) 腳位：執行一次硬體重置
    if (config.pin_reset != -1) {
        pinMode(config.pin_reset, OUTPUT);
        // 重置序列：LOW (重置) -> HIGH (釋放重置)
        digitalWrite(config.pin_reset, LOW);
        delay(50); // 重置脈衝
        digitalWrite(config.pin_reset, HIGH);
        delay(50); // 等待感應器喚醒
        sendLogMessage("RESET pin toggled for hard reset.");
    }
    
    // ====================================================================
    // 3. 安全級聯初始化邏輯 (移植自 main.c 的 init_camera_safe)
    // ====================================================================

    // 檢查是否有 PSRAM (SPIRAM)
    bool psram_available = psramFound();
    int initial_fb_count = psram_available ? 2 : 1;
    
    if (psram_available) {
        sendLogMessage("PSRAM detected, attempting 2 frame buffers.");
    } else {
        sendLogMessage("No PSRAM detected, attempting 1 frame buffer.");
    }

    // A. 嘗試最高品質/速度配置: QVGA (320x240) + PSRAM/內存緩衝區
    config.frame_size = FRAMESIZE_QVGA; 
    config.fb_count = initial_fb_count;
    if (attemptCameraInit(config)) return true;

    // B. 降級嘗試 1: QVGA (320x240) + 僅 1 緩衝區 (即使有 PSRAM，也可能在 2 個緩衝區時失敗)
    if (initial_fb_count == 2) {
        config.fb_count = 1;
        sendLogMessage("Downgrade: PSRAM 2-buffer failed. Trying 1 buffer at QVGA.");
        if (attemptCameraInit(config)) return true;
    }
    
    // C. 降級嘗試 2: 最小解析度 QQVGA (160x120) + 1 緩衝區
    config.frame_size = FRAMESIZE_QQVGA; 
    config.fb_count = 1;
    sendLogMessage("Downgrade: QVGA failed. Trying minimal size QQVGA.");
    if (attemptCameraInit(config)) return true;

    // 所有嘗試均失敗
    sendLogMessage("FATAL: All camera initialization attempts failed!");
    return false;
}

// MJPEG 串流處理器
void httpStreamHandler(AsyncWebServerRequest *request) {
    // 設置 MJPEG 串流的 HTTP 標頭
    
    // 建立一個 multipart/x-mixed-replace 格式的串流回應
    AsyncWebServerResponse *response = request->beginResponseStream("multipart/x-mixed-replace;boundary=123456789000000000000987654321");

    // 設置串流標頭
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Content-Disposition", "inline; filename=stream.jpg");
    response->addHeader("X-Content-Type-Options", "nosniff");
    request->send(response);
    
    // 串流迴圈：只要客戶端連線，就持續發送影像幀
    while (request->client()->connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            // 影像抓取失敗時，發送錯誤日誌並暫停
            sendLogMessage("Camera Frame Grab Failed! Retrying...");
            delay(50); // 稍微縮短等待時間
            continue;
        }

        // 構建 MJPEG part 標頭
        char header_buf[128];
        snprintf(header_buf, 128,
                 "--123456789000000000000987654321\r\n"
                 "Content-Type: image/jpeg\r\n"
                 "Content-Length: %u\r\n"
                 "\r\n",
                 fb->len);
                 
        // 發送標頭和影像資料
        request->client()->write(header_buf, strlen(header_buf));
        // 顯式轉換 fb->buf
        request->client()->write((const char*)fb->buf, fb->len); 
        request->client()->write("\r\n", 2);

        esp_camera_fb_return(fb);
        // 限制幀率，避免佔用過多 CPU
        delay(1); 
    }
}


// ----------------------------------------------------------------------
// V. 網路事件處理 (Network Event Handling)
// ----------------------------------------------------------------------

void emergencyStopNow() {
  targetA = targetB = 0;
  // 立即停止 PWM 輸出並禁用 STBY
  digitalWrite(motor_stby, LOW); // 禁用馬達
  ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0);
  ledcWrite(CH_B_LEFT, 0); ledcWrite(CH_B_RIGHT, 0);
  sendLogMessage("!!! EMERGENCY STOP Triggered !!!");
}

// 處理來自 WebSocket 客戶端的命令 (單字元或 JSON)
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: { 
        IPAddress ip = webSocket.remoteIP(num);
        sendLogMessage("--- WS Client Connected from " + ip.toString() + " ---");
      }
      break;
    case WStype_DISCONNECTED:
      sendLogMessage("--- WS Client Disconnected ---");
      // 斷線時立即停止馬達
      emergencyStopNow();
      break;
    case WStype_TEXT:
      {
        String msg = String((char*)payload);
        // V. 命令解析 (Command Parsing) - 單字元命令
        if (msg.length() == 1) {
          char cmd = msg.charAt(0);
          switch (cmd) {
            case 'A': 
              currentMode = AUTO; 
              sendLogMessage("Mode Switched: AUTO"); 
              break;
            case 'M': 
              currentMode = MANUAL; 
              sendLogMessage("Mode Switched: MANUAL"); 
              break;
            case 'S':
              emergencyStopNow();
              break;
          }
          lastCommandTime = millis(); 
        } else {
          // V. 命令解析 (Command Parsing) - JSON 遙控命令
          JsonDocument doc;
          DeserializationError err = deserializeJson(doc, (const char*)payload); 
          if (!err) {
            int steer = doc["steer"] | 0; // 轉向 (-100 ~ 100)
            int throttle = doc["throttle"] | 0; // 油門 (-100 ~ 100)
            
            // VI. 馬達控制 (Motor Control)
            if (currentMode == MANUAL) {
                // 更新目標速度
                targetA = throttle * MAX_DUTY / 100; // 前後
                targetB = steer * MAX_DUTY / 100;    // 左右
                
                // 應用速度並限制在 MAX_DUTY 範圍內
                int speedA = constrain(targetA, -MAX_DUTY, MAX_DUTY);
                int speedB = constrain(targetB, -MAX_DUTY, MAX_DUTY);

                // 如果有任何動作，則啟用馬達
                if (speedA != 0 || speedB != 0) {
                   digitalWrite(motor_stby, HIGH);
                } else {
                   // 停止時，持續維持 STBY LOW
                   digitalWrite(motor_stby, LOW); 
                }


                // 馬達 A: 前後
                if (speedA > 0) { ledcWrite(CH_A_FWD, speedA); ledcWrite(CH_A_REV, 0); } 
                else if (speedA < 0) { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, abs(speedA)); } 
                else { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0); }

                // 馬達 B: 轉向
                if (speedB > 0) { ledcWrite(CH_B_RIGHT, speedB); ledcWrite(CH_B_LEFT, 0); } // 右轉
                else if (speedB < 0) { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, abs(speedB)); } // 左轉
                else { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, 0); }

                // Reset timeout on every joystick command
                lastCommandTime = millis(); 
            }

            // 發送實時狀態回瀏覽器
            JsonDocument status;
            status["motorA"] = targetA;
            status["motorB"] = targetB;
            status["mode"] = currentMode == AUTO ? "AUTO" : "MANUAL";
            
            // 由於客戶端已經過濾了 (0, 0) 命令，此處的 debug 訊息只會在有動作時發送。
            status["debug"] = String("JSTK:") + throttle + "/" + steer + " | Mode:" + String(currentMode == AUTO ? "AUTO" : "MANUAL");
            
            size_t json_len = measureJson(status);
            char buffer[json_len + 1];
            size_t len = serializeJson(status, buffer, json_len + 1);
            webSocket.broadcastTXT(buffer, len);
            
          } else {
            sendLogMessage("WS Error: JSON parse failed: " + String(err.c_str()));
          }
        }
      }
      break;
    default:
      break;
  }
}

// ----------------------------------------------------------------------
// VII. 網頁服務 (Web Services)
// ----------------------------------------------------------------------

// 網頁前端 HTML 內容 (已更新影像串流 URL)
const char index_html[] = R"rawliteral(
<!doctype html>
<html lang="zh-TW">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Car Remote Control (User App)</title>
  <style>
    :root{--bg:#0b0d11;--card:#0f1720;--accent:#3b82f6;--muted:#98a2b3}
    html,body{height:100%;margin:0;background:linear-gradient(180deg,var(--bg),#071022);color:#e6eef6;font-family:Inter,system-ui,Segoe UI,Roboto,"Noto Sans TC",sans-serif}
    .app{display:grid;grid-template-columns:1fr;grid-template-rows:1fr;height:100vh;padding:12px;box-sizing:border-box;position:relative}
    .viewer{background:rgba(255,255,255,0.02);border-radius:12px;padding:0;position:relative;overflow:hidden;}
    /* FIX: 確保影像元素能夠佔滿容器並維持長寬比 */
    .videoFrame{width:100%;height:100%;object-fit:contain;background:#000} 
    .overlay{position:absolute;left:12px;top:12px;background:rgba(0,0,0,0.45);padding:6px 8px;border-radius:8px;font-size:13px;color:var(--muted);z-index:5}
    .controls{position:absolute;top:0;left:0;width:100%;height:100%;display:flex;justify-content:space-between;align-items:flex-end;pointer-events:none}
    .stick{width:120px;height:120px;border-radius:50%;background:rgba(255,255,255,0.15);display:grid;place-items:center;position:relative;pointer-events:auto; touch-action: none;}
    .base{width:70px;height:70px;border-radius:50%;background:rgba(255,255,255,0.05);border:2px dashed rgba(255,255,255,0.03);display:grid;place-items:center}
    .knob{width:40px;height:40px;border-radius:50%;background:linear-gradient(180deg,#fff,#cbd5e1);transform:translate(-50%,-50%);position:absolute;left:50%;top:50%;box-shadow:0 6px 18px rgba(2,6,23,0.6)}
    .value{font-size:12px;color:var(--muted);text-align:center;margin-top:4px}
    /* Mobile Layout Adjustments */
    @media (max-width: 600px) {
        .app { padding: 8px; }
        .stick { width: 90px; height: 90px; }
        .base { width: 50px; height: 50px; }
        .knob { width: 30px; height: 30px; }
        .controls { flex-direction: row; justify-content: space-around; align-items: flex-end; padding-bottom: 20px;}
    }
  </style>
</head>
<body>
  <div class="app">
    <div class="viewer">
      <!-- 影像串流在這裡顯示 -->
      <!-- 移除 onerror 中的 placeholder，避免不必要的請求，改用純色背景或簡單文字 -->
      <img id="video" class="videoFrame" alt="遠端影像" src="" style="background:#000; color:white; text-align:center; line-height:100vh;" />
      <div class="overlay">IP: <span id="imgSource">N/A</span> | WS: <span id="wsStatus">未連線</span></div>
      <div class="controls">
        <div style="margin:12px; display:flex; flex-direction:column; gap:8px;">
          <div class="stick" id="stickLeft" data-role="steer"><div class="base"></div><div class="knob" id="knobLeft"></div></div>
          <div class="value">方向: <span id="valSteer">0</span></div>
        </div>
        <div style="margin:12px; display:flex; flex-direction:column; gap:8px;">
          <div class="stick" id="stickRight" data-role="throttle"><div class="base"></div><div class="knob" id="knobRight"></div></div>
          <div class="value">油門: <span id="valThrottle">0</span></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    class VirtualStick {
      constructor(stickEl, knobEl, onChange){
        this.el = stickEl; this.knob = knobEl; this.cb = onChange; this.max = Math.min(stickEl.clientWidth, stickEl.clientHeight)/2 - 8;
        this.center = {x: this.el.clientWidth/2, y: this.el.clientHeight/2};
        this.pointerId = null; this.pos = {x:0,y:0}; this.deadzone = 6;
        this._bind();
      }
      _bind(){
        this.el.style.touchAction = 'none';
        this.el.addEventListener('pointerdown', e=>this._start(e));
        window.addEventListener('pointermove', e=>this._move(e));
        window.addEventListener('pointerup', e=>this._end(e));
        window.addEventListener('pointercancel', e=>this._end(e));
        // 確保 resize 時更新搖桿範圍
        window.addEventListener('resize', ()=>{this.center = {x:this.el.clientWidth/2,y:this.el.clientHeight/2};this.max = Math.min(this.el.clientWidth,this.el.clientHeight)/2 - 8});
      }
      _start(e){ if(this.pointerId!==null) return; this.pointerId = e.pointerId; this.el.setPointerCapture?.(e.pointerId); this._move(e); }
      _move(e){ if(this.pointerId===null || e.pointerId!==this.pointerId) return; const rect = this.el.getBoundingClientRect(); let x = e.clientX - rect.left - rect.width/2; let y = e.clientY - rect.top - rect.height/2; const d = Math.hypot(x,y); if(d>this.max){ const r = this.max/d; x*=r; y*=r; } this.pos = {x,y}; this.knob.style.left = (50 + (x/rect.width*100))+'%'; this.knob.style.top = (50 + (y/rect.height*100))+'%'; this._fire(); }
      _end(e){ if(this.pointerId===null || e.pointerId!==this.pointerId) return; this.pointerId=null; this.pos={x:0,y:0}; this.knob.style.left='50%'; this.knob.style.top='50%'; this._fire(); }
      _fire(){ const norm = {x: Math.abs(this.pos.x) < this.deadzone ? 0 : this.pos.x/this.max, y: Math.abs(this.pos.y) < this.deadzone ? 0 : this.pos.y/this.max}; if(this.cb) this.cb(norm); }
    }

    const wsStatusEl = document.getElementById('wsStatus');
    const valSteer = document.getElementById('valSteer');
    const valThrottle = document.getElementById('valThrottle');
    const stickL = document.getElementById('stickLeft');
    const stickR = document.getElementById('stickRight');
    const videoEl = document.getElementById('video');

    const state = {steer:0, throttle:0, ws:null, sendInterval:null, videoInterval:null, config:{videoUrl:'',videoFps:10,wsUrl:'',sendRate:50}};

    const left = new VirtualStick(stickL, document.getElementById('knobLeft'), n=>{ state.steer = Math.round(n.x*100); valSteer.textContent=state.steer; });
    const right = new VirtualStick(stickR, document.getElementById('knobRight'), n=>{ state.throttle = Math.round(-n.y*100); valThrottle.textContent=state.throttle; });
    
    // --- 日誌輔助函式: 輸出到瀏覽器 Console ---
    function appendLog(message) {
        const timestamp = new Date().toLocaleTimeString('en-US', {hour12: false});
        console.log(`[ESP32 LOG] [${timestamp}] ${message}`); 
    }
    // ----------------------

    function connectWs(){ 
        if(state.ws){ try{state.ws.close()}catch(e){} state.ws=null; } 
        const wsUrl = `ws://${window.location.hostname}:81`;
        
        appendLog(`嘗試連線到 WebSocket: ${wsUrl}`);
        wsStatusEl.textContent = 'Connecting...';

        try{ 
            state.ws = new WebSocket(wsUrl); 
            state.ws.binaryType='arraybuffer'; 
            
            state.ws.onopen=()=>{
                wsStatusEl.textContent = 'OPEN';
                appendLog('WebSocket 連線成功。');
            }; 
            
            state.ws.onclose=()=>{
                wsStatusEl.textContent = 'CLOSED';
                appendLog('WebSocket 已斷線，3秒後重試連線...');
                setTimeout(connectWs, 3000); // 重試連線
            }; 
            
            state.ws.onerror=()=>{
                wsStatusEl.textContent = 'ERROR';
                appendLog('WebSocket 連線錯誤。');
                // 錯誤時也嘗試重連
                setTimeout(connectWs, 5000); 
            }; 
            
            state.ws.onmessage = (event) => {
                const data = event.data;
                
                // 嘗試解析 JSON (控制狀態/遠端日誌)
                try {
                    const json = JSON.parse(data);
                    if (json.debug) {
                        // 這是來自 ESP32 的遠端日誌 (JSON 格式)
                        appendLog(json.debug);
                    } else if (json.motorA !== undefined) {
                        // 馬達狀態更新 (可選)
                        // appendLog(`Motor A:${json.motorA}, B:${json.motorB}`);
                    }
                } catch(e) {
                    // 如果不是 JSON，則視為遠端日誌文本
                    appendLog(data);
                }
            };
        }catch(e){ 
            wsStatusEl.textContent = 'ERROR'; 
            appendLog(`WebSocket 建立失敗: ${e.message}`);
        } 
    }

    // 發送搖桿命令到 WebSocket
    function startSending(rate){ 
      if(state.sendInterval) clearInterval(state.sendInterval); 
      state.sendInterval=setInterval(()=>{ 
        if(state.ws && state.ws.readyState===WebSocket.OPEN){ 
          // 新增檢查: 只有當 throttle 或 steer 不為 0 時才發送命令 (或者目標速度改變時)
          if(state.throttle !== 0 || state.steer !== 0){
            // t: timestamp, steer: 轉向 (-100 to 100), throttle: 油門 (-100 to 100)
            state.ws.send(JSON.stringify({t:Date.now(),steer:state.steer,throttle:state.throttle})); 
          }
        } 
      }, rate); 
    }
    
    // 影像串流處理 (直接指向 M-JPEG 串流)
    function startVideoPoll(){ 
        state.config.videoUrl = 'http://' + window.location.hostname + '/stream';
        videoEl.src = state.config.videoUrl;
        document.getElementById('imgSource').textContent = window.location.hostname + '/stream';
        appendLog("影像串流已啟動: " + state.config.videoUrl);
        
        // 註冊影像載入失敗事件，用於調試
        videoEl.onerror = () => {
            appendLog("WARNING: Video stream connection error (M-JPEG stream may be down or initial load failed).");
            // 由於 M-JPEG 是持續連線，一旦連線中斷，通常需要重新設定 src 才能重新連線
            // 但過於頻繁的重試會造成伺服器負擔，因此只在頁面載入時啟動一次。
            // 保持 src 不變，讓瀏覽器嘗試自動重連。
        };
    }

    window.addEventListener('beforeunload', ()=>{ if(state.ws) state.ws.close(); clearInterval(state.sendInterval); });
    
    window.onload = () => {
        connectWs();
        startSending(50); // 每 50ms 發送一次控制命令
        startVideoPoll(); // 啟動影像串流
    };
  </script>
</body>
</html>
)rawliteral";

// 設置 HTTP Server 和 WebSocket
void setupWebServer() {
  String hostname = "esp32car-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  
  if (MDNS.begin(hostname.c_str())) {
    Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
  } else {
    sendLogMessage("Error setting up mDNS!");
  }

  // Handle favicon.ico request (防止 404 錯誤)
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204); 
  });

  // 根目錄提供遙控網頁
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });
  
  // 相機串流端點
  server.on("/stream", HTTP_GET, httpStreamHandler);


  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  sendLogMessage("Web UI Ready on port 80. Remote Control Active at http://" + WiFi.localIP().toString());
}

// ----------------------------------------------------------------------
// VIII. OTA 服務 (Over-The-Air Update)
// ----------------------------------------------------------------------

void setupOTA() {
  String hostname = "esp32car-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  
  // 設定 OTA 參數
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.setPassword("mysecurepassword"); // 替換為您的密碼
  
  // OTA 事件處理
  ArduinoOTA.onStart([]() { sendLogMessage("OTA: Start updating " + String(ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem")); });
  ArduinoOTA.onEnd([]() { sendLogMessage("OTA: Update Finished. Rebooting..."); });
  ArduinoOTA.onError([](ota_error_t error) { sendLogMessage("OTA Error: " + String(error)); });

  ArduinoOTA.begin();
  sendLogMessage("OTA Ready. Hostname: " + hostname + ".local");
}

// ----------------------------------------------------------------------
// IX. 馬達初始化 (Motor Initialization)
// ----------------------------------------------------------------------

void setupPWM() {
  // 設置 LEDC 通道頻率與解析度
  ledcSetup(CH_A_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(CH_A_REV, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B_RIGHT, PWM_FREQ, PWM_RES);

  // 將 LEDC 通道連接到 GPIO 引腳 (使用 gpio_pins.h 的定義)
  ledcAttachPin(motorA_pwm_fwd, CH_A_FWD);
  ledcAttachPin(motorA_pwm_rev, CH_A_REV);
  ledcAttachPin(motorB_pwm_left, CH_B_LEFT);
  ledcAttachPin(motorB_pwm_right, CH_B_RIGHT);
}


// ----------------------------------------------------------------------
// X. PSRAM 外部記憶體檢查 (PSRAM External Memory Check)
// ----------------------------------------------------------------------

/**
 * @brief 僅檢查並輸出 PSRAM 大小，避免長時間的 R/W 迴圈導致看門狗超時。
 */
void psramTest() {
  sendLogMessage("--- X. PSRAM External Memory Check ---");

  // 使用 psramFound() 判斷 PSRAM 是否被核心成功初始化
  if (!psramFound()) {
    sendLogMessage("PSRAM: Not detected or core initialization failed.");
    return;
  }
  
  size_t totalPsram = ESP.getPsramSize();
  size_t freePsram = ESP.getFreePsram();
  
  sendLogMessage("PSRAM Check: Success");
  sendLogMessage("Total PSRAM size: " + String(totalPsram / 1024) + " KB");
  sendLogMessage("Free PSRAM (SPIRAM): " + String(freePsram / 1024) + " KB");
}

// ----------------------------------------------------------------------
// 程式進入點 (Arduino setup/loop)
// ----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000); 

  // X. PSRAM 外部記憶體檢查
  // **關鍵修復：移除 psramInit()，依賴核心初始化**
  //psramTest(); 
  if (psramFound()) {
    Serial.println("✅ PSRAM detected and initialized!");
  } else {
    Serial.println("❌ No PSRAM detected.");
  }

  // XI. 馬達開關 (Motor Enable)
  // 使用 gpio_pins.h 中的 motor_stby
  pinMode(motor_stby, OUTPUT);
  digitalWrite(motor_stby, LOW); // 預設禁用馬達

  // IX. 馬達初始化 (Motor Initialization)
  setupPWM();

  // II. 網路連線 (Network Connection) - 需有 Launcher App 儲存的憑證
  connectToWiFi();

  // IV. 相機初始化 (Camera Initialization) - 在連線後執行
  // 已經更新為更穩定的級聯初始化邏輯
  if (initCamera()) {
      sendLogMessage("Camera initialization successful! Stream available at /stream");
  } else {
      sendLogMessage("!!! FATAL: Camera initialization failed! Cannot start video stream. !!!");
  }

  // VIII. OTA 服務 (Over-The-Air Update)
  setupOTA();

  // VII. 網頁服務 (Web Services)
  setupWebServer();
  
  sendLogMessage("User App setup complete. Ready to receive commands.");
}


void loop() {
  // 保持 OTA 服務運行
  ArduinoOTA.handle();
  // 保持 WebSocket 服務運行
  webSocket.loop();
  
  // 馬達命令超時邏輯：若超過 COMMAND_TIMEOUT 且馬達正在運行，則停止所有馬達
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (targetA != 0 || targetB != 0) { 
      // 僅在目標速度為非零時才發送停止日誌
      sendLogMessage("Motors stopped due to command timeout.");
      targetA = targetB = 0; // 重設目標速度
      digitalWrite(motor_stby, LOW); // 禁用馬達
    }
  }

  // 心跳日誌
  static unsigned long lastLogMillis = 0;
  if (millis() - lastLogMillis > 50000) { 
    // 輸出當前 PSRAM 狀態
    size_t freePsram = psramFound() ? heap_caps_get_free_size(MALLOC_CAP_SPIRAM) : 0;
    sendLogMessage("Heartbeat: Car system active, Mode=" + String(currentMode == AUTO ? "AUTO" : "MANUAL") + 
                   " | Free PSRAM: " + String(freePsram / 1024) + " KB");
    lastLogMillis = millis();
  }
}
