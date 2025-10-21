#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>      // Includes AsyncWebServerResponse
#include <esp_ota_ops.h>      // For OTA partition functions
#include <esp_partition.h>    // For finding partitions

// 引入 ESP32 Camera 函式庫
#include "esp_camera.h"
#include "esp_system.h" // Needed for ESP.getPsramSize() / ESP.getFreePsram()
#include "driver/ledc.h"

// === Camera Pin Definitions (UPDATED FROM camera_pins.h) ===
// 警告：請檢查這些腳位是否與您的硬體實際連接匹配！
struct CameraPins {
    int PWDN = -1;  // CAM_PIN_PWDN
    int RESET = -1; // CAM_PIN_RESET
    int XCLK = 15;  // CAM_PIN_XCLK
    int SIOD = 4;   // CAM_PIN_SIOD (SDA)
    int SIOC = 5;   // CAM_PIN_SIOC (SCL)
    int Y9 = 16;    // CAM_PIN_D7
    int Y8 = 17;    // CAM_PIN_D6
    int Y7 = 18;    // CAM_PIN_D5
    int Y6 = 12;    // CAM_PIN_D4
    int Y5 = 10;    // CAM_PIN_D3
    int Y4 = 8;     // CAM_PIN_D2
    int Y3 = 9;     // CAM_PIN_D1
    int Y2 = 11;    // CAM_PIN_D0
    int VSYNC = 6;  // CAM_PIN_VSYNC
    int HREF = 7;   // CAM_PIN_HREF
    int PCLK = 13;  // CAM_PIN_PCLK
} CAMERA_PINS;


// === 全域設定與連線狀態 ===
// OTA & Web Services
WebSocketsServer webSocket(81);
AsyncWebServer server(80);

// --- 馬達驅動晶片 GPIO 設定 (UPDATED FROM USER INPUT) ---
const int motorA_pwm_fwd = 44; // 新腳位
const int motorA_pwm_rev = 43; // 新腳位
const int motorB_pwm_left  = 20;
const int motorB_pwm_right = 21;
const int motor_stby = 40; // 新腳位 (馬達啟用/禁用)

// LEDC Channel for PWM
const int CH_A_FWD = 0;
const int CH_A_REV = 1;
const int CH_B_LEFT = 2;
const int CH_B_RIGHT = 3;
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES = 8;      // 8-bit, 0-255 duty cycle

// 馬達控制變數
const int MAX_DUTY = 200; 
volatile int targetA = 0; 
volatile int targetB = 0;
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
// IV. 攝影機初始化 (Camera Initialization)
// ----------------------------------------------------------------------

/**
 * @brief 初始化攝影機模組。
 */
esp_err_t initCamera() {
    // 1. PSRAM 檢查
    if (ESP.getPsramSize() == 0) {
        sendLogMessage("PSRAM not found. Camera operations might be limited.");
    } else {
        sendLogMessage("PSRAM found: " + String(ESP.getPsramSize()) + " bytes.");
    }

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    
    // 2. LEDC 定時器設定
    config.ledc_timer = LEDC_TIMER_3;
    
    // 數據腳位 (D0-D7) - 使用 Y2-Y9 命名對應 D0-D7
    config.pin_d0 = CAMERA_PINS.Y2; // D0
    config.pin_d1 = CAMERA_PINS.Y3; // D1
    config.pin_d2 = CAMERA_PINS.Y4; // D2
    config.pin_d3 = CAMERA_PINS.Y5; // D3
    config.pin_d4 = CAMERA_PINS.Y6; // D4
    config.pin_d5 = CAMERA_PINS.Y7; // D5
    config.pin_d6 = CAMERA_PINS.Y8; // D6
    config.pin_d7 = CAMERA_PINS.Y9; // D7

    // 同步腳位
    config.pin_vsync = CAMERA_PINS.VSYNC;
    config.pin_href = CAMERA_PINS.HREF;
    config.pin_pclk = CAMERA_PINS.PCLK;

    // 功率/重置腳位
    config.pin_reset = CAMERA_PINS.RESET;
    config.pin_pwdn = CAMERA_PINS.PWDN;

    // SCCB (I2C) 腳位
    config.pin_sccb_sda = CAMERA_PINS.SIOD;
    config.pin_sccb_scl = CAMERA_PINS.SIOC;

    // XCLK 設置
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // 設置幀緩衝區和大小
    config.frame_size = FRAMESIZE_VGA; // 640x480 (適合串流)
    config.jpeg_quality = 12; // 0-63, 數字越小品質越高
    config.fb_count = 2; // 使用 2 個幀緩衝區 (需要足夠 PSRAM)
    config.grab_mode = CAMERA_GRAB_LATEST; 

    // 初始化攝影機
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        sendLogMessage("Camera init failed with error: " + String(esp_err_to_name(err)));
        return err;
    }
    
    // 應用基本設定
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1); // 垂直翻轉
        s->set_hmirror(s, 1); // 水平鏡像
    }

    return ESP_OK;
}


// ----------------------------------------------------------------------
// V. 網路事件處理 (Network Event Handling)
// ----------------------------------------------------------------------

void emergencyStopNow() {
  targetA = targetB = 0;
  // immediately stop PWM and disable STBY pin
  digitalWrite(motor_stby, LOW);
  ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0);
  ledcWrite(CH_B_LEFT, 0); ledcWrite(CH_B_RIGHT, 0);
  sendLogMessage("!!! EMERGENCY STOP Triggered !!!");
}

// 處理來自 WebSocket 客戶端的命令
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
        // 命令解析 (Command Parsing) - 單字元命令
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
          // 命令解析 (Command Parsing) - JSON 遙控命令
          JsonDocument doc;
          DeserializationError err = deserializeJson(doc, (const char*)payload); 
          if (!err) {
            int steer = doc["steer"] | 0;
            int throttle = doc["throttle"] | 0;
            
            // 馬達控制 (Motor Control)
            if (currentMode == MANUAL) {
                targetA = throttle; // 前後
                targetB = steer;    // 左右
                
                digitalWrite(motor_stby, HIGH);
                
                int speedA = constrain(targetA, -MAX_DUTY, MAX_DUTY);
                int speedB = constrain(targetB, -MAX_DUTY, MAX_DUTY);

                if (speedA > 0) { ledcWrite(CH_A_FWD, speedA); ledcWrite(CH_A_REV, 0); } 
                else if (speedA < 0) { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, abs(speedA)); } 
                else { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0); }

                if (speedB > 0) { ledcWrite(CH_B_RIGHT, speedB); ledcWrite(CH_B_LEFT, 0); } 
                else if (speedB < 0) { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, abs(speedB)); } 
                else { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, 0); }

                lastCommandTime = millis(); 
            }

            // 發送實時狀態回瀏覽器 (Console log)
            JsonDocument status;
            status["motorA"] = targetA;
            status["motorB"] = targetB;
            status["debug"] = String("JSTK:") + throttle + "/" + steer + " | Mode:" + String(currentMode == AUTO ? "AUTO" : "MANUAL") + 
                              " | PSRAM Free: " + String(ESP.getFreePsram() / 1024) + " KB"; // 修正 PSRAM 函式
            
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
// VI. MJPEG 串流服務 (MJPEG Streaming Service)
// ----------------------------------------------------------------------

// 這是 AsyncWebServer 實作 MJPEG 串流的關鍵類別
class MJPEGStreamResponse: public AsyncWebServerResponse {
    private:
        // MJPEG Boundary 字符串
        const char* _boundary = "frameboundary"; 
        uint32_t _frameStartTime = 0;
        size_t _totalLength = 0; // 追蹤已發送的總字節數

    public:
        // *** FIX 2: 使用 default constructor 並在 body 內設定 protected 成員 ***
        MJPEGStreamResponse() : AsyncWebServerResponse() {
            // 初始化繼承的 protected 成員
            _contentType = "multipart/x-mixed-replace; boundary=frameboundary";
            _code = 200;
            _contentLength = 0;
            _chunked = true; // 使用分塊傳輸編碼 (關鍵)
        }
        
        // 此函數是 Streaming 響應的核心
        size_t _sendContent(AsyncWebServerRequest* request, size_t index, size_t available) {
            // Obtain the frame buffer
            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Camera capture failed or out of memory!");
                return index; // Allow the framework to retry or timeout later
            }

            // --- Frame image HTTP Header ---
            String header = 
                String("Content-Type: image/jpeg\r\n") +
                "Content-Length: " + String(fb->len) + "\r\n\r\n";

            // --- Write the image header ---
            request->sendChunked("multipart/x-mixed-replace; boundary=frame", [header, fb](uint8_t* buffer, size_t maxLen, size_t index) -> size_t {
                if (index < header.length()) {
                    size_t toCopy = std::min(maxLen, header.length() - index);
                    memcpy(buffer, header.c_str() + index, toCopy);
                    return toCopy;
                } else if (index < header.length() + fb->len) {
                    size_t dataIndex = index - header.length();
                    size_t toCopy = std::min(maxLen, fb->len - dataIndex);
                    memcpy(buffer, fb->buf + dataIndex, toCopy);
                    return toCopy;
                }
                return 0; // No more data to send
            });

            // Return the total bytes sent
            size_t totalBytes = index + header.length() + fb->len;

            // Return the frame buffer to the driver
            esp_camera_fb_return(fb);

            return totalBytes;
        }

};

// MJPEG 串流路徑處理函式
void handleJPEGSStream(AsyncWebServerRequest *request) {
    // 檢查攝影機是否初始化成功
    if (esp_camera_fb_get() == NULL) {
        request->send(503, "text/plain", "Camera not initialized or out of memory.");
        return;
    }
    // 傳送自定義的串流響應對象
    request->send(new MJPEGStreamResponse());
}

// ----------------------------------------------------------------------
// VII. 網頁服務 (Web Services)
// ----------------------------------------------------------------------

// 網頁前端 HTML 內容 (已移除影像輪詢，改為直接串流)
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
    /* 影像來源指向 /stream 實現 MJPEG 串流 */
    .videoFrame{width:100%;height:100%;object-fit:cover;background:#000;border:3px solid var(--accent);}
    .overlay{position:absolute;left:12px;top:12px;background:rgba(0,0,0,0.45);padding:6px 8px;border-radius:8px;font-size:13px;color:var(--muted);z-index:5}
    .controls{position:absolute;top:0;left:0;width:100%;height:100%;display:flex;justify-content:space-between;align-items:flex-end;pointer-events:none}
    .stick{width:120px;height:120px;border-radius:50%;background:rgba(255,255,255,0.15);display:grid;place-items:center;position:relative;pointer-events:auto; touch-action: none;}
    .base{width:70px;height:70px;border-radius:50%;background:rgba(255,255,255,0.05);border:2px dashed rgba(255,255,255,0.03);display:grid;place-items:center}
    .knob{width:40px;height:40px;border-radius:50%;background:linear-gradient(180deg,#fff,#cbd5e1);transform:translate(-50%,-50%);position:absolute;left:50%;top:50%;box-shadow:0 6px 18px rgba(2,6,23,0.6)}
    .value{font-size:12px;color:var(--muted);text-align:center;margin-top:4px}
  </style>
</head>
<body>
  <div class="app">
    <div class="viewer">
      <!-- 影像來源直接指向 /stream 實現 MJPEG 串流 -->
      <img id="video" class="videoFrame" alt="遠端影像" src="/stream" />
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
    const imgSourceEl = document.getElementById('imgSource');

    const state = {steer:0, throttle:0, ws:null, sendInterval:null, config:{wsUrl:'',sendRate:50}};

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
            }; 
            
            state.ws.onmessage = (event) => {
                const data = event.data;
                
                // 嘗試解析 JSON (控制狀態/遠端日誌)
                try {
                    const json = JSON.parse(data);
                    if (json.debug) {
                        appendLog(json.debug);
                    } else if (json.motorA !== undefined) {
                        // 馬達狀態更新 (可選)
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
          // t: timestamp, steer: 轉向 (-100 to 100), throttle: 油門 (-100 to 100)
          state.ws.send(JSON.stringify({t:Date.now(),steer:state.steer,throttle:state.throttle})); 
        } 
      }, rate); 
    }
    
    // 移除舊的 fetchFrame / startVideoPoll 邏輯，改為直接使用 /stream

    window.addEventListener('beforeunload', ()=>{ if(state.ws) state.ws.close(); startSending(0); });
    
    window.onload = () => {
        const ip = window.location.hostname;
        imgSourceEl.textContent = ip + "/stream";
        connectWs();
        startSending(50); // 每 50ms 發送一次控制命令
    };
  </script>
</body>
</html>
)rawliteral";

// 設置 HTTP Server 和 WebSocket
void setupWebServer() {
  //String hostname = "esp32car-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  String hostname = "esp32s3-" + String(WiFi.macAddress());
  hostname.replace(":", ""); // remove colons for clean name
  
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
  
  // 新增 MJPEG 串流路由
  server.on("/stream", HTTP_GET, handleJPEGSStream);

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  sendLogMessage("Web UI Ready on port 80. Remote Control Active at http://" + WiFi.localIP().toString());
}

// ----------------------------------------------------------------------
// VIII. OTA 服務 (Over-The-Air Update)
// ----------------------------------------------------------------------

void setupOTA() {
  //String hostname = "esp32car-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  String hostname = "esp32s3-" + String(WiFi.macAddress());
  hostname.replace(":", ""); // remove colons for clean name
  
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

  // 將 LEDC 通道連接到 GPIO 引腳 (已更新 motorA 和 STBY 腳位)
  ledcAttachPin(motorA_pwm_fwd, CH_A_FWD);
  ledcAttachPin(motorA_pwm_rev, CH_A_REV);
  ledcAttachPin(motorB_pwm_left, CH_B_LEFT);
  ledcAttachPin(motorB_pwm_right, CH_B_RIGHT);
}

// ----------------------------------------------------------------------
// 程式進入點
// ----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  // 馬達開關 (Motor Enable) - 已更新腳位
  pinMode(motor_stby, OUTPUT);
  digitalWrite(motor_stby, LOW); // 預設禁用馬達

  // 馬達初始化 (Motor Initialization)
  setupPWM();

  // 網路連線 (Network Connection)
  connectToWiFi();
  
  // 攝影機初始化 (Camera Initialization) - 已使用新的腳位定義
  initCamera();

  // OTA 服務 (Over-The-Air Update)
  setupOTA();

  // 網頁服務 (Web Services)
  setupWebServer();
  
  sendLogMessage("User App setup complete. Ready to receive commands and stream video.");
}


void loop() {
  // 保持 OTA 服務運行
  ArduinoOTA.handle();
  // 保持 WebSocket 服務運行
  webSocket.loop();
  
  // 馬達命令超時邏輯
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (targetA != 0 || targetB != 0) { 
      sendLogMessage("Motors stopped due to command timeout.");
      targetA = targetB = 0; // 重設目標速度
      digitalWrite(motor_stby, LOW); // 禁用馬達
    }
  }

  // 心跳日誌
  static unsigned long lastLogMillis = 0;
  if (millis() - lastLogMillis > 5000) { 
    // 修正 PSRAM 函式：使用 ESP.getFreePsram()
    sendLogMessage("Heartbeat: Car active, Mode=" + String(currentMode == AUTO ? "AUTO" : "MANUAL") + 
                   " | PSRAM Free: " + String(ESP.getFreePsram() / 1024) + " KB");
    lastLogMillis = millis();
  }
}
