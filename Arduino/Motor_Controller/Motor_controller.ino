#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <CytronMotorDriver.h>

// ================= WIFI SETTINGS =================
const char* ssid = "GATI_BOT";
const char* password = "password123";
WebServer server(80);

// ================= MOTOR PINS (MDD3A PWM-PWM) =================
CytronMD motorLeft(PWM_PWM, 25, 26);   
CytronMD motorRight(PWM_PWM, 27, 14);  

// ================= ENCODER PINS =================
#define ENC_L_A 34  
#define ENC_L_B 33
#define ENC_R_A 35  
#define ENC_R_B 32

// ================= PHYSICAL CALIBRATION =================
#define PID_INTERVAL 50        // 20Hz Loop (50ms)
#define TICKS_PER_REV_L 275.0  
#define TICKS_PER_REV_R 273.800  
#define MAX_RPM 155.0          
#define MIN_PWM 20             
#define MAX_PWM 120

const float MAX_TICKS_L = (TICKS_PER_REV_L * MAX_RPM / 60.0) * (PID_INTERVAL / 1000.0);
const float MAX_TICKS_R = (TICKS_PER_REV_R * MAX_RPM / 60.0) * (PID_INTERVAL / 1000.0);

// ================= PID SETTINGS =================
float Kp = 7.5; 
float Ki = 3.5; 
float Kd = 0.5;

// ================= GLOBALS =================
volatile long leftCount = 0;
volatile long rightCount = 0;
float targetTicksL = 0, targetTicksR = 0;
float iTermL = 0, iTermR = 0;
float lastErrL = 0, lastErrR = 0;
unsigned long lastPIDTime = 0;

// ================= ISR FUNCTIONS =================
void IRAM_ATTR leftEncoderISR() { (digitalRead(ENC_L_B) == HIGH) ? leftCount++ : leftCount--; }
void IRAM_ATTR rightEncoderISR() { (digitalRead(ENC_R_B) == HIGH) ? rightCount++ : rightCount--; }

// ================= GUI HTML/CSS/JS (GATI BOT DESIGN) =================
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<style>
  body { font-family: 'Arial', sans-serif; background-color: #ffffff; display: flex; flex-direction: column; align-items: center; margin: 0; padding: 10px; height: 100vh; overflow: hidden; }
  h1 { font-size: 48px; font-weight: 900; margin: 10px 0 30px 0; letter-spacing: -1px; }
  .main-container { display: flex; flex-direction: row; width: 100%; max-width: 900px; justify-content: space-around; align-items: center; flex-wrap: wrap; }
  .label-pill { background-color: #999; color: #333; padding: 5px 25px; border-radius: 15px; font-weight: bold; font-size: 14px; display: inline-block; margin-bottom: 15px; text-transform: uppercase; }
  .left-panel { display: flex; flex-direction: column; align-items: center; gap: 40px; }
  
  /* Status Toggle */
  .toggle-switch { position: relative; width: 100px; height: 40px; background: #ccc; border-radius: 20px; display: flex; align-items: center; justify-content: space-between; padding: 0 5px; font-size: 10px; font-weight: bold; color: white; }
  .toggle-knob { position: absolute; top: 2px; left: 2px; width: 36px; height: 36px; background: white; border-radius: 50%; transition: 0.3s; box-shadow: 0 2px 5px rgba(0,0,0,0.2); }
  .status-text { z-index: 1; padding: 0 10px; }
  .online { background: #00d4ff; } 
  .offline { background: #ff0000; }
  .online .toggle-knob { left: 62px; }

  /* Speedometer */
  .speed-wrapper { display: flex; align-items: center; gap: 15px; }
  .adjust-btn { width: 70px; height: 60px; background-color: #1a1a1a; border-radius: 20px 5px 5px 20px; color: #00d4ff; font-size: 40px; font-weight: bold; border: none; cursor: pointer; display: flex; align-items: center; justify-content: center; padding-bottom: 5px; }
  .btn-plus { border-radius: 5px 20px 20px 5px; } 
  .gauge-svg { width: 180px; height: 100px; }

  /* Right Panel */
  .right-panel { display: flex; flex-direction: column; align-items: center; position: relative; }
  .control-grid { display: grid; grid-template-columns: 90px 90px 90px; grid-template-rows: 90px 90px 90px; gap: 5px; align-items: center; justify-items: center; }
  .arrow-btn { width: 80px; height: 80px; background: #00d4ff; border: none; cursor: pointer; clip-path: polygon(40% 0%, 60% 0%, 60% 40%, 100% 40%, 50% 100%, 0% 40%, 40% 40%); }
  
  .btn-up { transform: rotate(180deg); grid-column: 2; grid-row: 1; }
  .btn-down { transform: rotate(0deg); grid-column: 2; grid-row: 3; }
  .btn-left { transform: rotate(90deg); grid-column: 1; grid-row: 2; }
  .btn-right { transform: rotate(-90deg); grid-column: 3; grid-row: 2; }

  .stop-btn { width: 70px; height: 70px; background-color: #ff0000; color: white; border: none; border-radius: 15px; font-weight: bold; font-size: 16px; grid-column: 2; grid-row: 2; cursor: pointer; box-shadow: 0 4px 0 #b30000; }
  .stop-btn:active { box-shadow: none; transform: translateY(4px); }
  .arrow-btn:active { opacity: 0.7; }
  @media (max-width: 600px) { .main-container { flex-direction: column; gap: 30px; } h1 { font-size: 36px; } }
</style>
</head>
<body>
  <h1>GATI BOT</h1>
  <div class="main-container">
    <div class="left-panel">
      <div><div id="status-display" class="toggle-switch offline"><span class="status-text">ON</span><span class="status-text">OFF</span><div class="toggle-knob"></div></div></div>
      <div style="text-align:center;">
        <div class="label-pill">SPEED</div>
        <div class="speed-wrapper">
          <button class="adjust-btn" onclick="changeSpeed(-0.1)">-</button>
          <svg class="gauge-svg" viewBox="0 0 100 60">
            <path d="M10 50 A 40 40 0 0 1 90 50" fill="none" stroke="#eee" stroke-width="12" stroke-linecap="round"/>
            <path d="M10 50 A 40 40 0 0 1 30 15" fill="none" stroke="#00d4ff" stroke-width="12" stroke-dasharray="10 2" />
            <path d="M30 15 A 40 40 0 0 1 60 10" fill="none" stroke="#ff6600" stroke-width="12" stroke-dasharray="10 2" />
            <path d="M60 10 A 40 40 0 0 1 90 50" fill="none" stroke="#ff0000" stroke-width="12" stroke-dasharray="10 2" />
            <line id="needle" x1="50" y1="50" x2="10" y2="50" stroke="#333" stroke-width="3" stroke-linecap="round" transform="rotate(0, 50, 50)" />
            <circle cx="50" cy="50" r="4" fill="#333" />
          </svg>
          <button class="adjust-btn btn-plus" onclick="changeSpeed(0.1)">+</button>
        </div>
      </div>
    </div>
    <div class="right-panel">
      <div class="label-pill">CONTROLS</div>
      <div class="control-grid">
        <button class="arrow-btn btn-up" ontouchstart="sendMove('f')" ontouchend="sendMove('s')" onmousedown="sendMove('f')" onmouseup="sendMove('s')"></button>
        <button class="arrow-btn btn-left" ontouchstart="sendMove('l')" ontouchend="sendMove('s')" onmousedown="sendMove('l')" onmouseup="sendMove('s')"></button>
        <button class="stop-btn" onclick="sendMove('s')">STOP</button>
        <button class="arrow-btn btn-right" ontouchstart="sendMove('r')" ontouchend="sendMove('s')" onmousedown="sendMove('r')" onmouseup="sendMove('s')"></button>
        <button class="arrow-btn btn-down" ontouchstart="sendMove('b')" ontouchend="sendMove('s')" onmousedown="sendMove('b')" onmouseup="sendMove('s')"></button>
      </div>
    </div>
  </div>
<script>
  let speed = 0.5;
  function changeSpeed(delta) {
    speed = Math.min(Math.max(speed + delta, 0.1), 1.0);
    const angle = speed * 180;
    document.getElementById('needle').setAttribute('transform', `rotate(${angle}, 50, 50)`);
  }
  changeSpeed(0);
  function sendMove(dir) {
    let l=0, r=0;
    if(dir=='f'){ l=speed; r=speed; }
    else if(dir=='b'){ l=-speed; r=-speed; }
    else if(dir=='l'){ l=-speed*0.6; r=speed*0.6; }
    else if(dir=='r'){ l=speed*0.6; r=-speed*0.6; }
    else if(dir=='s'){ l=0; r=0; }
    fetch(`/move?l=${l.toFixed(2)}&r=${r.toFixed(2)}`).catch(e => console.log(e));
  }
  setInterval(() => {
    fetch('/ping').then(r => {
      if(r.ok) {
        document.getElementById('status-display').classList.add('online');
        document.getElementById('status-display').classList.remove('offline');
      }
    }).catch(() => {
      document.getElementById('status-display').classList.remove('online');
      document.getElementById('status-display').classList.add('offline');
    });
  }, 2000);
</script></body></html>
)=====";

// ================= HELPER FUNCTIONS =================
void handleMove() {
  if (server.hasArg("l") && server.hasArg("r")) {
    targetTicksL = server.arg("l").toFloat() * MAX_TICKS_L;
    targetTicksR = server.arg("r").toFloat() * MAX_TICKS_R;
  }
  server.send(200, "text/plain", "OK");
}

int applyOutput(float pid_output) {
  if (abs(pid_output) < 0.5) return 0; 
  int pwm = map(abs((int)pid_output), 0, 255, MIN_PWM, MAX_PWM);
  return (pid_output > 0) ? constrain(pwm, 0, 255) : -constrain(pwm, 0, 255);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // Fast timeout for serial commands

  // Initialize Motor & Encoder Pins
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);

  // Initialize Wi-Fi & Web Server
  WiFi.softAP(ssid, password);
  server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });
  server.on("/move", handleMove);
  server.on("/ping", []() { server.send(200, "text/plain", "pong"); });
  server.begin();

  Serial.println("System Ready: Wi-Fi + Serial/ROS Teleop");
  Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
}

// ================= LOOP =================
void loop() {
  // 1. Handle Web Requests (Phone Control)
  server.handleClient();

  // 2. Handle Serial/ROS Commands (Teleop)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("CMD")) {
      float l_ratio, r_ratio;
      // Parses "CMD 0.5 -0.5" -> Left 50%, Right -50%
      if (sscanf(line.c_str(), "CMD %f %f", &l_ratio, &r_ratio) == 2) {
        targetTicksL = l_ratio * MAX_TICKS_L;
        targetTicksR = r_ratio * MAX_TICKS_R;
      }
    } else if (line == "k" || line == "s") { // ROS Emergency Stop
      targetTicksL = 0; targetTicksR = 0;
      iTermL = 0; iTermR = 0;
    }
  }

  // 3. PID Control Loop (Runs every 50ms)
  unsigned long now = millis();
  if (now - lastPIDTime >= PID_INTERVAL) {
    lastPIDTime = now;

    // Capture delta ticks
    static long prevL = 0, prevR = 0;
    long currL = leftCount;
    long currR = rightCount;
    float actualL = (float)(currL - prevL);
    float actualR = (float)(currR - prevR);
    prevL = currL; prevR = currR;

    // --- LEFT PID ---
    float errorL = targetTicksL - actualL;
    iTermL = constrain(iTermL + (errorL * Ki), -255, 255);
    float outL = (Kp * errorL) + iTermL + ((errorL - lastErrL) * Kd);
    lastErrL = errorL;

    // --- RIGHT PID ---
    float errorR = targetTicksR - actualR;
    iTermR = constrain(iTermR + (errorR * Ki), -255, 255);
    float outR = (Kp * errorR) + iTermR + ((errorR - lastErrR) * Kd);
    lastErrR = errorR;

    // Safety: Stop integrating if target is zero
    if (abs(targetTicksL) < 0.01) { outL = 0; iTermL = 0; }
    if (abs(targetTicksR) < 0.01) { outR = 0; iTermR = 0; }

    // Execute Motor Speeds
    motorLeft.setSpeed(applyOutput(outL));
    motorRight.setSpeed(applyOutput(outR));

    // 4. Feedback for Odom (Required for ROS)
    Serial.printf("ODOM %ld %ld\n", leftCount, rightCount);
  }
}
