#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <HX711.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ==================== Pinos ====================
#define TFT_CS   8
#define TFT_DC   17
#define TFT_RST  18

#define TOUCH_CS  16
#define TOUCH_IRQ 15

#define THERM_PIN   4
#define HEATER_PIN  5

#define DIR   9
#define STEP  46
#define ENABLE 3

// HX711
#define HX711_DOUT  41
#define HX711_SCK   42

// ==================== Objetos ====================
Adafruit_ILI9341 tft(&SPI, TFT_DC, TFT_CS, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
HX711 scale;

// ==================== Layout/UI ====================
const int SCREEN_W = 320, SCREEN_H = 240;
const int MARGIN = 10;
const int BTN_W  = 140;
const int BTN_H  = 60;
const int BTN_SP = 20;

const int BTN1_X = MARGIN;                        // Motor
const int BTN1_Y = 20;
const int BTN2_X = BTN1_X + BTN_W + BTN_SP;       // Hotend
const int BTN2_Y = BTN1_Y;

const int TEMP_AREA_Y = 110;

#define COL_BG     ILI9341_WHITE
#define COL_ON     ILI9341_GREEN
#define COL_OFF    ILI9341_RED
#define COL_TEXT   ILI9341_BLACK
#define COL_FRAME  ILI9341_BLACK

// Touch calibração
int16_t TOUCH_X_MIN = 200,  TOUCH_X_MAX = 3800;
int16_t TOUCH_Y_MIN = 200,  TOUCH_Y_MAX = 3800;

// ==================== Termistor / PID / PWM ====================
const float VCC    = 3.30f;
const int   SAMPLES= 32;
const float R_PULL = 4700.0f;
const float R0     = 100000.0f;
const float t0     = 298.15f;   // 25°C (K)
const float BETA   = 3950.0f;

// SEU leitor (mantido)
float readV(){
  analogReadResolution(12);
  analogSetPinAttenuation(THERM_PIN, ADC_11db);
  uint32_t acc=0;
  for(int i=0;i<SAMPLES;i++) acc += analogRead(THERM_PIN);
  float adc = (float)acc / (float)SAMPLES;      // 0..4095
  return (adc / 4095.0f) * VCC;                 // volts no nó
}
float readTempC_raw(float &vnode, float &r_ntc){
  vnode = readV();
  r_ntc = R_PULL * (VCC - vnode) / max(vnode, 1e-6f); // pull-down
  float invT = (1.0f/t0) + (1.0f/BETA) * logf(r_ntc / R0);
  return (1.0f/invT) - 273.15f;
}

// PWM (LEDC 3.x)
const uint32_t PWM_HZ   = 500;
const uint8_t  PWM_RES  = 10;    // duty 0..1023
const bool     INVERT_OUTPUT = false;
inline uint16_t pwmMax(){ return (1u<<PWM_RES)-1u; }
inline void pwmBegin(){
  bool ok = ledcAttach(HEATER_PIN, PWM_HZ, PWM_RES);
  if (!ok) Serial.println(F("[ERRO] ledcAttach falhou!"));
  if (INVERT_OUTPUT) ledcOutputInvert(HEATER_PIN, true);
  ledcWrite(HEATER_PIN, 0);
}
inline void pwmWriteDuty(uint16_t duty){
  if (duty > pwmMax()) duty = pwmMax();
  if (INVERT_OUTPUT) duty = pwmMax() - duty;
  ledcWrite(HEATER_PIN, duty);
}
inline void heaterOff(){ pwmWriteDuty(0); }

// PID
double SetpointC = 280.0;
double InputC = 0.0, OutputPWM = 0.0;
double Kp = 4.0;     // 16 / 4
double Ki = 0.005;   // 0.02 / 4
double Kd = 11.0;    // 44 / 4
PID hotendPID(&InputC, &OutputPWM, &SetpointC, Kp, Ki, Kd, DIRECT);

const TickType_t PID_DT_TICKS = pdMS_TO_TICKS(100);
const float MAX_C = 300.0f;

// HX711 calibração
float CALIBRATION_FACTOR = 0.038126f;

// ==================== Estados (compartilhados) ====================
volatile bool motorOn  = false;
volatile bool heaterOn = false;

float tempC_shared   = NAN;
float grams_shared   = NAN;
uint16_t pwm_shared  = 0;

// Proteção (mutex)
SemaphoreHandle_t gMutex;

// ==================== UI helpers ====================
void drawButton(int x, int y, int w, int h, const char* label, bool on) {
  tft.fillRoundRect(x, y, w, h, 8, on ? COL_ON : COL_OFF);
  tft.drawRoundRect(x, y, w, h, 8, COL_FRAME);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t bx = x + 10;
  int16_t by = y + (h/2) - 8;
  tft.setCursor(bx, by);
  tft.print(label);
}
void drawUI() {
  tft.fillScreen(COL_BG);
  tft.drawRect(0, 0, SCREEN_W, SCREEN_H, COL_FRAME);
  drawButton(BTN1_X, BTN1_Y, BTN_W, BTN_H, motorOn  ? "Motor: ON"  : "Motor: OFF",  motorOn);
  drawButton(BTN2_X, BTN2_Y, BTN_W, BTN_H, heaterOn ? "Hotend: ON" : "Hotend: OFF", heaterOn);
}
void updateInfoOnScreen(float tC, float grams, bool heater, bool motor, uint16_t pwm) {
  tft.fillRect(0, TEMP_AREA_Y, SCREEN_W, SCREEN_H - TEMP_AREA_Y, COL_BG);
  tft.setTextColor(COL_TEXT);
  tft.setTextSize(3);
  tft.setCursor(MARGIN, TEMP_AREA_Y + 8);
  tft.print("Temp: ");
  tft.print(tC, 1);
  tft.print(" C");

  tft.setCursor(MARGIN, TEMP_AREA_Y + 48);
  tft.print("Peso: ");
  if (isnan(grams)) tft.print("--.-");
  else tft.print(grams, 1);
  tft.print(" g");

  tft.setTextSize(2);
  tft.setCursor(MARGIN, TEMP_AREA_Y + 90);
  tft.print("SP: ");
  tft.print(SetpointC, 0);
  tft.print(" C  |  Heater: ");
  tft.print(heater ? "ON" : "OFF");
  tft.print("  |  Motor: ");
  tft.print(motor ? "ON" : "OFF");
  tft.print("  |  PWM: ");
  tft.print(pwm);
}
bool touchToScreen(int16_t &sx, int16_t &sy) {
  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();
  sx = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, SCREEN_W);
  sy = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, SCREEN_H);
  return true;
}
bool pointInRect(int16_t x, int16_t y, int rx, int ry, int rw, int rh) {
  return (x >= rx && x < rx+rw && y >= ry && y < ry+rh);
}

// ==================== HX711 helpers ====================
bool hx711Init() {
  scale.begin(HX711_DOUT, HX711_SCK);
  vTaskDelay(pdMS_TO_TICKS(100));
  if (!scale.is_ready()) return false;
  scale.set_scale(CALIBRATION_FACTOR);
  scale.set_offset(2557);
  scale.tare(10);
  return true;
}
float readWeightGrams() {
  if (!scale.is_ready()) return NAN;
  return scale.get_units(5);
}

// ==================== Tasks ====================
void TaskUI(void *pv){
  // UI INIT
  tft.begin();
  ts.begin();
  tft.setRotation(1);
  ts.setRotation(1);
  drawUI();

  uint32_t lastUi = 0;
  uint32_t lastTouch = 0;

  for(;;){
    // Touch (debounce ~150 ms)
    int16_t x, y;
    if (ts.touched()){
      uint32_t now = millis();
      if (now - lastTouch > 150){
        lastTouch = now;
        if (touchToScreen(x,y)){
          if (pointInRect(x,y,BTN1_X,BTN1_Y,BTN_W,BTN_H)){
            motorOn = !motorOn;
            drawButton(BTN1_X, BTN1_Y, BTN_W, BTN_H, motorOn ? "Motor: ON" : "Motor: OFF", motorOn);
            digitalWrite(ENABLE, motorOn ? LOW : HIGH);
          } else if (pointInRect(x,y,BTN2_X,BTN2_Y,BTN_W,BTN_H)){
            heaterOn = !heaterOn;
            drawButton(BTN2_X, BTN2_Y, BTN_W, BTN_H, heaterOn ? "Hotend: ON" : "Hotend: OFF", heaterOn);
            if (!heaterOn) heaterOff();
          }
        }
      }
    }

    // Atualização da área de info ~5 Hz
    if (millis() - lastUi >= 200){
      lastUi = millis();
      float t, g; bool h, m; uint16_t pwm;
      // snapshot protegido
      if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(5)) == pdTRUE){
        t = tempC_shared; g = grams_shared; h = heaterOn; m = motorOn; pwm = pwm_shared;
        xSemaphoreGive(gMutex);
      } else { t = tempC_shared; g = grams_shared; h = heaterOn; m = motorOn; pwm = pwm_shared; }
      updateInfoOnScreen(t, g, h, m, pwm);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskHeater(void *pv){
  // PWM + PID init
  pinMode(THERM_PIN, INPUT);
  pwmBegin();

  hotendPID.SetMode(AUTOMATIC);
  hotendPID.SetOutputLimits(0, (double)pwmMax());
  hotendPID.SetSampleTime(pdTICKS_TO_MS(PID_DT_TICKS));

  for(;;){
    float V, R;
    float tC = readTempC_raw(V, R);

    if (heaterOn){
      if (tC >= MAX_C){
        heaterOn = false;
        heaterOff();
        Serial.println(F("[FAILSAFE] Overtemp. Heater OFF."));
      } else {
        InputC = (double)tC;
        hotendPID.Compute();
        uint16_t duty = (uint16_t)OutputPWM;
        pwmWriteDuty(duty);
        // publica PWM
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(1)) == pdTRUE){
          pwm_shared = duty;
          xSemaphoreGive(gMutex);
        } else {
          pwm_shared = duty;
        }
      }
    } else {
      heaterOff();
      if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(1)) == pdTRUE){
        pwm_shared = 0;
        xSemaphoreGive(gMutex);
      } else { pwm_shared = 0; }
    }

    // publica temperatura
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      tempC_shared = tC;
      xSemaphoreGive(gMutex);
    } else {
      tempC_shared = tC;
    }

    vTaskDelay(PID_DT_TICKS); // ~100 ms
  }
}

void TaskStepper(void *pv){
  // Driver stepper
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH); // desliga no boot
  stepper.setMaxSpeed(100);
  stepper.setSpeed(10);

  for(;;){
    if (motorOn){
      stepper.runSpeed(); // deve ser chamado MUITO frequente
      // 1-2 ms dá conta de 50*16 steps/s
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void TaskHX711(void *pv){
  if (!hx711Init()){
    Serial.println(F("[HX711] Nao pronto. Verifique pinos/fios."));
  } else {
    Serial.println(F("[HX711] OK. Tara feita."));
  }

  for(;;){
    float g = readWeightGrams();
    // se quiser offsetzinho, aplique aqui (ex.: +2 g)
    // g += 2.0f;

    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(2)) == pdTRUE){
      grams_shared = g;
      xSemaphoreGive(gMutex);
    } else {
      grams_shared = g;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
  }
}

// ==================== Arduino setup/loop ====================
void setup(){
  Serial.begin(115200);
  SPI.begin();

  // Mutex
  gMutex = xSemaphoreCreateMutex();

  // Inicializa display/touch na task de UI (para evitar draws de outro core)
  // Cria Tasks
  xTaskCreatePinnedToCore(TaskUI,     "UI",     8192, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskHeater, "HEATER", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(TaskStepper,"STEPPER",4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(TaskHX711,  "HX711",  4096, nullptr, 2, nullptr, 0);

  // nada de heavy init aqui além do SPI.begin(); display é iniciado na TaskUI
}

void loop(){
  // vazio — tudo roda nas tasks
  // vTaskDelay(pdMS_TO_TICKS(1000));
}
