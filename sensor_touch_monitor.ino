/*
  STTHK3113 Assignment 2 - Interactive ESP32 Sensor System
  Room Climate & Smoke Monitor

  Sensors : DHT22 (temperature + humidity), MQ-2 (smoke / combustible gas, analog)
  Display : 0.96" OLED SSD1306, I2C (0x3C)
  Input   : Capacitive touch on a male pin -> GPIO 4(T0)
  Pages   : 0 = Climate, 1 = Smoke, 2 = Summary
  Libraries (Library Manager): Adafruit SSD1306, Adafruit GFX, DHT sensor library, Adafruit Unified Sensor

  GPIO map
    OLED SDA -> GPIO 21      OLED SCL -> GPIO 22
    DHT22 DATA -> GPIO 5
    MQ-2 AO -> GPIO 34 (ADC1, input only). MQ-2 is powered from 3.3 V so AO never exceeds the ESP32's 3.3 V limit
    OLED VCC, DHT22 VCC, MQ-2 VCC -> 3.3 V      all GND -> GND
    Touch pin -> GPIO 4(T0)
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ---------- Pins ----------
constexpr uint8_t PIN_DHT   = 5;
constexpr uint8_t PIN_MQ2   = 34;
constexpr uint8_t PIN_TOUCH = 4;   // touch-capable: 4, 13, 14, 27, 32, 33

// ---------- Display ----------
constexpr uint8_t OLED_W = 128, OLED_H = 64, OLED_ADDR = 0x3C;
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

// ---------- Sensors ----------
DHT dht(PIN_DHT, DHT22);
float temperatureC = NAN, humidityPct = NAN;
int   smokeRaw = 0;          // 0..4095 ADC reading (MQ-2 powered at 3.3 V)
int   smokePct = 0;          // 0..100 of ADC range
int   smokeBaseline = 0;     // clean-air reading captured after warm-up

constexpr unsigned long DHT_INTERVAL_MS  = 2000;   // DHT22 needs >= 2 s between reads
constexpr unsigned long MQ2_INTERVAL_MS  = 250;
constexpr unsigned long DRAW_INTERVAL_MS = 200;
constexpr unsigned long MQ2_WARMUP_MS    = 20000;  // short warm-up for demo (MQ-2 is best pre-heated longer)
constexpr int SMOKE_WARN_DELTA   = 150;            // raw counts above baseline -> WARNING (tune in testing)
constexpr int SMOKE_DANGER_DELTA = 400;            // raw counts above baseline -> DANGER

// ---------- Touch ----------
constexpr uint8_t  TOUCH_SAMPLES_NEEDED = 3;       // consecutive "touched" reads = debounce
constexpr unsigned long TOUCH_LOCKOUT_MS = 400;    // minimum time between page changes
uint32_t touchBaseline = 0;                        // untouched value measured at start-up
uint32_t touchDelta = 0;                           // change from baseline that counts as a touch
uint32_t touchValue = 0;                           // latest touchRead() value
uint8_t  touchCount = 0;
bool     touchHeld = false;
unsigned long lastPageChange = 0;

constexpr uint8_t NUM_PAGES = 3;
uint8_t page = 0;

unsigned long tDht = 0, tMq2 = 0, tDraw = 0, tTouch = 0, bootMs = 0;
bool baselineSet = false;

uint32_t readTouchAvg(uint8_t n) {
  uint64_t s = 0;
  for (uint8_t i = 0; i < n; i++) { s += touchRead(PIN_TOUCH); delay(5); }
  return (uint32_t)(s / n);
}

void calibrateTouch() {
  touchBaseline = readTouchAvg(30);          // untouched baseline
  touchDelta = touchBaseline * 3 / 10;       // a 30 % change from the baseline = touched
  if (touchDelta < 5) touchDelta = 5;
  Serial.printf("Touch baseline=%lu delta=%lu\n", (unsigned long)touchBaseline, (unsigned long)touchDelta);
}

bool isTouched(uint32_t v) {                 // works whether a touch lowers or raises the value
  return v + touchDelta < touchBaseline || v > touchBaseline + touchDelta;
}

const char* smokeStatus() {
  int d = smokeRaw - smokeBaseline;
  if (d >= SMOKE_DANGER_DELTA) return "DANGER";
  if (d >= SMOKE_WARN_DELTA)   return "WARNING";
  return "NORMAL";
}

void updateTouch() {                          // call often, never blocks
  touchValue = touchRead(PIN_TOUCH);
  bool touched = isTouched(touchValue);
  if (touched) { if (touchCount < 255) touchCount++; } else { touchCount = 0; touchHeld = false; }
  if (touchCount >= TOUCH_SAMPLES_NEEDED && !touchHeld && millis() - lastPageChange > TOUCH_LOCKOUT_MS) {
    touchHeld = true;                         // one page change per touch
    lastPageChange = millis();
    page = (page + 1) % NUM_PAGES;
    Serial.printf("Touch -> page %u\n", page);
  }
}

void updateSensors() {
  unsigned long now = millis();
  if (now - tMq2 >= MQ2_INTERVAL_MS) {
    tMq2 = now;
    uint32_t s = 0;
    for (uint8_t i = 0; i < 8; i++) s += analogRead(PIN_MQ2);
    smokeRaw = s / 8;
    smokePct = map(smokeRaw, 0, 4095, 0, 100);
    if (!baselineSet && now - bootMs >= MQ2_WARMUP_MS) { smokeBaseline = smokeRaw; baselineSet = true; }
  }
  if (now - tDht >= DHT_INTERVAL_MS) {
    tDht = now;
    float h = dht.readHumidity(), t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) { humidityPct = h; temperatureC = t; }
    else Serial.println("DHT22 read failed - check wiring");
  }
}

void header(const char* title) {
  oled.setTextSize(1); oled.setCursor(0, 0);
  oled.print(title);
  oled.setCursor(104, 0); oled.printf("%u/%u", page + 1, NUM_PAGES);
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
}

void draw() {
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
  if (page == 0) {
    header("CLIMATE (DHT22)");
    oled.setTextSize(2);
    oled.setCursor(0, 18); if (isnan(temperatureC)) oled.print("Temp --"); else oled.printf("T:%.1f C", temperatureC);
    oled.setCursor(0, 40); if (isnan(humidityPct)) oled.print("Hum --"); else oled.printf("H:%.1f %%", humidityPct);
  } else if (page == 1) {
    header("SMOKE (MQ-2)");
    if (!baselineSet) {
      oled.setTextSize(1); oled.setCursor(0, 22);
      oled.printf("Warming up... %lus", (MQ2_WARMUP_MS - (millis() - bootMs)) / 1000 + 1);
    } else {
      oled.setTextSize(2); oled.setCursor(0, 16); oled.printf("%d %%", smokePct);
      oled.setTextSize(1); oled.setCursor(0, 36); oled.printf("Raw: %d (base %d)", smokeRaw, smokeBaseline);
      oled.setTextSize(2); oled.setCursor(0, 48); oled.print(smokeStatus());
    }
  } else {
    header("SUMMARY");
    oled.setTextSize(1);
    oled.setCursor(0, 16); if (isnan(temperatureC)) oled.print("Temp: --"); else oled.printf("Temp : %.1f C", temperatureC);
    oled.setCursor(0, 28); if (isnan(humidityPct)) oled.print("Humid: --"); else oled.printf("Humid: %.1f %%", humidityPct);
    oled.setCursor(0, 40); oled.printf("Smoke: %d %%  %s", smokePct, baselineSet ? smokeStatus() : "WARMUP");
    oled.setCursor(0, 54); oled.print("Touch = next page");
  }
  oled.display();
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ2, ADC_11db);
  dht.begin();
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) { Serial.println("OLED not found"); while (true) delay(1000); }
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  oled.setCursor(0, 20); oled.println("Do NOT touch the pin"); oled.println("Calibrating touch...");
  oled.display();
  delay(2000);        // keep the calibration message visible (do not touch the pin)
  calibrateTouch();
}

unsigned long tSerial = 0;

void loop() {
  if (millis() - tSerial >= 1000) {            // debug output for testing / tuning
    tSerial = millis();
    Serial.printf("T=%.1fC H=%.1f%% smokeRaw=%d base=%d touch=%lu touchBase=%lu touched=%d page=%u\n",
                  temperatureC, humidityPct, smokeRaw, smokeBaseline, (unsigned long)touchValue,
                  (unsigned long)touchBaseline, isTouched(touchValue), page);
  }
  updateTouch();      // touch is polled every loop, so sensors are never blocked (FR9)
  updateSensors();
  if (millis() - tDraw >= DRAW_INTERVAL_MS) { tDraw = millis(); draw(); }
}
