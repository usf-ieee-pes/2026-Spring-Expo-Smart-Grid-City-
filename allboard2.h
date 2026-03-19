/*
 * ── PIN ASSIGNMENTS ──────────────────────────────────────────────────
 *   Pin  3  → Hospital MOSFET gate (PWM) — HIGHEST priority
 *   Pin  5  → School MOSFET gate (PWM)
 *   Pin  6  → Homes MOSFET gate (PWM)
 *   Pin  9  → Factory MOSFET gate (PWM)
 *   Pin 10  → Park MOSFET gate (PWM)  — LOWEST priority
 *   Pin 11  → Servo signal (wind turbine spinner)
 *   Pin A0  → Road LED strip MOSFET gate (PWM)
 *   Pin  7  → WS2812B road animation strip data (via 300Ω)
 *   Pin 13  → WS2812B transmission line strip data (via 300Ω)
 *   Pin  8  → Active buzzer
 *   Pin  2  → SoftwareSerial RX ← ESP32 GPIO17 TX
 *   Pin A1  → SoftwareSerial TX → ESP32 GPIO16 RX (via 1kΩ)
 *   Pin  4  → 74HC595 LATCH (RCLK/ST_CP)
 *   Pin A2  → 74HC595 DATA  (SER/DS)
 *   Pin A3  → 74HC595 CLOCK (SRCLK/SH_CP)
 *   Pin A4  → I2C SDA (INA219 ×3)
 *   Pin A5  → I2C SCL (INA219 ×3)
 *
 * ── INA219 ADDRESSES ─────────────────────────────────────────────────
 *   0x40 — Solar combined (2× panels in parallel, 1N4007 on each)
 *   0x41 — Crank combined (2× cranks in parallel, 1N4007 on each)
 *   0x44 — 5V bus output (after LM2596 buck converters)
 *
 * ── 74HC595 BIT MAP ──────────────────────────────────────────────────
 *   Bit 0 → Intersection A — Red    LED (via 220Ω)
 *   Bit 1 → Intersection A — Yellow LED (via 220Ω)
 *   Bit 2 → Intersection A — Green  LED (via 220Ω)
 *   Bit 3 → Intersection B — Red    LED (via 220Ω)
 *   Bit 4 → Intersection B — Yellow LED (via 220Ω)
 *   Bit 5 → Intersection B — Green  LED (via 220Ω)
 *   Bit 6 → (spare)
 *   Bit 7 → (spare)
 *
 * ── 74HC595 WIRING ───────────────────────────────────────────────────
 *   Pin 14 (SER/DS)     → Arduino A2
 *   Pin 11 (SRCLK)      → Arduino A3
 *   Pin 12 (RCLK/LATCH) → Arduino 4
 *   Pin 10 (SRCLR)      → Arduino 5V  (active low — tie HIGH to disable clear)
 *   Pin 13 (OE)         → GND          (active low — tie LOW to enable output)
 *   Pin  8 (GND)        → GND
 *   Pin 16 (VCC)        → 5V
 *   Pins Q0–Q5          → 220Ω → traffic light LEDs
 *
 * ── SERVO WIRING ─────────────────────────────────────────────────────
 *   Red wire   → 5V
 *   Black wire → GND
 *   White/Yellow signal wire → Arduino Pin 11
 *
 * ── TRANSMISSION LINE STRIP ──────────────────────────────────────────
 *   Separate WS2812B strip routed along "power lines" between
 *   the source area and the city buildings.
 *   Data → Pin 13 via 300Ω resistor
 *   Power → MB102 5V rail (same as road strip)
 *   GND   → common GND
 *   Shows blue "energy packets" flowing source→city at a speed
 *   proportional to current output. Slows and dims as grid drops.
 *
 * ── SENSOR WIRING (split circuit) ───────────────────────────────────
 *   Solar panels: both (+) → 1N4007 each → joined → INA219 VIN+
 *                 INA219 VIN– → 100Ω → joined (–) → GND
 *   Cranks:       same pattern with their own INA219
 *   Bus sensor:   after LM2596 output → INA219 VIN+ → VIN– → bus
 *
 * Compile: Tools → Board → Arduino UNO
 * =====================================================================
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <FastLED.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// With 2 panels + 2 cranks these will be roughly double the single values
#define CURRENT_FULL      110.0  // mA — Level 5
#define CURRENT_DROP1      90.0  // mA — Level 4
#define CURRENT_DROP2      70.0  // mA — Level 3
#define CURRENT_DROP3      50.0  // mA — Level 2
#define CURRENT_DROP4      30.0  // mA — Level 1
                                 //       Level 0: below CURRENT_DROP4

// ── Smoothing & easing ────────────────────────────────────────────────
#define SMOOTH            0.85   // EMA factor
#define PWM_EASE_STEP     4      // max PWM units moved per loop

// ── PWM levels ────────────────────────────────────────────────────────
#define PWM_FULL          255
#define PWM_DIM           80
#define PWM_OFF           0

// ── Building pins ─────────────────────────────────────────────────────
#define PIN_HOSPITAL      3
#define PIN_SCHOOL        5
#define PIN_HOMES         6
#define PIN_FACTORY       9
#define PIN_PARK          10
#define PIN_ROAD          A0

// ── New feature pins ──────────────────────────────────────────────────
#define PIN_SERVO         11     // wind turbine servo
#define PIN_ROAD_STRIP    7      // WS2812B road animation
#define PIN_TXLINE_STRIP  13     // WS2812B transmission line
#define PIN_BUZZER        8

// ── SoftwareSerial  ────────────────────────
#define PIN_SERIAL_RX     2
#define PIN_SERIAL_TX     A1    // via 1kΩ to ESP32 GPIO16

// ── 74HC595 shift register (traffic lights) ───────────────────────────
#define PIN_SR_LATCH      4
#define PIN_SR_DATA       A2
#define PIN_SR_CLOCK      A3

// ── WS2812B strips ────────────────────────────────────────────────────
#define NUM_ROAD_LEDS     30     // road animation strip
#define NUM_TXLINE_LEDS   20     // transmission line strip
CRGB roadLeds[NUM_ROAD_LEDS];
CRGB txLeds[NUM_TXLINE_LEDS];

// ── INA219 ────────────────────────────────────────────────────────────
Adafruit_INA219 ina_solar(0x40);   // 2× solar panels combined
Adafruit_INA219 ina_crank(0x41);   // 2× cranks combined
Adafruit_INA219 ina_bus(0x44);     // 5V bus output

// ── Serial & Servo ────────────────────────────────────────────────────
SoftwareSerial espSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
Servo turbineServo;

// ── Raw sensor readings ───────────────────────────────────────────────
float solarVoltage = 0.0, solarCurrent = 0.0;
float crankVoltage = 0.0, crankCurrent = 0.0;
float busVoltage   = 0.0, busCurrent   = 0.0;

// ── Smoothed readings ─────────────────────────────────────────────────
float smoothSolarV = 0.0, smoothSolarI = 0.0;
float smoothCrankV = 0.0, smoothCrankI = 0.0;
float smoothBusV   = 0.0, smoothBusI   = 0.0;

// ── Grid state ────────────────────────────────────────────────────────
int gridLevel     = 5;
int lastGridLevel = -1;

// ── Target brightness ─────────────────────────────────────────────────
int targetHospital = PWM_FULL, targetSchool  = PWM_FULL;
int targetHomes    = PWM_FULL, targetFactory = PWM_FULL;
int targetPark     = PWM_FULL, targetRoad    = PWM_FULL;

// ── Current eased brightness ──────────────────────────────────────────
int brHospital = PWM_FULL, brSchool  = PWM_FULL;
int brHomes    = PWM_FULL, brFactory = PWM_FULL;
int brPark     = PWM_FULL, brRoad    = PWM_FULL;

// ── Servo ─────────────────────────────────────────────────────────────
int currentServoAngle = 0;   // 0=stopped, 180=full speed
int targetServoAngle  = 0;

// ── Traffic light state ───────────────────────────────────────────────
// Each intersection cycles: Green → Yellow → Red → repeat
// Phase 0=Green, 1=Yellow, 2=Red
int  tlPhase[2]         = {0, 2};   // offset phases so they alternate
unsigned long tlLastChange  = 0;
unsigned long tlInterval    = 3000; // ms per phase — slows with grid level
bool tlFlashing             = false;
bool tlFlashState           = false;
unsigned long tlLastFlash   = 0;

// ── Transmission line animation ───────────────────────────────────────
uint8_t txOffset      = 0;
int     txPacketSpeed = 3;   // pixels per frame — proportional to current

// ── Timers ────────────────────────────────────────────────────────────
unsigned long lastSerialSend    = 0;
unsigned long lastRoadLedUpdate = 0;
unsigned long lastTxLedUpdate   = 0;
unsigned long lastBuzzerToggle  = 0;
bool          buzzerState       = false;
uint8_t       roadOffset        = 0;

// ── Helpers ───────────────────────────────────────────────────────────
int easePWM(int current, int target) {
  if (current < target) return min(current + PWM_EASE_STEP, target);
  if (current > target) return max(current - PWM_EASE_STEP, target);
  return current;
}

void shiftOut595(uint8_t data) {
  digitalWrite(PIN_SR_LATCH, LOW);
  shiftOut(PIN_SR_DATA, PIN_SR_CLOCK, MSBFIRST, data);
  digitalWrite(PIN_SR_LATCH, HIGH);
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  // Building MOSFETs
  pinMode(PIN_HOSPITAL, OUTPUT); pinMode(PIN_SCHOOL,  OUTPUT);
  pinMode(PIN_HOMES,    OUTPUT); pinMode(PIN_FACTORY, OUTPUT);
  pinMode(PIN_PARK,     OUTPUT); pinMode(PIN_ROAD,    OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  analogWrite(PIN_HOSPITAL, PWM_FULL); analogWrite(PIN_SCHOOL,  PWM_FULL);
  analogWrite(PIN_HOMES,    PWM_FULL); analogWrite(PIN_FACTORY, PWM_FULL);
  analogWrite(PIN_PARK,     PWM_FULL); analogWrite(PIN_ROAD,    PWM_FULL);

  // Shift register pins
  pinMode(PIN_SR_LATCH, OUTPUT);
  pinMode(PIN_SR_DATA,  OUTPUT);
  pinMode(PIN_SR_CLOCK, OUTPUT);
  shiftOut595(0b00100100);  // both intersections start on Green (bits 2 and 5)

  // Servo
  turbineServo.attach(PIN_SERVO);
  turbineServo.write(90);  // stopped (continuous servo: 90=stop, 0=full CCW, 180=full CW)

  // WS2812B strips — road and transmission line on same FastLED call
  FastLED.addLeds<WS2812B, PIN_ROAD_STRIP,   GRB>(roadLeds, NUM_ROAD_LEDS);
  FastLED.addLeds<WS2812B, PIN_TXLINE_STRIP, GRB>(txLeds,   NUM_TXLINE_LEDS);
  FastLED.setBrightness(80);
  fill_solid(roadLeds, NUM_ROAD_LEDS, CRGB::Green);
  fill_solid(txLeds,   NUM_TXLINE_LEDS, CRGB::Black);
  FastLED.show();

  // INA219 sensors
  if (!ina_solar.begin()) Serial.println(F("WARN: INA219 solar (0x40) not found"));
  if (!ina_crank.begin()) Serial.println(F("WARN: INA219 crank (0x41) not found"));
  if (!ina_bus.begin())   Serial.println(F("WARN: INA219 bus   (0x44) not found"));

  delay(1000);
  Serial.println(F("Grid controller v2 ready."));
  Serial.println(F("SolV|SolI|CrkV|CrkI|BusV|BusI(s)|Lvl|Servo|TL"));
}

// ── Main loop ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  readSensors();
  smoothSensors();
  gridLevel = getGridLevel(smoothBusI);
  applyLoadShedding(gridLevel);
  easeBrightness();
  updateServo(gridLevel);
  updateTrafficLights(gridLevel, now);

  if (now - lastSerialSend >= 1000) {
    lastSerialSend = now;
    sendToESP32();
  }
  if (now - lastRoadLedUpdate >= 40) {
    lastRoadLedUpdate = now;
    animateRoadStrip(gridLevel);
  }
  if (now - lastTxLedUpdate >= 30) {
    lastTxLedUpdate = now;
    animateTxLine(gridLevel);
  }

  handleBuzzer(gridLevel, now);
  delay(30);
}

// ── Sensors ───────────────────────────────────────────────────────────
void readSensors() {
  solarVoltage = ina_solar.getBusVoltage_V();
  solarCurrent = ina_solar.getCurrent_mA();
  crankVoltage = ina_crank.getBusVoltage_V();
  crankCurrent = ina_crank.getCurrent_mA();
  busVoltage   = ina_bus.getBusVoltage_V();
  busCurrent   = ina_bus.getCurrent_mA();
}

void smoothSensors() {
  smoothSolarV = (smoothSolarV * SMOOTH) + (solarVoltage * (1.0 - SMOOTH));
  smoothSolarI = (smoothSolarI * SMOOTH) + (solarCurrent * (1.0 - SMOOTH));
  smoothCrankV = (smoothCrankV * SMOOTH) + (crankVoltage * (1.0 - SMOOTH));
  smoothCrankI = (smoothCrankI * SMOOTH) + (crankCurrent * (1.0 - SMOOTH));
  smoothBusV   = (smoothBusV   * SMOOTH) + (busVoltage   * (1.0 - SMOOTH));
  smoothBusI   = (smoothBusI   * SMOOTH) + (busCurrent   * (1.0 - SMOOTH));
}

int getGridLevel(float current) {
  if (current >= CURRENT_FULL)   return 5;
  if (current >= CURRENT_DROP1)  return 4;
  if (current >= CURRENT_DROP2)  return 3;
  if (current >= CURRENT_DROP3)  return 2;
  if (current >= CURRENT_DROP4)  return 1;
  return 0;
}

// ── Load shedding ─────────────────────────────────────────────────────
void applyLoadShedding(int level) {
  switch (level) {
    case 5:
      targetHospital=PWM_FULL; targetSchool=PWM_FULL;
      targetHomes=PWM_FULL;    targetFactory=PWM_FULL;
      targetPark=PWM_FULL;     targetRoad=PWM_FULL; break;
    case 4:
      targetHospital=PWM_FULL; targetSchool=PWM_FULL;
      targetHomes=PWM_FULL;    targetFactory=PWM_FULL;
      targetPark=PWM_DIM;      targetRoad=200; break;
    case 3:
      targetHospital=PWM_FULL; targetSchool=PWM_FULL;
      targetHomes=PWM_FULL;    targetFactory=PWM_DIM;
      targetPark=PWM_OFF;      targetRoad=140; break;
    case 2:
      targetHospital=PWM_FULL; targetSchool=PWM_FULL;
      targetHomes=PWM_DIM;     targetFactory=PWM_OFF;
      targetPark=PWM_OFF;      targetRoad=80; break;
    case 1:
      targetHospital=PWM_FULL; targetSchool=PWM_DIM;
      targetHomes=PWM_OFF;     targetFactory=PWM_OFF;
      targetPark=PWM_OFF;      targetRoad=30; break;
    default:
      targetHospital=PWM_OFF; targetSchool=PWM_OFF;
      targetHomes=PWM_OFF;    targetFactory=PWM_OFF;
      targetPark=PWM_OFF;     targetRoad=PWM_OFF; break;
  }
  if (gridLevel != lastGridLevel) {
    Serial.print(F("Grid level changed: ")); Serial.println(level);
    lastGridLevel = gridLevel;
  }
}

void easeBrightness() {
  brHospital = easePWM(brHospital, targetHospital);
  brSchool   = easePWM(brSchool,   targetSchool);
  brHomes    = easePWM(brHomes,    targetHomes);
  brFactory  = easePWM(brFactory,  targetFactory);
  brPark     = easePWM(brPark,     targetPark);
  brRoad     = easePWM(brRoad,     targetRoad);

  analogWrite(PIN_HOSPITAL, brHospital); analogWrite(PIN_SCHOOL,  brSchool);
  analogWrite(PIN_HOMES,    brHomes);    analogWrite(PIN_FACTORY, brFactory);
  analogWrite(PIN_PARK,     brPark);     analogWrite(PIN_ROAD,    brRoad);
}

// ── Servo — wind turbine ──────────────────────────────────────────────
// Continuous rotation servo: 90=stop, <90=CCW, >90=CW
// Map grid level to speed: 0=stop(90), 5=full speed(180)
// Use map() to interpolate between 90 (stop) and 180 (full CW)
void updateServo(int level) {
  // Target angle: 90 (stopped) at level 0, 180 (full speed) at level 5
  targetServoAngle = map(level, 0, 5, 90, 180);

  // Ease servo angle toward target (max 2 degrees per loop)
  if (currentServoAngle < targetServoAngle)
    currentServoAngle = min(currentServoAngle + 2, targetServoAngle);
  else if (currentServoAngle > targetServoAngle)
    currentServoAngle = max(currentServoAngle - 2, targetServoAngle);

  turbineServo.write(currentServoAngle);
}

// ── Traffic lights ────────────────────────────────────────────────────
// Two intersections, each with R/Y/G LEDs on shift register bits 0-5.
// Normal operation (level 5): 3s Green, 1s Yellow, 3s Red
// Slow (level 3-4):           5s Green, 1s Yellow, 5s Red
// Emergency (level 1-2):      Flash Red only (500ms on/off)
// Blackout (level 0):         All off
//
// The two intersections are offset by one phase so they alternate:
// When A is Green, B is Red — just like a real intersection.
void updateTrafficLights(int level, unsigned long now) {
  if (level == 0) {
    shiftOut595(0b00000000);  // all off
    return;
  }

  if (level <= 2) {
    // Flash red on both intersections
    if (now - tlLastFlash >= 500) {
      tlLastFlash = now;
      tlFlashState = !tlFlashState;
      // Bit 0 = A Red, Bit 3 = B Red
      shiftOut595(tlFlashState ? 0b00001001 : 0b00000000);
    }
    return;
  }

  // Normal or slow cycle
  tlInterval = (level >= 4) ? 3000 : 5000;

  if (now - tlLastChange >= tlInterval) {
    tlLastChange = now;
    // Advance each intersection to next phase
    tlPhase[0] = (tlPhase[0] + 1) % 3;
    tlPhase[1] = (tlPhase[1] + 1) % 3;
  }

  // Build shift register byte
  // Phase 0=Green, 1=Yellow, 2=Red
  // Intersection A: bits 0(R), 1(Y), 2(G)
  // Intersection B: bits 3(R), 4(Y), 5(G)
  uint8_t srByte = 0;
  srByte |= (1 << tlPhase[0]);        // A: set bit for current phase (0=R,1=Y,2=G)
  srByte |= (1 << (tlPhase[1] + 3));  // B: same but offset by 3
  shiftOut595(srByte);
}

// ── Road animation strip ──────────────────────────────────────────────
void animateRoadStrip(int level) {
  CRGB baseColor;
  switch (level) {
    case 5: baseColor = CRGB(0,   180, 0);  break;
    case 4: baseColor = CRGB(100, 180, 0);  break;
    case 3: baseColor = CRGB(200, 140, 0);  break;
    case 2: baseColor = CRGB(220, 80,  0);  break;
    case 1: baseColor = CRGB(200, 0,   0);  break;
    default:baseColor = CRGB(20,  20,  20); break;
  }
  fill_solid(roadLeds, NUM_ROAD_LEDS, baseColor);
  int pulsePos = (roadOffset / 3) % NUM_ROAD_LEDS;
  roadLeds[pulsePos] = CRGB::White;
  if (pulsePos > 0) roadLeds[pulsePos - 1].fadeToBlackBy(150);
  FastLED.show();
  roadOffset++;
}

// ── Transmission line animation ───────────────────────────────────────
// Blue "energy packets" travel from source end (index 0) to city end.
// Packet count and speed scale with smoothed bus current.
// At level 0: strip goes dark to show power loss visually.
void animateTxLine(int level) {
  // Number of active packets: 1 at level 1, up to 4 at level 5
  int numPackets = max(0, level - 0);
  // Packet speed: map level 0-5 → skip 0-4 pixels per frame
  int skip = map(level, 0, 5, 0, 4);

  // Fade everything slightly each frame
  for (int i = 0; i < NUM_TXLINE_LEDS; i++) {
    txLeds[i].fadeToBlackBy(60);
  }

  if (level == 0) {
    FastLED.show();
    return;
  }

  // Draw packets evenly spaced along the strip
  int spacing = NUM_TXLINE_LEDS / max(numPackets, 1);
  for (int p = 0; p < numPackets; p++) {
    int pos = (txOffset + p * spacing) % NUM_TXLINE_LEDS;
    // Color: blue-white at full power, dim blue at low power
    uint8_t brightness = map(level, 1, 5, 60, 255);
    txLeds[pos] = CRGB(0, brightness / 4, brightness);      // blue packet
    if (pos > 0) txLeds[pos - 1] = CRGB(0, brightness / 8, brightness / 2);  // trail
  }

  FastLED.show();
  txOffset = (txOffset + 1 + skip) % NUM_TXLINE_LEDS;
}

// ── Send JSON to ESP32 ────────────────────────────────────────────────
void sendToESP32() {
  espSerial.print(F("{"));
  espSerial.print("\"bv\":"); espSerial.print(smoothBusV,   2); espSerial.print(F(","));
  espSerial.print("\"bc\":"); espSerial.print(smoothBusI,   1); espSerial.print(F(","));
  espSerial.print("\"sv\":"); espSerial.print(smoothSolarV, 2); espSerial.print(F(","));
  espSerial.print("\"sc\":"); espSerial.print(smoothSolarI, 1); espSerial.print(F(","));
  espSerial.print("\"cv\":"); espSerial.print(smoothCrankV, 2); espSerial.print(F(","));
  espSerial.print("\"cc\":"); espSerial.print(smoothCrankI, 1); espSerial.print(F(","));
  espSerial.print("\"lv\":"); espSerial.print(gridLevel);       espSerial.print(F(","));
  espSerial.print("\"h\":");  espSerial.print(brHospital);      espSerial.print(F(","));
  espSerial.print("\"s\":");  espSerial.print(brSchool);        espSerial.print(F(","));
  espSerial.print("\"ho\":"); espSerial.print(brHomes);         espSerial.print(F(","));
  espSerial.print("\"f\":");  espSerial.print(brFactory);       espSerial.print(F(","));
  espSerial.print("\"p\":");  espSerial.print(brPark);          espSerial.print(F(","));
  espSerial.print("\"rd\":"); espSerial.print(brRoad);          espSerial.print(F(","));
  espSerial.print("\"sv_angle\":"); espSerial.print(currentServoAngle); espSerial.print(F(","));
  espSerial.print("\"tl0\":"); espSerial.print(tlPhase[0]);     espSerial.print(F(","));
  espSerial.print("\"tl1\":"); espSerial.print(tlPhase[1]);
  espSerial.println(F("}"));

  // Serial Monitor mirror
  Serial.print(smoothSolarV,1); Serial.print(F("V|"));
  Serial.print(smoothSolarI,0); Serial.print(F("mA|"));
  Serial.print(smoothCrankV,1); Serial.print(F("V|"));
  Serial.print(smoothCrankI,0); Serial.print(F("mA|"));
  Serial.print(smoothBusV,  1); Serial.print(F("V|"));
  Serial.print(smoothBusI,  0); Serial.print(F("mA|Lvl:"));
  Serial.print(gridLevel);      Serial.print(F("|Servo:"));
  Serial.print(currentServoAngle); Serial.print(F("|TL:"));
  Serial.print(tlPhase[0]); Serial.print(F("/"));
  Serial.println(tlPhase[1]);
}

// ── Buzzer ────────────────────────────────────────────────────────────
void handleBuzzer(int level, unsigned long now) {
  if (level <= 1) {
    if (now - lastBuzzerToggle >= 800) {
      lastBuzzerToggle = now;
      buzzerState = !buzzerState;
      digitalWrite(PIN_BUZZER, buzzerState ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerState = false;
  }
}  

