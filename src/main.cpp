#include "Adafruit_DotStar.h"
#include "elapsedMillis.h"

#include "SPI.h"
#include <Wire.h>
#include "Adafruit_MCP9808.h"

// #define SIMULATE_HEAT
#define FLAT_BOARD_LAYOUT

#ifdef DO_SERIAL
#define Debug_print(...)    Serial.print(__VA_ARGS__)
#define Debug_println(...)  Serial.println(__VA_ARGS__)
#define Debug_begin(...)  Serial.begin(__VA_ARGS__)
#define Debug_delay(...)  delay(__VA_ARGS__)
#else
#define Debug_print(...)
#define Debug_println(...)
#define Debug_begin(...)
#define Debug_delay(...)
#endif

#define MAX_HEAT_HOURS 12L
#define MAX_HEAT_TIME (MAX_HEAT_HOURS * 60L * 60L * 1000L)

#define VALVE_MAX_CLOSE_TIME 12000
#define VALVE_WAIT 1000
#define VALVE_OPEN_FINISH_TIME 500
#define VALVE_CLOSE_FINISH_TIME 700

#define MOTOR_PIN_1 11
#define MOTOR_PIN_2 12

#ifdef FLAT_BOARD_LAYOUT
#define TEMP_LIMIT 85
#define MOTOR_SLEEP_PIN 13
#define KNOB_LIMIT_SENSOR_PIN MISO
#define FIRE_REQUEST_PIN A4
#define SWITCH_GROUND_PIN_1 2
#define SWITCH_GROUND_PIN_2 A3
#define TEMP_SENSOR_POWER_PIN 7
#else
#define TEMP_LIMIT 80
#define MOTOR_SLEEP_PIN 7
#define KNOB_LIMIT_SENSOR_PIN 9
#define FIRE_REQUEST_PIN 10
#endif

#define DOTSTAR_CLOCK_PIN 40
#define DOTSTAR_DATA_PIN 41

#define COLOR_RED 0xFF0000
#define COLOR_GREEN 0x00FF00
#define COLOR_BLUE 0x0000FF
#define COLOR_YELLOW 0xFFFF00
#define COLOR_ORANGE 0xFF8000
#define COLOR_PURPLE 0xA020F0
#define COLOR_CYAN 0x00FFFF
#define COLOR_BLACK 0x000000

Adafruit_DotStar statusLED = Adafruit_DotStar(1, DOTSTAR_DATA_PIN, DOTSTAR_CLOCK_PIN, DOTSTAR_BGR);

Adafruit_MCP9808 tempSensor = Adafruit_MCP9808();
bool validSensor = false;
bool overTemp = false;

typedef enum {
  valveClosed = 0,
  valveOpening,
  valveFinishOpening,
  valveOpen,
  valveClosing,
  valveFinishClosing,
  valveUnknown
} ValveState;

ValveState currentValveState = valveClosed;
elapsedMillis valveWaitTime;

uint32_t statusCount = 0;

uint64_t millis64() {
    static uint32_t low32 = 0, high32 = 0;
    uint32_t new_low32 = millis();
    if (new_low32 < low32) high32++;
    low32 = new_low32;
    return (uint64_t) high32 << 32 | low32;
}

class elapsedMillis64
{
private:
  uint64_t ms;
public:
  elapsedMillis64(void) { ms = millis64(); }
  elapsedMillis64(uint64_t val) { ms = millis64() - val; }
  elapsedMillis64(const elapsedMillis64 &orig) { ms = orig.ms; }
  operator uint64_t () const { return millis64() - ms; }
  elapsedMillis64 & operator = (const elapsedMillis64 &rhs) { ms = rhs.ms; return *this; }
  elapsedMillis64 & operator = (uint64_t val) { ms = millis64() - val; return *this; }
  elapsedMillis64 & operator -= (uint64_t val)      { ms += val ; return *this; }
  elapsedMillis64 & operator += (uint64_t val)      { ms -= val ; return *this; }
  elapsedMillis64 operator - (int val) const           { elapsedMillis64 r(*this); r.ms += val; return r; }
  elapsedMillis64 operator - (unsigned int val) const  { elapsedMillis64 r(*this); r.ms += val; return r; }
  elapsedMillis64 operator - (long val) const          { elapsedMillis64 r(*this); r.ms += val; return r; }
  elapsedMillis64 operator - (unsigned long val) const { elapsedMillis64 r(*this); r.ms += val; return r; }
  elapsedMillis64 operator - (int64_t val) const          { elapsedMillis64 r(*this); r.ms += val; return r; }
  elapsedMillis64 operator - (uint64_t val) const { elapsedMillis64 r(*this); r.ms += val; return r; }
  elapsedMillis64 operator + (int val) const           { elapsedMillis64 r(*this); r.ms -= val; return r; }
  elapsedMillis64 operator + (unsigned int val) const  { elapsedMillis64 r(*this); r.ms -= val; return r; }
  elapsedMillis64 operator + (long val) const          { elapsedMillis64 r(*this); r.ms -= val; return r; }
  elapsedMillis64 operator + (unsigned long val) const { elapsedMillis64 r(*this); r.ms -= val; return r; }
  elapsedMillis64 operator + (int64_t val) const          { elapsedMillis64 r(*this); r.ms -= val; return r; }
  elapsedMillis64 operator + (uint64_t val) const { elapsedMillis64 r(*this); r.ms -= val; return r; }
};

elapsedMillis64 fireOnTime;

void setStatusColor(uint32_t color) {
  #ifdef FLAT_BOARD_LAYOUT
  color = (color & 0xFCFCFC) >> 2;    // reduce color to 1/4
  #else
  color = (color & 0xF0F0F0) >> 4;    // reduce coloor to 1/16
  #endif

  statusLED.setPixelColor(0, color);
  statusLED.show();
}

bool heatRequested() {
  #ifdef SIMULATE_HEAT
  return ((statusCount/10)%2);
  #endif

  return digitalRead(FIRE_REQUEST_PIN) == LOW;
}

bool knobIsAtLimit() {
  #ifdef SIMULATE_HEAT
  if (valveWaitTime > 3000)
    return true;
  #endif

  return (valveWaitTime>VALVE_WAIT) && (digitalRead(KNOB_LIMIT_SENSOR_PIN) == LOW);
}

void motorForward() {
  digitalWrite(MOTOR_PIN_2, HIGH);
  digitalWrite(MOTOR_PIN_1, LOW);
  digitalWrite(MOTOR_SLEEP_PIN, HIGH);
  valveWaitTime = 0;
}

void motorBackward() {
  digitalWrite(MOTOR_PIN_1, HIGH);
  digitalWrite(MOTOR_PIN_2, LOW);
  digitalWrite(MOTOR_SLEEP_PIN, HIGH);
  valveWaitTime = 0;
}

void motorStop() {
  digitalWrite(MOTOR_PIN_1, LOW);
  digitalWrite(MOTOR_PIN_2, LOW);
  digitalWrite(MOTOR_SLEEP_PIN, LOW);
}

void updateStatusLED() {
  switch (currentValveState) {
    case valveClosed: setStatusColor((overTemp) ? COLOR_CYAN : COLOR_GREEN);   break;
    case valveFinishOpening:
    case valveOpening: setStatusColor(COLOR_YELLOW);   break;
    case valveOpen: setStatusColor(COLOR_ORANGE);   break;
    case valveFinishClosing:
    case valveClosing: setStatusColor(COLOR_YELLOW);   break;
    case valveUnknown: setStatusColor(COLOR_RED);   break;
  }
}

void setValveState(ValveState desiredState) {
  if (desiredState != currentValveState) {
    if (desiredState == valveClosed) {
      if (currentValveState != valveClosing && currentValveState != valveFinishClosing) {
        motorBackward();
        Debug_println("Valve is Closing");
        currentValveState = valveClosing;
      }
    }
    else if (desiredState == valveOpen) {
      if (currentValveState != valveOpening && currentValveState != valveFinishOpening) {
        motorForward();
        Debug_println("Valve is Opening");
        currentValveState = valveOpening;
      }
    }
  }
}

void updateValveState() {
  static elapsedMillis finishTime;

  if (knobIsAtLimit()) {
    if (currentValveState == valveOpening) {
      finishTime = 0;
      currentValveState = valveFinishOpening;
      Debug_println("Ensure fully open...");
    }
    else if (currentValveState == valveFinishOpening && finishTime > VALVE_OPEN_FINISH_TIME) {
      currentValveState = valveOpen;
      motorStop();
      Debug_println("Valve is Open");
    }
    else if (currentValveState == valveClosing) {
      finishTime = 0;
      currentValveState = valveFinishClosing;
      Debug_println("Ensure fully closed...");
    }
    else if (currentValveState == valveFinishClosing && finishTime > VALVE_CLOSE_FINISH_TIME) {
      currentValveState = valveClosed;
      motorStop();
      Debug_println("Valve is Closed");
    }
  }
}

void setup() {
    delay(2000);

    Debug_begin(9600);
    Debug_println("Fireplace Controller Setup");

    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(MOTOR_SLEEP_PIN, OUTPUT);
    motorStop();

    #ifdef FLAT_BOARD_LAYOUT
    pinMode(SWITCH_GROUND_PIN_1, OUTPUT);
    digitalWrite(SWITCH_GROUND_PIN_1, LOW);
    pinMode(SWITCH_GROUND_PIN_2, OUTPUT);
    digitalWrite(SWITCH_GROUND_PIN_2, LOW);
    pinMode(TEMP_SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(TEMP_SENSOR_POWER_PIN, HIGH);
    delay(200);
    #endif

    pinMode(KNOB_LIMIT_SENSOR_PIN, INPUT_PULLUP);
    pinMode(FIRE_REQUEST_PIN, INPUT_PULLUP);

    statusLED.begin();
    statusLED.show();

    if (tempSensor.begin()) {
      validSensor = true;
    }
    else {
      Serial.println("Couldn't find MCP9808!");
    }

    elapsedMillis resetTime;
    bool flag = false;
    elapsedMillis flagTime;

    Debug_println("Verify Valve is Closed...");
    setStatusColor(COLOR_CYAN);
    motorBackward();
    while ((resetTime < 1000 || !knobIsAtLimit()) && (resetTime<VALVE_MAX_CLOSE_TIME)) {
      if (flagTime > 300) {
        flagTime = 0;
        flag = !flag;
        setStatusColor((flag) ? COLOR_PURPLE : COLOR_CYAN);
      }
    };
    motorStop();

    if (resetTime>=VALVE_MAX_CLOSE_TIME) {
      Debug_println("*** Unable to Close Valve!");
      while (1) {
        setStatusColor(COLOR_RED);
        delay(100);
        setStatusColor(COLOR_BLACK);
        delay(100);
      };
    }

    Debug_println("Valve is Closed. Ready.");
}

elapsedMillis statusTime;

void loop() {
  millis64();

  bool wantsHeat = heatRequested();

  if (!wantsHeat) {
    fireOnTime = 0;
  }

  overTemp = false;

  if (wantsHeat && validSensor) {
    float temp = tempSensor.readTempC() * 1.8 + 32.0;

    if (temp > TEMP_LIMIT) {
      wantsHeat = false;
      overTemp = true;
    }
  }

  if (wantsHeat && fireOnTime > MAX_HEAT_TIME) {
    wantsHeat = false;
  }

  ValveState desiredState = (wantsHeat) ? valveOpen : valveClosed;

  setValveState(desiredState);
  updateValveState();

  updateStatusLED();

  if (statusTime > 1000) {
    statusTime = 0;

    #ifdef SIMULATE_HEAT
    Debug_print(statusCount++);
    Debug_print(": Loop - heat pin=");
    Debug_print(digitalRead(FIRE_REQUEST_PIN));
    Debug_println();
    #endif
  }

  delay(10);
}
