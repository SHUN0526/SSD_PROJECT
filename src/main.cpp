#include <Wire.h>                       // I2C 통신(화면/OLED에 쓰임)을 쓰기 위한 라이브러리
#include <Adafruit_GFX.h>               // 그래픽(글자/도형) 그리는 라이브러리
#include <Adafruit_SSD1306.h>           // SSD1306 OLED 화면을 쓰기 위한 라이브러리
#include <EEPROM.h>                     // 전원을 꺼도 값이 남는 저장공간(EEPROM)을 쓰기 위한 라이브러리
#include <Arduino.h>                    // pinMode, digitalWrite 등 Arduino API
#include <avr/io.h>                     // AVR 레지스터
#include <avr/interrupt.h>              // ISR, 인터럽트 제어

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// ===== 좌표/동작 기본 설정 =====
const long DISPLAY_UNIT = 95;
const long SET_STEP_X = 1000;
const long SET_STEP_Y = 1000;
const long SET_STEP_Z = 1000;
const int  STEP_HALF_US = 150;

const long Y_PRE_DOWN_STEPS   = 100;

// ----- 플런저(주사기) -----
const long DROP_UNIT_STEPS = 200;
// dropSteps = 플런저 "목표 절대 위치(스텝)"로 사용 (200의 배수 단위로 표시/조정)
long dropSteps = DROP_UNIT_STEPS;
const long MIN_DROP = -20000;
const long MAX_DROP =  20000;

const unsigned long DROP_REPEAT_START_MS = 700;
const unsigned long DROP_REPEAT_RATE_MS  = 220;

// ===== 모터 방향 =====
const uint8_t X_DIR_POS   = LOW;
const uint8_t X_DIR_NEG   = HIGH;
const uint8_t STAGEY_DIR_POS = HIGH;
const uint8_t STAGEY_DIR_NEG = LOW;
const uint8_t LIFT_DIR_UP    = HIGH;
const uint8_t LIFT_DIR_DOWN  = LOW;
const uint8_t UD_DIR_DOWN = HIGH;  // 플런저를 눌러서 액체 나오는 방향
const uint8_t UD_DIR_UP   = LOW;   // 플런저를 올리는 방향

// ===== 드라이버 핀 =====
const uint8_t UD_PUL = 8,  UD_DIR = 9;
const uint8_t X_PUL  = 10, X_DIR  = 11; // 보드 X → 실제 Y
const uint8_t Y_PUL  = 12, Y_DIR  = 13; // 실제 Z
const uint8_t Z_PUL  = 14, Z_DIR  = 15; // 보드 Z → 실제 X

// ===== 버튼 핀 =====
const uint8_t BTN_SETX_UP    = 18;
const uint8_t BTN_SETX_DN    = 19;
const uint8_t BTN_SETY_UP    = 20;
const uint8_t BTN_SETY_DN    = 21;
const uint8_t BTN_SETZ_UP    = 22;
const uint8_t BTN_SETZ_DN    = 23;
const uint8_t BTN_GO         = 24;
const uint8_t BTN_RESET00    = 25;
const uint8_t BTN_PLUNGER_UP = 6;
const uint8_t BTN_PLUNGER_DN = 7;

// ===== 현재/목표 위치 =====
long curX = 0, curY = 0, curZ = 0;
long tgtX = 0, tgtY = 0, tgtZ = 0;

// ★ 플런저 현재/목표 위치(스텝): curUD는 실제 물리 위치, dropSteps를 절대 목표로 사용
long curUD = 0;   // +방향: 누름(UD_DIR_DOWN), -방향: 올림(UD_DIR_UP)
long tgtUD = 0;

// ===== 이동 한계 =====
const long MIN_X = 0;
const long MAX_X = 170 * DISPLAY_UNIT;
const long MIN_Y = -220 * DISPLAY_UNIT;
const long MAX_Y = 0;

const long MIN_Z = -500 * DISPLAY_UNIT;
const long MAX_Z = 0 * DISPLAY_UNIT;

// Z 전용 속도 파라미터
const int  STEP_HALF_US_Z_MIN   = 220;
const int  STEP_HALF_US_Z_START = 500;
const long Z_ACCEL_STEPS        = 1500; // (현재는 사용 안 함)


// ===== 클램프 =====
inline void clampXY() {
  if (tgtX < MIN_X) tgtX = MIN_X; else if (tgtX > MAX_X) tgtX = MAX_X;
  if (tgtY < MIN_Y) tgtY = MIN_Y; else if (tgtY > MAX_Y) tgtY = MAX_Y;
}
inline void clampZ() {
  if (tgtZ < MIN_Z) tgtZ = MIN_Z; else if (tgtZ > MAX_Z) tgtZ = MAX_Z;
}
inline void clampDrop() {
  if (dropSteps < MIN_DROP) dropSteps = MIN_DROP;
  else if (dropSteps > MAX_DROP) dropSteps = MAX_DROP;
}
inline void clampUDPos() {
  if (curUD < MIN_DROP) curUD = MIN_DROP;
  else if (curUD > MAX_DROP) curUD = MAX_DROP;
}

// ===== EEPROM 구조 =====
struct PersistV5 { uint32_t magic; long curX; long curY; long curZ; long curUD; long dropSteps; };
struct PersistV4 { uint32_t magic; long curX; long curY; long curZ; long dropSteps; };
struct PersistV3 { uint32_t magic; long tgtX; long tgtY; long tgtZ; long dropSteps; };

const uint32_t MAGIC_V5 = 0xC0FFEE7A;
const uint32_t MAGIC_V4 = 0xC0FFEE79;
const uint32_t MAGIC_V3 = 0xC0FFEE78;
const int EEPROM_ADDR = 0;

bool needSave = false;
unsigned long lastEditMs = 0;
const unsigned long SAVE_DELAY_MS = 100;

const unsigned long POS_SAVE_INTERVAL_MS = 50;
const long          POS_SAVE_DELTA_STEPS = 50;
unsigned long lastPosSaveMs = 0;
long lastSavedX = 0, lastSavedY = 0, lastSavedZ = 0, lastSavedUD = 0;

long lastOLEDUnitX = 0, lastOLEDUnitY = 0, lastOLEDUnitZ = 0;
// ★ 플런저 현재 N(200스텝 단위) 표시 상태
long lastOLEDBurstsUD = 0;

// ===== 버튼 이벤트 =====
enum { EV_NONE = 0, EV_PRESS = 1, EV_REPEAT = 2 };

// 스텝 펄스(블로킹 버전, 플런저 등에서 사용)
static inline void stepHalf(uint8_t PUL, int half_us) {
  digitalWrite(PUL, HIGH);  delayMicroseconds(half_us);
  digitalWrite(PUL, LOW);   delayMicroseconds(half_us);
}

// ─────────────────────────────────────────────────────────────
// 타이머 기반 스텝 드라이버
// Timer1 = Y축(보드 X_PUL/X_DIR) / Timer3 = X축(보드 Z_PUL/Z_DIR) / Timer2 = Z축(보드 Y_PUL/Y_DIR)
// ─────────────────────────────────────────────────────────────
volatile bool isMoving = false;

// Timer1(Y)
volatile uint8_t* T1_PUL_PORT;
volatile uint8_t  T1_PUL_MASK;
volatile long     T1_steps_remaining = 0;
volatile long     T1_steps_done = 0;
volatile int8_t   T1_step_sign  = +1;
volatile bool     T1_pulse_state = false;

// Timer3(X)
volatile uint8_t* T3_PUL_PORT;
volatile uint8_t  T3_PUL_MASK;
volatile long     T3_steps_remaining = 0;
volatile long     T3_steps_done = 0;
volatile int8_t   T3_step_sign  = +1;
volatile bool     T3_pulse_state = false;

// Timer2(Z)
volatile uint8_t* T2_PUL_PORT;
volatile uint8_t  T2_PUL_MASK;
volatile long     T2_steps_remaining = 0;
volatile long     T2_steps_done = 0;
volatile int8_t   T2_step_sign  = +1;
volatile bool     T2_pulse_state = false;

// Timer1 ISR
ISR(TIMER1_COMPA_vect) {
  if (T1_steps_remaining > 0) {
    if (T1_pulse_state) { *T1_PUL_PORT |=  T1_PUL_MASK; }
    else { *T1_PUL_PORT &= ~T1_PUL_MASK; T1_steps_remaining--; T1_steps_done++; }
    T1_pulse_state = !T1_pulse_state;
  } else {
    TIMSK &= ~(1 << OCIE1A);
    *T1_PUL_PORT &= ~T1_PUL_MASK;
  }
}

// Timer3 ISR
ISR(TIMER3_COMPA_vect) {
  if (T3_steps_remaining > 0) {
    if (T3_pulse_state) { *T3_PUL_PORT |=  T3_PUL_MASK; }
    else { *T3_PUL_PORT &= ~T3_PUL_MASK; T3_steps_remaining--; T3_steps_done++; }
    T3_pulse_state = !T3_pulse_state;
  } else {
    ETIMSK &= ~(1 << OCIE3A);
    *T3_PUL_PORT &= ~T3_PUL_MASK;
  }
}

// Timer2 ISR (ATmega128: TIMER2_COMP_vect)
ISR(TIMER2_COMP_vect) {
  if (T2_steps_remaining > 0) {
    if (T2_pulse_state) { *T2_PUL_PORT |=  T2_PUL_MASK; }
    else { *T2_PUL_PORT &= ~T2_PUL_MASK; T2_steps_remaining--; T2_steps_done++; }
    T2_pulse_state = !T2_pulse_state;
  } else {
    TIMSK &= ~(1 << OCIE2);
    *T2_PUL_PORT &= ~T2_PUL_MASK;
  }
}

// half_us = 반주기(us) → 1스텝 주기 = 2*half_us
static inline void startTimerMotor(
  uint8_t timer_id, uint8_t pulPin, uint8_t dirPin, uint8_t dirLevel,
  long steps, int half_us, int8_t step_sign
){
  if (steps <= 0) return;

  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, dirLevel);
  delayMicroseconds(100);

  noInterrupts();

  if (timer_id == 1) {
    // 16-bit /8 CTC
    uint32_t ocr = (uint32_t)(((uint64_t)F_CPU / 8ULL) * (uint64_t)half_us / 1000000ULL);
    if (ocr == 0) ocr = 1; ocr -= 1;

    T1_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin));
    T1_PUL_MASK = digitalPinToBitMask(pulPin);
    T1_steps_remaining = steps; T1_steps_done = 0; T1_step_sign = step_sign; T1_pulse_state = false;

    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
    TCCR1B |= (1 << WGM12);
    OCR1A   = (uint16_t)ocr;
    TCCR1B |= (1 << CS11);          // /8
    TIMSK  |= (1 << OCIE1A);
  }
  else if (timer_id == 3) {
    // 16-bit /8 CTC
    uint32_t ocr = (uint32_t)(((uint64_t)F_CPU / 8ULL) * (uint64_t)half_us / 1000000ULL);
    if (ocr == 0) ocr = 1; ocr -= 1;

    T3_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin));
    T3_PUL_MASK = digitalPinToBitMask(pulPin);
    T3_steps_remaining = steps; T3_steps_done = 0; T3_step_sign = step_sign; T3_pulse_state = false;

    TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;
    TCCR3B |= (1 << WGM32);
    OCR3A   = (uint16_t)ocr;
    TCCR3B |= (1 << CS31);          // /8
    ETIMSK |= (1 << OCIE3A);
  }
  else if (timer_id == 2) {
    // 8-bit Timer2: 프리스케일러를 골라 OCR2(0..255) 범위에 맞춤
    // ticks(half) = F_CPU * half_us / 1e6  @ /1
    uint32_t ticks = ((uint64_t)F_CPU * (uint64_t)half_us) / 1000000ULL;

    struct Opt { uint16_t presc; uint8_t cs; } opts[] = {
      { 8,    (1<<CS21) },
      { 32,   (1<<CS21)|(1<<CS20) },
      { 64,   (1<<CS22) },
      { 128,  (1<<CS22)|(1<<CS20) },
      { 256,  (1<<CS22)|(1<<CS21) },
      { 1024, (1<<CS22)|(1<<CS21)|(1<<CS20) }
    };

    uint8_t csbits = 0;
    uint8_t ocr8 = 255;
    for (uint8_t i=0; i<sizeof(opts)/sizeof(opts[0]); i++) {
      uint32_t cnt = ticks / opts[i].presc;
      if (cnt >= 2 && cnt <= 256) { ocr8 = (uint8_t)(cnt - 1); csbits = opts[i].cs; break; }
    }

    T2_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin));
    T2_PUL_MASK = digitalPinToBitMask(pulPin);
    T2_steps_remaining = steps; T2_steps_done = 0; T2_step_sign = step_sign; T2_pulse_state = false;

    TCCR2 = 0; TCNT2 = 0;
    TCCR2 |= (1 << WGM21);   // CTC
    OCR2   = ocr8;
    TCCR2 |= csbits;
    TIMSK |= (1 << OCIE2);
  }

  interrupts();
}

// 위치 저장 스로틀
inline void maybeSaveCurPosDuringMove() {
  unsigned long now = millis();
  if ((now - lastPosSaveMs) >= POS_SAVE_INTERVAL_MS) {
    long dx = curX - lastSavedX, dy = curY - lastSavedY, dz = curZ - lastSavedZ, du = curUD - lastSavedUD;
    if ( (dx>=POS_SAVE_DELTA_STEPS || dx<=-POS_SAVE_DELTA_STEPS) ||
         (dy>=POS_SAVE_DELTA_STEPS || dy<=-POS_SAVE_DELTA_STEPS) ||
         (dz>=POS_SAVE_DELTA_STEPS || dz<=-POS_SAVE_DELTA_STEPS) ||
         (du>=POS_SAVE_DELTA_STEPS || du<=-POS_SAVE_DELTA_STEPS) ) {
      PersistV5 p { MAGIC_V5, curX, curY, curZ, curUD, dropSteps };
      EEPROM.put(EEPROM_ADDR, p);
      lastPosSaveMs = now; lastSavedX = curX; lastSavedY = curY; lastSavedZ = curZ; lastSavedUD = curUD;
    }
  }
}

// OLED 갱신
void updateOLED() {
  long x = curX / DISPLAY_UNIT;
  long y = curY / DISPLAY_UNIT;
  long z = curZ / DISPLAY_UNIT;
  // ★ 목표가 아니라 "현재" N 표시
  long burstsCur = curUD / DROP_UNIT_STEPS;

  display.clearDisplay();
  display.setTextColor(WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);   display.print(F("X:")); display.print(x);

  display.setCursor(50, 0);  display.print(F("Y:")); display.print(y);

  display.setCursor(0, 18);  display.print(F("Z:")); display.print(z);
  display.setTextSize(1);
  display.setCursor(88, 23); display.print(F("D:")); display.print(burstsCur);

  display.display();
}

// 즉시 저장
void savePersistNow() {
  PersistV5 p { MAGIC_V5, curX, curY, curZ, curUD, dropSteps };
  EEPROM.put(EEPROM_ADDR, p);
  lastSavedX = curX; lastSavedY = curY; lastSavedZ = curZ; lastSavedUD = curUD;
  lastPosSaveMs = millis();
  lastOLEDUnitX = curX / DISPLAY_UNIT;
}

inline void maybeUpdateOLEDWhileMoving(char axis) {
  long ux = curX / DISPLAY_UNIT, uy = curY / DISPLAY_UNIT, uz = curZ / DISPLAY_UNIT;
  bool changed = false;
  if (axis == 'X' && ux != lastOLEDUnitX) { lastOLEDUnitX = ux; changed = true; }
  if (axis == 'Y' && uy != lastOLEDUnitY) { lastOLEDUnitY = uy; changed = true; }
  if (axis == 'Z' && uz != lastOLEDUnitZ) { lastOLEDUnitZ = uz; changed = true; }

  // ★ 추가: 플런저 U축 진행(200스텝마다 N이 1씩 변할 때만 갱신)
  if (axis == 'U') {
    long ub = curUD / DROP_UNIT_STEPS;
    if (ub != lastOLEDBurstsUD) {
      lastOLEDBurstsUD = ub;
      changed = true;
      // 버스트 단위마다 즉시 저장(전원 중단 시 재개 정확도↑)
      savePersistNow();
    }
  }

  if (changed) updateOLED();
}

// 진행량 반영
inline void serviceAxisProgress() {
  long d1 = 0, d3 = 0, d2 = 0;
  noInterrupts();
  if (T1_steps_done) { d1 = T1_steps_done; T1_steps_done = 0; }
  if (T3_steps_done) { d3 = T3_steps_done; T3_steps_done = 0; }
  if (T2_steps_done) { d2 = T2_steps_done; T2_steps_done = 0; }
  interrupts();

  if (d3) { curX += (long)T3_step_sign * d3; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('X'); }
  if (d1) { curY += (long)T1_step_sign * d1; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('Y'); }
  if (d2) { curZ += (long)T2_step_sign * d2; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('Z'); }
}

// 블로킹 스텝(플런저/기타)
void moveAxisSteps(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                   long steps, int half_us, long &curAxis, int stepSign, char axisTag) {
  if (steps <= 0) return;
  digitalWrite(DIR, dirLevel);
  delayMicroseconds(100);
  for (long i = 0; i < steps; i++) {
    stepHalf(PUL, half_us);
    curAxis += stepSign;
    maybeSaveCurPosDuringMove();
    // ★ 모든 축에 대해 OLED 갱신 훅 호출 (U 포함)
    maybeUpdateOLEDWhileMoving(axisTag);
  }
}

void requestSave() { needSave = true; lastEditMs = millis(); }
void trySaveIfIdle() {
  if (needSave && (millis() - lastEditMs >= SAVE_DELAY_MS)) { savePersistNow(); needSave = false; }
}

// 로드
void loadPersist() {
  uint32_t magic = 0; EEPROM.get(EEPROM_ADDR, magic);
  if (magic == MAGIC_V5) {
    PersistV5 p; EEPROM.get(EEPROM_ADDR, p);
    curX = p.curX; curY = p.curY; curZ = p.curZ; curUD = p.curUD; dropSteps = p.dropSteps;
  } else if (magic == MAGIC_V4) {
    PersistV4 p; EEPROM.get(EEPROM_ADDR, p);
    curX = p.curX; curY = p.curY; curZ = p.curZ; dropSteps = p.dropSteps; curUD = 0;
  } else if (magic == MAGIC_V3) {
    PersistV3 p; EEPROM.get(EEPROM_ADDR, p);
    curX = p.tgtX; curY = p.tgtY; curZ = p.tgtZ; dropSteps = p.dropSteps; curUD = 0;
  } else {
    curX = curY = curZ = 0; dropSteps = DROP_UNIT_STEPS; curUD = 0;
  }
  clampUDPos();
  tgtX = curX; tgtY = curY; tgtZ = curZ;
  clampXY(); clampZ(); clampDrop();
  lastOLEDUnitX = curX / DISPLAY_UNIT;
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
  lastSavedUD  = curUD;
  // ★ 현재 N 초기화
  lastOLEDBurstsUD = curUD / DROP_UNIT_STEPS;
}

// 플런저: 목표 절대 위치(dropSteps)까지 이동
void doOneDrop() {
  // 목표를 절대 위치로 해석
  tgtUD = dropSteps;          // 목표 스텝
  clampDrop();                // 목표 한계
  clampUDPos();               // 현재 한계

  long delta = tgtUD - curUD; // 남은 거리(스텝)
  if (delta == 0) { savePersistNow(); return; }

  if (delta > 0) {
    // 더 눌러야 함(UD_DIR_DOWN), curUD가 + 방향으로 증가
    moveAxisSteps(UD_PUL, UD_DIR, UD_DIR_DOWN, delta, STEP_HALF_US, curUD, +1, 'U');
  } else {
    // 올려야 함(UD_DIR_UP), curUD가 - 방향으로 감소
    moveAxisSteps(UD_PUL, UD_DIR, UD_DIR_UP,  -delta, STEP_HALF_US, curUD, -1, 'U');
  }
  savePersistNow();
}

// 버튼 배열/인덱스
const uint8_t BTN_PINS[] = {
  BTN_SETX_UP, BTN_SETX_DN,
  BTN_SETY_UP, BTN_SETY_DN,
  BTN_SETZ_UP, BTN_SETZ_DN,
  BTN_GO, BTN_RESET00,
  BTN_PLUNGER_UP, BTN_PLUNGER_DN
};
const uint8_t NBTN = sizeof(BTN_PINS) / sizeof(BTN_PINS[0]);

enum {
  IDX_XP = 0, IDX_XM,
  IDX_YP,     IDX_YM,
  IDX_ZP,     IDX_ZM,
  IDX_GO,     IDX_RESET,
  IDX_DROP_UP, IDX_DROP_DN
};

uint8_t  btnStable[NBTN];
uint8_t  btnLastReading[NBTN];
unsigned long btnLastChange[NBTN];
unsigned long btnPressStart[NBTN];
unsigned long btnLastRepeat[NBTN];

const unsigned long DEBOUNCE_MS     = 100;
const unsigned long REPEAT_START_MS = 400;
const unsigned long REPEAT_RATE_MS  = 90;

uint8_t pollBtn(uint8_t idx) {
  uint8_t pin = BTN_PINS[idx];
  uint8_t raw = digitalRead(pin);
  unsigned long now = millis();

  if (raw != btnLastReading[idx]) { btnLastReading[idx] = raw; btnLastChange[idx] = now; }

  if (now - btnLastChange[idx] > DEBOUNCE_MS) {
    if (btnStable[idx] != raw) {
      btnStable[idx] = raw;
      if (btnStable[idx] == LOW) { btnPressStart[idx] = now; btnLastRepeat[idx] = now; return EV_PRESS; }
    } else {
      if (btnStable[idx] == LOW) {
        if (now - btnPressStart[idx] >= REPEAT_START_MS &&
            now - btnLastRepeat[idx]  >= REPEAT_RATE_MS) {
          btnLastRepeat[idx] = now; return EV_REPEAT;
        }
      }
    }
  }
  return EV_NONE;
}

// D(6/7) 전용 처리 : dropSteps(=플런저 목표 절대 위치)를 200씩 증감
int8_t dropOwner = -1;
void handleDropButtons(bool &changed) {
  const long STEP_EDIT = DROP_UNIT_STEPS;
  bool upLow = (digitalRead(BTN_PLUNGER_UP) == LOW);
  bool dnLow = (digitalRead(BTN_PLUNGER_DN) == LOW);
  if (upLow && dnLow) return;

  static bool dropFired = false;
  static unsigned long dropPressStartMs = 0;
  static unsigned long dropLastRepeatMs  = 0;
  unsigned long now = millis();

  if (dropOwner == -1) {
    if (upLow && !dnLow) { dropOwner = 0; dropFired = false; dropPressStartMs = now; dropLastRepeatMs = now; }
    else if (dnLow && !upLow) { dropOwner = 1; dropFired = false; dropPressStartMs = now; dropLastRepeatMs = now; }
    else return;
  }

  if (dropOwner == 0) {
    if (upLow) {
      if (!dropFired) { dropSteps += STEP_EDIT; clampDrop(); changed = true; requestSave(); dropFired = true; }
      else if ((now - dropPressStartMs) >= DROP_REPEAT_START_MS &&
               (now - dropLastRepeatMs)  >= DROP_REPEAT_RATE_MS) {
        dropSteps += STEP_EDIT; clampDrop(); changed = true; requestSave(); dropLastRepeatMs = now;
      }
    } else { dropOwner = -1; dropFired = false; }
  } else if (dropOwner == 1) {
    if (dnLow) {
      if (!dropFired) { dropSteps -= STEP_EDIT; clampDrop(); changed = true; requestSave(); dropFired = true; }
      else if ((now - dropPressStartMs) >= DROP_REPEAT_START_MS &&
               (now - dropLastRepeatMs)  >= DROP_REPEAT_RATE_MS) {
        dropSteps -= STEP_EDIT; clampDrop(); changed = true; requestSave(); dropLastRepeatMs = now;
      }
    } else { dropOwner = -1; dropFired = false; }
  }
}

// 초기화
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.begin(115200);

  pinMode(UD_PUL, OUTPUT); pinMode(UD_DIR, OUTPUT);
  pinMode(X_PUL,  OUTPUT); pinMode(X_DIR,  OUTPUT);
  pinMode(Y_PUL,  OUTPUT); pinMode(Y_DIR,  OUTPUT);
  pinMode(Z_PUL,  OUTPUT); pinMode(Z_DIR,  OUTPUT);

  pinMode(BTN_SETX_UP,    INPUT_PULLUP);
  pinMode(BTN_SETX_DN,    INPUT_PULLUP);
  pinMode(BTN_SETY_UP,    INPUT_PULLUP);
  pinMode(BTN_SETY_DN,    INPUT_PULLUP);
  pinMode(BTN_SETZ_UP,    INPUT_PULLUP);
  pinMode(BTN_SETZ_DN,    INPUT_PULLUP);
  pinMode(BTN_GO,         INPUT_PULLUP);
  pinMode(BTN_RESET00,    INPUT_PULLUP);
  pinMode(BTN_PLUNGER_UP, INPUT_PULLUP);
  pinMode(BTN_PLUNGER_DN, INPUT_PULLUP);

  unsigned long t_init = millis();
  for (uint8_t i=0; i<NBTN; i++) {
    uint8_t raw = digitalRead(BTN_PINS[i]);
    btnStable[i]      = raw;
    btnLastReading[i] = raw;
    btnLastChange[i]  = t_init;
    btnPressStart[i]  = 0;
    btnLastRepeat[i]  = 0;
  }

  loadPersist();
  clampXY(); clampZ(); clampDrop(); clampUDPos();

  lastOLEDUnitX = curX / DISPLAY_UNIT;
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
  // ★ 현재 N 초기화(안전용)
  lastOLEDBurstsUD = curUD / DROP_UNIT_STEPS;

  updateOLED();
}

/*
// 프로토콜 설명(생략)
*/

unsigned char command0 = 0;
unsigned char InputSirialData0 = 0;
unsigned char Uart0ProtocolPointer = 0;
unsigned char Uart0ReciveCheckEnd = 0;

unsigned char Uart0_Data_Count = 0;
unsigned char Check_Xor =0;
unsigned char Check_Xor_Check =0;
unsigned char Check_Xor_TX = 0;

unsigned char Error_Tx = 0;
unsigned char Error = 0;
unsigned char Error_COM = 0;
unsigned char Error_BCC = 0;
unsigned char Error_LEN = 0;
unsigned char Error_LEN_Count = 0;

int X1_Rx = 0;
int Y1_Rx = 0;
int Z1_Rx = 0;
int F1_Rx = 0;

int X1 = 0;
int Y1 = 0;
int Z1 = 0;
int F1 = 0;

char X1_L_Rx = 0;
char Y1_L_Rx = 0;
char Z1_L_Rx = 0;
char F1_L_Rx = 0;
char X1_L = 0;
char Y1_L = 0;
char Z1_L = 0;
char F1_L = 0;

char X1_H_Rx = 0;
char Y1_H_Rx = 0;
char Z1_H_Rx = 0;
char F1_H_Rx = 0;
char X1_H = 0;
char Y1_H = 0;
char Z1_H = 0;
char F1_H = 0;

char FN_Port_Rx = 0;
char Error1_Rx = 0;
char Error1 = 0;

char FN_Port0_Rx = 0;
char FN_Port1_Rx = 0;
char FN_Port2_Rx = 0;
char FN_Port3_Rx = 0;
char FN_Port4_Rx = 0;
char FN_Port5_Rx = 0;
char FN_Port6_Rx = 0;
char FN_Port7_Rx = 0;

char START_MODE_STATUE = 0;
char END_MODE_STATUE = 0;

void UartRxProtocol()
{
  char Uart0_Data;
  if(Serial.available()) {
      Uart0_Data = Serial.read();
      switch(Uart0ProtocolPointer)
      {
        case 0:     if(0x7b == Uart0_Data ) { Uart0ProtocolPointer = 1; Uart0_Data_Count++; Error_COM = 0; }
                    else { Uart0ProtocolPointer = 0; Check_Xor =0; }
                    break;
        case 1:     if(0x5b == Uart0_Data ){ Uart0ProtocolPointer = 2; }
                    else { Uart0ProtocolPointer = 0; Check_Xor =0; }
                    break;
        case 2:     if(0x02 == Uart0_Data )  { Uart0ProtocolPointer = 10;  Error_LEN_Count = 0; }
                    else { Uart0ProtocolPointer = 0; Check_Xor =0; }
                    break;
        case 10:    if(0X00 == Uart0_Data){ Uart0ProtocolPointer = 11; Check_Xor ^= Uart0_Data; Error_LEN = 0; }
                    else { Uart0ProtocolPointer = 11; Check_Xor ^= Uart0_Data; Error_LEN = 1; }
                    break;
        case 11:    if(0x0D == Uart0_Data){ Uart0ProtocolPointer = 20; Check_Xor ^= Uart0_Data; Error_LEN = 0; Error = Error & 0x00; }
                    else { Uart0ProtocolPointer = 20; Check_Xor ^= Uart0_Data; Error_LEN = 1; Error = Error | 0x08; }
                    break;
        case 20:    if(0x31 == Uart0_Data){ Uart0ProtocolPointer = 21; Check_Xor ^= Uart0_Data; Error_LEN_Count++; }
                    else { Uart0ProtocolPointer = 21; Check_Xor ^= Uart0_Data; Error_BCC = 1; }
                    break;
        case 21:    if(0x51 == Uart0_Data){ Uart0ProtocolPointer = 30; Check_Xor ^= Uart0_Data; Error_LEN_Count++; }
                    else { Uart0ProtocolPointer = 30; Check_Xor ^= Uart0_Data; Error_BCC = 1; }
                    break;
        case 30:    Uart0ProtocolPointer = 31; X1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 31:    Uart0ProtocolPointer = 32; X1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 32:    Uart0ProtocolPointer = 33; Y1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 33:    Uart0ProtocolPointer = 34; Y1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 34:    Uart0ProtocolPointer = 35; Z1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 35:    Uart0ProtocolPointer = 36; Z1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 36:    Uart0ProtocolPointer = 37; F1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 37:    Uart0ProtocolPointer = 38; F1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 38:    Uart0ProtocolPointer = 39; FN_Port_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 39:    Uart0ProtocolPointer = 90; Error1_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break;
        case 90:    if(Check_Xor == Uart0_Data || 0xBC == Uart0_Data) {
                         Check_Xor_Check = Check_Xor; command0 = 1; Uart0ProtocolPointer = 100;
                         Check_Xor =0; Error_BCC = 0; Error_LEN_Count++; Error = Error & 0x00;
                    } else {
                         Check_Xor_Check = Check_Xor; command0 = 1; Uart0ProtocolPointer = 100;
                         Check_Xor =0; Error_BCC = 1; Error_LEN_Count++; Error = Error | 0x04;
                    }
                    break;
        case 100:   if(0x03 == Uart0_Data )  { Uart0ProtocolPointer = 101; } else { Uart0ProtocolPointer = 0; Check_Xor =0; } break;
        case 101:   if(0x5d == Uart0_Data )  { Uart0ProtocolPointer = 102; } else { Uart0ProtocolPointer = 0; Check_Xor =0; } break;
        case 102:   if(0x7d == Uart0_Data )  { Uart0ProtocolPointer = 0; Uart0ReciveCheckEnd = 1; Check_Xor =0; Error_COM = 0; }
                    else { Uart0ProtocolPointer = 0; }
                    break;
        default:    Uart0ProtocolPointer = 0; Check_Xor =0; break;
      }
  }
}

// ISR 기반 시작(블로킹 안 함)
void startAxisSteps_ISR(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                        long steps, int half_us, int8_t stepSign, char axisTag) {
  if (steps <= 0) return;
  uint8_t timer_id = 0;
  if (axisTag == 'X') timer_id = 3;      // 보드 Z → 실제 X
  else if (axisTag == 'Y') timer_id = 1; // 보드 X → 실제 Y
  else if (axisTag == 'Z') timer_id = 2; // 보드 Y → 실제 Z
  else return;
  startTimerMotor(timer_id, PUL, DIR, dirLevel, steps, half_us, stepSign);
}

// ISR 기반 이동(완료까지 대기, 진행량/수신/저장만 처리)
void moveAxisSteps_ISR(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                       long steps, int half_us, long &curAxis, int stepSign, char axisTag) {
  if (steps <= 0) return;

  uint8_t timer_id = 0;
  if (axisTag == 'X') timer_id = 3;
  else if (axisTag == 'Y') timer_id = 1;
  else if (axisTag == 'Z') timer_id = 2;
  else {
    // (플런저 등) 타이머 미사용 축
    moveAxisSteps(PUL, DIR, dirLevel, steps, half_us, curAxis, stepSign, axisTag);
    return;
  }

  startTimerMotor(timer_id, PUL, DIR, dirLevel, steps, half_us, (int8_t)stepSign);

  for (;;) {
    serviceAxisProgress();
    UartRxProtocol();
    trySaveIfIdle();
    bool done = (timer_id == 3) ? (T3_steps_remaining == 0) :
                (timer_id == 1) ? (T1_steps_remaining == 0) :
                                  (T2_steps_remaining == 0);
    if (done) break;
  }
  serviceAxisProgress();
}

// 목표로 이동 + 드롭 + Z복귀(절대 0)
void goToTargetXYZ() {
  if (isMoving) return;
  isMoving = true;

  clampXY(); clampZ();

  long dx = tgtX - curX;
  long dy = tgtY - curY;
  long dz = tgtZ - curZ;

  int8_t sx = (dx >= 0) ? +1 : -1;
  int8_t sy = (dy >= 0) ? +1 : -1;

  long adx = (dx >= 0) ? dx : -dx;
  long ady = (dy >= 0) ? dy : -dy;

  // X/Y 동착을 위한 속도 보정(선택)
  int half_x = STEP_HALF_US;
  int half_y = STEP_HALF_US;
  long maxSteps = (adx > ady) ? adx : ady;
  if (maxSteps > 0) {
    if (adx > 0) half_x = (int)((long)STEP_HALF_US * maxSteps / adx);
    if (ady > 0) half_y = (int)((long)STEP_HALF_US * maxSteps / ady);
    if (half_x < 1) half_x = 1; if (half_y < 1) half_y = 1;
  }

  // X/Y 동시 시작
  bool startedX = false, startedY = false;
  if (adx > 0) { startAxisSteps_ISR(Z_PUL, Z_DIR, (sx > 0 ? X_DIR_POS : X_DIR_NEG), adx, half_x, sx, 'X'); startedX = true; }
  if (ady > 0) { startAxisSteps_ISR(X_PUL, X_DIR, (sy > 0 ? STAGEY_DIR_POS : STAGEY_DIR_NEG), ady, half_y, sy, 'Y'); startedY = true; }

  while ( (startedX && T3_steps_remaining > 0) || (startedY && T1_steps_remaining > 0) ) {
    serviceAxisProgress(); UartRxProtocol(); trySaveIfIdle();
  }
  serviceAxisProgress();

  // ── Z 이동 (Timer2 기반, 균일 펄스) ──
  if (dz > 0) {
    moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_UP,   dz, STEP_HALF_US_Z_MIN, curZ, +1, 'Z');
  } else if (dz < 0) {
    moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_DOWN, -dz, STEP_HALF_US_Z_MIN, curZ, -1, 'Z');
  }

  // 플런저: 절대 목표까지 수행
  doOneDrop();

  // ── Z 복귀: 항상 절대 0(=MAX_Z)으로 복귀 ──
  if (curZ != MAX_Z) {
    long dz_home = MAX_Z - curZ;
    if (dz_home > 0) {
      moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_UP,   dz_home, STEP_HALF_US_Z_MIN, curZ, +1, 'Z');
    } else {
      moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_DOWN, -dz_home, STEP_HALF_US_Z_MIN, curZ, -1, 'Z');
    }
  }

  savePersistNow();
  isMoving = false;
}

void Serial_Main0(void)
{
  if(Uart0ReciveCheckEnd) {
    if(command0 == 1) {
        X1_L = X1_H_Rx; X1_H = X1_L_Rx;
        Y1_L = Y1_H_Rx; Y1_H = Y1_L_Rx;
        Z1_L = Z1_H_Rx; Z1_H = Z1_L_Rx;
        F1_L = F1_H_Rx; F1_H = F1_L_Rx;

        X1 = ((uint16_t)X1_H << 8) | (uint8_t)X1_L;
        Y1 = ((uint16_t)Y1_H << 8) | (uint8_t)Y1_L;
        Z1 = ((uint16_t)Z1_H << 8) | (uint8_t)Z1_L;
        F1 = ((uint16_t)F1_H << 8) | (uint8_t)F1_L;

        tgtX      = (long)X1 * DISPLAY_UNIT;
        tgtY      = (long)Y1 * DISPLAY_UNIT;
        tgtZ      = (long)Z1 * DISPLAY_UNIT;
        dropSteps = (long)F1 * DROP_UNIT_STEPS;   // ★ 플런저 "절대 목표"로 해석
        clampXY(); clampZ(); clampDrop();
        updateOLED();

        if (!isMoving) { goToTargetXYZ(); }
        requestSave();
        trySaveIfIdle();

        FN_Port0_Rx = FN_Port_Rx & 0x80;
        FN_Port1_Rx = FN_Port_Rx & 0x40;
        FN_Port2_Rx = FN_Port_Rx & 0x20;
        FN_Port3_Rx = FN_Port_Rx & 0x10;
        FN_Port4_Rx = FN_Port_Rx & 0x08;
        FN_Port5_Rx = FN_Port_Rx & 0x04;
        FN_Port6_Rx = FN_Port_Rx & 0x02;
        FN_Port7_Rx = FN_Port_Rx & 0x01;

        Error_Tx = Error;

        Check_Xor_TX = 0X00 ^ 0x0E ^ 0X31 ^ 0x52 ^
                       X1_L ^ X1_H ^
                       Y1_L ^ Y1_H ^
                       Z1_L ^ Z1_H ^
                       F1_L ^ F1_H ^
                       START_MODE_STATUE ^ START_MODE_STATUE ^
                       Error_Tx;

        Serial.write(0x7B); Serial.write(0x5B); Serial.write(0x02);
        Serial.write(0X00); Serial.write(0x0E);
        Serial.write(0x31); Serial.write(0x52);

        Serial.write(X1_L); Serial.write(X1_H);
        Serial.write(Y1_L); Serial.write(Y1_H);
        Serial.write(Z1_L); Serial.write(Z1_H);
        Serial.write(F1_L); Serial.write(F1_H);

        Serial.write(START_MODE_STATUE);
        Serial.write(END_MODE_STATUE);

        Serial.write(Error_Tx);
        Serial.write(Check_Xor_TX);

        Serial.write(0x03); Serial.write(0x5D); Serial.write(0x7D);

        command0 = 0;
    }
    Uart0ReciveCheckEnd = 0;
  }
}

// 메인 루프
void loop() {
  bool changed = false;

  serviceAxisProgress();

  UartRxProtocol();
  Serial_Main0();

  // X
  uint8_t e0 = pollBtn(IDX_XP);
  uint8_t e1 = pollBtn(IDX_XM);
  if (e0 == EV_PRESS || e0 == EV_REPEAT) { tgtX += SET_STEP_X; changed = true; requestSave(); }
  if (e1 == EV_PRESS || e1 == EV_REPEAT) { tgtX -= SET_STEP_X; changed = true; requestSave(); }
  clampXY();

  // Y
  uint8_t e2 = pollBtn(IDX_YP);
  uint8_t e3 = pollBtn(IDX_YM);
  if (e2 == EV_PRESS || e2 == EV_REPEAT) { tgtY += SET_STEP_Y; changed = true; requestSave(); }
  if (e3 == EV_PRESS || e3 == EV_REPEAT) { tgtY -= SET_STEP_Y; changed = true; requestSave(); }
  clampXY();

  // Z
  uint8_t eZp = pollBtn(IDX_ZP);
  uint8_t eZm = pollBtn(IDX_ZM);
  if (eZp == EV_PRESS || eZp == EV_REPEAT) { tgtZ += SET_STEP_Z; changed = true; requestSave(); }
  if (eZm == EV_PRESS || eZm == EV_REPEAT) { tgtZ -= SET_STEP_Z; changed = true; requestSave(); }
  clampZ();

  // D(6/7) → 플런저 절대 목표 조정
  handleDropButtons(changed);

  // RESET
  uint8_t e5 = pollBtn(IDX_RESET);
  if (e5 == EV_PRESS) {
    tgtX = 0; tgtY = 0; tgtZ = 0;
    dropSteps = DROP_UNIT_STEPS;   // 플런저 목표는 기본값(200)으로
    clampXY(); clampZ(); clampDrop();
    updateOLED();
    if (!isMoving) { goToTargetXYZ(); }
    savePersistNow();
    changed = true;
  }

  if (changed) updateOLED();

  // GO (중복 드롭 금지: goToTargetXYZ()에서 이미 doOneDrop()+Z복귀 수행)
  uint8_t e4 = pollBtn(IDX_GO);
  if (e4 == EV_PRESS) {
    if (!isMoving) {
      goToTargetXYZ();
      updateOLED();
    }
  }

  trySaveIfIdle();
}
