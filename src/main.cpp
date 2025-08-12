#include <Wire.h>                       // I2C 통신(화면/OLED에 쓰임)을 쓰기 위한 라이브러리
#include <Adafruit_GFX.h>               // 그래픽(글자/도형) 그리는 라이브러리
#include <Adafruit_SSD1306.h>           // SSD1306 OLED 화면을 쓰기 위한 라이브러리
#include <EEPROM.h>                     // 전원을 꺼도 값이 남는 저장공간(EEPROM)을 쓰기 위한 라이브러리
#include <Arduino.h>                    // pinMode, digitalWrite 등 Arduino API
#include <avr/io.h>                     // AVR 레지스터
#include <avr/interrupt.h>              // ISR, 인터럽트 제어

#define SCREEN_WIDTH 128                // OLED 가로 픽셀 수(128칸)
#define SCREEN_HEIGHT 32                // OLED 세로 픽셀 수(32칸)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire); // OLED 화면 객체 만들기(가로/세로/I2C)


// ===== 좌표/동작 기본 설정(숫자들 역할) =====

const long DISPLAY_UNIT = 95;         // 현재 종이 위의 칸 간격(1cm) 만큼 이동
const long SET_STEP_X = 1000;           // X버튼을 한 번 누를 때 목표 X를 1000스텝(=화면 1) 바꿈
const long SET_STEP_Y = 1000;           // Y버튼을 한 번 누를 때 목표 Y를 1000스텝 바꿈
const long SET_STEP_Z = 1000;           // Z버튼을 한 번 누를 때 목표 Z를 1000스텝 바꿈
const int  STEP_HALF_US = 150;          // 작으면 더 빨리, 크면 더 천천히 움직임

const long Y_PRE_DOWN_STEPS   = 100;    // 액체를 떨어뜨리기 전에 Z(리프트)를 살짝 내리는 양


// ----- 플런저(주사기를 눌러서 액체 나오는 부분) 설정 -----

const long DROP_UNIT_STEPS = 200;       // 플런저를 "한 번" 누르는 양을 200스텝으로 정함(방울 1개 느낌)
long dropSteps = DROP_UNIT_STEPS;       // 총 누를 양(200의 배수로 관리). D:200xN에서 N을 결정하는 값
const long MIN_DROP = -20000;                // D 최소값
const long MAX_DROP = 20000;            // D 최대값(너무 많이 누르지 않게 제한)

// --- 플런저(6/7) 전용 반복 타이밍(“짧게 1번, 충분히 길게 눌러야 천천히 반복”) ---
const unsigned long DROP_REPEAT_START_MS = 700;  // 길게 눌렀을 때 첫 반복까지 대기(0.7초)
const unsigned long DROP_REPEAT_RATE_MS  = 220;  // 반복 시작 후 반복 간격(0.22초)


// ===== 모터 회전 방향(어느 쪽으로 돌릴지) =====

const uint8_t X_DIR_POS   = LOW;        // X축을 +방향(오른쪽처럼)으로 움직일 때 DIR 핀에 줄 값
const uint8_t X_DIR_NEG   = HIGH;       // X축을 -방향(왼쪽처럼)으로 움직일 때 DIR 핀에 줄 값
const uint8_t STAGEY_DIR_POS = HIGH;    // Y축을 +방향(앞쪽처럼)으로 움직일 때 줄 값
const uint8_t STAGEY_DIR_NEG = LOW;     // Y축을 -방향(뒤쪽처럼)으로 움직일 때 줄 값
const uint8_t LIFT_DIR_UP    = HIGH;    // Z축을 위로 올릴 때 줄 값
const uint8_t LIFT_DIR_DOWN  = LOW;     // Z축을 아래로 내릴 때 줄 값
const uint8_t UD_DIR_DOWN = HIGH;       // 플런저를 아래로 눌러서 액체가 나오게 할 때 줄 값
const uint8_t UD_DIR_UP = LOW;

// ===== 드라이버 핀(모터와 버튼이 연결된 보드의 핀 번호) =====

const uint8_t UD_PUL = 8,  UD_DIR = 9;  // 플런저 모터의 펄스(PUL)/방향(DIR) 핀
const uint8_t X_PUL  = 10, X_DIR  = 11; // 스테이지 Y모터에 연결(보드 핀 이름은 X지만 Y로 씀) 펄스/방향 핀
const uint8_t Y_PUL  = 12, Y_DIR  = 13; // 리프트(Z축) 모터의 펄스/방향 핀
const uint8_t Z_PUL  = 14, Z_DIR  = 15; // 스테이지 X모터의 펄스/방향 핀


// ===== 버튼 핀(손으로 누르는 스위치가 연결된 핀 번호) =====

const uint8_t BTN_SETX_UP    = 18;      // X+ 버튼(누르면 오른쪽/플러스 방향으로 목표 X 증가)
const uint8_t BTN_SETX_DN    = 19;      // X- 버튼(누르면 왼쪽/마이너스 방향으로 목표 X 감소)
const uint8_t BTN_SETY_UP    = 20;      // Y+ 버튼(앞쪽/플러스 방향 목표 Y 증가)
const uint8_t BTN_SETY_DN    = 21;      // Y- 버튼(뒤쪽/마이너스 방향 목표 Y 감소)
const uint8_t BTN_SETZ_UP    = 22;      // Z+ 버튼(위로/플러스 방향 목표 Z 증가)
const uint8_t BTN_SETZ_DN    = 23;      // Z- 버튼(아래로/마이너스 방향 목표 Z 감소)
const uint8_t BTN_GO         = 24;      // GO 버튼(목표 X,Y,Z로 이동하고 그 자리에서 액체 떨어뜨리기)
const uint8_t BTN_RESET00    = 25;      // RESET 버튼(X=0,Y=0,Z=0 으로 초기화하고 이동)
const uint8_t BTN_PLUNGER_UP = 6;       // 플런저 D값을 +200(방울 +1) 늘리는 버튼
const uint8_t BTN_PLUNGER_DN = 7;       // 플런저 D값을 -200(방울 -1) 줄이는 버튼


// ===== 현재 위치/목표 위치(단위: 스텝) =====

long curX = 0, curY = 0, curZ = 0;      // curX/Y/Z: 지금 모터가 실제로 서 있는 자리(스텝)
long tgtX = 0, tgtY = 0, tgtZ = 0;      // tgtX/Y/Z: 우리가 가고 싶은 목표 자리(스텝)


// ===== 이동 가능한 범위(스텝 단위, 안전/기계 한계) =====

const long MIN_X = 0;                    // X의 최소 스텝(더 왼쪽으로 못 감)
const long MAX_X = 170 * DISPLAY_UNIT;    // X의 최대 스텝(오른쪽 한계). 화면으로는 170
const long MIN_Y = -220 * DISPLAY_UNIT;   // Y의 최소 스텝(뒤쪽 한계). 화면으로는 -250
const long MAX_Y = 0;                    // Y의 최대 스텝(앞쪽 한계). 화면으로는 0

const long MIN_Z = -500 * DISPLAY_UNIT;     // Z의 최소 스텝(아래쪽 한계). 화면으로는 -20
const long MAX_Z = 0 * DISPLAY_UNIT;    // Z의 최대 스텝(위쪽 한계). 화면으로는 300


// 목표 X,Y가 허용 범위를 넘지 않게 맞춰주는 함수
inline void clampXY() {
  if (tgtX < MIN_X) tgtX = MIN_X;        // 목표 X가 너무 작으면(왼쪽으로 너무 가면) 최소값으로 고정
  else if (tgtX > MAX_X) tgtX = MAX_X;   // 목표 X가 너무 크면(오른쪽으로 너무 가면) 최대값으로 고정

  if (tgtY < MIN_Y) tgtY = MIN_Y;        // 목표 Y가 너무 작으면(뒤로 너무 가면) 최소값으로 고정
  else if (tgtY > MAX_Y) tgtY = MAX_Y;   // 목표 Y가 너무 크면(앞으로 너무 가면) 최대값으로 고정
}

// 목표 Z가 허용 범위를 넘지 않게 맞춰주는 함수
inline void clampZ() {
  if (tgtZ < MIN_Z) tgtZ = MIN_Z;        // 목표 Z가 너무 작으면(아래로 너무 가면) 최소값으로 고정
  else if (tgtZ > MAX_Z) tgtZ = MAX_Z;   // 목표 Z가 너무 크면(위로 너무 가면) 최대값으로 고정
}

// 플런저 D(총 누르는 양) 안전 범위 안에서 맞춰주는 함수
inline void clampDrop() {
  if (dropSteps < MIN_DROP) dropSteps = MIN_DROP; // D가 0보다 작아지면 0으로
  else if (dropSteps > MAX_DROP) dropSteps = MAX_DROP; // D가 너무 크면 최대값으로 20000
}


// ===== EEPROM(전원 꺼도 남는 저장소)에 저장할 자료 모양 =====
struct PersistV4 {                       // ★ 실제 위치(curX/Y/Z)와 dropSteps를 저장
  uint32_t magic;
  long curX;
  long curY;
  long curZ;
  long dropSteps;
};
struct PersistV3 {                       // 과거 호환: X,Y,Z, dropSteps(D) 저장
  uint32_t magic;
  long tgtX;
  long tgtY;
  long tgtZ;
  long dropSteps;
};

const uint32_t MAGIC_V4 = 0xC0FFEE79;    // V4 형식(현재)
const uint32_t MAGIC_V3 = 0xC0FFEE78;    // V3 형식(과거)
const int EEPROM_ADDR = 0;               // EEPROM의 0번 주소부터 저장 시작

bool needSave = false;                   // 지금 값이 바뀌었고, 조금 있다 저장해야 함을 알리는 깃발
unsigned long lastEditMs = 0;            // 마지막으로 값이 바뀐 시간(밀리초)
const unsigned long SAVE_DELAY_MS = 100;// 값이 바뀐 뒤 1.5초 동안 더 안 바뀌면 그때 저장

// ★ 이동 중 현재 위치를 너무 자주 쓰지 않도록 쓰기 쓰로틀
const unsigned long POS_SAVE_INTERVAL_MS = 50;     // 최소 저장 간격(ms)
const long          POS_SAVE_DELTA_STEPS = 50;     // 위치 변화가 이 이상이면 저장 고려
unsigned long lastPosSaveMs = 0;
long lastSavedX = 0, lastSavedY = 0, lastSavedZ = 0;

// ★ OLED 갱신 쓰로틀(현재 표시중인 정수 단위)
long lastOLEDUnitX = 0, lastOLEDUnitY = 0, lastOLEDUnitZ = 0;


// ===== 버튼 이벤트(버튼을 눌렀는지, 오래 눌렀는지) =====
enum { EV_NONE = 0, EV_PRESS = 1, EV_REPEAT = 2 }; // NONE:없음, PRESS:딱 한 번 눌림, REPEAT:길게 눌러서 반복

// 모터 움직임 제어 함수
static inline void stepHalf(uint8_t PUL, int half_us) {
  digitalWrite(PUL, HIGH);  delayMicroseconds(half_us); // PUL 핀을 HIGH로 올렸다가
  digitalWrite(PUL, LOW);   delayMicroseconds(half_us); // 다시 LOW로 내림(모터 한 번 스텝)
}


// ─────────────────────────────────────────────────────────────────────
// [추가] 타이머 기반 스텝 드라이버 (Timer1 = Y축, Timer3 = X축)
// ─────────────────────────────────────────────────────────────────────

// 진행 중 여부(재진입 방지용)
volatile bool isMoving = false;

// ===== Timer1(여기서는 'Y축'= 보드 X_PUL/X_DIR) 진행량 =====
volatile uint8_t* T1_PUL_PORT;
volatile uint8_t  T1_PUL_MASK;
volatile long     T1_steps_remaining = 0;
volatile long     T1_steps_done = 0;     // ISR에서 완료 스텝 누적
volatile int8_t   T1_step_sign  = +1;
volatile bool     T1_pulse_state = false;

// ===== Timer3(여기서는 'X축'= 보드 Z_PUL/Z_DIR) 진행량 =====
volatile uint8_t* T3_PUL_PORT;
volatile uint8_t  T3_PUL_MASK;
volatile long     T3_steps_remaining = 0;
volatile long     T3_steps_done = 0;
volatile int8_t   T3_step_sign  = +1;
volatile bool     T3_pulse_state = false;

// 타이머1 비교일치 A ISR (Y축)
ISR(TIMER1_COMPA_vect) {
  if (T1_steps_remaining > 0) {
    if (T1_pulse_state) {
      *T1_PUL_PORT |=  T1_PUL_MASK;   // HIGH
    } else {
      *T1_PUL_PORT &= ~T1_PUL_MASK;   // LOW
      T1_steps_remaining--;
      T1_steps_done++;                // 한 스텝 완료
    }
    T1_pulse_state = !T1_pulse_state;
  } else {
    TIMSK &= ~(1 << OCIE1A);          // Timer1 IRQ off
    *T1_PUL_PORT &= ~T1_PUL_MASK;     // 핀 LOW 유지
  }
}

// 타이머3 비교일치 A ISR (X축)  ※ ATmega128: ETIMSK 사용
ISR(TIMER3_COMPA_vect) {
  if (T3_steps_remaining > 0) {
    if (T3_pulse_state) {
      *T3_PUL_PORT |=  T3_PUL_MASK;   // HIGH
    } else {
      *T3_PUL_PORT &= ~T3_PUL_MASK;   // LOW
      T3_steps_remaining--;
      T3_steps_done++;                // 한 스텝 완료
    }
    T3_pulse_state = !T3_pulse_state;
  } else {
    ETIMSK &= ~(1 << OCIE3A);         // Timer3 IRQ off
    *T3_PUL_PORT &= ~T3_PUL_MASK;     // 핀 LOW 유지
  }
}

// half_us = 반주기(us) → 1스텝 주기 = 2*half_us
static inline void startTimerMotor(
  uint8_t timer_id, uint8_t pulPin, uint8_t dirPin, uint8_t dirLevel,
  long steps, int half_us, int8_t step_sign
){
  if (steps <= 0) return;

  // 핀 설정
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, dirLevel);
  delayMicroseconds(100);   // 방향 안정화

  noInterrupts();

  // OCR 계산 (분주 8, CTC)
  // OCR = (F_CPU/8)*half_us/1e6 - 1
  uint32_t ocr = (uint32_t)(((uint64_t)F_CPU / 8ULL) * (uint64_t)half_us / 1000000ULL);
  if (ocr == 0) ocr = 1;
  ocr -= 1;

  if (timer_id == 1) {
    T1_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin));
    T1_PUL_MASK = digitalPinToBitMask(pulPin);
    T1_steps_remaining = steps;
    T1_steps_done = 0;
    T1_step_sign = step_sign;
    T1_pulse_state = false;

    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
    TCCR1B |= (1 << WGM12);         // CTC
    OCR1A   = (uint16_t)ocr;
    TCCR1B |= (1 << CS11);          // /8
    TIMSK  |= (1 << OCIE1A);        // IRQ on
  }
  else if (timer_id == 3) {
    T3_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin));
    T3_PUL_MASK = digitalPinToBitMask(pulPin);
    T3_steps_remaining = steps;
    T3_steps_done = 0;
    T3_step_sign = step_sign;
    T3_pulse_state = false;

    TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;
    TCCR3B |= (1 << WGM32);         // CTC
    OCR3A   = (uint16_t)ocr;
    TCCR3B |= (1 << CS31);          // /8
    ETIMSK |= (1 << OCIE3A);        // IRQ on (Timer3는 ETIMSK)
  }

  interrupts();
}


// ★ 이동 중 주기적으로 현재 위치 저장
inline void maybeSaveCurPosDuringMove() {
  unsigned long now = millis();
  if ((now - lastPosSaveMs) >= POS_SAVE_INTERVAL_MS) {
    long dx = curX - lastSavedX;
    long dy = curY - lastSavedY;
    long dz = curZ - lastSavedZ;
    if ( (dx>=POS_SAVE_DELTA_STEPS || dx<=-POS_SAVE_DELTA_STEPS) ||
         (dy>=POS_SAVE_DELTA_STEPS || dy<=-POS_SAVE_DELTA_STEPS) ||
         (dz>=POS_SAVE_DELTA_STEPS || dz<=-POS_SAVE_DELTA_STEPS) ) {
      PersistV4 p { MAGIC_V4, curX, curY, curZ, dropSteps };
      EEPROM.put(EEPROM_ADDR, p);
      lastPosSaveMs = now;
      lastSavedX = curX; lastSavedY = curY; lastSavedZ = curZ;
    }
  }
}
// OLED 화면에 X,Y,Z,D를 보기 좋게 보여주는 함수
void updateOLED() {
  // ★ 수정: OLED는 “목표(tgt)”가 아니라 “현재(cur)”를 보여줌
  long x = curX / DISPLAY_UNIT;                // 스텝을 화면 숫자로 바꿈(X)
  long y = curY / DISPLAY_UNIT;                // 스텝을 화면 숫자로 바꿈(Y)
  long z = curZ / DISPLAY_UNIT;                // 스텝을 화면 숫자로 바꿈(Z)
  long bursts = dropSteps / DROP_UNIT_STEPS;   // D:200xN의 N값(방울 개수 느낌)

  display.clearDisplay();                      // 화면 지우기
  display.setTextColor(WHITE);                 // 글자색은 하양

  display.setTextSize(1);                      // 작은 글자 크기
  display.setCursor(0, 0);   display.print(F("X")); // 왼쪽 위에 X라는 글자
  display.setTextSize(2);                      // 큰 글자 크기
  display.setCursor(16, 0);  display.print(x); // X값 숫자 보여주기

  display.setTextSize(1);                      // 다시 작은 글자
  display.setCursor(64, 0);  display.print(F("Y")); // Y 글자
  display.setTextSize(2);                      // 큰 글자
  display.setCursor(80, 0);  display.print(y); // Y값 숫자

  display.setTextSize(1);                      // 작은 글자
  display.setCursor(0, 18);                    // 둘째 줄 왼쪽
  display.print(F("Z:")); display.print(z);    // Z:숫자
  display.setCursor(64, 18);                   // 둘째 줄 중간쯤
  display.print(F("D:200x")); display.print(bursts); // D:200xN 형태로 표시

  display.display();                            // 실제로 화면에 그리기
}

// ★ 이동 중 OLED를 ‘현재 위치 기준’으로 1칸(=DISPLAY_UNIT) 바뀔 때만 갱신
inline void maybeUpdateOLEDWhileMoving(char axis) {
  long ux = curX / DISPLAY_UNIT;
  long uy = curY / DISPLAY_UNIT;
  long uz = curZ / DISPLAY_UNIT;
  bool changed = false;

  if (axis == 'X' && ux != lastOLEDUnitX) { lastOLEDUnitX = ux; changed = true; }
  if (axis == 'Y' && uy != lastOLEDUnitY) { lastOLEDUnitY = uy; changed = true; }
  if (axis == 'Z' && uz != lastOLEDUnitZ) { lastOLEDUnitZ = uz; changed = true; }

  if (changed) updateOLED();
}

// ──────────────────────────────────────────────────────────────
// [추가] 인터럽트 드라이버 진행량을 메인 상태(curX/curY)에 반영
// ──────────────────────────────────────────────────────────────
inline void serviceAxisProgress() {
  long d1 = 0, d3 = 0;

  noInterrupts();
  if (T1_steps_done) { d1 = T1_steps_done; T1_steps_done = 0; }
  if (T3_steps_done) { d3 = T3_steps_done; T3_steps_done = 0; }
  interrupts();

  if (d3) { // Timer3 == X축 (보드 Z_PUL/Z_DIR을 실제 X로 사용)
    curX += (long)T3_step_sign * d3;
    maybeSaveCurPosDuringMove();
    maybeUpdateOLEDWhileMoving('X');
  }
  if (d1) { // Timer1 == Y축 (보드 X_PUL/X_DIR을 실제 Y로 사용)
    curY += (long)T1_step_sign * d1;
    maybeSaveCurPosDuringMove();
    maybeUpdateOLEDWhileMoving('Y');
  }
}

// 모터를 정해진 스텝 수만큼 “현재값을 갱신하며” 움직이는 함수(한 축 전용)
void moveAxisSteps(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                   long steps, int half_us, long &curAxis, int stepSign, char axisTag) {
  if (steps <= 0) return;
  digitalWrite(DIR, dirLevel);
  delayMicroseconds(100);
  for (long i = 0; i < steps; i++) {
    stepHalf(PUL, half_us);
    curAxis += stepSign;                // ★ 한 스텝마다 실제 위치 갱신
    maybeSaveCurPosDuringMove();        // ★ 이동 체크포인트 저장
    maybeUpdateOLEDWhileMoving(axisTag);// ★ OLED 1칸 단위 갱신
  }
}


// 지금 설정(tgtX,tgtY,tgtZ,dropSteps)을 EEPROM에 즉시 저장
void savePersistNow() {
  PersistV4 p { MAGIC_V4, curX, curY, curZ, dropSteps }; // ★ 실제 위치 저장
  EEPROM.put(EEPROM_ADDR, p);                            // EEPROM에 쓰기
  lastSavedX = curX; lastSavedY = curY; lastSavedZ = curZ;
  lastPosSaveMs = millis();
  // OLED 기준값도 동기화
  lastOLEDUnitX = curX / DISPLAY_UNIT;
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
}

// “조금 있다가 저장하자”라고 표시만 해두는 함수
void requestSave() {
  needSave = true;                                       // 저장 예약
  lastEditMs = millis();                                 // 지금 시간을 기록
}

// 값이 더 이상 바뀌지 않으면, 정해진 시간 뒤에 저장하는 함수
void trySaveIfIdle() {
  if (needSave && (millis() - lastEditMs >= SAVE_DELAY_MS)) { // 마지막 변경 후 1.5초 지남?
    savePersistNow();                                         // 그럼 진짜 저장
    needSave = false;                                         // 저장 예약 해제
  }
}

//
// 전원을 켰을 때 EEPROM에서 이전 값을 읽어오는 함수
//
void loadPersist() {
  uint32_t magic = 0;                                // 먼저 버전 번호(매직)를 읽을 변수
  EEPROM.get(EEPROM_ADDR, magic);                    // EEPROM에서 번호를 읽음

  if (magic == MAGIC_V4) {                           
    PersistV4 p; EEPROM.get(EEPROM_ADDR, p);         // 실제 위치/드롭값 복원
    curX = p.curX; curY = p.curY; curZ = p.curZ;
    dropSteps = p.dropSteps;
    tgtX = curX; tgtY = curY; tgtZ = curZ;           // 목표도 현재와 맞춤(절대좌표 유지)
  } 
  else if (magic == MAGIC_V3) {                      // 구버전 호환: 목표를 실제로 간주
    PersistV3 p; EEPROM.get(EEPROM_ADDR, p);
    curX = p.tgtX; curY = p.tgtY; curZ = p.tgtZ;
    dropSteps = p.dropSteps;
    tgtX = curX; tgtY = curY; tgtZ = curZ;
  }
  else {                                             // 저장된 게 없으면
    curX = 0; curY = 0; curZ = 0;
    tgtX = 0; tgtY = 0; tgtZ = 0;
    dropSteps = DROP_UNIT_STEPS;                     // 모두 0/기본값
  }

  clampXY(); clampZ(); clampDrop();                  // 읽은 값이 범위를 넘지 않게 정리

  // OLED 기준값 초기화
  lastOLEDUnitX = curX / DISPLAY_UNIT;
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
}

void doOneDrop() {
  long bursts = dropSteps / DROP_UNIT_STEPS;                    // 200스텝을 몇 번 누를지 계산(N)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (bursts > 0) {
    for (long i = 0; i < bursts; i++) {               // N번 반복
      moveAxisSteps(UD_PUL, UD_DIR, UD_DIR_DOWN, DROP_UNIT_STEPS, STEP_HALF_US, /*curAxis*/curZ, 0, 'X');
      delay(100);                                               // 잠깐 대기(플런저가 올라가는 시간)
    }
  } else if (bursts < 0) {
    for (long i = 0; i < -bursts; i++) {          // N번 반복(마이너스면 위로 올리기)
      moveAxisSteps(UD_PUL, UD_DIR, UD_DIR_UP,   DROP_UNIT_STEPS, STEP_HALF_US, /*curAxis*/curZ, 0, 'X');
      delay(100);                                       // 잠깐 대기(플런저가 내려가는 시간)
    }
  }
  savePersistNow();
}

//
// 현재 위치(curX/Y/Z)에서 목표 위치(tgtX/Y/Z)까지 실제로 모터를 움직이는 함수
//


// GO 버튼을 눌렀을 때 하는 동작(한 번 작업: 이동 → 프리다운 → 플런저 누르기 → 복귀)

//
// 버튼들을 한 자리에서 관리하기 위한 배열과 번호들
//
const uint8_t BTN_PINS[] = {
  BTN_SETX_UP, BTN_SETX_DN,   // X+ / X-
  BTN_SETY_UP, BTN_SETY_DN,   // Y+ / Y-
  BTN_SETZ_UP, BTN_SETZ_DN,   // Z+ / Z-
  BTN_GO, BTN_RESET00,        // GO / RESET
  BTN_PLUNGER_UP, BTN_PLUNGER_DN // D +200 / -200
};
const uint8_t NBTN = sizeof(BTN_PINS) / sizeof(BTN_PINS[0]); // 버튼 개수 계산

// 각 버튼이 배열에서 몇 번인지 기억하기 위한 번호(읽기 쉽게 하려고 이름 붙임)
enum {
  IDX_XP = 0, IDX_XM,         // X+ / X-
  IDX_YP,     IDX_YM,         // Y+ / Y-
  IDX_ZP,     IDX_ZM,         // Z+ / Z-
  IDX_GO,     IDX_RESET,      // GO / RESET
  IDX_DROP_UP, IDX_DROP_DN    // D +200 / -200
};

uint8_t  btnStable[NBTN];             // 안정된(확실한) 버튼 값(HIGH/LOW)을 기억
uint8_t  btnLastReading[NBTN];        // 마지막으로 읽었던 생 버튼 값
unsigned long btnLastChange[NBTN];    // 버튼 값이 바뀐 시각
unsigned long btnPressStart[NBTN];    // 버튼을 누르기 시작한 시각
unsigned long btnLastRepeat[NBTN];    // 길게 누른 상태에서 마지막으로 반복 신호를 낸 시각

// 버튼 동작 시간 설정(얼마나 눌러야 반복으로 볼지 등)
const unsigned long DEBOUNCE_MS     = 100; // 버튼이 튀는 것(노이즈)을 무시하기 위해 100ms 기다림
const unsigned long REPEAT_START_MS = 400; // 0.4초 이상 누르고 있으면 “길게 눌림 반복” 시작
const unsigned long REPEAT_RATE_MS  = 90;  // 반복이 시작되면 0.09초마다 한 번씩 반복 신호

//
// 버튼 하나의 상태를 판별해서 PRESS/REPEAT/NONE 을 돌려주는 함수
//
uint8_t pollBtn(uint8_t idx) {
  uint8_t pin = BTN_PINS[idx];                      // 몇 번 핀인지 알아옴
  uint8_t raw = digitalRead(pin);                   // 지금 실제 버튼 값을 읽음(HIGH/LOW)
  unsigned long now = millis();                     // 현재 시간을 ms로 읽음

  if (raw != btnLastReading[idx]) {                 // 직전에 읽은 값과 다르면
    btnLastReading[idx] = raw;                      // 새 값으로 바꾸고
    btnLastChange[idx] = now;                       // 바뀐 시각을 기록
  }

  if (now - btnLastChange[idx] > DEBOUNCE_MS) {     // 바뀌고 나서 100ms가 지나면(튀는 것 지나감)
    if (btnStable[idx] != raw) {                    // 안정된 값이 지금 값과 다르면(진짜로 바뀜)
      btnStable[idx] = raw;                         // 안정된 값 갱신
      if (btnStable[idx] == LOW) {                  // 풀업 입력이라 LOW면 “눌림”
        btnPressStart[idx] = now;                   // 누르기 시작한 시각 저장
        btnLastRepeat[idx] = now;                   // 반복 타이머도 초기화
        return EV_PRESS;                            // “한 번 눌림” 이벤트 보내기
      }
    } else {                                        // 안정된 값이 계속 같으면(변화 없음)
      if (btnStable[idx] == LOW) {                  // 계속 눌린 상태면
        if (now - btnPressStart[idx] >= REPEAT_START_MS && // 눌린 지 0.4초 지났고
            now - btnLastRepeat[idx]  >= REPEAT_RATE_MS) { // 마지막 반복에서 0.09초 지났으면
          btnLastRepeat[idx] = now;                // 반복 시각 갱신
          return EV_REPEAT;                        // “길게 눌러서 반복” 이벤트 보내기
        }
      }
    }
  }
  return EV_NONE;                                   // 아무 이벤트도 아님
}

//
// 6번/7번 버튼(D값 조절)을 특별히 다루는 부분
// → 한쪽을 누르고 있는 동안에는 반대쪽은 무시
// → “짧게 1번, 충분히 길게 눌러야 천천히 반복” (EV_REPEAT 사용 안 함)
//
int8_t dropOwner = -1;                              // -1:아무도 안 누름, 0:6번(UP)이 주도권, 1:7번(DN)이 주도권

void handleDropButtons(bool &changed) {
  const long STEP_EDIT = DROP_UNIT_STEPS;           // 한 번 바뀔 때 D는 200씩 변함

  // 물리 버튼 현재 상태(풀업: LOW = 눌림)
  bool upLow = (digitalRead(BTN_PLUNGER_UP) == LOW);
  bool dnLow = (digitalRead(BTN_PLUNGER_DN) == LOW);

  // 두 개를 동시에 누르면 아무 것도 하지 않음(충돌 방지)
  if (upLow && dnLow) return;

  static bool dropFired = false;                    // 이번 “누름”에서 1회 동작했는지
  static unsigned long dropPressStartMs = 0;        // 누르기 시작한 시각
  static unsigned long dropLastRepeatMs  = 0;       // 마지막 반복 시각
  unsigned long now = millis();

  // 아직 주도권 없으면, 눌린 쪽에 주도권 부여 + 타이머 초기화
  if (dropOwner == -1) {
    if (upLow && !dnLow) {
      dropOwner = 0;
      dropFired = false;
      dropPressStartMs = now;
      dropLastRepeatMs  = now;
    } else if (dnLow && !upLow) {
      dropOwner = 1;
      dropFired = false;
      dropPressStartMs = now;
      dropLastRepeatMs  = now;
    } else {
      return; // 아무도 안 눌림
    }
  }

  // 주도권 가진 쪽 처리
  if (dropOwner == 0) { // 6번(UP)
    if (upLow) {
      if (!dropFired) {
        // 첫 눌림: 딱 1회 +200
        dropSteps += STEP_EDIT;
        clampDrop();
        changed = true;
        requestSave();
        dropFired = true;
      } else if ((now - dropPressStartMs) >= DROP_REPEAT_START_MS &&
                 (now - dropLastRepeatMs)  >= DROP_REPEAT_RATE_MS) {
        // 충분히 길게 눌렀을 때만 천천히 반복(+200)
        dropSteps += STEP_EDIT;
        clampDrop();
        changed = true;
        requestSave();
        dropLastRepeatMs = now;
      }
    } else {
      // 6번에서 손을 떼면 주도권 해제
      dropOwner = -1;
      dropFired = false;
    }
  } else if (dropOwner == 1) { // 7번(DN)
    if (dnLow) {
      if (!dropFired) {
        // 첫 눌림: 딱 1회 -200
        dropSteps -= STEP_EDIT;
        clampDrop();
        changed = true;
        requestSave();
        dropFired = true;
      } else if ((now - dropPressStartMs) >= DROP_REPEAT_START_MS &&
                 (now - dropLastRepeatMs)  >= DROP_REPEAT_RATE_MS) {
        // 충분히 길게 눌렀을 때만 천천히 반복(-200)
        dropSteps -= STEP_EDIT;
        clampDrop();
        changed = true;
        requestSave();
        dropLastRepeatMs = now;
      }
    } else {
      // 7번에서 손을 떼면 주도권 해제
      dropOwner = -1;
      dropFired = false;
    }
  }
}

//
// 전원을 켰을 때 한 번만 실행되는 부분(초기 준비)
//
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);        // OLED 시작(전압 방식/주소 0x3C)

  Serial.begin(115200);

  pinMode(UD_PUL, OUTPUT); pinMode(UD_DIR, OUTPUT); // 플런저 모터 핀을 출력으로
  pinMode(X_PUL,  OUTPUT); pinMode(X_DIR,  OUTPUT); // Y모터(보드 X) 핀을 출력으로
  pinMode(Y_PUL,  OUTPUT); pinMode(Y_DIR,  OUTPUT); // Z모터 핀을 출력으로
  pinMode(Z_PUL,  OUTPUT); pinMode(Z_DIR,  OUTPUT); // X모터 핀을 출력으로

  pinMode(BTN_SETX_UP,    INPUT_PULLUP);            // 버튼들은 풀업 입력(안 누르면 HIGH, 누르면 LOW)
  pinMode(BTN_SETX_DN,    INPUT_PULLUP);
  pinMode(BTN_SETY_UP,    INPUT_PULLUP);
  pinMode(BTN_SETY_DN,    INPUT_PULLUP);
  pinMode(BTN_SETZ_UP,    INPUT_PULLUP);
  pinMode(BTN_SETZ_DN,    INPUT_PULLUP);
  pinMode(BTN_GO,         INPUT_PULLUP);
  pinMode(BTN_RESET00,    INPUT_PULLUP);
  pinMode(BTN_PLUNGER_UP, INPUT_PULLUP);
  pinMode(BTN_PLUNGER_DN, INPUT_PULLUP);

  unsigned long t_init = millis();                  // 지금 시간을 기록
  for (uint8_t i=0; i<NBTN; i++) {                  // 모든 버튼에 대해
    uint8_t raw = digitalRead(BTN_PINS[i]);         // 현재 값을 읽고
    btnStable[i]      = raw;                        // 안정된 값으로 저장
    btnLastReading[i] = raw;                        // 마지막 읽은 값으로 저장
    btnLastChange[i]  = t_init;                     // 바뀐 시간 초기화
    btnPressStart[i]  = 0;                          // 눌린 시작 시각 초기화
    btnLastRepeat[i]  = 0;                          // 반복 시각 초기화
  }

  loadPersist();                                    // EEPROM에서 마지막 실제 위치/드롭값 복원
  clampXY(); clampZ(); clampDrop();                 // 안전 범위로 정리

  // OLED 기준값 초기화
  lastOLEDUnitX = curX / DISPLAY_UNIT;
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;

  updateOLED();                                      // 화면에 처음 값 표시(현재 위치 기준)
}



/*
// 송신측 --------------------------------------------------------------
총 21BYTE
EX : 7B 5B 02 00 0B 35 51 01 02 03 04 05 06 07 08 09 BC 03 5D 7D
EX : STX     ID LEN FN    DATA(9BYTE)                BCC ETX

STX : 7B 5B 02 ( { [ STX ) 3BYTE
ID : 1BYTE
     장비 ID
LEN : 1BYTE 
     프로토콜 길이 FN , DATA, ERR, BCC 까지
FN :  2BYTE
      송신 39 51
DATA : 10BYTE  
       1 : MODE  unsigned char 1~13
           1)자동,        2)수동,         3)주차모드,       4)위험정지        5)평지자유이동모드,
           6)평지후진     7)최초상승,     8)2개단상승,      9)계단에서상승,   10)최종상승후착지, 
           11)평지전진    12)최초하강,    13)계단에서하강   14)2개단착지,     15)최종하강후착지,    
           16)둔턱승월,   17)둔턱하강 
       2 : 구동신호 unsigned char 1~4
           1)후진, 0x01   2)전진, 0x02    3)정지, 0x03    4)위험 0x04
       3 : Battery_Level 배터리량
           1) 배터리량 0 ~ 200 X 0.5 값으로 환산됨. 
       4 : Main_Motor 주 모터 속도 및 방향
           1) 1BIT EN신호 0x80
           2) 1BIT 정역 0x40
           3) 6BIT Speed 0x00 ~ 0x3f(63) X 4배 값이 출력됨. (최대 252)
       5 : Center_Lift 중심 리프트 모터 속도 및 방향
           1) 1BIT EN신호 0x80
           2) 1BIT 정역 0x40
           3) 6BIT Speed 0x00 ~ 0x3f(63) X 4배 값이 출력됨. (최대 252)
       6 : Side1_Lift 날개1 모터 속도 및 방향 
           1) 1BIT EN신호 0x80
           2) 1BIT 정역 0x40
           3) 6BIT Speed 0x00 ~ 0x3f(63) X 4배 값이 출력됨. (최대 252)
       7 : Side2_Lift 날개2 모터 속도 및 방향 
           1) 1BIT EN신호 0x80
           2) 1BIT 정역 0x40
           3) 6BIT Speed 0x00 ~ 0x3f(63) X 4배 값이 출력됨. (최대 252)
       8 : Output_Port 출력 PORT 1PORT
           1) Output출력 
       9 : Handle_Acc_X 손잡이 측 가속도 x축
       10 : Handle_Acc_Y 손잡이 측 가속도 Y축
BCC : 1BYTE ( FN , DATA, ERR )의 XOR    
ETX : 03 5D 7D ( ETX ] } ) 3BYTE
*/

unsigned char command0 = 0;               // 시리얼 통신 내용의 그에 따르는 내용을 할수 있도록 하는 변수
unsigned char InputSirialData0 = 0;       // UDR버퍼의 내용을 잠시 보관하는 변수
unsigned char Uart0ProtocolPointer = 0;   // UDR0버퍼의 연속적인 데이터에 대한 점프
unsigned char Uart0ReciveCheckEnd = 0;    // UDR0의 내용의 마지막을 의미

unsigned char Uart0_Data_Count = 0;       // 통신 데이터가 입력되는지 확인하는 변수 
unsigned char Check_Xor =0;               // BCC를 체크할때 활용하는 변수 
unsigned char Check_Xor_Check =0;         // BCC를 체크할때 활용할때 임시 체크 변수 
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
  
  if(Serial.available())
  { 
      //digitalWrite(T_LED, HIGH);  
      Uart0_Data = Serial.read();

      //Serial.write(Uart0_Data);
      
      // 
      switch(Uart0ProtocolPointer)
      { 
        // 프로토콜 예)
        //      STX      LEN   FN    DATA                          BCC ETX
        //      7B 5B 02 00 0D 31 51 00	00 00 00 00 00 00 00 00 00 BC	03 5D 7D
        //---------------------------------------------------------------------
        // 시작 byte  7b 5b 02
        case 0:     if(0x7b == Uart0_Data ) { Uart0ProtocolPointer = 1;
                      Uart0_Data_Count++;     Error_COM = 0 ;              
                    }  
                    else {  Uart0ProtocolPointer = 0; Check_Xor =0;   }
                    break; 
        case 1:     if(0x5b == Uart0_Data ){  Uart0ProtocolPointer = 2;  }  
                    else {Uart0ProtocolPointer = 0; Check_Xor =0;        }
                    break; 
        case 2:     if(0x02 == Uart0_Data )  {Uart0ProtocolPointer = 10;  Error_LEN_Count = 0;       }
                    else {Uart0ProtocolPointer = 0; Check_Xor =0;  }
                    break;            
        //---------------------------------------------------------------------
        // LEN 길이 0x00 
        case 10:    if(0X00 == Uart0_Data)   {     
                          Uart0ProtocolPointer = 11;    Check_Xor ^= Uart0_Data;    Error_LEN = 0;    
                    }     
                    else {    
                          Uart0ProtocolPointer = 11;    Check_Xor ^= Uart0_Data;    Error_LEN = 1; 
                    }                          
                    break;
        //---------------------------------------------------------------------
        // LEN 길이 0x0C 
        case 11:    if(0x0D == Uart0_Data)    {    
                          Uart0ProtocolPointer = 20;    Check_Xor ^= Uart0_Data;  Error_LEN = 0;   Error = Error & 0x00;  }    
                    else {    
                          Uart0ProtocolPointer = 20;    Check_Xor ^= Uart0_Data;  Error_LEN = 1;    Error = Error | 0x08;    }                  
                    break;
        //---------------------------------------------------------------------
        // 기능  5Q
        case 20:    if(0x31 == Uart0_Data)    {    Uart0ProtocolPointer = 21;   Check_Xor ^= Uart0_Data;  Error_LEN_Count = Error_LEN_Count + 1;  }     
                    else {    Uart0ProtocolPointer = 21; Check_Xor ^= Uart0_Data;   Error_BCC = 1;  }                     
                    break;  
                    
        case 21:    if(0x51 == Uart0_Data)    {    Uart0ProtocolPointer = 30;   Check_Xor ^= Uart0_Data;  Error_LEN_Count = Error_LEN_Count + 1;  }     
                    else {    Uart0ProtocolPointer = 30; Check_Xor ^= Uart0_Data;   Error_BCC = 1; }            
                    break;
        //----------------------------------------------------------------------
        // 데이터 
        
        case 30:    Uart0ProtocolPointer = 31;    // Uart1ProtocolPointer을 31으로 점프
                    X1_L_Rx = Uart0_Data;         //    
                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break; 

        case 31:    Uart0ProtocolPointer = 32;    // Uart1ProtocolPointer을 32로 점프
                    X1_H_Rx = Uart0_Data;     
                    Check_Xor ^= Uart0_Data;   
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;

        case 32:    Uart0ProtocolPointer = 33;    // Uart1ProtocolPointer을 33로 점프
                    Y1_L_Rx = Uart0_Data;    
                    Check_Xor ^= Uart0_Data; 
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;

        case 33:    Uart0ProtocolPointer = 34;    // Uart1ProtocolPointer을 34으로 점프
                    Y1_H_Rx = Uart0_Data;        
                    Check_Xor ^= Uart0_Data;  
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;

        case 34:    Uart0ProtocolPointer = 35;    // Uart1ProtocolPointer을 35으로 점프
                    Z1_L_Rx = Uart0_Data;         //
                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;  
                    
        case 35:    Uart0ProtocolPointer = 36;    // Uart1ProtocolPointer을 35으로 점프
                    Z1_H_Rx = Uart0_Data;         //  
                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;  

        case 36:    Uart0ProtocolPointer = 37;    // Uart1ProtocolPointer을 35으로 점프
                    F1_L_Rx = Uart0_Data;    //
                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;  

        case 37:    Uart0ProtocolPointer = 38;    // Uart1ProtocolPointer을 38 점프
                    F1_H_Rx = Uart0_Data;    //
                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break; 

        case 38:    Uart0ProtocolPointer = 39;      // Uart1ProtocolPointer을 90 점프
                    FN_Port_Rx = Uart0_Data;    //  

                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;  

        case 39:    Uart0ProtocolPointer = 90;      // Uart1ProtocolPointer을 90 점프
                    Error1_Rx = Uart0_Data;    //  

                    Check_Xor ^= Uart0_Data;    
                    Error_LEN_Count = Error_LEN_Count + 1;
                    break;  

        //----------------------------------------------------------------------
        // BCC 
        case 90:    if(Check_Xor == Uart0_Data) { 
                         Check_Xor_Check = Check_Xor;
                         command0 = 1;  Uart0ProtocolPointer = 100; 
                         Check_Xor =0;  Error_BCC = 0; Error_LEN_Count = Error_LEN_Count + 1;  
                         Error = Error & 0x00;      
                    }   
                    else if(0xBC == Uart0_Data)   {
                         Check_Xor_Check = Check_Xor;
                         command0 = 1;  Uart0ProtocolPointer = 100; 
                         Check_Xor =0;  Error_BCC = 0; Error_LEN_Count = Error_LEN_Count + 1;  
                         Error = Error & 0x00;      
                    }         
                    else  {
                         Check_Xor_Check = Check_Xor;  
                         command0 = 1;  Uart0ProtocolPointer = 100;  
                         Check_Xor =0; Error_BCC = 1;  Error_LEN_Count = Error_LEN_Count + 1; 
                         Error = Error | 0x04;  
                    }                  
                    break;

        //---------------------------------------------------------------------
        // 끝  byte  03 5d 7d  
        case 100:   if(0x03 == Uart0_Data )  {    Uart0ProtocolPointer = 101;     }
                    else
                    {   Uart0ProtocolPointer = 0;   Check_Xor =0;  } 
                    break; 
        case 101:   if(0x5d == Uart0_Data ) { Uart0ProtocolPointer = 102;    }
                    else 
                    {   Uart0ProtocolPointer = 0;  Check_Xor =0;    }
                    break;                     
        case 102:   if(0x7d == Uart0_Data)  {   
                    Uart0ProtocolPointer = 0;   Uart0ReciveCheckEnd = 1;  Check_Xor =0;     Error_COM = 0 ;
                    }   
                    else {  Uart0ProtocolPointer = 0; } 
                    break;
        default:    Uart0ProtocolPointer = 0;
                    Check_Xor =0;
                    break;
        
      }
  }
}
void moveAxisSteps_ISR(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                       long steps, int half_us, long &curAxis, int stepSign, char axisTag) {
  if (steps <= 0) return;

  // 축→타이머 매핑
  uint8_t timer_id = 0;
  if (axisTag == 'X') timer_id = 3;      // X == Timer3
  else if (axisTag == 'Y') timer_id = 1; // Y == Timer1
  else {
    // Z나 플런저 등은 기존 블로킹 버전 사용
    moveAxisSteps(PUL, DIR, dirLevel, steps, half_us, curAxis, stepSign, axisTag);
    return;
  }

  // 타이머 드라이버 시동
  startTimerMotor(timer_id, PUL, DIR, dirLevel, steps, half_us, (int8_t)stepSign);

  // 완료까지 대기하되, 진행량/저장/UART 수신은 계속 처리
  for (;;) {
    serviceAxisProgress();   // ISR 누적분을 curX/curY에 반영 + OLED/저장 스로틀
    UartRxProtocol();        // 시리얼 수신 버퍼링(파싱만, 실행은 메인에서)
    trySaveIfIdle();         // 지연 저장

    bool done = (timer_id == 3) ? (T3_steps_remaining == 0) : (T1_steps_remaining == 0);
    if (done) break;
  }

  // 잔여 반영
  serviceAxisProgress();
}
void goToTargetXYZ() { //x축과 y축 동시 움직임
  

  if (isMoving) return;                         // 재진입 방지
  isMoving = true;

  clampXY(); clampZ();                          // 목표가 범위를 넘었으면 먼저 정리

  long dx = tgtX - curX;                        // X축으로 얼마나 더 가야 하는지
  if (dx > 0) {                                 // 목표가 오른쪽(+)
    // ★ X축: 타이머 ISR 버전 사용 (Timer3, 핀은 Z_PUL/Z_DIR)
    moveAxisSteps_ISR(Z_PUL, Z_DIR, X_DIR_POS,  dx, STEP_HALF_US, curX, +1, 'X');
  }
  else if (dx < 0) {                            // 목표가 왼쪽(-)
    moveAxisSteps_ISR(Z_PUL, Z_DIR, X_DIR_NEG, -dx, STEP_HALF_US, curX, -1, 'X');
  }

  long dy = tgtY - curY;                        // Y축으로 얼마나 더 가야 하는지
  if (dy > 0) {                                 // 목표가 앞쪽(+)
    // ★ Y축: 타이머 ISR 버전 사용 (Timer1, 핀은 X_PUL/X_DIR)
    moveAxisSteps_ISR(X_PUL, X_DIR, STAGEY_DIR_POS,  dy, STEP_HALF_US, curY, +1, 'Y');
  }
  else if (dy < 0) {                            // 목표가 뒤쪽(-)
    moveAxisSteps_ISR(X_PUL, X_DIR, STAGEY_DIR_NEG, -dy, STEP_HALF_US, curY, -1, 'Y');
  }

  long dz = tgtZ - curZ;                        // Z축으로 얼마나 더 가야 하는지
  if (dz > 0) {                                 // 목표가 위쪽(+)
    // ★ Z는 기존 블로킹 버전 유지
    moveAxisSteps(Y_PUL, Y_DIR, LIFT_DIR_UP,   dz, STEP_HALF_US, curZ, +1, 'Z');
  }
  else if (dz < 0) {                            // 목표가 아래(-)
    moveAxisSteps(Y_PUL, Y_DIR, LIFT_DIR_DOWN, -dz, STEP_HALF_US, curZ, -1, 'Z');
  }

  doOneDrop();                                   // 플런저 동작

  // Z 복귀
  moveAxisSteps(Y_PUL, Y_DIR, LIFT_DIR_UP, -dz, STEP_HALF_US, curZ, +1, 'Z');

  // 목표 도달 스냅샷
  savePersistNow();

  isMoving = false;
}


void Serial_Main0(void) 
{
  if(Uart0ReciveCheckEnd)
  {
    if(command0 == 1)
    {

        X1_L = X1_H_Rx;
        X1_H = X1_L_Rx;
        
        Y1_L = Y1_H_Rx;
        Y1_H = Y1_L_Rx;

        Z1_L = Z1_H_Rx;
        Z1_H = Z1_L_Rx;

        F1_L = F1_H_Rx;
        F1_H = F1_L_Rx;


        X1 = ((uint16_t)X1_H << 8) | (uint8_t)X1_L;
        Y1 = ((uint16_t)Y1_H << 8) | (uint8_t)Y1_L;
        Z1 = ((uint16_t)Z1_H << 8) | (uint8_t)Z1_L;
        F1 = ((uint16_t)F1_H << 8) | (uint8_t)F1_L;

        tgtX      = (long)X1 * DISPLAY_UNIT;     
        tgtY      = (long)Y1 * DISPLAY_UNIT;
        tgtZ      = (long)Z1 * DISPLAY_UNIT;
        dropSteps = (long)F1 * DROP_UNIT_STEPS;   
        clampXY(); clampZ(); clampDrop();
        updateOLED();         

        // ★ 재진입 방지: 이미 이동 중이면 즉시 goToTargetXYZ() 재호출을 막음
        if (!isMoving) {
          goToTargetXYZ();
        }
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


        //Serial.print(X1); Serial.print(" ");
        //Serial.print(Y1); Serial.print(" ");
        //Serial.print(Z1); Serial.print(" ");
        //Serial.println(F1);
        
        // STX : 7B 5B 02 ( { [ STX ) 3BYTE
        Serial.write(0x7B); Serial.write(0x5B); Serial.write(0x02);

        // Len : 1BYTE
        Serial.write(0X00);
        Serial.write(0x0E);
        // FN :  2BYTE
        Serial.write(0x31);
        Serial.write(0x52);

        Serial.write(X1_L);
        Serial.write(X1_H);
        
        Serial.write(Y1_L);
        Serial.write(Y1_H);

        Serial.write(Z1_L);
        Serial.write(Z1_H);

        Serial.write(F1_L);
        Serial.write(F1_H);

        Serial.write(START_MODE_STATUE);
        Serial.write(END_MODE_STATUE);

        Serial.write(Error_Tx);
        Serial.write(Check_Xor_TX);

        // ETX : 03 5D 7D ( ETX ] } )
        Serial.write(0x03); Serial.write(0x5D); Serial.write(0x7D);
        
        command0 = 0;
    }
    Uart0ReciveCheckEnd = 0;
  }
}


//
// 프로그램이 계속 도는 곳(계속 반복)
//
void loop() {
  bool changed = false;                              // 화면을 새로 그릴지 알려주는 표시

  // ★ 인터럽트 진행량 반영을 주기적으로 수행 (OLED/저장 스로틀도 같이)
  serviceAxisProgress();
 
  UartRxProtocol();
  Serial_Main0();


  // ----- X 위치 조정 -----
  uint8_t e0 = pollBtn(IDX_XP);                      // X+ 버튼 상태 읽기(눌림/반복/없음)
  uint8_t e1 = pollBtn(IDX_XM);                      // X- 버튼 상태 읽기
  if (e0 == EV_PRESS || e0 == EV_REPEAT) {           // X+를 짧게/길게 누르면
    tgtX += SET_STEP_X;                              // 목표 X를 +1000
    changed = true;                                  // 화면 갱신 필요
    requestSave();                                   // 잠시 뒤 저장 예약
  }
  if (e1 == EV_PRESS || e1 == EV_REPEAT) {           // X-를 누르면
    tgtX -= SET_STEP_X;                              // 목표 X를 -1000
    changed = true;                                  // 화면 갱신 필요
    requestSave();                                   // 저장 예약
  }
  clampXY();                                         // X,Y 목표가 범위를 벗어나지 않게 정리

  // ----- Y 위치 조정 -----
  uint8_t e2 = pollBtn(IDX_YP);                      // Y+ 버튼
  uint8_t e3 = pollBtn(IDX_YM);                      // Y- 버튼
  if (e2 == EV_PRESS || e2 == EV_REPEAT) {           // Y+를 누르면
    tgtY += SET_STEP_Y;                              // 목표 Y를 +1000
    changed = true; requestSave();                   // 화면 갱신/저장 예약
  }
  if (e3 == EV_PRESS || e3 == EV_REPEAT) {           // Y-를 누르면
    tgtY -= SET_STEP_Y;                              // 목표 Y를 -1000
    changed = true; requestSave();                   // 화면 갱신/저장 예약
  }
  clampXY();                                         // X,Y 범위 정리

  // ----- Z 위치 조정 -----
  uint8_t eZp = pollBtn(IDX_ZP);                     // Z+ 버튼
  uint8_t eZm = pollBtn(IDX_ZM);                     // Z- 버튼
  if (eZp == EV_PRESS || eZp == EV_REPEAT) {         // Z+를 누르면
    tgtZ += SET_STEP_Z;                              // 목표 Z를 +1000
    changed = true; requestSave();                   // 화면 갱신/저장 예약
  }
  if (eZm == EV_PRESS || eZm == EV_REPEAT) {         // Z-를 누르면
    tgtZ -= SET_STEP_Z;                              // 목표 Z를 -1000
    changed = true; requestSave();                   // 화면 갱신/저장 예약
  }
  clampZ();                                          // Z 범위 정리

  // ----- 플런저 D 조정(6=+200, 7=-200) : 한쪽만 동작 + 짧게 1번, 충분히 길게 눌러야 천천히 반복 -----
  handleDropButtons(changed);                        // 전용 처리(여기서만 6/7을 다룸)

  // ----- RESET: X=0,Y=0,Z=0, D=200으로 만들고, 실제 그 자리로 이동 -----
  uint8_t e5 = pollBtn(IDX_RESET);                   // RESET 버튼 상태
  if (e5 == EV_PRESS) {                              // 딱 한 번 눌림이면
    tgtX = 0; tgtY = 0; tgtZ = 0;                    // 목표를 0으로
    dropSteps = DROP_UNIT_STEPS;                     // D를 200으로
    clampXY(); clampZ(); clampDrop();                // 안전 범위 정리
    updateOLED();                                    // 화면을 먼저 바꾸고

    // 실제로 이동 (이동 중이면 재진입 방지)
    if (!isMoving) {
      goToTargetXYZ();                               // 이동 수행(타이머 기반 X/Y + 기존 Z/플런저)
    }

    savePersistNow();                                // 바로 저장
    changed = true;                                  // 화면 갱신 표시
  }

  if (changed) updateOLED();                         // 값이 바뀌었으면 화면 새로 그림(현재 위치 기준)

  // ----- GO: 목표 자리로 이동하고, D만큼(200xN) 플런저를 눌러 액체를 떨어뜨림 -----
  uint8_t e4 = pollBtn(IDX_GO);                      // GO 버튼 상태
  if (e4 == EV_PRESS) {                              // 딱 한 번 눌림이면
    if (!isMoving) {
      goToTargetXYZ();                               // 목표로 이동(절대 좌표)
      doOneDrop();                                   // 한 번 작업 실행
      savePersistNow();                              // 끝난 뒤 저장
      updateOLED();                                  // 화면 갱신(현재 위치 기준)
    }
  }

  trySaveIfIdle();                                   // 값이 더 안 바뀌면 1.5초 뒤 EEPROM 저장
}