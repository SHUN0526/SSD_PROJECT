#include <Wire.h>                       // I2C(아이투씨) 통신을 쓰기 위한 기본 라이브러리
#include <Adafruit_GFX.h>               // 글자/그림을 OLED에 그릴 때 쓰는 라이브러리
#include <Adafruit_SSD1306.h>           // SSD1306 칩을 쓰는 OLED를 쉽게 다루는 라이브러리
#include <EEPROM.h>                     // 전원을 꺼도 남는 저장공간(EEPROM)을 쓰는 라이브러리
#include <Arduino.h>                    // 아두이노 기본 함수(pinMode 등)를 쓰려면 필요
#include <avr/io.h>                     // AVR 칩의 레지스터를 직접 쓸 때 필요
#include <avr/interrupt.h>              // 인터럽트(특별히 급한 일 처리)를 쓸 때 필요

#define SCREEN_WIDTH 128                // OLED 가로 픽셀 수
#define SCREEN_HEIGHT 32                // OLED 세로 픽셀 수
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire); // 위 크기와 I2C로 화면 객체 만들기

// ===== 좌표/동작 기본 설정 =====
const long DISPLAY_UNIT = 95;           // 우리가 쓰는 좌표 1칸을 실제 모터 95스텝으로 바꿔주는 값
const int  STEP_HALF_US = 150;          // 작을수록 더 빨리 움직임
// ----- 플런저(주사기) -----
const long DROP_UNIT_STEPS = 200;       // 플런저 1칸(N) = 200스텝으로 정의
long dropSteps = DROP_UNIT_STEPS;       // 플런저의 목표 위치(절대 스텝) — 처음엔 200으로 시작
const long MIN_DROP = -10000;           // 플런저가 올라갈 수 있는 최댓치(음수 임향 한계)
const long MAX_DROP =  10000;           // 플런저가 눌릴 수 있는 최댓치(양수 방향 한계)

// ===== 모터 방향(하드웨어 결선에 맞춘 값) =====
const uint8_t X_DIR_POS   = LOW;        // X(실제 기계의 X)에 +방향으로 움직일 때 DIR핀 값
const uint8_t X_DIR_NEG   = HIGH;       // X에 -방향일 때 DIR핀 값
const uint8_t STAGEY_DIR_POS = HIGH;    // Y에 +방향일 때 DIR핀 값
const uint8_t STAGEY_DIR_NEG = LOW;     // Y에 -방향일 때 DIR핀 값
const uint8_t LIFT_DIR_UP    = HIGH;    // Z에 위로(+방향) 움직일 때 DIR핀 값
const uint8_t LIFT_DIR_DOWN  = LOW;     // Z에 아래로(-방향) 움직일 때 DIR핀 값
const uint8_t UD_DIR_DOWN = HIGH;       // 플런저를 누르는(+방향) DIR핀 값
const uint8_t UD_DIR_UP   = LOW;        // 플런저를 올리는(-방향) DIR핀 값

// ===== 드라이버 핀 연결(보드 핀 번호) =====
const uint8_t UD_PUL = 8,  UD_DIR = 9;  // 플런저 모터의 펄스/방향 핀
const uint8_t X_PUL  = 10, X_DIR  = 11; // 보드 X → 실제 Y축(결선상 이렇게 대응)
const uint8_t Y_PUL  = 12, Y_DIR  = 13; // 보드 Y → 실제 Z축
const uint8_t Z_PUL  = 14, Z_DIR  = 15; // 보드 Z → 실제 X축

// ===== 버튼(원점만 사용) =====
const uint8_t BTN_RESET00 = 25;         // 원점 복귀(목표 0으로) 버튼

// ===== 현재 위치/목표 위치 변수(단위: 스텝) =====
long curX = 0, curY = 0, curZ = 0;      // 지금 실제 모터가 있는 위치
long tgtX = 0, tgtY = 0, tgtZ = 0;      // 가고 싶은 목표 위치

// 플런저(U축) 현재/목표 위치(단위: 스텝)
long curUD = 0;                          // 플런저 현재 스텝 위치(+:눌림, -:올림)
long tgtUD = 0;                          // 플런저 목표 스텝 위치

// ===== 이동 한계(안전 범위) =====
const long MIN_X = 0;                    // X가 갈 수 있는 최소치
const long MAX_X = 170 * DISPLAY_UNIT;   // X가 갈 수 있는 최대치
const long MIN_Y = -220 * DISPLAY_UNIT;  // Y 최소치
const long MAX_Y = 0;                    // Y 최대치(0)

const long MIN_Z = -140 * DISPLAY_UNIT;  // Z 최소치
const long MAX_Z = 0 * DISPLAY_UNIT;     // Z 최대치(0)

// Z 전용 속도(여긴 균일 속도만 사용)
const int  STEP_HALF_US_Z_MIN   = 220;   // Z축 펄스의 반 주기(숫자가 작을수록 더 빠름)

// ===== 목표값을 안전 범위로 잠그는(클램프) 함수들 =====
inline void clampXY() {                  // X/Y 목표가 한계를 넘지 않게 보정
  if (tgtX < MIN_X) tgtX = MIN_X; else if (tgtX > MAX_X) tgtX = MAX_X; // X 범위 고정
  if (tgtY < MIN_Y) tgtY = MIN_Y; else if (tgtY > MAX_Y) tgtY = MAX_Y; // Y 범위 고정
}
inline void clampZ() {                   // Z 목표 범위 보정
  if (tgtZ < MIN_Z) tgtZ = MIN_Z; else if (tgtZ > MAX_Z) tgtZ = MAX_Z;
}
inline void clampDrop() {                // 플런저 목표(dropSteps) 범위 보정
  if (dropSteps < MIN_DROP) dropSteps = MIN_DROP;
  else if (dropSteps > MAX_DROP) dropSteps = MAX_DROP;
}
inline void clampUDPos() {               // 플런저 현재 위치(curUD)도 범위 보정
  if (curUD < MIN_DROP) curUD = MIN_DROP;
  else if (curUD > MAX_DROP) curUD = MAX_DROP;
}

// ===== EEPROM(저장공간) 구조 =====
struct PersistV5 { uint32_t magic; long curX; long curY; long curZ; long curUD; long dropSteps; }; // 저장할 데이터 묶음

const uint32_t MAGIC_V5 = 0xC0FFEE7A;   // "이 데이터는 V5다!"라고 표시하는 번호
const int EEPROM_ADDR = 0;               // EEPROM에서 저장을 시작할 주소(0번부터)

// ===== 저장 관련 보조 변수들 =====
bool needSave = false;                   // 조금 있다 저장할 게 있는지 표시
unsigned long lastEditMs = 0;            // 마지막으로 값이 바뀐 시간
const unsigned long SAVE_DELAY_MS = 100; // 바뀐 뒤 0.1초 지나면 실제 저장

const unsigned long POS_SAVE_INTERVAL_MS = 100; // 이동 중, 0.1초마다 저장할지 확인
const long          POS_SAVE_DELTA_STEPS = 200; // 200스텝 이상 움직였을 때만 저장
unsigned long lastPosSaveMs = 0;               // 마지막으로 저장한 시각
long lastSavedX = 0, lastSavedY = 0, lastSavedZ = 0, lastSavedUD = 0; // 마지막 저장된 위치들

long lastOLEDUnitX = 0, lastOLEDUnitY = 0, lastOLEDUnitZ = 0; // 마지막으로 OLED에 표시한 값(좌표 단위)
long lastOLEDBurstsUD = 0;               // 플런저 N(=200스텝 단위)로 마지막 표시한 값

// ===== 스텝 펄스를 직접 만드는 간단 함수(블로킹) — 플런저에서 사용 =====
static inline void stepHalf(uint8_t PUL, int half_us) { // 펄스 핀을 반 주기만큼 HIGH, 그다음 반 주기 LOW
  digitalWrite(PUL, HIGH);  delayMicroseconds(half_us); // 펄스 HIGH 유지
  digitalWrite(PUL, LOW);   delayMicroseconds(half_us); // 펄스 LOW 유지 → 합치면 1스텝
}

// ─────────────────────────────────────────────────────────────
// 타이머 기반 스텝 드라이버(인터럽트가 자동으로 펄스를 만들어줌)
// Timer1 = Y축(보드 X) / Timer3 = X축(보드 Z) / Timer2 = Z축(보드 Y)
// ─────────────────────────────────────────────────────────────
volatile bool isMoving = false;          // 지금 모터들이 움직이는 중인지 표시

// Timer1(Y)에서 쓸 변수들(인터럽트가 접근하므로 volatile)
volatile uint8_t* T1_PUL_PORT;           // Y 펄스 핀이 달린 포트 주소
volatile uint8_t  T1_PUL_MASK;           // 그 포트에서 펄스 핀의 비트 마스크
volatile long     T1_steps_remaining = 0;// 아직 움직여야 할 스텝 수
volatile long     T1_steps_done = 0;     // 방금까지 진행된 스텝 수(루프에서 꺼내 씀)
volatile int8_t   T1_step_sign  = +1;    // +방향인지 -방향인지
volatile bool     T1_pulse_state = false;// 지금 HIGH를 낼 차례인지 LOW를 낼 차례인지

// Timer3(X)에서 쓸 변수들
volatile uint8_t* T3_PUL_PORT;           // X 펄스 포트 주소
volatile uint8_t  T3_PUL_MASK;           // X 펄스 마스크
volatile long     T3_steps_remaining = 0;// X 남은 스텝
volatile long     T3_steps_done = 0;     // X 진행 스텝
volatile int8_t   T3_step_sign  = +1;    // X 방향
volatile bool     T3_pulse_state = false;// X 펄스 토글 상태

// Timer2(Z)에서 쓸 변수들
volatile uint8_t* T2_PUL_PORT;           // Z 펄스 포트 주소
volatile uint8_t  T2_PUL_MASK;           // Z 펄스 마스크
volatile long     T2_steps_remaining = 0;// Z 남은 스텝
volatile long     T2_steps_done = 0;     // Z 진행 스텝
volatile int8_t   T2_step_sign  = +1;    // Z 방향
volatile bool     T2_pulse_state = false;// Z 펄스 토글 상태

// Timer1 ISR — 일정 시간마다 자동으로 불려 Y 펄스를 만듦
ISR(TIMER1_COMPA_vect) {
  if (T1_steps_remaining > 0) {          // 아직 갈 스텝이 남아있으면
    if (T1_pulse_state) { *T1_PUL_PORT |=  T1_PUL_MASK; }  // 이번엔 HIGH -- OR..... 레지스터 오픈 - 핀 번호 수정 ... 그 비트만 1로 만들기
    else { *T1_PUL_PORT &= ~T1_PUL_MASK; T1_steps_remaining--; T1_steps_done++; } // 다음엔 LOW + 1스텝 완료 -- AND그리고 값 다 바꾸기... 그 비트만 0으로 만들기
    T1_pulse_state = !T1_pulse_state;    // 다음 번엔 반대로.... 토글
  } else {                                // 다 갔으면
    TIMSK &= ~(1 << OCIE1A);             // 이 타이머 인터럽트 끄기
    *T1_PUL_PORT &= ~T1_PUL_MASK;        // 펄스 핀 LOW로 정리
  }
}

// Timer3 ISR — X 펄스를 만듦
ISR(TIMER3_COMPA_vect) {
  if (T3_steps_remaining > 0) {
    if (T3_pulse_state) { *T3_PUL_PORT |=  T3_PUL_MASK; }  // HIGH
    else { *T3_PUL_PORT &= ~T3_PUL_MASK; T3_steps_remaining--; T3_steps_done++; } // LOW + 1스텝 완료
    T3_pulse_state = !T3_pulse_state;    // 다음엔 반대로
  } else {                                // 다 갔으면
    ETIMSK &= ~(1 << OCIE3A);            // 타이머3 인터럽트 끄기
    *T3_PUL_PORT &= ~T3_PUL_MASK;        // 펄스 핀 LOW로
  }
}

// Timer2 ISR — Z 펄스를 만듦
ISR(TIMER2_COMP_vect) {
  if (T2_steps_remaining > 0) {
    if (T2_pulse_state) { *T2_PUL_PORT |=  T2_PUL_MASK; }  // HIGH
    else { *T2_PUL_PORT &= ~T2_PUL_MASK; T2_steps_remaining--; T2_steps_done++; } // LOW + 1스텝 완료
    T2_pulse_state = !T2_pulse_state;    // 다음엔 반대로
  } else {                                // 다 갔으면
    TIMSK &= ~(1 << OCIE2);              // 타이머2 인터럽트 끄기
    *T2_PUL_PORT &= ~T2_PUL_MASK;        // 펄스 핀 LOW로
  }
}

// 타이머로 모터를 출발시키는 함수(속도는 half_us로 결정)
static inline void startTimerMotor(
  uint8_t timer_id, uint8_t pulPin, uint8_t dirPin, uint8_t dirLevel,
  long steps, int half_us, int8_t step_sign
){
  if (steps <= 0) return;                // 갈 게 없으면 아무것도 안 함

  pinMode(pulPin, OUTPUT);               // 펄스 핀 출력으로
  pinMode(dirPin, OUTPUT);               // 방향 핀 출력으로
  digitalWrite(dirPin, dirLevel);        // 원하는 방향으로 미리 세팅
  delayMicroseconds(100);                // 아주 잠깐 기다려 안정화

  noInterrupts();                        // 설정 중엔 인터럽트 잠시 끄기(안전)

  if (timer_id == 1) {                   // 타이머1(Y) 설정
    uint32_t ocr = (uint32_t)(((uint64_t)F_CPU / 8ULL) * (uint64_t)half_us / 1000000ULL); // 비교값 계산
    if (ocr == 0) ocr = 1; ocr -= 1;     // 최소 1 보장

    T1_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin)); // 포트/마스크 기록
    T1_PUL_MASK = digitalPinToBitMask(pulPin);
    T1_steps_remaining = steps; T1_steps_done = 0; T1_step_sign = step_sign; T1_pulse_state = false; // 상태 초기화

    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;   // 타이머1 초기화
    TCCR1B |= (1 << WGM12);              // CTC 모드
    OCR1A   = (uint16_t)ocr;             // 비교값 넣기
    TCCR1B |= (1 << CS11);               // 분주 /8
    TIMSK  |= (1 << OCIE1A);             // 비교매치 인터럽트 켜기
  }
  else if (timer_id == 3) {              // 타이머3(X) 설정
    uint32_t ocr = (uint32_t)(((uint64_t)F_CPU / 8ULL) * (uint64_t)half_us / 1000000ULL);
    if (ocr == 0) ocr = 1; ocr -= 1;

    T3_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin));
    T3_PUL_MASK = digitalPinToBitMask(pulPin);
    T3_steps_remaining = steps; T3_steps_done = 0; T3_step_sign = step_sign; T3_pulse_state = false;

    TCCR3A = 0; TCCR3B = 0; TCNT3 = 0;   // 타이머3 초기화
    TCCR3B |= (1 << WGM32);              // CTC 모드
    OCR3A   = (uint16_t)ocr;             // 비교값
    TCCR3B |= (1 << CS31);               // 분주 /8
    ETIMSK |= (1 << OCIE3A);             // 인터럽트 켜기
  }
  else if (timer_id == 2) {              // 타이머2(Z) 설정
    uint32_t ticks = ((uint64_t)F_CPU * (uint64_t)half_us) / 1000000ULL; // 기본 틱 계산

    struct Opt { uint16_t presc; uint8_t cs; } opts[] = { // 가능한 분주기 조합들
      { 8,    (1<<CS21) },
      { 32,   (1<<CS21)|(1<<CS20) },
      { 64,   (1<<CS22) },
      { 128,  (1<<CS22)|(1<<CS20) },
      { 256,  (1<<CS22)|(1<<CS21) },
      { 1024, (1<<CS22)|(1<<CS21)|(1<<CS20) }
    };

    uint8_t csbits = 0;                  // 선택한 분주 비트
    uint8_t ocr8 = 255;                  // 비교값
    for (uint8_t i=0; i<sizeof(opts)/sizeof(opts[0]); i++) { // 각 옵션을 시험
      uint32_t cnt = ticks / opts[i].presc;                  // 카운트 계산
      if (cnt >= 2 && cnt <= 256) { ocr8 = (uint8_t)(cnt - 1); csbits = opts[i].cs; break; } // 가능하면 선택
    }

    T2_PUL_PORT = portOutputRegister(digitalPinToPort(pulPin)); // 포트/마스크 기록
    T2_PUL_MASK = digitalPinToBitMask(pulPin);
    T2_steps_remaining = steps; T2_steps_done = 0; T2_step_sign = step_sign; T2_pulse_state = false; // 상태 초기화

    TCCR2 = 0; TCNT2 = 0;                // 타이머2 초기화
    TCCR2 |= (1 << WGM21);               // CTC 모드
    OCR2   = ocr8;                        // 비교값
    TCCR2 |= csbits;                      // 분주 설정
    TIMSK |= (1 << OCIE2);               // 인터럽트 켜기
  }

  interrupts();                           // 설정 끝. 다시 인터럽트 허용
}

// 이동 중에 일정 간격으로 위치를 EEPROM에 저장(전원 꺼져도 기억하려고)
inline void maybeSaveCurPosDuringMove() {
  unsigned long now = millis();           // 지금 시간(ms)
  if ((now - lastPosSaveMs) >= POS_SAVE_INTERVAL_MS) { // 마지막 저장에서 50ms 지났으면
    long dx = curX - lastSavedX, dy = curY - lastSavedY, dz = curZ - lastSavedZ, du = curUD - lastSavedUD; // 변화량
    if ( (dx>=POS_SAVE_DELTA_STEPS || dx<=-POS_SAVE_DELTA_STEPS) ||   // 어느 축이든 50스텝 이상 바뀌면
         (dy>=POS_SAVE_DELTA_STEPS || dy<=-POS_SAVE_DELTA_STEPS) ||
         (dz>=POS_SAVE_DELTA_STEPS || dz<=-POS_SAVE_DELTA_STEPS) ||
         (du>=POS_SAVE_DELTA_STEPS || du<=-POS_SAVE_DELTA_STEPS) ) {
      PersistV5 p { MAGIC_V5, curX, curY, curZ, curUD, dropSteps }; // 저장할 묶음 만들고
      EEPROM.put(EEPROM_ADDR, p);            // EEPROM에 쓰기
      lastPosSaveMs = now;                   // 저장 시각 기억
      lastSavedX = curX; lastSavedY = curY; lastSavedZ = curZ; lastSavedUD = curUD; // 저장한 값 기억
    }
  }
}

// ===== OLED(작은 화면) 도움 함수: 숫자를 오른쪽 정렬로 찍기 =====
static void printRightAligned(long value, int rightX, int y, uint8_t size, bool invertedText=false) {
  char buf[16];                            // 숫자를 글자로 담을 공간
  int len = snprintf(buf, sizeof(buf), "%ld", value); // 숫자를 글자로 바꾸기
  if (len < 0) len = 0;                    // 혹시 에러면 0
  int w = len * 6 * size;                  // 글자 폭 계산(6픽셀 × 크기)
  int x = rightX - w + 1; if (x < 0) x = 0;// 오른쪽 끝에 맞춰 x 계산
  if (invertedText) display.setTextColor(BLACK, WHITE); // 반전이면 흰 배경에 검은 글자
  else              display.setTextColor(WHITE);        // 아니면 검은 배경에 흰 글자
  display.setTextSize(size);               // 글자 크기
  display.setCursor(x, y);                 // 글자 시작 위치
  display.print(buf);                      // 숫자 출력
  display.setTextColor(WHITE);             // 색 복구
}

// 작은 배지(X/Y/Z/D 글자) 그리기
static void badgeFilled(int x, int y, const __FlashStringHelper* label) {
  display.fillRoundRect(x, y, 18, 10, 2, WHITE);    // 흰 동그란 네모(테두리)
  display.fillRoundRect(x+1, y+1, 16, 8, 2, BLACK); // 안쪽을 검정으로
  display.setTextSize(1);                            // 작은 글자 크기
  display.setTextColor(WHITE);                       // 흰 글자
  display.setCursor(x + 5, y + 2);                   // 글자 위치
  display.print(label);                              // 라벨 출력
  display.setTextColor(WHITE);                       // 색 유지
}

// OLED 화면 갱신(현재 X/Y/Z와 플런저 N 보여주기)
void updateOLED() {
  long x = curX / DISPLAY_UNIT;            // X를 보기 쉬운 단위로
  long y = curY / DISPLAY_UNIT;            // Y를 보기 쉬운 단위로
  long z = curZ / DISPLAY_UNIT;            // Z를 보기 쉬운 단위로
  long n = curUD / DROP_UNIT_STEPS;        // 플런저는 200스텝=1칸으로 표시

  display.clearDisplay();                   // 화면 지우기

  // 카드 한 칸(64×16) 그려서 값 표시하는 작은 함수
  auto drawCard = [&](int x0, int y0, const __FlashStringHelper* lab, long val, bool showX200=false) {
    display.fillRoundRect(x0, y0, 64, 16, 3, WHITE);  // 흰 카드
    display.drawRoundRect(x0, y0, 64, 16, 3, BLACK);  // 검은 테두리
    badgeFilled(x0 + 2, y0 + 2, lab);                 // 왼쪽 위 라벨(X/Y/Z/D)
    printRightAligned(val, x0 + 63, y0 + 0, 2, true); // 오른쪽에 큰 숫자(반전 글자)
  };

  drawCard(  0,  0, F("X"), x, false);     // 왼쪽 위 카드: X
  drawCard( 64,  0, F("Y"), y, false);     // 오른쪽 위 카드: Y
  drawCard(  0, 16, F("Z"), z, false);     // 왼쪽 아래 카드: Z
  drawCard( 64, 16, F("D"), n, true);      // 오른쪽 아래 카드: D(플런저 N)

  display.display();                        // 실제로 화면에 보여주기
}

// 지금 상태를 즉시 EEPROM에 저장
void savePersistNow() {
  PersistV5 p { MAGIC_V5, curX, curY, curZ, curUD, dropSteps }; // 저장 묶음 만들기
  EEPROM.put(EEPROM_ADDR, p);             // EEPROM에 쓰기
  lastSavedX = curX; lastSavedY = curY; lastSavedZ = curZ; lastSavedUD = curUD; // 기록 업데이트
  lastPosSaveMs = millis();               // 저장 시간
  lastOLEDUnitX = curX / DISPLAY_UNIT;    // 표시 기준도 새로 기억
}

// 움직일 때 너무 자주 화면을 그리지 않도록, 단위가 바뀔 때만 갱신
inline void maybeUpdateOLEDWhileMoving(char axis) {
  long ux = curX / DISPLAY_UNIT, uy = curY / DISPLAY_UNIT, uz = curZ / DISPLAY_UNIT; // 단위로 변환
  bool changed = false;                    // 이번에 화면을 바꿀지 표시
  if (axis == 'X' && ux != lastOLEDUnitX) { lastOLEDUnitX = ux; changed = true; } // X 단위 바뀌면
  if (axis == 'Y' && uy != lastOLEDUnitY) { lastOLEDUnitY = uy; changed = true; } // Y 단위 바뀌면
  if (axis == 'Z' && uz != lastOLEDUnitZ) { lastOLEDUnitZ = uz; changed = true; } // Z 단위 바뀌면

  if (axis == 'U') {                      // 플런저(U축)일 때
    long ub = curUD / DROP_UNIT_STEPS;    // N 단위로
    if (ub != lastOLEDBurstsUD) {         // 바뀌었으면
      lastOLEDBurstsUD = ub;              // 새 값 기억
      changed = true;                     // 화면 갱신할 것
      savePersistNow();                   // 이때마다 저장해 두면 전원 꺼져도 이어서 정확
    }
  }
  if (changed) updateOLED();              // 바꿔야 하면 화면 갱신
}

// 인터럽트가 세어준 진행 스텝을 실제 위치(curX/Y/Z)에 반영
inline void serviceAxisProgress() {
  long d1 = 0, d3 = 0, d2 = 0;            // 각 축 진행량 임시 저장
  noInterrupts();                          // 값 읽는 동안 인터럽트 잠깐 끄기(안전)
  if (T1_steps_done) { d1 = T1_steps_done; T1_steps_done = 0; } // Y 진행량 꺼내기
  if (T3_steps_done) { d3 = T3_steps_done; T3_steps_done = 0; } // X 진행량 꺼내기
  if (T2_steps_done) { d2 = T2_steps_done; T2_steps_done = 0; } // Z 진행량 꺼내기
  interrupts();                            // 다시 인터럽트 켜기

  if (d3) { curX += (long)T3_step_sign * d3; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('X'); } // X 반영
  if (d1) { curY += (long)T1_step_sign * d1; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('Y'); } // Y 반영
  if (d2) { curZ += (long)T2_step_sign * d2; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('Z'); } // Z 반영
}

// 블로킹 방식으로 스텝을 직접 만드는 이동(플런저 등에서 사용)
void moveAxisSteps(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                   long steps, int half_us, long &curAxis, int stepSign, char axisTag) {
  if (steps <= 0) return;                 // 갈 게 없으면 끝
  digitalWrite(DIR, dirLevel);            // 방향 먼저 세팅
  delayMicroseconds(100);                 // 잠깐 기다리기
  for (long i = 0; i < steps; i++) {      // 필요한 스텝 수만큼 반복
    stepHalf(PUL, half_us);               // 펄스 1번 = 1스텝
    curAxis += stepSign;                  // 현재 위치 1스텝 반영(+ 또는 -)
    maybeSaveCurPosDuringMove();          // 가끔 저장
    maybeUpdateOLEDWhileMoving(axisTag);  // 가끔 화면 갱신
  }
}

// 저장 예약 표시 함수(나중에 저장하도록)
void requestSave() { needSave = true; lastEditMs = millis(); } // 지금 바뀌었으니 조금 뒤 저장하자고 표시
void trySaveIfIdle() {                     // 잠깐 쉴 때 저장 실행
  if (needSave && (millis() - lastEditMs >= SAVE_DELAY_MS)) { savePersistNow(); needSave = false; }
}

// 전원을 켰을 때, 지난번에 저장한 상태를 불러오기(V5만)
void loadPersist() {
  uint32_t magic = 0; EEPROM.get(EEPROM_ADDR, magic); // 먼저 magic을 읽어서 버전 확인
  if (magic == MAGIC_V5) {               // V5 형식이면
    PersistV5 p; EEPROM.get(EEPROM_ADDR, p); // 전부 읽어오고
    curX = p.curX; curY = p.curY; curZ = p.curZ; curUD = p.curUD; dropSteps = p.dropSteps; // 복구
  } else {                                // 그 외면(빈 칩/다른 버전)
    curX = curY = curZ = 0;              // 좌표 0으로 시작
    dropSteps = DROP_UNIT_STEPS;          // 플런저 목표는 200(=N 1칸)
    curUD = 0;                            // 플런저 현재 위치 0
  }
  clampUDPos();                           // 플런저 현재 위치도 안전 범위로
  tgtX = curX; tgtY = curY; tgtZ = curZ;  // 목표를 현재와 같게(부팅 직후 갑자기 움직이지 않게)
  clampXY(); clampZ(); clampDrop();       // 목표값들도 안전 범위 보정
  lastOLEDUnitX = curX / DISPLAY_UNIT;    // OLED 표시 기준 초기화
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
  lastSavedUD  = curUD;                   // 마지막 저장된 플런저 위치 기억
  lastOLEDBurstsUD = curUD / DROP_UNIT_STEPS; // 플런저 N 기준 값도 세팅
}

// 플런저 동작: dropSteps(절대 목표)까지 실제로 움직이기
void doOneDrop() {
  tgtUD = dropSteps;                      // 목표 스텝 복사
  clampDrop();                            // 목표 안전 범위
  clampUDPos();                           // 현재 위치도 안전 범위

  long delta = tgtUD - curUD;             // 남은 거리(스텝)
  if (delta == 0) { savePersistNow(); return; } // 이미 목표면 저장만 하고 끝

  if (delta > 0) {                        // 더 눌러야 하면(+방향)
    moveAxisSteps(UD_PUL, UD_DIR, UD_DIR_DOWN, delta, STEP_HALF_US, curUD, +1, 'U'); // 눌러서 전진
  } else {                                // 올려야 하면(-방향)
    moveAxisSteps(UD_PUL, UD_DIR, UD_DIR_UP,  -delta, STEP_HALF_US, curUD, -1, 'U'); // 올려서 후진
  }
  savePersistNow();                       // 끝났으니 저장
}

// ISR 방식으로 축 이동을 "시작"만 하는 함수(즉, 비동기 출발)
void startAxisSteps_ISR(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                        long steps, int half_us, int8_t stepSign, char axisTag) {
  if (steps <= 0) return;                 // 스텝 없으면 끝
  uint8_t timer_id = 0;                   // 어떤 타이머 쓸지 고르기
  if (axisTag == 'X') timer_id = 3;      // X는 타이머3
  else if (axisTag == 'Y') timer_id = 1; // Y는 타이머1
  else if (axisTag == 'Z') timer_id = 2; // Z는 타이머2
  else return;                            // 그 외는 없음
  startTimerMotor(timer_id, PUL, DIR, dirLevel, steps, half_us, stepSign); // 타이머 출발!
}

// ISR 방식으로 이동하고, 끝날 때까지 기다리는 함수
void moveAxisSteps_ISR(uint8_t PUL, uint8_t DIR, uint8_t dirLevel,
                       long steps, int half_us, long &curAxis, int stepSign, char axisTag) {
  if (steps <= 0) return;                 // 스텝 없으면 끝

  uint8_t timer_id = 0;                   // 타이머 고르기
  if (axisTag == 'X') timer_id = 3;
  else if (axisTag == 'Y') timer_id = 1;
  else if (axisTag == 'Z') timer_id = 2;
  else {                                  // 타이머를 안 쓰는 축이면(플런저 등)
    moveAxisSteps(PUL, DIR, dirLevel, steps, half_us, curAxis, stepSign, axisTag); // 직접 1스텝씩
    return;                               // 끝
  }

  startTimerMotor(timer_id, PUL, DIR, dirLevel, steps, half_us, (int8_t)stepSign); // 타이머 출발

  for (;;) {                               // 이 축이 끝날 때까지 반복
    serviceAxisProgress();                 // 인터럽트가 쌓은 진행량을 위치에 반영
    trySaveIfIdle();                       // 저장할 게 있으면 지금 저장
    bool done = (timer_id == 3) ? (T3_steps_remaining == 0) : // 이 축이 다 끝났는지 확인
                (timer_id == 1) ? (T1_steps_remaining == 0) :
                                  (T2_steps_remaining == 0);
    if (done) break;                       // 끝났으면 반복 종료
  }
  serviceAxisProgress();                   // 마지막으로 한 번 더 반영
}

// 목표로 이동 + 플런저 동작 + Z를 다시 0으로 복귀
void goToTargetXYZ() {
  if (isMoving) return;                    // 이미 움직이는 중이면 겹치지 않게 종료
  isMoving = true;                          // 이제부터 움직인다고 표시

  clampXY(); clampZ();                      // 목표값이 안전한지 확인

  long dx = tgtX - curX;                    // X로 얼마나 가야 하는지
  long dy = tgtY - curY;                    // Y로 얼마나 가야 하는지
  long dz = tgtZ - curZ;                    // Z로 얼마나 가야 하는지

  int8_t sx = (dx >= 0) ? +1 : -1;          // X 방향(+ 또는 -)
  int8_t sy = (dy >= 0) ? +1 : -1;          // Y 방향(+ 또는 -)

  long adx = (dx >= 0) ? dx : -dx;          // X 절댓값
  long ady = (dy >= 0) ? dy : -dy;          // Y 절댓값

  // X와 Y가 "같은 시간에" 도착하도록 속도를 보정(둘 중 스텝이 더 많은 축에 속도를 맞춤)
  int half_x = STEP_HALF_US;                // X 기본 반 주기
  int half_y = STEP_HALF_US;                // Y 기본 반 주기
  long maxSteps = (adx > ady) ? adx : ady;  // 더 많이 가는 축의 스텝 수
  if (maxSteps > 0) {                       // 0이 아니면 보정
    if (adx > 0) half_x = (int)((long)STEP_HALF_US * maxSteps / adx); // X 속도 보정
    if (ady > 0) half_y = (int)((long)STEP_HALF_US * maxSteps / ady); // Y 속도 보정
    if (half_x < 1) half_x = 1; if (half_y < 1) half_y = 1;           // 최소값 보호
  }

  // X와 Y는 동시에 출발(요청사항: "동시에 움직이는 건 X/Y뿐")
  bool startedX = false, startedY = false;
  if (adx > 0) { startAxisSteps_ISR(Z_PUL, Z_DIR, (sx > 0 ? X_DIR_POS : X_DIR_NEG), adx, half_x, sx, 'X'); startedX = true; } // X 시작
  if (ady > 0) { startAxisSteps_ISR(X_PUL, X_DIR, (sy > 0 ? STAGEY_DIR_POS : STAGEY_DIR_NEG), ady, half_y, sy, 'Y'); startedY = true; } // Y 시작

  while ( (startedX && T3_steps_remaining > 0) || (startedY && T1_steps_remaining > 0) ) { // 둘 중 하나라도 남아있으면
    serviceAxisProgress(); trySaveIfIdle(); // 진행 반영 + 저장
  }
  serviceAxisProgress();                   // 마지막 반영

  // Z는 따로 혼자 움직임(요청 그대로: X/Y만 동시, Z는 별도)
  if (dz > 0) {
    moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_UP,   dz, STEP_HALF_US_Z_MIN, curZ, +1, 'Z'); // 위로 이동
  } else if (dz < 0) {
    moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_DOWN, -dz, STEP_HALF_US_Z_MIN, curZ, -1, 'Z'); // 아래로 이동
  }

  // 플런저 동작(절대 목표 dropSteps까지)
  doOneDrop();

  // Z는 항상 0(=MAX_Z)으로 복귀 → 작업 높이를 일정하게 유지
  if (curZ != MAX_Z) {
    long dz_home = MAX_Z - curZ;          // 0까지 남은 거리
    if (dz_home > 0) {
      moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_UP,   dz_home, STEP_HALF_US_Z_MIN, curZ, +1, 'Z');
    } else {
      moveAxisSteps_ISR(Y_PUL, Y_DIR, LIFT_DIR_DOWN, -dz_home, STEP_HALF_US_Z_MIN, curZ, -1, 'Z');
    }
  }

  savePersistNow();                        // 모든 동작 후 상태 저장
  isMoving = false;                        // 움직임 끝 표시
}

/* ===== 시리얼(컴퓨터↔보드) 프로토콜 =====
   들어오는 패킷을 해석해 X/Y/Z 목표와 F(플런저 N 변위)를 적용하고
   바로 goToTargetXYZ()를 실행합니다.
*/
unsigned char command0 = 0;               // 명령을 다 받았는지 표시(1이면 처리할 것 있음)
unsigned char Uart0ProtocolPointer = 0;   // 프로토콜의 진행 단계
unsigned char Uart0ReciveCheckEnd = 0;    // 패킷 끝까지 받았는지 표시

unsigned char Uart0_Data_Count = 0;       // 받은 바이트 개수(디버그용)
unsigned char Check_Xor =0;               // 수신 체크섬 계산값
unsigned char Check_Xor_Check =0;         // 체크섬 확인용 저장
unsigned char Check_Xor_TX = 0;           // 송신 체크섬 계산값

unsigned char Error_Tx = 0;               // 보낼 에러 비트
unsigned char Error = 0;                  // 내부 에러 비트
unsigned char Error_COM = 0;              // 통신 에러
unsigned char Error_BCC = 0;              // 체크섬 에러
unsigned char Error_LEN = 0;              // 길이 에러
unsigned char Error_LEN_Count = 0;        // 길이 카운트

int X1 = 0, Y1 = 0, Z1 = 0, F1 = 0;       // 해석된 X/Y/Z/플런저N 값(16비트)

// 수신 바이트들(L/H로 쪼갠 값들)
char X1_L_Rx = 0, Y1_L_Rx = 0, Z1_L_Rx = 0, F1_L_Rx = 0;
char X1_L = 0,    Y1_L = 0,    Z1_L = 0,    F1_L = 0;

char X1_H_Rx = 0, Y1_H_Rx = 0, Z1_H_Rx = 0, F1_H_Rx = 0;
char X1_H = 0,    Y1_H = 0,    Z1_H = 0,    F1_H = 0;

char FN_Port_Rx = 0;                      // 상태 바이트
char Error1_Rx = 0;                       // 에러 바이트(추가)

char FN_Port0_Rx = 0, FN_Port1_Rx = 0, FN_Port2_Rx = 0, FN_Port3_Rx = 0; // 상태 바이트의 각 비트
char FN_Port4_Rx = 0, FN_Port5_Rx = 0, FN_Port6_Rx = 0, FN_Port7_Rx = 0;

char START_MODE_STATUE = 0;               // 시작 상태(보낼 때 사용)
char END_MODE_STATUE = 0;                 // 끝 상태(보낼 때 사용)

// 수신 바이트들을 한 글자씩 읽어서 정해진 형태로 해석
void UartRxProtocol()
{
  char Uart0_Data;                        // 이번에 읽은 바이트
  if(Serial.available()) {                // 읽을 게 있으면
      Uart0_Data = Serial.read();         // 1바이트 읽기
      switch(Uart0ProtocolPointer)        // 현재 단계에 따라 처리
      {
        case 0:     if(0x7b == Uart0_Data ) { Uart0ProtocolPointer = 1; Uart0_Data_Count++; Error_COM = 0; } // '{'
                    else { Uart0ProtocolPointer = 0; Check_Xor =0; } // 아니면 처음부터
                    break;
        case 1:     if(0x5b == Uart0_Data ){ Uart0ProtocolPointer = 2; } // '['
                    else { Uart0ProtocolPointer = 0; Check_Xor =0; }
                    break;
        case 2:     if(0x02 == Uart0_Data )  { Uart0ProtocolPointer = 10;  Error_LEN_Count = 0; } // STX
                    else { Uart0ProtocolPointer = 0; Check_Xor =0; }
                    break;
        case 10:    if(0X00 == Uart0_Data){ Uart0ProtocolPointer = 11; Check_Xor ^= Uart0_Data; Error_LEN = 0; } // 길이 상위
                    else { Uart0ProtocolPointer = 11; Check_Xor ^= Uart0_Data; Error_LEN = 1; }
                    break;
        case 11:    if(0x0D == Uart0_Data){ Uart0ProtocolPointer = 20; Check_Xor ^= Uart0_Data; Error_LEN = 0; Error = Error & 0x00; } // 길이 하위=13
                    else { Uart0ProtocolPointer = 20; Check_Xor ^= Uart0_Data; Error_LEN = 1; Error = Error | 0x08; }
                    break;
        case 20:    if(0x31 == Uart0_Data){ Uart0ProtocolPointer = 21; Check_Xor ^= Uart0_Data; Error_LEN_Count++; } // 명령 코드 1
                    else { Uart0ProtocolPointer = 21; Check_Xor ^= Uart0_Data; Error_BCC = 1; }
                    break;
        case 21:    if(0x51 == Uart0_Data){ Uart0ProtocolPointer = 30; Check_Xor ^= Uart0_Data; Error_LEN_Count++; } // 명령 코드 2
                    else { Uart0ProtocolPointer = 30; Check_Xor ^= Uart0_Data; Error_BCC = 1; }
                    break;
        case 30:    Uart0ProtocolPointer = 31; X1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // X L
        case 31:    Uart0ProtocolPointer = 32; X1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // X H
        case 32:    Uart0ProtocolPointer = 33; Y1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // Y L
        case 33:    Uart0ProtocolPointer = 34; Y1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // Y H
        case 34:    Uart0ProtocolPointer = 35; Z1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // Z L
        case 35:    Uart0ProtocolPointer = 36; Z1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // Z H
        case 36:    Uart0ProtocolPointer = 37; F1_L_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // F L
        case 37:    Uart0ProtocolPointer = 38; F1_H_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // F H
        case 38:    Uart0ProtocolPointer = 39; FN_Port_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // 상태 바이트
        case 39:    Uart0ProtocolPointer = 90; Error1_Rx = Uart0_Data; Check_Xor ^= Uart0_Data; Error_LEN_Count++; break; // 에러 바이트
        case 90:    if(Check_Xor == Uart0_Data || 0xBC == Uart0_Data) { // 체크섬 맞으면 OK
                         Check_Xor_Check = Check_Xor; command0 = 1; Uart0ProtocolPointer = 100;
                         Check_Xor =0; Error_BCC = 0; Error_LEN_Count++; Error = Error & 0x00;
                    } else {                                              // 틀리면 체크섬 에러
                         Check_Xor_Check = Check_Xor; command0 = 1; Uart0ProtocolPointer = 100;
                         Check_Xor =0; Error_BCC = 1; Error_LEN_Count++; Error = Error | 0x04;
                    }
                    break;
        case 100:   if(0x03 == Uart0_Data )  { Uart0ProtocolPointer = 101; } else { Uart0ProtocolPointer = 0; Check_Xor =0; } break; // ETX
        case 101:   if(0x5d == Uart0_Data )  { Uart0ProtocolPointer = 102; } else { Uart0ProtocolPointer = 0; Check_Xor =0; } break; // ']'
        case 102:   if(0x7d == Uart0_Data )  { Uart0ProtocolPointer = 0; Uart0ReciveCheckEnd = 1; Check_Xor =0; Error_COM = 0; } // '}'
                    else { Uart0ProtocolPointer = 0; }
                    break;
        default:    Uart0ProtocolPointer = 0; Check_Xor =0; break; // 예상 밖엔 처음부터
      }
  }
}

// 수신 패킷을 다 받았으면 실제 값들로 바꾸고 동작 실행 + 회신 보내기
void Serial_Main0(void)
{
  if(Uart0ReciveCheckEnd) {               // 패킷 하나를 끝까지 받았으면
    if(command0 == 1) {                    // 처리할 게 있으면
        X1_L = X1_H_Rx; X1_H = X1_L_Rx;   // 바이트 순서 뒤집기(상대 포맷 맞추기)
        Y1_L = Y1_H_Rx; Y1_H = Y1_L_Rx;
        Z1_L = Z1_H_Rx; Z1_H = Z1_L_Rx;
        F1_L = F1_H_Rx; F1_H = F1_L_Rx;

        X1 = ((uint16_t)X1_H << 8) | (uint8_t)X1_L; // 16비트 값으로 합치기(X)
        Y1 = ((uint16_t)Y1_H << 8) | (uint8_t)Y1_L; // Y
        Z1 = ((uint16_t)Z1_H << 8) | (uint8_t)Z1_L; // Z
        F1 = ((uint16_t)F1_H << 8) | (uint8_t)F1_L; // F(플런저 N 변화량)

        tgtX = (long)X1 * DISPLAY_UNIT;   // 목표 X를 스텝으로 바꾸기
        tgtY = (long)Y1 * DISPLAY_UNIT;   // 목표 Y를 스텝으로 바꾸기
        tgtZ = (long)Z1 * DISPLAY_UNIT;   // 목표 Z를 스텝으로 바꾸기

        dropSteps = curUD + (long)F1 * DROP_UNIT_STEPS; // 플런저: "지금에서 F1칸" → 절대 목표로 변환
        clampDrop();                                 // 안전 범위로

        clampXY(); clampZ(); clampDrop();            // 목표들 안전 범위 보정
        updateOLED();                                // 화면에 보이기

        if (!isMoving) { goToTargetXYZ(); }          // 움직이는 중이 아니면 바로 실행
        requestSave();                                // 저장 예약
        trySaveIfIdle();                              // 가능하면 지금 저장

        // 상태 바이트의 각 비트(필요하면 사용)
        FN_Port0_Rx = FN_Port_Rx & 0x80;
        FN_Port1_Rx = FN_Port_Rx & 0x40;
        FN_Port2_Rx = FN_Port_Rx & 0x20;
        FN_Port3_Rx = FN_Port_Rx & 0x10;
        FN_Port4_Rx = FN_Port_Rx & 0x08;
        FN_Port5_Rx = FN_Port_Rx & 0x04;
        FN_Port6_Rx = FN_Port_Rx & 0x02;
        FN_Port7_Rx = FN_Port_Rx & 0x01;

        Error_Tx = Error;                            // 현재 에러 상태 복사

        // 회신 패킷 체크섬 만들기
        Check_Xor_TX = 0X00 ^ 0x0E ^ 0X31 ^ 0x52 ^
                       X1_L ^ X1_H ^
                       Y1_L ^ Y1_H ^
                       Z1_L ^ Z1_H ^
                       F1_L ^ F1_H ^
                       START_MODE_STATUE ^ START_MODE_STATUE ^
                       Error_Tx;

        // 실제로 회신 보내기(시작문자 → 데이터 → 끝문자)
        Serial.write(0x7B); Serial.write(0x5B); Serial.write(0x02); // '{' '[' STX
        Serial.write(0X00); Serial.write(0x0E);                      // 길이 상/하(0x000E)
        Serial.write(0x31); Serial.write(0x52);                      // 명령 코드

        Serial.write(X1_L); Serial.write(X1_H);                      // X L/H
        Serial.write(Y1_L); Serial.write(Y1_H);                      // Y L/H
        Serial.write(Z1_L); Serial.write(Z1_H);                      // Z L/H
        Serial.write(F1_L); Serial.write(F1_H);                      // F L/H

        Serial.write(START_MODE_STATUE);                              // 시작 상태
        Serial.write(END_MODE_STATUE);                                // 끝 상태

        Serial.write(Error_Tx);                                       // 에러 비트
        Serial.write(Check_Xor_TX);                                   // 체크섬

        Serial.write(0x03); Serial.write(0x5D); Serial.write(0x7D);   // ETX, ']', '}'

        command0 = 0;                                                // 처리 완료 표시
    }
    Uart0ReciveCheckEnd = 0;                                         // 수신 완료 플래그 해제
  }
}

// ===== 원점(RESET) 버튼만 디바운스 처리 =====
const unsigned long DEBOUNCE_MS = 100;   // 버튼이 튀는(덜컹) 시간을 거르기 위한 100ms
static uint8_t resetStable = HIGH;       // 안정된 버튼 값(HIGH=안눌림)
static uint8_t resetLastReading = HIGH;  // 마지막으로 읽은 원시 버튼 값
static unsigned long resetLastChange = 0;// 마지막으로 값이 바뀐 시각

// 원점 버튼이 "딱 한번 눌렸는지" 확인(에지 감지)
bool resetButtonPressedOnce() {
  uint8_t raw = digitalRead(BTN_RESET00); // 지금 버튼 값 읽기
  unsigned long now = millis();           // 지금 시각

  if (raw != resetLastReading) { resetLastReading = raw; resetLastChange = now; } // 값이 바뀌면 시각 갱신

  if (now - resetLastChange > DEBOUNCE_MS) { // 100ms 이상 값이 같았다면 안정된 값으로 인정
    if (resetStable != raw) {            // 안정값이 바뀌었으면
      resetStable = raw;                 // 안정값 업데이트
      if (resetStable == LOW) return true;  // LOW가 된 그 순간이 "눌림" 에지
    }
  }
  return false;                           // 아니면 안 눌림
}

// 초기화(전원 켰을 때 한 번 실행)
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED 시작(주소 0x3C)
  Serial.begin(115200);                       // 시리얼 통신 속도 115200

  pinMode(UD_PUL, OUTPUT); pinMode(UD_DIR, OUTPUT); // 플런저 핀들을 출력으로
  pinMode(X_PUL,  OUTPUT); pinMode(X_DIR,  OUTPUT); // X(실제 Y) 핀들 출력
  pinMode(Y_PUL,  OUTPUT); pinMode(Y_DIR,  OUTPUT); // Y(실제 Z) 핀들 출력
  pinMode(Z_PUL,  OUTPUT); pinMode(Z_DIR,  OUTPUT); // Z(실제 X) 핀들 출력

  pinMode(BTN_RESET00, INPUT_PULLUP);          // 원점 버튼은 풀업 입력(눌리면 LOW)

  resetStable = resetLastReading = digitalRead(BTN_RESET00); // 버튼 초기 안정값 기록
  resetLastChange = millis();                // 마지막 변화 시각 초기화

  loadPersist();                             // 지난번 저장 상태를 불러오기
  clampXY(); clampZ(); clampDrop(); clampUDPos(); // 안전 범위로 정리

  lastOLEDUnitX = curX / DISPLAY_UNIT;       // OLED 표시 기준값 초기화
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
  lastOLEDBurstsUD = curUD / DROP_UNIT_STEPS; // 플런저 N 표시 기준 초기화

  updateOLED();                              // 처음 화면 그리기
}

// 메인 루프(계속 반복)
void loop() {
  serviceAxisProgress();                     // 인터럽트가 만든 진행량을 실제 위치에 반영

  UartRxProtocol();                          // 시리얼로 들어오는 바이트 해석
  Serial_Main0();                            // 패킷 다 받았으면 처리/응답/동작 실행

  // 원점(RESET) 버튼만 사용 — 누르면 목표 0, 플런저 목표는 N=1(200스텝)
  if (resetButtonPressedOnce()) {            // 버튼이 딱 눌린 순간이면
    tgtX = 0; tgtY = 0; tgtZ = 0;           // X/Y/Z 목표 0
    dropSteps = DROP_UNIT_STEPS;             // 플런저 목표 200(=N 1)
    clampXY(); clampZ(); clampDrop();        // 안전 보정
    updateOLED();                            // 화면 갱신
    if (!isMoving) { goToTargetXYZ(); }      // 바로 실행( X/Y 동시 → Z → 플런저 → Z복귀 )
    savePersistNow();                        // 상태 저장
  }

  trySaveIfIdle();                           // 저장 예약된 것이 있으면 지금 저장
}
