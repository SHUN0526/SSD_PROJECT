#include <Wire.h>                       // I2C(아이투씨) 통신 라이브러리
#include <Adafruit_GFX.h>               // 글자/그림 그리기 라이브러리
#include <Adafruit_SSD1306.h>           // SSD1306 OLED 라이브러리
#include <EEPROM.h>                     // 전원 꺼져도 남는 저장공간
#include <Arduino.h>                    // 아두이노 기본 함수들
#include <avr/io.h>                     // AVR 레지스터 직접 제어
#include <avr/interrupt.h>              // 인터럽트 사용

#define SCREEN_WIDTH 128                // OLED 가로 픽셀
#define SCREEN_HEIGHT 32                // OLED 세로 픽셀
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire); // OLED 객체

// ===== 좌표/속도 기본 =====
const long DISPLAY_UNIT = 95;           // 좌표 1칸 = 모터 95스텝
const int  STEP_HALF_US = 150;          // 스텝 반 주기(us) — 작을수록 빠름

// ----- 플런저(U축) -----
const long DROP_UNIT_STEPS = 200;       // 플런저 1칸(N) = 200스텝
long dropSteps = DROP_UNIT_STEPS;       // 플런저 절대 목표(스텝)
const long MIN_DROP = -10000;           // 플런저 최소 한계
const long MAX_DROP =  10000;           // 플런저 최대 한계

// ===== 모터 방향(배선에 맞춤) =====
const uint8_t X_DIR_POS   = LOW;        // X +방향
const uint8_t X_DIR_NEG   = HIGH;       // X -방향
const uint8_t STAGEY_DIR_POS = HIGH;    // Y +방향
const uint8_t STAGEY_DIR_NEG = LOW;     // Y -방향
const uint8_t LIFT_DIR_UP    = HIGH;    // Z 위(+)
const uint8_t LIFT_DIR_DOWN  = LOW;     // Z 아래(-)
const uint8_t UD_DIR_DOWN = HIGH;       // 플런저 누름(+)
const uint8_t UD_DIR_UP   = LOW;        // 플런저 올림(-)

// ===== 드라이버 핀 연결(보드 핀 번호) =====
const uint8_t UD_PUL = 8,  UD_DIR = 9;  // 플런저
const uint8_t X_PUL  = 10, X_DIR  = 11; // 보드 X → 실제 Y
const uint8_t Y_PUL  = 12, Y_DIR  = 13; // 보드 Y → 실제 Z
const uint8_t Z_PUL  = 14, Z_DIR  = 15; // 보드 Z → 실제 X

// ===== 버튼(원점) =====
const uint8_t BTN_RESET00 = 25;         // 원점(0,0,0) 버튼

// ===== 현재/목표 위치(스텝) =====
long curX=0, curY=0, curZ=0;            // 현재 위치
long tgtX=0, tgtY=0, tgtZ=0;            // 목표 위치
long curUD=0, tgtUD=0;                   // 플런저 현재/목표

// ===== 이동 한계(안전 범위) =====
const long MIN_X = 0;
const long MAX_X = 170 * DISPLAY_UNIT;
const long MIN_Y = -220 * DISPLAY_UNIT;
const long MAX_Y = 0;
const long MIN_Z = -140 * DISPLAY_UNIT;
const long MAX_Z = 0;

// Z 전용 속도(균일)
const int  STEP_HALF_US_Z_MIN = 220;

// ===== 목표값을 안전 범위로 고정(클램프) =====
inline void clampXY(){ if(tgtX<MIN_X)tgtX=MIN_X; else if(tgtX>MAX_X)tgtX=MAX_X;
                       if(tgtY<MIN_Y)tgtY=MIN_Y; else if(tgtY>MAX_Y)tgtY=MAX_Y; }
inline void clampZ(){  if(tgtZ<MIN_Z)tgtZ=MIN_Z; else if(tgtZ>MAX_Z)tgtZ=MAX_Z; }
inline void clampDrop(){ if(dropSteps<MIN_DROP)dropSteps=MIN_DROP; else if(dropSteps>MAX_DROP)dropSteps=MAX_DROP; }
inline void clampUDPos(){ if(curUD<MIN_DROP)curUD=MIN_DROP; else if(curUD>MAX_DROP)curUD=MAX_DROP; }

// ===== EEPROM(저장 형식: V5만 사용) =====
struct PersistV5{ uint32_t magic; long curX; long curY; long curZ; long curUD; long dropSteps; }; // 저장 묶음
const uint32_t MAGIC_V5 = 0xC0FFEE7A;   // "이건 V5다" 표시값
const int EEPROM_ADDR = 0;               // EEPROM 시작 주소

// ===== 저장 관련 =====
bool needSave=false; unsigned long lastEditMs=0;
const unsigned long SAVE_DELAY_MS=100;   // 바뀐 뒤 0.1초 지나면 저장
const unsigned long POS_SAVE_INTERVAL_MS=100; // 이동 중 저장 체크 주기
const long POS_SAVE_DELTA_STEPS=200;     // 200스텝 이상 움직였을 때만 저장
unsigned long lastPosSaveMs=0;
long lastSavedX=0,lastSavedY=0,lastSavedZ=0,lastSavedUD=0;
long lastOLEDUnitX=0,lastOLEDUnitY=0,lastOLEDUnitZ=0,lastOLEDBurstsUD=0;

// ===== 플런저 스텝(간단, 블로킹) =====
static inline void stepHalf(uint8_t PUL,int half_us){
  digitalWrite(PUL,HIGH); delayMicroseconds(half_us); // HIGH 유지
  digitalWrite(PUL,LOW ); delayMicroseconds(half_us); // LOW  유지 → 1스텝
}

// ─────────────────────────────────────────────
// 인터럽트 기반 스텝 드라이버(자동 펄스)
// T1: Y(보드X) / T3: X(보드Z) / T2: Z(보드Y)
// ─────────────────────────────────────────────
volatile bool isMoving=false;            // 지금 움직이는 중?

// T1(Y)에서 쓰는 값들
volatile uint8_t* T1_PUL_PORT; volatile uint8_t  T1_PUL_MASK;
volatile long T1_steps_remaining=0, T1_steps_done=0;
volatile int8_t T1_step_sign=+1; volatile bool T1_pulse_state=false;

// T3(X)에서 쓰는 값들
volatile uint8_t* T3_PUL_PORT; volatile uint8_t  T3_PUL_MASK;
volatile long T3_steps_remaining=0, T3_steps_done=0;
volatile int8_t T3_step_sign=+1; volatile bool T3_pulse_state=false;

// T2(Z)에서 쓰는 값들
volatile uint8_t* T2_PUL_PORT; volatile uint8_t  T2_PUL_MASK;
volatile long T2_steps_remaining=0, T2_steps_done=0;
volatile int8_t T2_step_sign=+1; volatile bool T2_pulse_state=false;

// ===== “이동 중 수신 패킷 무효화” 강제 장치 =====
const unsigned long RX_GUARD_MS = 300;   // 이동 전후로 이 시간 동안은 명령 무조건 거부
volatile unsigned long rejectUntilMs=0;  // 이 시각 전까지는 수신 전부 거부
inline bool inRejectWindow(){            // 지금 거부 창 안인가?
  return (long)(millis() - rejectUntilMs) < 0;
}

// ===== 수신 파서에서 쓸 상태 플래그(패킷 단위 폐기용) =====
bool rxInPacket = false;                 // 패킷이 시작됐는지
bool rxRejectThisPacket = false;         // 이 패킷을 끝까지 버릴지

// ===== 각 타이머 인터럽트(펄스 토글) =====
ISR(TIMER1_COMPA_vect){
  if(T1_steps_remaining>0){
    if(T1_pulse_state){ *T1_PUL_PORT |=  T1_PUL_MASK; }      // HIGH
    else               { *T1_PUL_PORT &= ~T1_PUL_MASK; T1_steps_remaining--; T1_steps_done++; } // LOW + 1스텝
    T1_pulse_state=!T1_pulse_state;                           // 다음엔 반대로
  }else{
    TIMSK &= ~(1<<OCIE1A); *T1_PUL_PORT &= ~T1_PUL_MASK;      // 끝: 인터럽트 끄고 LOW로
  }
}
ISR(TIMER3_COMPA_vect){
  if(T3_steps_remaining>0){
    if(T3_pulse_state){ *T3_PUL_PORT |=  T3_PUL_MASK; }
    else               { *T3_PUL_PORT &= ~T3_PUL_MASK; T3_steps_remaining--; T3_steps_done++; }
    T3_pulse_state=!T3_pulse_state;
  }else{
    ETIMSK &= ~(1<<OCIE3A); *T3_PUL_PORT &= ~T3_PUL_MASK;
  }
}
ISR(TIMER2_COMP_vect){
  if(T2_steps_remaining>0){
    if(T2_pulse_state){ *T2_PUL_PORT |=  T2_PUL_MASK; }
    else               { *T2_PUL_PORT &= ~T2_PUL_MASK; T2_steps_remaining--; T2_steps_done++; }
    T2_pulse_state=!T2_pulse_state;
  }else{
    TIMSK &= ~(1<<OCIE2); *T2_PUL_PORT &= ~T2_PUL_MASK;
  }
}

// ===== 타이머로 축 출발(속도 = half_us) =====
static inline void startTimerMotor(uint8_t timer_id,uint8_t pulPin,uint8_t dirPin,uint8_t dirLevel,long steps,int half_us,int8_t step_sign){
  if(steps<=0) return;                    // 갈 게 없으면 끝
  pinMode(pulPin,OUTPUT); pinMode(dirPin,OUTPUT); // 핀을 출력으로
  digitalWrite(dirPin,dirLevel); delayMicroseconds(100); // 방향 세팅 후 잠깐 대기
  noInterrupts();                          // 설정 중엔 잠시 인터럽트 OFF

  if(timer_id==1){                        // 타이머1(Y)
    uint32_t ocr=(uint32_t)(((uint64_t)F_CPU/8ULL)*(uint64_t)half_us/1000000ULL);
    if(ocr==0) ocr=1; ocr-=1;             // 최소 1
    T1_PUL_PORT=portOutputRegister(digitalPinToPort(pulPin));
    T1_PUL_MASK=digitalPinToBitMask(pulPin);
    T1_steps_remaining=steps; T1_steps_done=0; T1_step_sign=step_sign; T1_pulse_state=false;
    TCCR1A=0; TCCR1B=0; TCNT1=0;
    TCCR1B|=(1<<WGM12); OCR1A=(uint16_t)ocr; TCCR1B|=(1<<CS11);
    TIMSK|=(1<<OCIE1A);
  }else if(timer_id==3){                  // 타이머3(X)
    uint32_t ocr=(uint32_t)(((uint64_t)F_CPU/8ULL)*(uint64_t)half_us/1000000ULL);
    if(ocr==0) ocr=1; ocr-=1;
    T3_PUL_PORT=portOutputRegister(digitalPinToPort(pulPin));
    T3_PUL_MASK=digitalPinToBitMask(pulPin);
    T3_steps_remaining=steps; T3_steps_done=0; T3_step_sign=step_sign; T3_pulse_state=false;
    TCCR3A=0; TCCR3B=0; TCNT3=0;
    TCCR3B|=(1<<WGM32); OCR3A=(uint16_t)ocr; TCCR3B|=(1<<CS31);
    ETIMSK|=(1<<OCIE3A);
  }else if(timer_id==2){                  // 타이머2(Z)
    uint32_t ticks=((uint64_t)F_CPU*(uint64_t)half_us)/1000000ULL;
    struct Opt{ uint16_t presc; uint8_t cs; } opts[]={
      {8,(1<<CS21)},{32,(1<<CS21)|(1<<CS20)},{64,(1<<CS22)},
      {128,(1<<CS22)|(1<<CS20)},{256,(1<<CS22)|(1<<CS21)},{1024,(1<<CS22)|(1<<CS21)|(1<<CS20)}
    };
    uint8_t csbits=0, ocr8=255;
    for(uint8_t i=0;i<sizeof(opts)/sizeof(opts[0]);i++){
      uint32_t cnt=ticks/opts[i].presc;
      if(cnt>=2 && cnt<=256){ ocr8=(uint8_t)(cnt-1); csbits=opts[i].cs; break; }
    }
    T2_PUL_PORT=portOutputRegister(digitalPinToPort(pulPin));
    T2_PUL_MASK=digitalPinToBitMask(pulPin);
    T2_steps_remaining=steps; T2_steps_done=0; T2_step_sign=step_sign; T2_pulse_state=false;
    TCCR2=0; TCNT2=0; TCCR2|=(1<<WGM21); OCR2=ocr8; TCCR2|=csbits; TIMSK|=(1<<OCIE2);
  }
  interrupts();                            // 설정 끝
}

// ===== 이동 중 위치를 가끔 저장(EEPROM) =====
inline void maybeSaveCurPosDuringMove(){
  unsigned long now=millis();
  if(now-lastPosSaveMs>=POS_SAVE_INTERVAL_MS){
    long dx=curX-lastSavedX,dy=curY-lastSavedY,dz=curZ-lastSavedZ,du=curUD-lastSavedUD;
    if( (dx>=POS_SAVE_DELTA_STEPS||dx<=-POS_SAVE_DELTA_STEPS) ||
        (dy>=POS_SAVE_DELTA_STEPS||dy<=-POS_SAVE_DELTA_STEPS) ||
        (dz>=POS_SAVE_DELTA_STEPS||dz<=-POS_SAVE_DELTA_STEPS) ||
        (du>=POS_SAVE_DELTA_STEPS||du<=-POS_SAVE_DELTA_STEPS) ){
      PersistV5 p{MAGIC_V5,curX,curY,curZ,curUD,dropSteps}; EEPROM.put(EEPROM_ADDR,p);
      lastPosSaveMs=now; lastSavedX=curX; lastSavedY=curY; lastSavedZ=curZ; lastSavedUD=curUD;
    }
  }
}

// ===== OLED 출력 유틸(오른쪽 정렬 숫자) =====
static void printRightAligned(long value,int rightX,int y,uint8_t size,bool inv=false){
  char buf[16]; int len=snprintf(buf,sizeof(buf),"%ld",value); if(len<0)len=0;
  int w=len*6*size; int x=rightX-w+1; if(x<0)x=0;
  if(inv) display.setTextColor(BLACK,WHITE); else display.setTextColor(WHITE);
  display.setTextSize(size); display.setCursor(x,y); display.print(buf); display.setTextColor(WHITE);
}
static void badgeFilled(int x,int y,const __FlashStringHelper* label){
  display.fillRoundRect(x,y,18,10,2,WHITE);
  display.fillRoundRect(x+1,y+1,16,8,2,BLACK);
  display.setTextSize(1); display.setTextColor(WHITE); display.setCursor(x+5,y+2); display.print(label);
}
void updateOLED(){
  long x=curX/DISPLAY_UNIT, y=curY/DISPLAY_UNIT, z=curZ/DISPLAY_UNIT, n=curUD/DROP_UNIT_STEPS;
  display.clearDisplay();
  auto card=[&](int x0,int y0,const __FlashStringHelper* lab,long v){
    display.fillRoundRect(x0,y0,64,16,3,WHITE); display.drawRoundRect(x0,y0,64,16,3,BLACK);
    badgeFilled(x0+2,y0+2,lab); printRightAligned(v,x0+63,y0,2,true);
  };
  card(  0,  0, F("X"), x);
  card( 64,  0, F("Y"), y);
  card(  0, 16, F("Z"), z);
  card( 64, 16, F("D"), n);
  display.display();
}

// ===== 즉시 저장 =====
void savePersistNow(){
  PersistV5 p{MAGIC_V5,curX,curY,curZ,curUD,dropSteps}; EEPROM.put(EEPROM_ADDR,p);
  lastSavedX=curX; lastSavedY=curY; lastSavedZ=curZ; lastSavedUD=curUD; lastPosSaveMs=millis();
  lastOLEDUnitX=curX/DISPLAY_UNIT;
}

// ===== 이동 중 최소한의 OLED 갱신 =====
inline void maybeUpdateOLEDWhileMoving(char axis){
  long ux=curX/DISPLAY_UNIT, uy=curY/DISPLAY_UNIT, uz=curZ/DISPLAY_UNIT; bool changed=false;
  if(axis=='X' && ux!=lastOLEDUnitX){ lastOLEDUnitX=ux; changed=true; }
  if(axis=='Y' && uy!=lastOLEDUnitY){ lastOLEDUnitY=uy; changed=true; }
  if(axis=='Z' && uz!=lastOLEDUnitZ){ lastOLEDUnitZ=uz; changed=true; }
  if(axis=='U'){
    long ub=curUD/DROP_UNIT_STEPS;
    if(ub!=lastOLEDBurstsUD){ lastOLEDBurstsUD=ub; changed=true; savePersistNow(); }
  }
  if(changed) updateOLED();
}

// ===== ISR가 쌓은 스텝을 실제 위치에 반영 =====
inline void serviceAxisProgress(){
  long d1=0,d3=0,d2=0; noInterrupts();
  if(T1_steps_done){ d1=T1_steps_done; T1_steps_done=0; }
  if(T3_steps_done){ d3=T3_steps_done; T3_steps_done=0; }
  if(T2_steps_done){ d2=T2_steps_done; T2_steps_done=0; }
  interrupts();
  if(d3){ curX += (long)T3_step_sign*d3; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('X'); }
  if(d1){ curY += (long)T1_step_sign*d1; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('Y'); }
  if(d2){ curZ += (long)T2_step_sign*d2; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving('Z'); }
}

// ===== 블로킹 이동(플런저 등) =====
void moveAxisSteps(uint8_t PUL,uint8_t DIR,uint8_t dirLevel,long steps,int half_us,long &curAxis,int stepSign,char axisTag){
  if(steps<=0) return;
  digitalWrite(DIR,dirLevel); delayMicroseconds(100);
  for(long i=0;i<steps;i++){ stepHalf(PUL,half_us); curAxis+=stepSign; maybeSaveCurPosDuringMove(); maybeUpdateOLEDWhileMoving(axisTag); }
}

// ===== 저장 예약/실행 =====
void requestSave(){ needSave=true; lastEditMs=millis(); }
void trySaveIfIdle(){ if(needSave && (millis()-lastEditMs>=SAVE_DELAY_MS)){ savePersistNow(); needSave=false; } }

// ===== 저장 로드(V5만) =====
void loadPersist(){
  uint32_t magic=0; EEPROM.get(EEPROM_ADDR,magic);
  if(magic==MAGIC_V5){ PersistV5 p; EEPROM.get(EEPROM_ADDR,p); curX=p.curX; curY=p.curY; curZ=p.curZ; curUD=p.curUD; dropSteps=p.dropSteps; }
  else{ curX=curY=curZ=0; curUD=0; dropSteps=DROP_UNIT_STEPS; }
  clampUDPos(); tgtX=curX; tgtY=curY; tgtZ=curZ; clampXY(); clampZ(); clampDrop();
  lastOLEDUnitX=curX/DISPLAY_UNIT; lastOLEDUnitY=curY/DISPLAY_UNIT; lastOLEDUnitZ=curZ/DISPLAY_UNIT;
  lastSavedUD=curUD; lastOLEDBurstsUD=curUD/DROP_UNIT_STEPS;
}

// ===== 플런저: 절대 목표까지 이동 =====
void doOneDrop(){
  tgtUD=dropSteps; clampDrop(); clampUDPos();
  long d=tgtUD-curUD; if(d==0){ savePersistNow(); return; }
  if(d>0) moveAxisSteps(UD_PUL,UD_DIR,UD_DIR_DOWN, d,STEP_HALF_US,curUD,+1,'U');
  else    moveAxisSteps(UD_PUL,UD_DIR,UD_DIR_UP,  -d,STEP_HALF_US,curUD,-1,'U');
  savePersistNow();
}

// ===== ISR 방식 이동 시작/대기 =====
void startAxisSteps_ISR(uint8_t PUL,uint8_t DIR,uint8_t dirLevel,long steps,int half_us,int8_t stepSign,char axisTag){
  if(steps<=0) return; uint8_t tid=0;
  if(axisTag=='X') tid=3; else if(axisTag=='Y') tid=1; else if(axisTag=='Z') tid=2; else return;
  startTimerMotor(tid,PUL,DIR,dirLevel,steps,half_us,stepSign);
}
void moveAxisSteps_ISR(uint8_t PUL,uint8_t DIR,uint8_t dirLevel,long steps,int half_us,long &curAxis,int stepSign,char axisTag){
  if(steps<=0) return; uint8_t tid=0;
  if(axisTag=='X') tid=3; else if(axisTag=='Y') tid=1; else if(axisTag=='Z') tid=2;
  else{ moveAxisSteps(PUL,DIR,dirLevel,steps,half_us,curAxis,stepSign,axisTag); return; }
  startTimerMotor(tid,PUL,DIR,dirLevel,steps,half_us,(int8_t)stepSign);
  for(;;){
    serviceAxisProgress(); trySaveIfIdle();
    bool done=(tid==3)?(T3_steps_remaining==0):(tid==1)?(T1_steps_remaining==0):(T2_steps_remaining==0);
    if(done) break;
  }
  serviceAxisProgress();
}

// ===== 목표로 이동(X/Y 동시 → Z → 플런저 → Z복귀) =====
void goToTargetXYZ(){
  if(isMoving) return;                    // 이미 움직이면 무시
  isMoving=true;                          // 이제부터 바쁨
  rejectUntilMs = millis() + RX_GUARD_MS; // ★이 시점부터 가드 시간 시작

  clampXY(); clampZ();
  long dx=tgtX-curX, dy=tgtY-curY, dz=tgtZ-curZ;
  int8_t sx=(dx>=0)?+1:-1, sy=(dy>=0)?+1:-1;
  long adx=(dx>=0)?dx:-dx, ady=(dy>=0)?dy:-dy;

  int half_x=STEP_HALF_US, half_y=STEP_HALF_US; long maxS=(adx>ady)?adx:ady;
  if(maxS>0){ if(adx>0) half_x=(int)((long)STEP_HALF_US*maxS/adx);
              if(ady>0) half_y=(int)((long)STEP_HALF_US*maxS/ady);
              if(half_x<1) half_x=1; if(half_y<1) half_y=1; }

  bool sxon=false, syon=false;
  if(adx>0){ startAxisSteps_ISR(Z_PUL,Z_DIR,(sx>0?X_DIR_POS:X_DIR_NEG), adx,half_x,sx,'X'); sxon=true; }
  if(ady>0){ startAxisSteps_ISR(X_PUL,X_DIR,(sy>0?STAGEY_DIR_POS:STAGEY_DIR_NEG), ady,half_y,sy,'Y'); syon=true; }

  while( (sxon && T3_steps_remaining>0) || (syon && T1_steps_remaining>0) ){
    serviceAxisProgress(); trySaveIfIdle();
  }
  serviceAxisProgress();

  if(dz>0)      moveAxisSteps_ISR(Y_PUL,Y_DIR,LIFT_DIR_UP,  dz,STEP_HALF_US_Z_MIN,curZ,+1,'Z');
  else if(dz<0) moveAxisSteps_ISR(Y_PUL,Y_DIR,LIFT_DIR_DOWN,-dz,STEP_HALF_US_Z_MIN,curZ,-1,'Z');

  doOneDrop();

  if(curZ!=MAX_Z){
    long d0=MAX_Z-curZ;
    if(d0>0)      moveAxisSteps_ISR(Y_PUL,Y_DIR,LIFT_DIR_UP,  d0,STEP_HALF_US_Z_MIN,curZ,+1,'Z');
    else if(d0<0) moveAxisSteps_ISR(Y_PUL,Y_DIR,LIFT_DIR_DOWN,-d0,STEP_HALF_US_Z_MIN,curZ,-1,'Z');
  }

  savePersistNow();
  isMoving=false;                          // 이제 한가
  rejectUntilMs = millis() + RX_GUARD_MS;  // ★끝난 직후에도 가드 시간 유지(잔상 패킷 차단)

  // 가드창 동안 들어온 찌꺼기 즉시 비우기(여기서 한 번 더 안전)
  while (inRejectWindow()) { while(Serial.available()) (void)Serial.read(); }
}

/* ===== 시리얼 프로토콜 =====
   규칙: “패킷 시작 시점”에 바쁘면 그 패킷은 끝날 때까지 rxRejectThisPacket=true(완전 폐기)
*/

// ---- 파서에서 쓰는 전역(원 코드와 동일) ----
unsigned char command0=0;
unsigned char Uart0ProtocolPointer=0;
unsigned char Uart0ReciveCheckEnd=0;

unsigned char Uart0_Data_Count=0;
unsigned char Check_Xor=0,Check_Xor_Check=0,Check_Xor_TX=0;

unsigned char Error_Tx=0,Error=0,Error_COM=0,Error_BCC=0,Error_LEN=0,Error_LEN_Count=0;

int X1=0,Y1=0,Z1=0,F1=0;
char X1_L_Rx=0,Y1_L_Rx=0,Z1_L_Rx=0,F1_L_Rx=0;
char X1_L=0,   Y1_L=0,   Z1_L=0,   F1_L=0;
char X1_H_Rx=0,Y1_H_Rx=0,Z1_H_Rx=0,F1_H_Rx=0;
char X1_H=0,   Y1_H=0,   Z1_H=0,   F1_H=0;

char FN_Port_Rx=0, Error1_Rx=0;
char FN_Port0_Rx=0,FN_Port1_Rx=0,FN_Port2_Rx=0,FN_Port3_Rx=0;
char FN_Port4_Rx=0,FN_Port5_Rx=0,FN_Port6_Rx=0,FN_Port7_Rx=0;

char START_MODE_STATUE=0, END_MODE_STATUE=0;

// 수신 파서(여러 바이트를 한 번에 처리)
void UartRxProtocol(){
  // 가드창/이동 중이면 즉시 버퍼 비우고, 파서 초기화(부분 패킷도 찢어버림)
  if (isMoving || inRejectWindow()){
    while(Serial.available()) (void)Serial.read(); // 싹 비우기
    Uart0ProtocolPointer=0; Check_Xor=0;           // 파서 리셋
    rxInPacket=false; rxRejectThisPacket=false;    // 패킷 상태 리셋
    return;                                        // 더 안 봄
  }

  while(Serial.available()){               // 남아 있는 동안 반복
    char Uart0_Data=Serial.read();         // 한 바이트 읽기

    switch(Uart0ProtocolPointer){
      case 0:                              // 패킷 시작 대기
        if(0x7b==Uart0_Data){              // '{' 받음 → 패킷 시작
          rxInPacket = true;               // 패킷 시작 표시
          // ★이 순간 바쁘면 이 패킷은 끝까지 폐기 표시
          rxRejectThisPacket = (isMoving || inRejectWindow());
          Uart0ProtocolPointer = 1; Uart0_Data_Count++; Error_COM = 0;
        }else{                             // 아니면 리셋
          Uart0ProtocolPointer = 0; Check_Xor = 0; rxInPacket=false; rxRejectThisPacket=false;
        }
        break;

      case 1: if(0x5b==Uart0_Data){ Uart0ProtocolPointer=2; }
              else { Uart0ProtocolPointer=0; Check_Xor=0; rxInPacket=false; rxRejectThisPacket=false; }
              break;

      case 2: if(0x02==Uart0_Data){ Uart0ProtocolPointer=10; Error_LEN_Count=0; }
              else { Uart0ProtocolPointer=0; Check_Xor=0; rxInPacket=false; rxRejectThisPacket=false; }
              break;

      case 10: if(0x00==Uart0_Data){ Uart0ProtocolPointer=11; Check_Xor^=Uart0_Data; Error_LEN=0; }
               else { Uart0ProtocolPointer=11; Check_Xor^=Uart0_Data; Error_LEN=1; }
               break;

      case 11: if(0x0D==Uart0_Data){ Uart0ProtocolPointer=20; Check_Xor^=Uart0_Data; Error_LEN=0; Error&=0x00; }
               else { Uart0ProtocolPointer=20; Check_Xor^=Uart0_Data; Error_LEN=1; Error|=0x08; }
               break;

      case 20: if(0x31==Uart0_Data){ Uart0ProtocolPointer=21; Check_Xor^=Uart0_Data; Error_LEN_Count++; }
               else { Uart0ProtocolPointer=21; Check_Xor^=Uart0_Data; Error_BCC=1; }
               break;

      case 21: if(0x51==Uart0_Data){ Uart0ProtocolPointer=30; Check_Xor^=Uart0_Data; Error_LEN_Count++; }
               else { Uart0ProtocolPointer=30; Check_Xor^=Uart0_Data; Error_BCC=1; }
               break;

      case 30: Uart0ProtocolPointer=31; X1_L_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 31: Uart0ProtocolPointer=32; X1_H_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 32: Uart0ProtocolPointer=33; Y1_L_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 33: Uart0ProtocolPointer=34; Y1_H_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 34: Uart0ProtocolPointer=35; Z1_L_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 35: Uart0ProtocolPointer=36; Z1_H_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 36: Uart0ProtocolPointer=37; F1_L_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 37: Uart0ProtocolPointer=38; F1_H_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 38: Uart0ProtocolPointer=39; FN_Port_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;
      case 39: Uart0ProtocolPointer=90; Error1_Rx=Uart0_Data; Check_Xor^=Uart0_Data; Error_LEN_Count++; break;

      case 90:
        // 체크섬 맞든 틀리든, 여기서는 "처리 예정" 마킹만 함
        Check_Xor_Check = Check_Xor;      // 참조용 저장
        if(!rxRejectThisPacket){          // ★버릴 패킷이 아니면
          command0 = 1;                   // 처리할 게 있다 표시
        }else{
          command0 = 0;                   // ★버릴 패킷이면 처리 금지
        }
        Uart0ProtocolPointer = 100; Check_Xor = 0; // 다음 단계
        break;

      case 100: if(0x03==Uart0_Data){ Uart0ProtocolPointer=101; } else { Uart0ProtocolPointer=0; Check_Xor=0; }
                break;
      case 101: if(0x5d==Uart0_Data){ Uart0ProtocolPointer=102; } else { Uart0ProtocolPointer=0; Check_Xor=0; }
                break;

      case 102: // '}' 도착 = 패킷 종료
        // ★패킷 전체를 버리기로 했으면 여기서 깔끔히 드랍
        if(rxRejectThisPacket){
          // 아무 플래그도 세우지 않고 리셋
          Uart0ProtocolPointer=0; Uart0ReciveCheckEnd=0; command0=0;
          rxInPacket=false; rxRejectThisPacket=false;    // 패킷 상태 해제
          // 끝.
          break;
        }
        // 정상 패킷이면 “받기 완료” 플래그 켜기
        Uart0ProtocolPointer=0; Uart0ReciveCheckEnd=1; Error_COM=0;
        rxInPacket=false; rxRejectThisPacket=false;      // 상태 해제
        break;

      default: Uart0ProtocolPointer=0; Check_Xor=0; rxInPacket=false; rxRejectThisPacket=false; break;
    }
  }
}

// 수신 패킷을 다 받았으면 실제 값들로 바꾸고 동작 실행 + 회신 보내기
void Serial_Main0(void){
  if(Uart0ReciveCheckEnd){               // 패킷 하나를 끝까지 받았으면
    // 혹시나: 가드창/이동 중이면 여기서도 처리 금지
    if (isMoving || inRejectWindow()){
      // 처리 플래그 리셋하고 그냥 버림
      command0=0; Uart0ReciveCheckEnd=0;
      return;
    }

    if(command0 == 1){                   // 처리할 게 있으면(버리는 패킷이면 0으로 되어 있음)
      X1_L = X1_H_Rx; X1_H = X1_L_Rx;   // 바이트 순서 뒤집기
      Y1_L = Y1_H_Rx; Y1_H = Y1_L_Rx;
      Z1_L = Z1_H_Rx; Z1_H = Z1_L_Rx;
      F1_L = F1_H_Rx; F1_H = F1_L_Rx;

      X1 = ((uint16_t)X1_H << 8) | (uint8_t)X1_L;
      Y1 = ((uint16_t)Y1_H << 8) | (uint8_t)Y1_L;
      Z1 = ((uint16_t)Z1_H << 8) | (uint8_t)Z1_L;
      F1 = ((uint16_t)F1_H << 8) | (uint8_t)F1_L;

      tgtX = (long)X1 * DISPLAY_UNIT;
      tgtY = (long)Y1 * DISPLAY_UNIT;
      tgtZ = (long)Z1 * DISPLAY_UNIT;

      dropSteps = curUD + (long)F1 * DROP_UNIT_STEPS; // 플런저: 상대 → 절대
      clampDrop();

      clampXY(); clampZ(); clampDrop();
      updateOLED();

      if (!isMoving) { goToTargetXYZ(); } // 지금 한가하면 바로 실행
      requestSave();
      trySaveIfIdle();

      // 상태 비트 (필요시 사용)
      FN_Port0_Rx = FN_Port_Rx & 0x80;
      FN_Port1_Rx = FN_Port_Rx & 0x40;
      FN_Port2_Rx = FN_Port_Rx & 0x20;
      FN_Port3_Rx = FN_Port_Rx & 0x10;
      FN_Port4_Rx = FN_Port_Rx & 0x08;
      FN_Port5_Rx = FN_Port_Rx & 0x04;
      FN_Port6_Rx = FN_Port_Rx & 0x02;
      FN_Port7_Rx = FN_Port_Rx & 0x01;

      Error_Tx = Error;

      // 회신 체크섬
      Check_Xor_TX = 0X00 ^ 0x0E ^ 0X31 ^ 0x52 ^
                     X1_L ^ X1_H ^ Y1_L ^ Y1_H ^ Z1_L ^ Z1_H ^
                     F1_L ^ F1_H ^ START_MODE_STATUE ^ START_MODE_STATUE ^ Error_Tx;

      // 회신 전송
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

      command0 = 0;                       // 처리 완료
    }
    Uart0ReciveCheckEnd = 0;              // 수신 완료 플래그 해제
  }
}

// ===== 원점(RESET) 버튼만 디바운스 처리 =====
const unsigned long DEBOUNCE_MS = 100;   // 버튼 덜컹 방지 100ms
static uint8_t resetStable = HIGH;       // 안정된 값(HIGH=안눌림)
static uint8_t resetLastReading = HIGH;  // 마지막 원시값
static unsigned long resetLastChange = 0;// 마지막 변화 시각

// 원점 버튼이 "딱 한번 눌렸는지" 확인
bool resetButtonPressedOnce() {
  uint8_t raw = digitalRead(BTN_RESET00); // 지금 값
  unsigned long now = millis();           // 지금 시각
  if (raw != resetLastReading) { resetLastReading = raw; resetLastChange = now; } // 변화 기록
  if (now - resetLastChange > DEBOUNCE_MS) { // 100ms 이상 그대로면 안정
    if (resetStable != raw) {            // 안정값이 바뀌었으면
      resetStable = raw;                 // 업데이트
      if (resetStable == LOW) return true;  // LOW 된 그 순간이 눌림
    }
  }
  return false;                           // 아니면 안 눌림
}

// ===== 초기화(전원 켰을 때 한 번) =====
void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED 시작
  Serial.begin(115200);                       // 시리얼 속도

  pinMode(UD_PUL, OUTPUT); pinMode(UD_DIR, OUTPUT); // 플런저
  pinMode(X_PUL,  OUTPUT); pinMode(X_DIR,  OUTPUT); // X(실제 Y)
  pinMode(Y_PUL,  OUTPUT); pinMode(Y_DIR,  OUTPUT); // Y(실제 Z)
  pinMode(Z_PUL,  OUTPUT); pinMode(Z_DIR,  OUTPUT); // Z(실제 X)

  pinMode(BTN_RESET00, INPUT_PULLUP);          // 원점 버튼 풀업

  resetStable = resetLastReading = digitalRead(BTN_RESET00); // 버튼 초기화
  resetLastChange = millis();

  loadPersist();                             // 지난 저장 상태 불러오기
  clampXY(); clampZ(); clampDrop(); clampUDPos(); // 안전 보정

  lastOLEDUnitX = curX / DISPLAY_UNIT;       // OLED 기준값
  lastOLEDUnitY = curY / DISPLAY_UNIT;
  lastOLEDUnitZ = curZ / DISPLAY_UNIT;
  lastOLEDBurstsUD = curUD / DROP_UNIT_STEPS;

  updateOLED();                              // 첫 화면
}

// ===== 메인 루프(계속 반복) =====
void loop() {
  serviceAxisProgress();                     // 인터럽트 진행 반영

  UartRxProtocol();                          // 들어온 바이트 해석/필요 시 폐기
  Serial_Main0();                            // 패킷 완성되었으면 처리

  // 원점(RESET) 버튼: 목표 0, 플런저 목표 N=1(200스텝)
  if (resetButtonPressedOnce()) {
    tgtX = 0; tgtY = 0; tgtZ = 0;
    dropSteps = DROP_UNIT_STEPS;
    clampXY(); clampZ(); clampDrop();
    updateOLED();
    if (!isMoving) { goToTargetXYZ(); }      // 즉시 실행
    savePersistNow();
  }

  trySaveIfIdle();                           // 저장 예약 처리
}
