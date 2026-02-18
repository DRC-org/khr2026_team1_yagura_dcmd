#include <Arduino.h>
#include <FastLED_NeoPixel.h>
#include <SPI.h>
#include <mcp_can.h>
#include <pwm.h>

void readSwitch();
void motorDrive(int direction);

// PWM 関係
#define PWM_N 11
#define PWM_L 10
#define DIS 7
// センサー関係
#define ENC_A 2
#define ENC_B 3
#define ENC_X 15
#define SW_A 16
#define SW_B 17
#define SENS 14
#define MCP_INT 12
// 通信関係
#define CAN_CS 5
#define SCL 19
#define SDA 18
// LED 関係
#define LED 13
#define RGB 6
// 入力関係
#define SW0 8
#define SW1 9
#define SW2 1
#define SW3 0

// duty 比
#define DUTY 25.0

// R4では"pwm.h"を用いることでPWM周波数を任意の値に指定できる。analogWrite()の上位互換
PwmOut pwm_n(PWM_N);
PwmOut pwm_l(PWM_L);

FastLED_NeoPixel<1, RGB, NEO_GRB> strip;  // RGBLED 制御用

MCP_CAN CAN(CAN_CS);  // MCP2515 CS ピン指定

unsigned int CAN_ID;  // CANのID（DC モータドライバは 0x300 番台）
int motorState = 0;   // モータの状態（正転: 1、逆転: -1、停止: 0）

void setup() {
  Serial.begin(115200);

  pinMode(PWM_N, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIS, OUTPUT);
  // pinMode(ENC_A, INPUT);
  // pinMode(ENC_B, INPUT);
  // pinMode(SENS, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(RGB, OUTPUT);
  pinMode(SW_A, INPUT_PULLUP);
  pinMode(SW_B, INPUT_PULLUP);
  pinMode(SW0, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(100);

  // モータ暴走防止のための初期化
  digitalWrite(PWM_N, LOW);
  digitalWrite(PWM_L, LOW);
  digitalWrite(DIS, LOW);
  pwm_n.begin((uint32_t)30, (uint32_t)0);  // スイッチング周期 30 µs（≒ 33 kHz）
  pwm_l.begin((uint32_t)30, (uint32_t)0);  // スイッチング周期 30 µs（≒ 33 kHz）

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("CAN.begin(...) failed.");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      strip.show();
      delay(100);
      strip.setPixelColor(0, strip.Color(0, 0, 0));
      strip.show();
      delay(100);
    }
  }
  CAN.setMode(MCP_NORMAL);

  readSwitch();  // DIP スイッチの読み出して CAN_ID を指定
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    unsigned char len;
    unsigned char buf[8];
    CAN.readMsgBuf(&rxId, &len, buf);  // 受信した CAN の読み出し

    if (rxId == CAN_ID) {
      unsigned char command = buf[0];

      if (command == 0x00) {  // 正転
        motorState = 1;
      } else if (command == 0x01) {  // 逆転
        motorState = -1;
      } else if (command == 0x02) {  // 停止
        motorState = 0;
      }
    }
    Serial.print("motorState : ");
    Serial.println(motorState);
  }

  motorDrive(motorState);

  Serial.print(digitalRead(SW_A));
  Serial.println(digitalRead(SW_B));
}

// DIPスイッチを読み取ってCAN_IDを指定する
void readSwitch() {
  unsigned int readNumber = !digitalRead(SW0) + 2 * !digitalRead(SW1) +
                            4 * !digitalRead(SW2) + 8 * !digitalRead(SW3);
  switch (readNumber) {
    case 0:
      strip.setPixelColor(0, strip.Color(255, 100, 0));
      break;
    case 1:
      strip.setPixelColor(0, strip.Color(255, 255, 0));
      break;
    case 2:
      strip.setPixelColor(0, strip.Color(0, 255, 0));
      break;
    case 3:
      strip.setPixelColor(0, strip.Color(0, 0, 255));
      break;
  }
  strip.show();
  CAN_ID = 0x300 + readNumber;
  Serial.print("CAN_ID: ");
  Serial.println(CAN_ID);
}

// モータ駆動
void motorDrive(int direction) {  // direction: 1 = 正転, -1 = 逆転, 0 = 停止
  if (direction == 1) {           // 正転
    pwm_n.pulse_perc(DUTY);
    pwm_l.pulse_perc(0);
  } else if (direction == -1) {  // 逆転
    pwm_n.pulse_perc(0);
    pwm_l.pulse_perc(DUTY);
  } else if (direction == 0) {  // 停止
    pwm_l.pulse_perc(0);
    pwm_n.pulse_perc(0);
  }
}
