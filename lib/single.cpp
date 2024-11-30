#include <Arduino.h>

const uint8_t in1 = 12; // L298N의 IN1 핀
const uint8_t in2 = 13; // L298N의 IN2 핀
const uint8_t ENA = 25; // L298N의 ENA 핀 (PWM 핀)

const int pwmChannel = 2; // PWM 채널
const int freq = 2000;    // PWM 주파수
const int resolution = 8; // PWM 해상도 (0 - 255)

// 모터 초기화 함수
void setup()
{
    // 시리얼 통신 초기화 (디버깅용)
    Serial.begin(115200);

    // 핀 모드 설정
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ENA, OUTPUT);

    // PWM 설정
    ledcSetup(pwmChannel, freq, resolution);
    ledcAttachPin(ENA, pwmChannel);

    // 모터 정지
}

// 모터 전진 함수
void moveForward(uint8_t speed)
{
    Serial.println("Moving forward...");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
}

// 모터 후진 함수
void moveBackward(uint8_t speed)
{
    Serial.println("Moving backward...");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, speed);
}

// 모터 정지 함수
void stopMotor()
{
    Serial.println("Stopping motor...");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
}

void loop()
{
    // 전진 (속도 200)
    moveForward(250);
    delay(2000); // 2초 동안 전진

    // 정지
    stopMotor();
    delay(1000); // 1초 동안 정지

    // 후진 (속도 150)
    moveBackward(250);
    delay(2000); // 2초 동안 후진

    // 정지
    stopMotor();
    delay(1000); // 1초 동안 정지
}