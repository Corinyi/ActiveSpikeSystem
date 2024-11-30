#include <WiFi.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Wire.h>
#include <ESP32Servo.h>

enum speedSettings
{
    SLOW = 150,
    NORMAL = 200,
    FAST = 250
};

class ActiveSpokeSystem
{
private:
    Servo servo1;
    int minUs = 500;
    int maxUs = 2500;
    const uint8_t servo1Pin = 2;

public:
    ActiveSpokeSystem() {}

    void begin()
    {
        // 서보 모터를 위한 타이머 설정 - 타이머 충돌 방지
        ESP32PWM::allocateTimer(0);
        servo1.attach(servo1Pin, minUs, maxUs);
        servo1.setPeriodHertz(300); // 서보 모터의 표준 주파수
    }

    void moveServos(int angle)
    {
        servo1.write(angle);
        vTaskDelay(500);
    }

    void defaultPosition()
    {
        moveServos(60);
    }

    void contraction()
    {
        Serial.println("ASS is contracting...");
        moveServos(20);
    }

    void expansion()
    {
        Serial.println("ASS is expanding...");
        moveServos(60);
    }

    void detachServos()
    {
        servo1.detach();
    }
};

class Car
{
private:
    const uint8_t in1 = 33;
    const uint8_t in2 = 32;
    const uint8_t SPEED_CONTROL_PIN_1 = 14;
    const int freq = 5000;
    const int channel_0 = 1; // 채널 번호가 겹치지 않도록 설정
    const int resolution = 8;
    speedSettings currentSpeedSettings;

public:
    Car()
    {
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);

        // DC 모터 PWM 설정
        ledcSetup(channel_0, freq, resolution);
        ledcAttachPin(SPEED_CONTROL_PIN_1, channel_0);

        setCurrentSpeed(speedSettings::NORMAL);
    }

    void moveForward()
    {
        Serial.println("Car is moving forward...");
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        ledcWrite(channel_0, 250);
    }

    void moveBackward()
    {
        Serial.println("Car is moving backward...");
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        ledcWrite(channel_0, 200);
    }

    void stop()
    {
        Serial.println("Car is stopping...");
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(channel_0, 0);
    }

    void setCurrentSpeed(speedSettings newSpeedSettings)
    {
        currentSpeedSettings = newSpeedSettings;
    }
};

const char *ssid = "ActiveSpike";
const char *password = "aaaa1111";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Car car;
ActiveSpokeSystem ass;

void sendCarCommand(const char *command)
{
    if (strcmp(command, "forward") == 0)
    {
        car.moveForward();
    }
    else if (strcmp(command, "backward") == 0)
    {
        car.moveBackward();
    }
    else if (strcmp(command, "stop") == 0)
    {
        car.stop();
        ass.defaultPosition();
    }
    else if (strcmp(command, "contraction") == 0)
    {
        ass.contraction();
    }
    else if (strcmp(command, "expansion") == 0)
    {
        ass.expansion();
    }
}

void setup()
{
    car.stop();
    ass.begin();

    Serial.begin(115200);
    // WiFi 초기화, 서버 설정 코드 생략
}

void loop()
{
    char command = Serial.read();
    switch (command)
    {
    case 'w':
        sendCarCommand("forward");
        break;
    case 's':
        sendCarCommand("backward");
        break;
    case 'j':
        sendCarCommand("stop");
        break;
    case 'n':
        sendCarCommand("contraction");
        break;
    case 'm':
        sendCarCommand("expansion");
        break;
    default:
        break;
    }
}