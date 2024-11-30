#include <WiFi.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Wire.h>

enum speedSettings
{
    SLOW = 210,
    NORMAL = 230,
    FAST = 255
};

class ActiveSpokeSystem
{
private:
    const uint8_t servo1Channel = 0;
    const uint8_t servo2Channel = 1;
    const uint8_t servo3Channel = 2;
    const uint8_t servo4Channel = 3;
    const uint8_t servo1Pin = 32; // LF
    const uint8_t servo2Pin = 4;  // RF
    const uint8_t servo3Pin = 5;  // LB
    const uint8_t servo4Pin = 2;  // RB
    const int servoFrequency = 50;

    const uint8_t servo1Init = 30;  // 30 // 80
    const uint8_t servo2Init = 145; // 165 // 110
    const uint8_t servo3Init = 50;  // 50 // 110
    const uint8_t servo4Init = 175; // 175 // 130

    const uint8_t expansionAngle = 40;

public:
    // Constructor
    ActiveSpokeSystem()
    {
        ledcSetup(servo1Channel, servoFrequency, 16);
        ledcSetup(servo2Channel, servoFrequency, 16);
        ledcSetup(servo3Channel, servoFrequency, 16);
        ledcSetup(servo4Channel, servoFrequency, 16);
        ledcAttachPin(servo1Pin, servo1Channel);
        ledcAttachPin(servo2Pin, servo2Channel);
        ledcAttachPin(servo3Pin, servo3Channel);
        ledcAttachPin(servo4Pin, servo4Channel);
    }

    void servoWrite(int channel, int deg)
    {
        // int duty = deg * 18.2 + 3277; // 16bit resolution servo anlge move formula
        int duty = map(deg, 0, 180, 1638, 8192); // 2.5% ~ 12.5%에 해당하는 듀티 사이클
        ledcWrite(channel, duty);
    }
    void moveServos(int angle1, int angle2, int angle3, int angle4)
    {
        // 각 서보모터를 지정된 각도로 이동하고 대기
        servoWrite(servo1Channel, angle1);
        servoWrite(servo2Channel, angle2);
        servoWrite(servo3Channel, angle3);
        servoWrite(servo4Channel, angle4);
        // vtaskDelay로 지정된 시간만큼 대기 (각도 이동 시간 보장)
        vTaskDelay(800);
    }

    void contraction()
    {
        Serial.println("ASS is contracting...");
        moveServos(servo1Init, servo2Init, servo3Init, servo4Init);
    }

    void expansion()
    {
        Serial.printf("ASS is expanding %d degree ...", expansionAngle);
        moveServos(servo1Init + expansionAngle, servo2Init - expansionAngle, servo3Init + expansionAngle, servo4Init - expansionAngle);
    }
};
class Car
{ // motordrive 1 > left, motordrive 2 > right
private:
    // front left motor connections
    const uint8_t md1in1 = 12;
    const uint8_t md1in2 = 13;
    // front right motor connections
    const uint8_t md1in3 = 14;
    const uint8_t md1in4 = 15;
    // rear left motor connections
    const uint8_t md2in1 = 16;
    const uint8_t md2in2 = 33;
    // rear right motor connections
    const uint8_t md2in3 = 18;
    const uint8_t md2in4 = 19;

    // PWM Setup to control motor speed
    const uint8_t SPEED_CONTROL_PIN_1 = 25; // LF
    const uint8_t SPEED_CONTROL_PIN_2 = 26; // RF
    const uint8_t SPEED_CONTROL_PIN_3 = 27; // LB
    const uint8_t SPEED_CONTROL_PIN_4 = 23; // RB

    // Play around with the frequency settings depending on the motor that you are using
    const int freq = 500;
    const int dcChannel = 4;

    // 8 Bit resolution for duty cycle so value is between 0 - 255
    const int resolution = 8;

    // holds the current speed settings, see values for SLOW, NORMAL, FAST
    speedSettings currentSpeedSettings;

public:
    Car()
    {
        // Set all pins to output
        pinMode(md1in1, OUTPUT);
        pinMode(md1in2, OUTPUT);
        pinMode(md1in3, OUTPUT);
        pinMode(md1in4, OUTPUT);
        pinMode(md2in1, OUTPUT);
        pinMode(md2in2, OUTPUT);
        pinMode(md2in3, OUTPUT);
        pinMode(md2in4, OUTPUT);

        pinMode(SPEED_CONTROL_PIN_1, OUTPUT);
        pinMode(SPEED_CONTROL_PIN_2, OUTPUT);
        pinMode(SPEED_CONTROL_PIN_3, OUTPUT);
        pinMode(SPEED_CONTROL_PIN_4, OUTPUT);

        // have to define pose motor

        // Set initial motor state to OFF
        digitalWrite(md1in1, LOW);
        digitalWrite(md1in2, LOW);
        digitalWrite(md1in3, LOW);
        digitalWrite(md1in4, LOW);
        digitalWrite(md2in1, LOW);
        digitalWrite(md2in2, LOW);
        digitalWrite(md2in3, LOW);
        digitalWrite(md2in4, LOW);

        // // Set the PWM Settings
        ledcSetup(dcChannel, freq, resolution);

        // Attach Pin to Channel
        ledcAttachPin(SPEED_CONTROL_PIN_1, dcChannel);
        ledcAttachPin(SPEED_CONTROL_PIN_2, dcChannel);
        ledcAttachPin(SPEED_CONTROL_PIN_3, dcChannel);
        ledcAttachPin(SPEED_CONTROL_PIN_4, dcChannel);

        // initialize default speed to SLOW
        setCurrentSpeed(speedSettings::NORMAL);
    }

    // Turn the car right
    void turnRight()
    {
        Serial.println("car is turning left...");
        digitalWrite(md1in1, HIGH);
        digitalWrite(md1in2, LOW);
        digitalWrite(md1in3, HIGH);
        digitalWrite(md1in4, LOW);
        digitalWrite(md2in1, HIGH);
        digitalWrite(md2in2, LOW);
        digitalWrite(md2in3, HIGH);
        digitalWrite(md2in4, LOW);
        ledcWrite(dcChannel, currentSpeedSettings);
    }

    // Turn the car left
    void turnLeft()
    {
        Serial.println("car is turning right...");
        digitalWrite(md1in1, LOW);
        digitalWrite(md1in2, HIGH);
        digitalWrite(md1in3, LOW);
        digitalWrite(md1in4, HIGH);
        digitalWrite(md2in1, LOW);
        digitalWrite(md2in2, HIGH);
        digitalWrite(md2in3, LOW);
        digitalWrite(md2in4, HIGH);
        ledcWrite(dcChannel, currentSpeedSettings);
    }

    // Move the car forward
    void moveForward()
    {
        Serial.println("car is moving forward...");
        digitalWrite(md1in1, HIGH);
        digitalWrite(md1in2, LOW);
        digitalWrite(md1in3, LOW);
        digitalWrite(md1in4, HIGH);
        digitalWrite(md2in1, HIGH);
        digitalWrite(md2in2, LOW);
        digitalWrite(md2in3, LOW);
        digitalWrite(md2in4, HIGH);
        ledcWrite(dcChannel, currentSpeedSettings);
    }

    // Move the car backward
    void moveBackward()
    {
        Serial.println("car is moving backward...");
        digitalWrite(md1in1, LOW);
        digitalWrite(md1in2, HIGH);
        digitalWrite(md1in3, HIGH);
        digitalWrite(md1in4, LOW);
        digitalWrite(md2in1, LOW);
        digitalWrite(md2in2, HIGH);
        digitalWrite(md2in3, HIGH);
        digitalWrite(md2in4, LOW);
        ledcWrite(dcChannel, currentSpeedSettings);
    }

    // Stop the car
    void stop()
    {
        Serial.println("car is stopping...");
        digitalWrite(md1in1, LOW);
        digitalWrite(md1in2, LOW);
        digitalWrite(md1in3, LOW);
        digitalWrite(md1in4, LOW);
        digitalWrite(md2in1, LOW);
        digitalWrite(md2in2, LOW);
        digitalWrite(md2in3, LOW);
        digitalWrite(md2in4, LOW);
    }

    // Set the motor speed
    void setMotorSpeed()
    {
        // change the duty cycle of the speed control pin connected to the motor
        Serial.print("Speed Settings: ");
        Serial.println(currentSpeedSettings);
        ledcWrite(dcChannel, currentSpeedSettings);
        //  ledcWrite(channel_1, currentSpeedSettings);
    }
    // Set the current speed
    void setCurrentSpeed(speedSettings newSpeedSettings)
    {
        Serial.println("car is changing speed...");
        currentSpeedSettings = newSpeedSettings;
    }
    // Get the current speed
    speedSettings getCurrentSpeed()
    {
        return currentSpeedSettings;
    }
    // set robot contraction
};

// Change this to your network SSID
const char *ssid = "ActiveSpike";
const char *password = "aaaa1111";

// AsyncWebserver runs on port 80 and the asyncwebsocket is initialize at this point also
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Our car object with ActiveSpoke System
Car car;
ActiveSpokeSystem ass;

// Function to send commands to car
void sendCarCommand(const char *command)
{
    // command could be either "left", "right", "forward" or "reverse" or "stop"
    // or speed settingg "slow-speed", "normal-speed", or "fast-speed"
    // or robot pose setting "contraction" or "expansion"
    if (strcmp(command, "left") == 0)
    {
        car.turnLeft();
    }
    else if (strcmp(command, "right") == 0)
    {
        car.turnRight();
    }
    else if (strcmp(command, "forward") == 0)
    {
        car.moveForward();
    }
    else if (strcmp(command, "backward") == 0)
    {
        car.moveBackward();
    }
    else if (strcmp(command, "slow-speed") == 0)
    {
        car.setCurrentSpeed(speedSettings::SLOW);
    }
    else if (strcmp(command, "normal-speed") == 0)
    {
        car.setCurrentSpeed(speedSettings::NORMAL);
    }
    else if (strcmp(command, "fast-speed") == 0)
    {
        car.setCurrentSpeed(speedSettings::FAST);
    }
    else if (strcmp(command, "contraction") == 0)
    {
        ass.contraction();
    }
    else if (strcmp(command, "expansion") == 0)
    {
        ass.expansion();
    }
    else if (strcmp(command, "stop") == 0)
    {
        car.stop();
    }
}
// Processor for index.html page template.  This sets the radio button to checked or unchecked
String indexPageProcessor(const String &var)
{
    String status = "";
    if (var == "SPEED_SLOW_STATUS")
    {
        if (car.getCurrentSpeed() == speedSettings::SLOW)
        {
            status = "checked";
        }
    }
    else if (var == "SPEED_NORMAL_STATUS")
    {
        if (car.getCurrentSpeed() == speedSettings::NORMAL)
        {
            status = "checked";
        }
    }
    else if (var == "SPEED_FAST_STATUS")
    {
        if (car.getCurrentSpeed() == speedSettings::FAST)
        {
            status = "checked";
        }
    }
    else if (var == "POSE_STATUS")
    {

        status = "checked";
    }
    // Serial.println(var);
    else if (var == "BUTTONPLACEHOLDER")
    {
        String buttons = "";
        String outputStateValue = "checked";
        buttons += "<h4>Real time Robot Status <span id=\"outputState\"></span></h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"output\" " + outputStateValue + "><span class=\"slider\"></span></label>";

        return buttons;
    }
    return status;
}

// cont relax sensing

// Callback function that receives messages from websocket client
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
    {
        Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
        // client->printf("Hello Client %u :)", client->id());
        // client->ping();
    }

    case WS_EVT_DISCONNECT:
    {
        Serial.printf("ws[%s][%u] disconnect\n", server->url(), client->id());
    }

    case WS_EVT_DATA:
    {
        // data packet
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len)
        {
            // the whole message is in a single frame and we got all of it's data
            if (info->opcode == WS_TEXT)
            {
                data[len] = 0;
                char *command = (char *)data;
                sendCarCommand(command);
            }
        }
    }

    case WS_EVT_PONG:
    {
        // Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
    }

    case WS_EVT_ERROR:
    {
        // Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
    }
    }
}

// Function called when resource is not found on the server
void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}

// Setup function
void setup()
{
    car.stop();
    ass.contraction();

    Serial.begin(115200);
    Serial.setTimeout(10000);

    // Initialize the Serial monitor baud rate
    Serial.println("Connecting to ");
    Serial.println(ssid);

    // Connect to your wifi
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // original - stamode not AP mode change to AP mode we use softAP
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid, password);
    // if (WiFi.waitForConnectResult() != WL_CONNECTED)
    // {
    //     Serial.printf("WiFi Failed!\n");
    //     return;
    // }

    // Serial.println(WiFi.localIP());

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // Add callback function to websocket server
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                          Serial.println("Requesting index page...");
                          request->send(SPIFFS, "/index.html", "text/html", false, indexPageProcessor); });

    // Route to load entireframework.min.css file
    server.on("/css/entireframework.min.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/css/entireframework.min.css", "text/css"); });

    // Route to load custom.css file
    server.on("/css/custom.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/css/custom.css", "text/css"); });

    // Route to load custom.js file
    server.on("/js/custom.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/js/custom.js", "text/javascript"); });

    // Send a GET request to <ESP_IP>/update?state=<inputMessage>
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                String inputMessage;
                String inputParam;

                Serial.println(inputMessage);
                request->send(200, "text/plain", "OK"); });

    // Send a GET request to <ESP_IP>/state
    // server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send(200, "text/plain", String(digitalRead(output)).c_str()); });
    // // On Not Found
    server.onNotFound(notFound);

    // Start server
    server.begin();
}

void loop()
{
    //     String angle_input;
    //     int angle = 90;
    //     char command;
    //     command = Serial.read();
    //     switch (command)
    //     {
    //     case 'w':
    //         Serial.println("forward");
    //         sendCarCommand("forward");
    //         break;
    //     case 'a':
    //         Serial.println("left");
    //         sendCarCommand("left");
    //         break;
    //     case 's':
    //         Serial.println("backward");
    //         sendCarCommand("backward");
    //         break;
    //     case 'd':
    //         Serial.println("right");
    //         sendCarCommand("right");
    //         break;
    //     case 'j':
    //         Serial.println("stop");
    //         sendCarCommand("stop");
    //         break;
    //     case 'i':
    //         Serial.println("slow-speed");
    //         sendCarCommand("slow-speed");
    //         break;
    //     case 'o':
    //         Serial.println("normal-speed");
    //         sendCarCommand("normal-speed");
    //         break;
    //     case 'p':
    //         Serial.println("fast-speed");
    //         sendCarCommand("fast-speed");
    //         break;
    //     case 'n':
    //         Serial.println("contraction");
    //         sendCarCommand("contraction");
    //         break;
    //     case 'm':
    //         Serial.println("expansion");
    //         sendCarCommand("expansion");
    //         break;
    //     case 'l':
    //         Serial.println("angle mode");
    //         while (true)
    //         {
    //             // if (Serial.available() > 0) // 입력 데이터가 있는 경우에만 처리
    //             // {
    //             // angle_input = Serial.readStringUntil('\n');
    //             angle = Serial.parseInt();
    //             if (angle == 0)
    //             {
    //                 Serial.println("Exiting servo control mode.");
    //                 break;
    //             }
    //             else
    //             {
    //                 Serial.printf("Moving servos to %d degrees.\n", angle);
    //                 ass.moveServos(angle, angle, angle, angle); // 입력받은 각도로 서보 이동
    //             }
    //         }

    //     default:
    //         break;
    //     }
}
