/**
 * @file Constants.h
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @author Tom Nurse (tjnurse@wpi.edu) - 2021/2022
 * @brief File containing variables common to the entire system, centralizing the settings of the sailbot
 * @version 2.0.2
 * @date 2022-4-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef Constants_h
#define Constants_h

// necessary libraries for the constants
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>    // Driver code for operating servo
#include <arduino-timer.h> // Library to handle non-blocking function calls at a set interval
#include <ArduinoJson.h>
#include <FastLED.h>

// WiFi credentials
#define HULL_SSID "sailbot_trimtab_ap"
#define PASSWORD "sailbot123"
#define SERVER_HOST "serverip_or_name"
#define SERVER_PORT 8000

/* Trim servo */
const int SERVO_CTR    = 90;
const int SERVO_LO_LIM = 0;
const int SERVO_HI_LIM = 180;
const int Amax = 3;
const int Vmax = 15;
extern bool move_flag;
extern int control_angle;

/* Wind vane*/ // 
const float POT_TICKS_PER_DEGREE = 8.22;
const int angle_tolerance = 5;
#define WIND_OFFSET 20
/* State angles */
const int MAX_LIFT_ANGLE = 15;

/* Pins */
const int potPin = 32;              //A19
const int servoPin = 15;            //Servo

enum TRIM_STATE {TRIM_STATE_MIN_LIFT, TRIM_STATE_MAX_DRAG_PORT, TRIM_STATE_MAX_DRAG_STBD, TRIM_STATE_MAX_LIFT_PORT, TRIM_STATE_MAX_LIFT_STBD, TRIM_STATE_MANUAL};
extern Servo servo; 
                             // Servo object            // Mapped reading from wind direction sensor on the front of the sail
#define POT_MAX 890
#define POT_MIN -830    

#define NUM_WIND_READINGS 5

#define TRIM_ADJUST_INTERVAL_MS 200

// LED setup
#define LED_DATA_PIN 13
#define NUM_LEDS 149
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

void VaneRead(void * pvParameters);
void LEDTimerTask(void * pvParameters);
void ServoTask(void * pvParameters);
void SendData(void * pvParameters);
bool servoControl(int currentWindAngle, TRIM_STATE state);
void movementHandler(int goal_angle, int currentAngle);
float moveToProfile(float diff, float curtime);
bool lightLED(bool reach_buoy,bool move_flag,bool found_buoy,bool trim_auto,bool rudder_auto,bool connection_status);
static void setSectionSolid(int sectionIndex, const CRGB &color);
void LED_setup();

#endif