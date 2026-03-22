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

/* Trim servo */
const int SERVO_CTR    = 90;
const int SERVO_LO_LIM = 0;
const int SERVO_HI_LIM = 180;
const int Amax = 3;
const int Vmax = 15;

/* Wind vane*/ // O to 2160 
const int POT_HEADWIND = 0;
const int POT_TICKS_PER_DEGREE = 6;

/* State angles */
const int MAX_LIFT_ANGLE = 30;

/* Pins */
const int potPin = 32;              //A19
const int batteryPin = 36;          //Battery V+
const int servoPin = 15;            //Servo
const int errorLED = 13;             //red
const int bleLED = 3;               //blue
const int powerLED = 5;             //green

#define POT_MAX 890
#define POT_MIN -830    

#define NUM_WIND_READINGS 20

#define TRIM_ADJUST_INTERVAL_MS 200

#endif