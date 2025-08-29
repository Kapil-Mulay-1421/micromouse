// I2C addresses for each sensor (use different addresses)
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31  
#define LOX3_ADDRESS 0x32


// // I2C addresses for each sensor
// #define LOX1_ADDRESS 0x30
// #define LOX2_ADDRESS 0x31
// #define LOX3_ADDRESS 0x29

// XSHUT pins for each sensor
#define SHT_LOX1 2
#define SHT_LOX2 4
#define SHT_LOX3 20

#define setPoint 0.0
#define offSet 1.0
#define Kp 0.75
#define Ki 0.01
#define Kd 0.01
#define maxPwm 125

#define LEFT_PWM_PIN     23    //pwm a
#define LEFT_DIR_PIN1    15    //ain1 
#define LEFT_DIR_PIN2    14    //ain2

#define RIGHT_PWM_PIN    22    //pwm b
#define RIGHT_DIR_PIN1   11    //bin1
#define RIGHT_DIR_PIN2   10    //bin2

#define STDBY 9
#define BASE_PWM         100  // Base speed for forward motion

///////////////////////////////////////////////////////////
// Encoder Configuration
#define RIGHT_ENC_DIR 5   //ylo
#define RIGHT_ENC_INT 6   //grn
#define LEFT_ENC_DIR 7    //ylo
#define LEFT_ENC_INT 8    //grn

#define SCREEN_ADDRESS 0x3C  // I2C address (most common for SSD1306)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Forward declarations for global sensors
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "IMUSensor.h"

// Global sensor objects - declared extern here, defined in main.cpp
extern IMUSensor g_BNOSensor;
extern Adafruit_SSD1306 g_Display;