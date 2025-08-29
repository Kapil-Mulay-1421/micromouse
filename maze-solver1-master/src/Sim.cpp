#include "sim/Sim.hpp"
#include <iostream>
#include "macros.hpp"
#include <PID_v1.h>

Adafruit_VL53L0X Sim::lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X Sim::lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X Sim::lox3 = Adafruit_VL53L0X();
 
VL53L0X_RangingMeasurementData_t Sim::measure1;
VL53L0X_RangingMeasurementData_t Sim::measure2;
VL53L0X_RangingMeasurementData_t Sim::measure3;

bool Sim::errCrct(double distance, float velocity) { 
    // Calculate error based on distance and velocity
    double error = distance - velocity; // Simple error calculation
    
    if (error < 0) {
        // Negative error - move backward with PWM 50
        Serial.println("Error correction: Moving backward");
        
        // Set motor directions for backward movement
        digitalWrite(LEFT_DIR_PIN1, LOW);   // Left motor backward
        digitalWrite(LEFT_DIR_PIN2, HIGH);
        digitalWrite(RIGHT_DIR_PIN1, HIGH); // Right motor backward
        digitalWrite(RIGHT_DIR_PIN2, LOW);
        
        // Apply PWM 50 for a short duration
        analogWrite(LEFT_PWM_PIN, 50);
        analogWrite(RIGHT_PWM_PIN, 50);
        delay(100); // Move for 100ms
        
        // Stop motors
        analogWrite(LEFT_PWM_PIN, 0);
        analogWrite(RIGHT_PWM_PIN, 0);
        
        // Restore forward direction
        digitalWrite(LEFT_DIR_PIN1, HIGH);  // Left motor forward
        digitalWrite(LEFT_DIR_PIN2, LOW);
        digitalWrite(RIGHT_DIR_PIN1, LOW);  // Right motor forward
        digitalWrite(RIGHT_DIR_PIN2, HIGH);
        
    } else {
        // Positive or zero error - move forward
        Serial.println("Error correction: Moving forward");
        
        // Set motor directions for forward movement
        digitalWrite(LEFT_DIR_PIN1, HIGH);  // Left motor forward
        digitalWrite(LEFT_DIR_PIN2, LOW);
        digitalWrite(RIGHT_DIR_PIN1, LOW);  // Right motor forward
        digitalWrite(RIGHT_DIR_PIN2, HIGH);
        
        // Apply PWM 50 for a short duration
        analogWrite(LEFT_PWM_PIN, 50);
        analogWrite(RIGHT_PWM_PIN, 50);
        delay(100); // Move for 100ms
        
        // Stop motors
        analogWrite(LEFT_PWM_PIN, 0);
        analogWrite(RIGHT_PWM_PIN, 0);
    }
    
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(", Distance: ");
    Serial.print(distance);
    Serial.print(", Velocity: ");
    Serial.println(velocity);

    return true;
}

bool Sim::turnLeft(int leftPwm, int rightPwm) {
    // Set motor directions for left turn: left motor backward, right motor forward
    digitalWrite(LEFT_DIR_PIN1, LOW);   // Left motor backward
    digitalWrite(LEFT_DIR_PIN2, HIGH);
    digitalWrite(RIGHT_DIR_PIN1, LOW);  // Right motor forward
    digitalWrite(RIGHT_DIR_PIN2, HIGH);

    // Get current yaw
    sensors_event_t orientation = g_BNOSensor.getOrientation();
    float startYaw = orientation.orientation.x;
    if (startYaw > 180) startYaw -= 360;
    else if (startYaw < -180) startYaw += 360;

    // Target is 90 deg left
    float targetYaw = startYaw + 90.0;
    if (targetYaw > 180) targetYaw -= 360;
    else if (targetYaw < -180) targetYaw += 360;

    // PID setpoint for yaw
    Setpoint = targetYaw;
    float currentYaw = startYaw;
    float angleDiff = 90.0;

    while (abs(angleDiff) > 1.5) {
        orientation = g_BNOSensor.getOrientation();
        currentYaw = orientation.orientation.x;
        if (currentYaw > 180) currentYaw -= 360;
        else if (currentYaw < -180) currentYaw += 360;

        // Run PID for yaw
        Input = currentYaw;
        myPID.Compute();

        // Output is correction for left/right speed
        int leftSpeed = constrain(leftPwm - Output, 0, 255);
        int rightSpeed = constrain(rightPwm + Output, 0, 255);

        analogWrite(LEFT_PWM_PIN, leftSpeed);
        analogWrite(RIGHT_PWM_PIN, rightSpeed);

        angleDiff = targetYaw - currentYaw;
        if (angleDiff > 180) angleDiff -= 360;
        else if (angleDiff < -180) angleDiff += 360;

        delay(20);
    }

    // Stop motors
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);

    // Restore forward direction
    digitalWrite(LEFT_DIR_PIN1, HIGH);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, HIGH);

    // Update PID setpoint to new heading
    Setpoint = targetYaw;
    Serial.print("Turn complete. Final yaw: ");
    Serial.println(currentYaw);
    return true;
}

bool Sim::turnRight(int leftPwm, int rightPwm) {
    // Set motor directions for right turn: left motor forward, right motor backward
    digitalWrite(LEFT_DIR_PIN1, HIGH);  // Left motor forward
    digitalWrite(LEFT_DIR_PIN2, LOW);
    digitalWrite(RIGHT_DIR_PIN1, LOW);  // Right motor backward
    digitalWrite(RIGHT_DIR_PIN2, HIGH);

    // Get current yaw
    sensors_event_t orientation = g_BNOSensor.getOrientation();
    float startYaw = orientation.orientation.x;
    if (startYaw > 180) startYaw -= 360;
    else if (startYaw < -180) startYaw += 360;

    // Target is 90 deg right
    float targetYaw = startYaw - 90.0;
    if (targetYaw > 180) targetYaw -= 360;
    else if (targetYaw < -180) targetYaw += 360;

    // PID setpoint for yaw
    Setpoint = targetYaw;
    float currentYaw = startYaw;
    float angleDiff = 90.0;

    while (abs(angleDiff) > 1.5) {
        orientation = g_BNOSensor.getOrientation();
        currentYaw = orientation.orientation.x;
        if (currentYaw > 180) currentYaw -= 360;
        else if (currentYaw < -180) currentYaw += 360;

        // Run PID for yaw
        Input = currentYaw;
        myPID.Compute();

        // Output is correction for left/right speed
        int leftSpeed = constrain(leftPwm - Output, 0, 255);
        int rightSpeed = constrain(rightPwm + Output, 0, 255);

        analogWrite(LEFT_PWM_PIN, leftSpeed);
        analogWrite(RIGHT_PWM_PIN, rightSpeed);

        angleDiff = targetYaw - currentYaw;
        if (angleDiff > 180) angleDiff -= 360;
        else if (angleDiff < -180) angleDiff += 360;

        delay(20);
    }

    // Stop motors
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);

    // Restore forward direction
    digitalWrite(LEFT_DIR_PIN1, HIGH);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, HIGH);

    // Update PID setpoint to new heading
    Setpoint = targetYaw;
    Serial.print("Turn complete. Final yaw: ");
    Serial.println(currentYaw);
    return true;
}


void Sim::setID() {
    Serial.println(F("Starting VL53L0X sensor initialization..."));
    
    // Configure XSHUT pins as outputs
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    
    // Initialize I2C communication
    Wire1.begin();
    Wire1.setClock(100000); // Set I2C to 100kHz for better stability
    
    Serial.println(F("I2C initialized, scanning for devices..."));
    scanI2C(); // Scan before initialization
    
    Serial.println(F("Resetting all sensors..."));
    
    // Reset all sensors by pulling XSHUT low
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    delay(100); // Hold in reset
    
    // Initialize LOX1 first
    Serial.println(F("Initializing LOX1..."));
    digitalWrite(SHT_LOX1, HIGH); // Enable LOX1
    delay(100); // Wait for sensor to boot
    
    if (!lox1.begin(0x29, false, &Wire1)) { // Start with default address
        Serial.println(F("Failed to initialize LOX1"));
        Serial.println(F("Check LOX1 wiring: SDA, SCL, VCC, GND, XSHUT"));
        scanI2C(); // Scan again to see what's on bus
        while(1) delay(1000);
    }
    lox1.setAddress(LOX1_ADDRESS); // Change to unique address
    Serial.print(F("LOX1 initialized with address: 0x"));
    Serial.println(LOX1_ADDRESS, HEX);
    
    // Initialize LOX2
    Serial.println(F("Initializing LOX2..."));
    digitalWrite(SHT_LOX2, HIGH); // Enable LOX2
    delay(100); // Wait for sensor to boot
    
    if (!lox2.begin(0x29, false, &Wire1)) { // Start with default address
        Serial.println(F("Failed to initialize LOX2"));
        Serial.println(F("Check LOX2 wiring: SDA, SCL, VCC, GND, XSHUT"));
        scanI2C(); // Scan again to see what's on bus
        while(1) delay(1000);
    }
    lox2.setAddress(LOX2_ADDRESS); // Change to unique address
    Serial.print(F("LOX2 initialized with address: 0x"));
    Serial.println(LOX2_ADDRESS, HEX);
    
    // Initialize LOX3
    Serial.println(F("Initializing LOX3..."));
    digitalWrite(SHT_LOX3, HIGH); // Enable LOX3
    delay(100); // Wait for sensor to boot
    
    if (!lox3.begin(0x29, false, &Wire1)) { // Start with default address
        Serial.println(F("Failed to initialize LOX3"));
        Serial.println(F("Check LOX3 wiring: SDA, SCL, VCC, GND, XSHUT"));
        scanI2C(); // Scan again to see what's on bus
        while(1) delay(1000);
    }
    lox3.setAddress(LOX3_ADDRESS); // Change to unique address
    Serial.print(F("LOX3 initialized with address: 0x"));
    Serial.println(LOX3_ADDRESS, HEX);
    
    Serial.println(F("All VL53L0X sensors initialized successfully!"));
    scanI2C(); // Final scan to confirm all addresses
    
    // Test all sensors
    Serial.println(F("Testing sensor readings..."));
    delay(100);
    readLeftSensor();
    delay(100);
    readFrontSensor(); 
    delay(100);
    readRightSensor();
    Serial.println(F("Sensor test complete."));
}

// I2C Scanner function for debugging
void Sim::scanI2C() {
    Serial.println(F("Scanning I2C bus..."));
    int deviceCount = 0;
    
    for (byte address = 1; address < 127; address++) {
        Wire1.beginTransmission(address);
        if (Wire1.endTransmission() == 0) {
            Serial.print(F("I2C device found at address 0x"));
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(F("!"));
            deviceCount++;
        }
    }
    
    if (deviceCount == 0) {
        Serial.println(F("No I2C devices found"));
    } else {
        Serial.print(F("Found "));
        Serial.print(deviceCount);
        Serial.println(F(" I2C devices"));
    }
    Serial.println(F("I2C scan complete\n"));
}

int Sim::readLeftSensor() {
    lox1.rangingTest(&measure1, false);

    // print left sensor reading
    Serial.print(F("Left: "));
    if(measure1.RangeStatus != 4) {
        Serial.print(measure1.RangeMilliMeter);
        Serial.print(F(" "));
        return measure1.RangeMilliMeter;
    } else {
        Serial.print(F("Out of range "));
        return -1; // Return -1 for out of range instead of stopping
    }
}
int Sim::readFrontSensor() {
    lox2.rangingTest(&measure2, false);

    // print front sensor reading
    Serial.print(F("Front: "));
    if(measure2.RangeStatus != 4) {
        Serial.print(measure2.RangeMilliMeter);
        Serial.print(F(" "));
        return measure2.RangeMilliMeter;
    } else {
        Serial.print(F("Out of range "));
        return -1; // Return -1 for out of range instead of stopping
    }
}

int Sim::readRightSensor() {
    lox3.rangingTest(&measure3, false);

    // print right sensor reading
    Serial.print(F("Right: "));
    if(measure3.RangeStatus != 4) {
        Serial.print(measure3.RangeMilliMeter);
        Serial.println(F(" "));
        return measure3.RangeMilliMeter;
    } else {
        Serial.println(F("Out of range "));
        return -1; // Return -1 for out of range instead of stopping
    }
}

