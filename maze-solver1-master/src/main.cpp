#include <Arduino.h>
#include <vector>
#include <utility>
#include "mouse/Mouse.hpp"
#include "macros.hpp"
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "IMUSensor.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>

IMUSensor g_BNOSensor(Wire);
Adafruit_SSD1306 g_Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int selectMode() {
    int mode = 0; // Default mode
    unsigned long lastButtonPress = millis();
    unsigned long timeout = 4000; // 4 seconds timeout
    bool buttonPressed = false;
    
    Serial.println("Mode selection - Press button 3 to cycle modes, 4 sec timeout");
    
    // Display initial mode
    g_Display.clearDisplay();
    g_Display.setTextSize(2);
    g_Display.setTextColor(SSD1306_WHITE);
    g_Display.setCursor(0, 0);
    g_Display.println("MODE: 0");
    g_Display.println("EXPLORATION");
    g_Display.display();
                   
    while (millis() - lastButtonPress < timeout) {
        int button3State = digitalRead(3);
        
        if (button3State == LOW && !buttonPressed) {
            buttonPressed = true;
            mode = (mode + 1) % 2; 
            lastButtonPress = millis(); 
            
            // Update display
            g_Display.clearDisplay();
            g_Display.setCursor(0, 0);
            g_Display.print("MODE: ");
            g_Display.println(mode);
            
            if (mode == 0) {
                g_Display.println("EXPLORATION");
            } else if (mode == 1) {
                g_Display.println("SPEEDRUN");
            } 
            g_Display.display();
            
            Serial.print("Mode changed to: ");
            Serial.println(mode);
            
            delay(50); // Debounce
        } else if (button3State == HIGH && buttonPressed) {
            buttonPressed = false; // Button released
        }
        
        delay(10);
    }
    
    // Mode locked
    g_Display.clearDisplay();
    g_Display.setCursor(0, 0);
    g_Display.print("LOCKED: ");
    g_Display.println(mode);
    if (mode == 0) {
        g_Display.println("EXPLORATION");
    } else if (mode == 1) {
        g_Display.println("SPEEDRUN");
    } 
    g_Display.display();
    
    Serial.print("Mode locked at: ");
    Serial.println(mode);
    
    return mode;
}

void setup() {
    Serial.begin(115200);
    delay(1000); 
    Serial.println("=== MAZE SOLVER STARTUP ===");

    pinMode(STDBY, OUTPUT);
    digitalWrite(STDBY, HIGH);

    pinMode(3, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);
    
    Serial.println("Pins configured. Testing button readings:");
    Serial.print("Button 3 state: ");
    Serial.println(digitalRead(3));
    Serial.print("Button 27 state: ");
    Serial.println(digitalRead(27));

    // Initialize I2C and OLED first (needed for mode selection display)
    Wire.begin();

    // if (!g_Display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //     Serial.println("OLED initialization failed!");
    //     while (1);
    // }
    
    // Select mode with button cycling and timeout
    // int mode = selectMode();
    int mode = 1;

    
    Serial.println("Starting maze solver");

    // if (!g_BNOSensor.begin()) {
    //     Serial.println("IMU initialization failed. Check wiring or address.");
    //     while (1);
    // }
    Serial.println("IMU initialized successfully.");
    
    std::vector<std::vector<int>> maze(16, std::vector<int>(16, 0));

    std::pair<int, int> start_point = {0, 0};
    std::pair<int, int> goal = {8, 8};

    Mouse mouse(start_point.first, start_point.second, {}, goal, maze, {}, "right_first");
    Serial.println("Mouse initialized.");
    
    Serial.print("Using mode: ");
    // if (mode == 0) {
    //     Serial.println("EXPLORATION");
    // } else if (mode == 1) {
    //     Serial.println("SPEEDRUN");
    // } 
    
    if (mode == 0) {
        // EXPLORATION MODE
        Serial.println("scanning walls");
        mouse.scanWalls();
        Serial.println("flood filling maze");
        maze = mouse.floodFill();
        Serial.println("navigating to goal");
        mouse.navigate(maze);

        mouse.setGoal(start_point);
        mouse.scanWalls();
        maze = mouse.floodFill();
        mouse.navigate(maze, true);

        // mouse.setGoal(goal);
        // mouse.scanWalls();
        // maze = mouse.floodFill();
        // mouse.navigate(maze);

        // Print all found paths
        Serial.println("All paths found:");
        for (const auto& path : mouse.getKnownPaths()) {
            Serial.print("Path: ");
            for (auto& p : path.positions) {
                Serial.print("(");
                Serial.print(p.first);
                Serial.print(",");
                Serial.print(p.second);
                Serial.print(") ");
            }
            Serial.print("\nLength: ");
            Serial.print(path.optimizedLength);
            Serial.print(", Turns: ");
            Serial.print(path.optimizedTurns);
            Serial.print(", Feasibility Score: ");
            Serial.println(path.feasibilityScore);
        }

        // Get and print best path
        auto best_path = mouse.getBestPath();
        Serial.println("The best path is:");
        for (auto& p : best_path->positions) {
            Serial.print("(");
            Serial.print(p.first);
            Serial.print(",");
            Serial.print(p.second);
            Serial.print(") ");
        }
        Serial.print("\nLength: ");
        Serial.print(best_path->optimizedLength);
        Serial.print(", Turns: ");
        Serial.print(best_path->optimizedTurns);
        Serial.print(", Feasibility Score: ");
        Serial.println(best_path->feasibilityScore);

    } 
    else {
        // SPEEDRUN MODE
        Serial.println("Starting speedrun mode");
        mouse.run();
    }

    Serial.println("Program completed");}

void loop() {

}