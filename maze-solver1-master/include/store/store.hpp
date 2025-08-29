#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <vector>
#include <set>

// Global vector in RAM
extern std::vector<int> values;

// === EEPROM Helpers ===

// Store vector<int> into EEPROM
void writeVectorToEEPROM(const std::vector<int>& vec);

// Store set of walls into EEPROM
void writeWallsToEEPROM(const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& walls);

// Read vector<int> from EEPROM
std::vector<int> readVectorFromEEPROM();

// Read walls set from EEPROM
std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> readWallsFromEEPROM();

// Clear EEPROM contents
void clearEEPROM();
