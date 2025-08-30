#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <vector>
#include <set>
#include "path/Path.hpp"

// Global vector in RAM
extern std::vector<int> values;

// === EEPROM Helpers ===

// Store vector<int> into EEPROM
void writeVectorToEEPROM(const std::vector<int>& vec);

// Store set of walls into EEPROM
void writeWallsToEEPROM(const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& walls);

// Store known paths to EEPROM
void writePathListToEEPROM(std::vector<Path>& path);

// Read vector<int> from EEPROM
std::vector<int> readVectorFromEEPROM();

// Read walls set from EEPROM
std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> readWallsFromEEPROM();

// Read paths from EEPROM
std::vector<Path> readPathListFromEEPROM();

// Clear EEPROM contents
void clearEEPROM();
