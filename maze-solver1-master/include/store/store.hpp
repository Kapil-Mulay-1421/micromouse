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
void writeVectorToFS(const std::vector<int>& vec);

// Store set of walls into EEPROM
void writeWallsToFS(const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& walls);

// Store known paths to EEPROM
void writePathListToFS(std::vector<Path>& path);

// Read vector<int> from EEPROM
std::vector<int> readVectorFromFS();

// Read walls set from EEPROM
std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> readWallsFromFS();

// Read paths from EEPROM
std::vector<Path> readPathListFromFS();

// Clear EEPROM contents
void clearEEPROM();
