#include <Arduino.h>
#include <EEPROM.h>
#include <vector>
#include <set>
#include "store/store.hpp"
#include "path/Path.hpp"

// Vector to hold input values in RAM
std::vector<int> values;

// EEPROM layout:
// [0] = number of elements (uint16_t, 2 bytes)
// [2..] = elements (each int stored as 2 bytes)

void writeVectorToEEPROM(const std::vector<int>& vec) {
  uint16_t size = vec.size();
  EEPROM.put(0, size); // store vector size at beginning
  int addr = 2;
  for (int v : vec) {
    EEPROM.put(addr, v);
    addr += sizeof(int);
  }
}

void writeWallsToEEPROM(const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& walls) {
    uint16_t size = walls.size();
    EEPROM.put(0, size); // store number of walls at beginning
    int addr = 2;
    for (const auto& wall : walls) {
        EEPROM.put(addr, wall.first.first);
        addr += sizeof(int);
        EEPROM.put(addr, wall.first.second);
        addr += sizeof(int);
        EEPROM.put(addr, wall.second.first);
        addr += sizeof(int);
        EEPROM.put(addr, wall.second.second);
        addr += sizeof(int);
    }
}

void writePathToEEPROM(Path& path) {
  
}

std::vector<int> readVectorFromEEPROM() {
  std::vector<int> vec;
  uint16_t size;
  EEPROM.get(0, size);
  int addr = 2;
  for (uint16_t i = 0; i < size; i++) {
    int v;
    EEPROM.get(addr, v);
    vec.push_back(v);
    addr += sizeof(int);
  }
  return vec;
}

std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> readWallsFromEEPROM() {
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> walls;
    uint16_t size;
    EEPROM.get(0, size);
    int addr = 2;
    for (uint16_t i = 0; i < size; i++) {
        std::pair<std::pair<int, int>, std::pair<int, int>> wall;
        EEPROM.get(addr, wall.first.first);
        addr += sizeof(int);
        EEPROM.get(addr, wall.first.second);
        addr += sizeof(int);
        EEPROM.get(addr, wall.second.first);
        addr += sizeof(int);
        EEPROM.get(addr, wall.second.second);
        addr += sizeof(int);
        walls.insert(wall);
    }
    return walls;
}

void clearEEPROM() {
  uint16_t zero = 0;
  EEPROM.put(0, zero);  // set vector size = 0
  // (Optional: wipe rest of EEPROM if you want full clear)
  for (int i = 2; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}

// void setup() {
//   Serial.begin(9600);
//   while (!Serial); // wait for Serial
//   Serial.println("Teensy Flash Vector Example");
//   Serial.println("Enter numbers:");
//   Serial.println(" - Enter 0 to print stored values");
//   Serial.println(" - Enter -1 to clear flash");
// }

