#include <Arduino.h>
#include <EEPROM.h>
#include <vector>
#include <set>
#include "store/store.hpp"
#include "path/Path.hpp"

constexpr size_t MAX_STEPS = 128;  // adjust depending on your maze size

struct PathData {
    uint16_t length;
    uint16_t turns;
    double optimizedLength;
    int optimizedTurns;
    double feasibilityScore;

    uint16_t positionsSize;
    std::pair<int, int> positions[MAX_STEPS];

    uint16_t movesSize;
    std::pair<int, int> moves[MAX_STEPS];
};

PathData serializePath(const Path& path) {
    PathData data{};
    data.length = path.length;
    data.turns = path.turns;
    data.optimizedLength = path.optimizedLength;
    data.optimizedTurns = path.optimizedTurns;
    data.feasibilityScore = path.feasibilityScore;

    data.positionsSize = std::min((size_t)MAX_STEPS, path.positions.size());
    for (size_t i = 0; i < data.positionsSize; i++) {
        data.positions[i] = path.positions[i];
    }

    data.movesSize = std::min((size_t)MAX_STEPS, path.moves.size());
    for (size_t i = 0; i < data.movesSize; i++) {
        data.moves[i] = path.moves[i];
    }

    return data;
}

Path deserializePath(const PathData& data) {
    std::vector<std::pair<int,int>> positions, moves;
    for (size_t i = 0; i < data.positionsSize; i++) {
        positions.push_back(data.positions[i]);
    }
    for (size_t i = 0; i < data.movesSize; i++) {
        moves.push_back(data.moves[i]);
    }
    return Path(positions, moves);
}


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

void writePathListToEEPROM(const std::vector<Path>& paths) {
    int count = paths.size();
    int addr = 0;

    // store number of paths
    EEPROM.put(addr, count);
    addr += sizeof(int);

    for (int i = 0; i < count; i++) {
        PathData data = serializePath(paths[i]);
        EEPROM.put(addr, data);
        addr += sizeof(PathData);
    }

    Serial.print("Saved ");
    Serial.print(count);
    Serial.println(" paths to EEPROM.");
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

std::vector<Path> readPathListFromEEPROM() {
    int count = 0;
    int addr = 0;
    EEPROM.get(addr, count);
    addr += sizeof(int);

    std::vector<Path> paths;
    for (int i = 0; i < count; i++) {
        PathData data;
        EEPROM.get(addr, data);
        addr += sizeof(PathData);
        paths.push_back(deserializePath(data));
    }

    return paths;
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

