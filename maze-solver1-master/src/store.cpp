#include <Arduino.h>
#include <LittleFS.h>
#include <vector>
#include <set>
#include "store/store.hpp"
#include "path/Path.hpp"

constexpr size_t MAX_STEPS = 511;  // adjust depending on your maze size

// ---------------- PathData serialization ----------------
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

// ---------------- LittleFS helpers ----------------

// define filesystem
LittleFS_Program myfs;   // uses onboard QSPI flash

// save vector<int> to file
void writeVectorToFS(const std::vector<int>& vec, const char* filename = "/vector.bin") {
    File f = myfs.open(filename, FILE_WRITE);
    if (!f) return;
    uint16_t size = vec.size();
    f.write((uint8_t*)&size, sizeof(size));
    for (int v : vec) {
        f.write((uint8_t*)&v, sizeof(v));
    }
    f.close();
}

std::vector<int> readVectorFromFS(const char* filename = "/vector.bin") {
    std::vector<int> vec;
    File f = myfs.open(filename, FILE_READ);
    if (!f) return vec;
    uint16_t size;
    f.read((uint8_t*)&size, sizeof(size));
    for (uint16_t i = 0; i < size; i++) {
        int v;
        f.read((uint8_t*)&v, sizeof(v));
        vec.push_back(v);
    }
    f.close();
    return vec;
}

// save set of walls
void writeWallsToFS(const std::set<std::pair<std::pair<int,int>, std::pair<int,int>>>& walls,
                    const char* filename = "/walls.bin") {
    File f = myfs.open(filename, FILE_WRITE);
    if (!f) return;
    uint16_t size = walls.size();
    f.write((uint8_t*)&size, sizeof(size));
    for (const auto& wall : walls) {
        f.write((uint8_t*)&wall, sizeof(wall));
    }
    f.close();
}

std::set<std::pair<std::pair<int,int>, std::pair<int,int>>> readWallsFromFS(const char* filename = "/walls.bin") {
    std::set<std::pair<std::pair<int,int>, std::pair<int,int>>> walls;
    File f = myfs.open(filename, FILE_READ);
    if (!f) return walls;
    uint16_t size;
    f.read((uint8_t*)&size, sizeof(size));
    for (uint16_t i = 0; i < size; i++) {
        std::pair<std::pair<int,int>, std::pair<int,int>> wall;
        f.read((uint8_t*)&wall, sizeof(wall));
        walls.insert(wall);
    }
    f.close();
    return walls;
}

// save list of paths
void writePathListToFS(const std::vector<Path>& paths, const char* filename = "/paths.bin") {
    File f = myfs.open(filename, FILE_WRITE);
    if (!f) return;
    int count = paths.size();
    f.write((uint8_t*)&count, sizeof(count));
    for (const auto& path : paths) {
        PathData data = serializePath(path);
        f.write((uint8_t*)&data, sizeof(data));
    }
    f.close();

    Serial.printf("Saved %d paths to FS.\n", count);
}

std::vector<Path> readPathListFromFS(const char* filename = "/paths.bin") {
    std::vector<Path> paths;
    File f = myfs.open(filename, FILE_READ);
    if (!f) return paths;
    int count;
    f.read((uint8_t*)&count, sizeof(count));
    for (int i = 0; i < count; i++) {
        PathData data;
        f.read((uint8_t*)&data, sizeof(data));
        paths.push_back(deserializePath(data));
    }
    f.close();
    return paths;
}

void clearFS(const char* filename) {
    myfs.remove(filename);
}

// ---------------- Example setup ----------------
void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (!myfs.begin(256 * 1024)) {
        Serial.println("LittleFS mount failed!");
        return;
    }

    Serial.println("LittleFS initialized.");
}

void loop() {
    // your logic here
}
