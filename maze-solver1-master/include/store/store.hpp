#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <vector>
#include <set>
#include "path/Path.hpp"

// ========== Filesystem (LittleFS) Helpers ==========

// Initialize filesystem
void setupFS();

// Store vector<int> into filesystem
void writeVectorToFS(const std::vector<int>& vec, const char* filename = "/vector.bin");

// Read vector<int> from filesystem
std::vector<int> readVectorFromFS(const char* filename = "/vector.bin");

// Store set of walls into filesystem
void writeWallsToFS(const std::set<std::pair<std::pair<int,int>, std::pair<int,int>>>& walls,
                    const char* filename = "/walls.bin");

// Read set of walls from filesystem
std::set<std::pair<std::pair<int,int>, std::pair<int,int>>> readWallsFromFS(const char* filename = "/walls.bin");

// Store known paths into filesystem
void writePathListToFS(const std::vector<Path>& paths, const char* filename = "/paths.bin");

// Read paths from filesystem
std::vector<Path> readPathListFromFS(const char* filename = "/paths.bin");

// Remove a file from filesystem
void clearFS(const char* filename);
