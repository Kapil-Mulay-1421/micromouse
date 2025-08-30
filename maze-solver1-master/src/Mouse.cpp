#include "mouse/Mouse.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <set>
// #include <memory>
#include "lidar/Lidar.hpp"
#include <Arduino.h>
#include <PID_v1.h>
#include "IMUSensor.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "macros.hpp"
#include "store/store.hpp"

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const float COUNTS_PER_REV = 366.375;
const float WHEEL_DIAMETER_CM = 4.52;
const float DIST_PER_COUNT = (PI * WHEEL_DIAMETER_CM) / COUNTS_PER_REV;


Mouse::Mouse(int x, int y,
    const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& known_walls,
    std::pair<int, int> goal,
    const std::vector<std::vector<int>>& maze,
    const std::vector<Path>& known_paths,
    const std::string& mode)
    : x_(x), y_(y), knownWalls_(known_walls), goal_(goal),
    maze_(maze), knownPaths_(known_paths), mode_(mode),
    direction_({0, 1}) {}  // assuming default facing North
    
    void Mouse::setGoal(const std::pair<int, int>& newGoal) {
        goal_ = newGoal;
}

std::vector<Path> Mouse::getKnownPaths() const {
    return knownPaths_;
}

std::tuple<bool, bool, bool, bool> Mouse::analyzeTofDistances(float front, float left, float right) {
    bool frontClose = 0.01 <= front && front <= 0.23;
    bool frontFar = 0.35 <= front && front <= 0.5;
    bool leftInRange = (0.2316 - 0.075) <= left && left <= (0.2316 + 0.075);
    bool rightInRange = (0.2316 - 0.075) <= right && right <= (0.2316 + 0.075);
    return std::make_tuple(frontClose, frontFar, leftInRange, rightInRange);
}

std::pair<int, int> Mouse::getLeft() {
    if (direction_ == std::make_pair(1, 0)) return {0, 1};
    if (direction_ == std::make_pair(0, 1)) return {-1, 0};
    if (direction_ == std::make_pair(-1, 0)) return {0, -1};
    return {1, 0}; // direction_ == (0, -1)
}

std::pair<int, int> Mouse::getRight() {
    if (direction_ == std::make_pair(1, 0)) return {0, -1};
    if (direction_ == std::make_pair(0, 1)) return {1, 0};
    if (direction_ == std::make_pair(-1, 0)) return {0, 1};
    return {-1, 0}; // direction_ == (0, -1)
}

// void Mouse::scanWalls() {
//     auto found_walls = scan(x_, y_, direction_);
//     for (const auto& wall : found_walls) {
//         auto reverse_wall = std::make_pair(wall.second, wall.first);
        // if (knownWalls_.find(wall) == knownWalls_.end() &&
//             knownWalls_.find(reverse_wall) == knownWalls_.end()) {
//             knownWalls_.insert(wall);
//         }
//     }
// }

void Mouse::scanWalls() {
    float tofFront = sim_.readFrontSensor();
    float tofLeft = sim_.readLeftSensor();
    float tofRight = sim_.readRightSensor();
    std::cout << "ToF readings: " << tofFront << ", " << tofLeft << ", " << tofRight << std::endl;

    bool frontClose, frontFar, leftInRange, rightInRange;
    std::tie(frontClose, frontFar, leftInRange, rightInRange) = analyzeTofDistances(tofFront, tofLeft, tofRight);

    if (frontClose) {
        calibrateFront();
        auto wall = std::make_pair(std::make_pair(x_, y_),
                                   std::make_pair(x_ + direction_.first, y_ + direction_.second));
        if (std::find(knownWalls_.begin(), knownWalls_.end(), wall) == knownWalls_.end()) {
            knownWalls_.insert(wall);
        }
        return;
    }

    if (frontFar) {
        auto wall = std::make_pair(
            std::make_pair(x_ + direction_.first, y_ + direction_.second),
            std::make_pair(x_ + 2 * direction_.first, y_ + 2 * direction_.second));
        if (std::find(knownWalls_.begin(), knownWalls_.end(), wall) == knownWalls_.end()) {
            knownWalls_.insert(wall);
        }
    }

    if (leftInRange) {
        auto left = getLeft();
        auto wall = std::make_pair(
            std::make_pair(x_ + direction_.first, y_ + direction_.second),
            std::make_pair(x_ + direction_.first + left.first, y_ + direction_.second + left.second));
        if (std::find(knownWalls_.begin(), knownWalls_.end(), wall) == knownWalls_.end()) {
            knownWalls_.insert(wall);
        }
    }

    if (rightInRange) {
        auto right = getRight();
        auto wall = std::make_pair(
            std::make_pair(x_ + direction_.first, y_ + direction_.second),
            std::make_pair(x_ + direction_.first + right.first, y_ + direction_.second + right.second));
        if (std::find(knownWalls_.begin(), knownWalls_.end(), wall) == knownWalls_.end()) {
            knownWalls_.insert(wall);
        }
    }
    // write walls to eeprom if no. of walls in knownWalls_ is at least 10 more than that in eeprom.
    auto eepromWalls_ = readWallsFromEEPROM();
    if (knownWalls_.size() >= eepromWalls_.size() + 5) {
        writeWallsToEEPROM(knownWalls_);
    }
    std::cout << "Known walls: " << knownWalls_.size() << std::endl;
}


// Utility movement methods
void Mouse::turnLeft(int leftPwm, int rightPwm) {
    direction_ = {-direction_.second, direction_.first};
    sim_.turnLeft();
}

void Mouse::turnRight(int leftPwm, int rightPwm) {
    direction_ = {direction_.second, -direction_.first};
    sim_.turnRight(leftPwm, rightPwm);
}

void Mouse::turnAround(int leftPwm, int rightPwm) {
    direction_ = {-direction_.first, -direction_.second};
    sim_.turnRight(leftPwm, rightPwm);
    sim_.turnRight(leftPwm, rightPwm);
}

void Mouse::changeDirection(std::pair<int, int> newDir) {
    if (newDir == direction_) return;
    if (newDir == std::make_pair(-direction_.second, direction_.first)) {
        turnLeft(100,100);
    } else if (newDir == std::make_pair(direction_.second, -direction_.first)) {
        turnRight(100,100);
    } else {
        turnAround(100,100);
    }
}

volatile long leftCount = 0;
volatile long rightCount = 0;

void Mouse::resetEncoders() {
    noInterrupts();
    leftCount = 0;
    rightCount = 0;
    interrupts();
}

float Mouse::getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
}

float Mouse::getLeftDistance() {
    return leftCount * DIST_PER_COUNT;
}

float Mouse::getRightDistance() {
    return rightCount * DIST_PER_COUNT;
}

bool Mouse::moveForward(int x, int leftPwm, int rightPwm) {
    
    // correctPosition(relX, relY);
    resetEncoders();
    
    delay(2000);  // Short delay to ensure motors are ready
    unsigned long startTime = millis();

    long unsigned int currDistance = -getAverageDistance();

    while(currDistance < x * 25) {  // 25 cm per block
        sensors_event_t orientation = g_BNOSensor.getOrientation();
        float yaw = orientation.orientation.x;

        if (yaw > 180) yaw -= 360;
        else if (yaw < -180) yaw += 360;

        // Run PID
        Input = yaw;
        myPID.Compute();  // Output is updated here

        if(yaw < 0.01 && yaw > -0.01) {
            Output = 0;
        }

        // Compute motor speeds
        if(yaw < 0){
            leftPwm = BASE_PWM -20- 2 * Output;
            rightPwm = BASE_PWM + 30 * Output;
        }
        else if(yaw >= 0){
            leftPwm = BASE_PWM -20 - 2 * Output;
            rightPwm = BASE_PWM + 30 * Output;
        }
        
        leftPwm = constrain(leftPwm, 0, 255);
        rightPwm = constrain(rightPwm, 0, 255);

        analogWrite(LEFT_PWM_PIN, leftPwm);
        analogWrite(RIGHT_PWM_PIN, rightPwm);

        // Display encoder distances during movement
        g_Display.clearDisplay();
        g_Display.setTextSize(1);
        g_Display.setCursor(0, 0);
        g_Display.print("Time: ");
        g_Display.print((millis() - startTime) / 1000.0, 1);
        g_Display.println("s");
        
        g_Display.setCursor(0, 12);
        g_Display.print("Yaw: ");
        g_Display.print(yaw, 1);
        g_Display.println(" deg");
        
        g_Display.setTextSize(1);
        g_Display.setCursor(0, 24);
        g_Display.println("Distance (cm):");
        
        g_Display.setTextSize(2);
        g_Display.setCursor(0, 36);
        g_Display.print("L:");
        g_Display.println(getLeftDistance(), 1);
        
        g_Display.setCursor(0, 52);
        g_Display.print("R:");
        g_Display.println(getRightDistance(), 1);
        
        g_Display.display();

        // Debug output
        Serial.print("Time: "); Serial.print((millis() - startTime) / 1000.0, 1);
        Serial.print("s | Yaw: "); Serial.print(yaw, 1);
        Serial.print(" | Left: "); Serial.print(getLeftDistance(), 2);
        Serial.print("cm | Right: "); Serial.print(getRightDistance(), 2);
        Serial.print("cm | Avg: "); Serial.print(getAverageDistance(), 2);
        Serial.println("cm");
        currDistance = -getAverageDistance();
    }
    
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);

    // Show final distances after movement
    g_Display.clearDisplay();
    g_Display.setTextSize(1);
    g_Display.setCursor(0, 0);
    g_Display.println("Movement Complete!");
    
    g_Display.setCursor(0, 16);
    g_Display.println("Final Distance:");
    
    g_Display.setTextSize(2);
    g_Display.setCursor(0, 28);
    g_Display.print("L:");
    g_Display.print(getLeftDistance(), 1);
    g_Display.println("cm");
    
    g_Display.setCursor(0, 48);
    g_Display.print("R:");
    g_Display.print(getRightDistance(), 1);
    g_Display.println("cm");
    
    g_Display.display();
    
    Serial.println("=== MOVEMENT COMPLETE ===");
    Serial.print("Final Left Distance: "); Serial.print(getLeftDistance(), 2); Serial.println(" cm");
    Serial.print("Final Right Distance: "); Serial.print(getRightDistance(), 2); Serial.println(" cm");
    Serial.print("Final Average Distance: "); Serial.print(getAverageDistance(), 2); Serial.println(" cm");

    delay(3000);


    x_ = x_ + direction_.first;
    y_ = y_ + direction_.second;
    return true;
}



void Mouse::moveForward() {
    moveForward(1.0, 100, 100); // Move forward by one cell (or suitable default)
}

void Mouse::calibrateFront() {
    float target = 0.094;
    float tof = sim_.readFrontSensor();
    float correction = tof - target;
    float velocity = (correction < 0) ? -0.1f : 0.1f;
    sim_.errCrct(std::abs(correction), velocity);
    std::cout << "Calibrated front distance with correction: " << correction << std::endl;
}

std::unordered_map<std::pair<int, int>, int, pair_hash> Mouse::floodFill(
    std::pair<int, int> goal,
    const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& walls,
    const std::set<std::pair<int, int>>& visited) {
    std::unordered_map<std::pair<int, int>, int, pair_hash> distances;
    std::queue<std::pair<int, int>> q;
    q.push(goal);
    distances[goal] = 0;

    auto isWall = [&](const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::find(walls.begin(), walls.end(), std::make_pair(from, to)) != walls.end() ||
               std::find(walls.begin(), walls.end(), std::make_pair(to, from)) != walls.end();
    };

    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while (!q.empty()) {
        auto curr = q.front();
        q.pop();
        int currDist = distances[curr];

        for (const auto& dir : directions) {
            auto next = std::make_pair(curr.first + dir.first, curr.second + dir.second);

            if (visited.find(next) != visited.end() &&
                distances.find(next) == distances.end() &&
                !isWall(curr, next)) {
                distances[next] = currDist + 1;
                q.push(next);
            }
        }
    }

    return distances;
}

std::vector<std::vector<int>> Mouse::floodFill() {
    // Prepare visited set
    std::set<std::pair<int, int>> visited;
    int l = maze_.size();
    int b = maze_[0].size();
    for (int i = 0; i < l; ++i)
        for (int j = 0; j < b; ++j)
            visited.insert({i, j});

    // Convert knownWalls_ to vector
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> walls(knownWalls_.begin(), knownWalls_.end());

    // Get distances from floodFill
    auto distances = floodFill(goal_, walls, visited);

    // Build maze grid from distances
    std::vector<std::vector<int>> maze(l, std::vector<int>(b, std::numeric_limits<int>::max()));
    for (const auto& [pos, dist] : distances) {
        maze[pos.first][pos.second] = dist;
    }
    return maze;
}


void Mouse::navigate(const std::vector<std::vector<int>>& maze, bool reverse) {
    moves_ = 0;
    int l = maze.size();
    int b = maze[0].size();

    std::vector<std::pair<int, int>> moves;
    std::vector<std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>> known_walls_at_each_step;
    known_walls_at_each_step.push_back(knownWalls_);  // For visualization

    std::vector<std::pair<int, int>> positions;
    positions.emplace_back(x_, y_);

    while (x_ != goal_.first || y_ != goal_.second) {
        // visualizer_.visualizeMaze(maze_, x_, y_, knownWalls_);
        int best_value = std::numeric_limits<int>::max();
        std::pair<int, int> best_move = {0, 0};

        std::vector<std::pair<int, int>> directions;
        if (mode_ == "left_first") {
            directions = {direction_, getLeft(), getRight(), {-direction_.first, -direction_.second}};
        } else {
            directions = {direction_, getRight(), getLeft(), {-direction_.first, -direction_.second}};
        }

        for (auto [dx, dy] : directions) {
            int new_x = x_ + dx;
            int new_y = y_ + dy;

            if (knownWalls_.count({{x_, y_}, {new_x, new_y}}) || knownWalls_.count({{new_x, new_y}, {x_, y_}}))
                continue;

            if (new_x >= 0 && new_x < l && new_y >= 0 && new_y < b) {
                int value = maze_[new_x][new_y];
                if (value < best_value) {
                    best_value = value;
                    best_move = {dx, dy};
                }
            }
        }

        if (direction_ == best_move) {
            moveForward();
            moves_++;
            moves.push_back(best_move);
            positions.emplace_back(x_, y_);
            known_walls_at_each_step.push_back(knownWalls_);
        } else {
            changeDirection(best_move);
        }

        scanWalls();
        maze_ = floodFill();
    }

    // visualizer_.visualizeMaze(maze_, x_, y_, knownWalls_);

    if (reverse) {
        std::tie(moves, positions) = reversePath(moves, positions);
    }

    removeLoopsAndMemorize(moves, positions);

    std::cout << "Goal reached in " << moves_ << " moves\n";
    std::cout << "Number of moves after loop removal and optimization: " << knownPaths_.back().optimizedLength << "\n";
    std::cout << "Number of turns after loop removal and optimization: " << knownPaths_.back().optimizedTurns << "\n";
    std::cout << "Path: ";
    for (auto& pos : knownPaths_.back().positions) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << "\n";
}
void Mouse::removeLoopsAndMemorize(std::vector<std::pair<int, int>>& moves, 
                                std::vector<std::pair<int, int>>& positions) {

    for (size_t i = 0; i < moves.size(); ++i) {
        for (size_t j = moves.size(); j > i; --j) {
            if (positions[i] == positions[j]) {
                // erase moves[i:j]
                moves.erase(moves.begin() + i, moves.begin() + j);
                // erase positions[i:j]
                positions.erase(positions.begin() + i, positions.begin() + j);
                break; // stop after first removal for this i
            }
        }
    }

    Path newPathObj(positions, moves);
    knownPaths_.push_back(newPathObj);
    writePathListToEEPROM(knownPaths_);
}

std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>>
Mouse::reversePath(std::vector<std::pair<int, int>> moves,
                   std::vector<std::pair<int, int>> positions) {
    for (auto& move : moves) {
        move.first = -move.first;
        move.second = -move.second;
    }
    std::reverse(moves.begin(), moves.end());
    std::reverse(positions.begin(), positions.end());

    return {moves, positions};
}

std::shared_ptr<Path> Mouse::getBestPath() const {
    std::shared_ptr<Path> bestPath = nullptr;
    for (const auto& path : knownPaths_) {
        if (!bestPath || path.feasibilityScore > bestPath->feasibilityScore) {
            bestPath = std::make_shared<Path>(path);
        }
    }
    return bestPath;
}

void Mouse::run() {

    sim_.setID();
    while(1){
        float left = sim_.readLeftSensor();
        delayMicroseconds(100);
        float front = sim_.readFrontSensor();
        delayMicroseconds(100);
        float right = sim_.readRightSensor();

    }
    auto bestPath = getBestPath();
        x_ = 0;
        y_ = 0;
        for (auto nextDir : bestPath->moves) {
        if (nextDir == direction_) {
            moveForward();
        } else if (nextDir == getLeft()) {
            turnLeft(100,100);
            moveForward();
        } else if (nextDir == getRight()) {
            turnRight(100,100);
            moveForward();
        } else {
            turnRight(100,100);
            turnRight(100,100);
            moveForward();
        }
        std::cout << x_ << " " << y_ << std::endl;
    }
}