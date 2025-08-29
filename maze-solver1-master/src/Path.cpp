#include "path/Path.hpp"
#include <cmath>
#include <limits>
#include <tuple>
#include <iostream>

// Default constructor
Path::Path() = default;

// Constructor with metrics calculation
Path::Path(const std::vector<std::pair<int, int>>& positions_,
           const std::vector<std::pair<int, int>>& moves_)
    : positions(positions_), moves(moves_) {

    length = static_cast<double>(moves.size());
    turns  = calculateTurns(moves);
    std::tie(optimizedLength, optimizedTurns) = optimizeLengthAndTurns(moves, length, turns);
    feasibilityScore = calculateFeasibilityScore(optimizedLength, optimizedTurns);
}

bool Path::isOpposite(const std::pair<int,int>& a, const std::pair<int,int>& b) {
    return a.first == -b.first && a.second == -b.second;
}

int Path::calculateTurns(const std::vector<std::pair<int,int>>& m) {
    int t = 0;
    for (size_t i = 0; i + 1 < m.size(); ++i) {
        if (m[i] == m[i+1]) {
            // straight
        } else if (isOpposite(m[i], m[i+1])) {
            t += 4; // 180° = 4 * 45°
        } else {
            t += 2; // 90° = 2 * 45°
        }
    }
    return t;
}

std::pair<double,int> Path::optimizeLengthAndTurns(
    const std::vector<std::pair<int,int>>& m,
    double rawLen,
    int rawTurns) 
{
    const double diag = std::sqrt(2.0);
    double optLen = 0.0;
    int optTurns = rawTurns;

    size_t i = 0;
    while (i < m.size()) {
        if (i + 1 < m.size() && m[i] != m[i+1]) {
            auto A = m[i];
            auto B = m[i+1];
            size_t s = 2;
            i += 2;
            while (i < m.size() && m[i] == m[i-2]) {
                ++s;
                ++i;
            }
            optLen += (static_cast<int>(s/2)) * diag + (s % 2) * 1.0;
            int cornersInRun = static_cast<int>(s) - 1;
            int reduction = std::max(0, 2 * cornersInRun - 2);
            optTurns -= reduction;
        } else {
            optLen += 1.0;
            ++i;
        }
    }

    if (optLen <= 0.0) optLen = rawLen;
    if (optTurns < 0)  optTurns = 0;

    return {optLen, optTurns};
}

double Path::calculateFeasibilityScore(double optLen, int optTurns) {
    auto wLen = 0.06056929;
    auto wTurns = 0.03632959;
    auto intercept = 0.001;
    double time = wLen * optLen + wTurns * static_cast<double>(optTurns) + intercept;

    if (!(time > 0.0)) {
        return 0.0;
    }
    return 1.0 / time;
}
