#pragma once

#include <vector>
#include <utility>

// Forward-declare regress() from Regression.hpp if needed
#include "regression/Regression.hpp"  

class Path {
public:
    std::vector<std::pair<int, int>> positions;   // optional, not required for metrics
    std::vector<std::pair<int, int>> moves;       // unit steps like (1,0),(0,1),(-1,0),(0,-1)

    int turns = 0;                // total 45° turn units
    double length = 0.0;          // raw path length (grid units)
    double optimizedLength = 0.0; // length after diagonal smoothing
    int optimizedTurns = 0;       // turns after smoothing (45° units)
    double feasibilityScore = 0;

    Path();  // default constructor

    Path(const std::vector<std::pair<int, int>>& positions_,
         const std::vector<std::pair<int, int>>& moves_);

private:
    static bool isOpposite(const std::pair<int,int>& a, const std::pair<int,int>& b);
    static int calculateTurns(const std::vector<std::pair<int,int>>& m);
    static std::pair<double,int> optimizeLengthAndTurns(const std::vector<std::pair<int,int>>& m,
                                                        double rawLen,
                                                        int rawTurns);
    static double calculateFeasibilityScore(double optLen, int optTurns);
};
