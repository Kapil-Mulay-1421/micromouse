#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <set>
#include <utility>

class Visualizer {
public:
    void visualizeMaze(const std::vector<std::vector<int>>& maze,
                       int x, int y,
                       const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& knownWalls);

private:
    bool wallExists(const std::pair<int, int>& a, const std::pair<int, int>& b,
                    const std::set<std::pair<std::pair<int, int>, std::pair<int, int>>>& knownWalls) const;
};

#endif
