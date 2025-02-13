// https://cokeandcode.com/tutorials/tilemap2.html

#ifndef PATHFINDING_MANHATTANHEURISTIC_HPP
#define PATHFINDING_MANHATTANHEURISTIC_HPP

#include "pathfinding/Heuristic.hpp"

#include <cmath>

namespace pathfinding {
    /**
     * A heuristic that drives the search based on the Manhattan distance
     * between the current location and the target
     */
    class ManhattanHeuristic: public Heuristic {

    public:
        float getCost(const TileBasedMap& map, const Coordinate& start, const Coordinate& target) override {
            return static_cast<float>(std::abs(start.x - target.x) + std::abs(start.y - target.y));
        }
    };

}// namespace pathfinding

#endif//PATHFINDING_MANHATTANHEURISTIC_HPP
