// https://cokeandcode.com/tutorials/tilemap2.html

#ifndef PATHFINDING_CLOSESTHEURISTIC_HPP
#define PATHFINDING_CLOSESTHEURISTIC_HPP

#include "pathfinding/Heuristic.hpp"

#include <cmath>


namespace pathfinding {
    /**
     * A heuristic that uses the tile that is closest to the target
     * as the next best tile.
     */
    class ClosestHeuristic: public Heuristic {

    public:
        float getCost(const TileBasedMap& map, const Coordinate& start, const Coordinate& target) override {
            const auto dx = static_cast<float>(target.x - start.x);
            const auto dy = static_cast<float>(target.y - start.y);

            const auto result = std::sqrt((dx * dx) + (dy * dy));
            return result;
        }
    };
}// namespace pathfinding

#endif//PATHFINDING_CLOSESTHEURISTIC_HPP
