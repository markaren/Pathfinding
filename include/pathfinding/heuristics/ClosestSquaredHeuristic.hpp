// https://cokeandcode.com/tutorials/tilemap2.html

#ifndef PATHFINDING_CLOSESTSQHEURISTIC_HPP
#define PATHFINDING_CLOSESTSQHEURISTIC_HPP

#include "pathfinding/Heuristic.hpp"

namespace pathfinding {
    /**
     * A heuristic that uses the tile that is closest to the target
     * as the next best tile. In this case the sqrt is removed
     * and the distance squared is used instead
     */
    class ClosestSquaredHeuristic: public Heuristic {

    public:
        float getCost(const TileBasedMap& map, const Coordinate& start, const Coordinate& target) override {
            const auto dx = static_cast<float>(target.x - start.x);
            const auto dy = static_cast<float>(target.y - start.y);

            const float result = (dx * dx) + (dy * dy);
            return result;
        }
    };
}// namespace pathfinding

#endif//PATHFINDING_CLOSESTSQHEURISTIC_HPP
