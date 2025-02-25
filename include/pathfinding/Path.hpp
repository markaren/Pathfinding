// https://cokeandcode.com/tutorials/tilemap2.html

#ifndef PATHFINDING_PATH_HPP
#define PATHFINDING_PATH_HPP

#include "Coordinate.hpp"

#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace pathfinding {
    /**
     * A path determined by some path finding algorithm. A series of steps from
     * the starting location to the target location. This includes a step for the
     * initial location.
     */
    class Path {

    public:
        /**
         * Get the length of the path, i.e. the number of steps
         *
         * @return The number of steps in this path
         */
        [[nodiscard]] size_t length() const {

            return steps_.size();
        }

        /**
         * Get the step at a given index in the path
         *
         * @param index The index of the step to retrieve.
         *
         * @return The step information, the position on the map.
         */
        const Coordinate& operator[](size_t index) const {

            if (index >= steps_.size()) {
                throw std::runtime_error("Index out of bounds: " + std::to_string(index));
            }
            return steps_[index];
        }

        /**
         * Prepend a step to the path.
         *
         * @param c The coordinate of the new step
         */
        void prependStep(const Coordinate& c) {

            steps_.insert(steps_.begin(), c);
        }

        [[nodiscard]] const Coordinate& start() const {

            return steps_.front();
        }

        [[nodiscard]] const Coordinate& target() const {

            return steps_.back();
        }

        [[nodiscard]] std::vector<Coordinate> steps() const {
            return steps_;
        }

        [[nodiscard]] std::vector<Coordinate> closed() const {
            return closed_;
        }

        void setClosed(const std::vector<Coordinate>& closed) {
            closed_ = closed;
        }

        /**
         * Check if a given Coordinate is part of the Path
         * @param c The coordinate to check
         *
         * @return True if contains the coordinate, false otherwise
         */
        [[nodiscard]] bool contains(const Coordinate& c) const {

            return std::ranges::find(steps_, c) != std::end(steps_);
        }

    private:
        std::vector<Coordinate> steps_;
        std::vector<Coordinate> closed_;
    };
}// namespace pathfinding

#endif//PATHFINDING_PATH_HPP
