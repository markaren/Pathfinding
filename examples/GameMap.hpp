#ifndef PATHFINDING_GAMEMAP_HPP
#define PATHFINDING_GAMEMAP_HPP

#include "pathfinding/TileBasedMap.hpp"

#include <string>
#include <utility>
#include <vector>

class GameMap : public TileBasedMap {
public:
    explicit GameMap(std::vector<std::string> data)
        : data_(std::move(data)) {
        height_ = data_.size();

        // check that width is consistent
        const unsigned width = data_[0].size();
        for (int i = 1; i < data_.size(); i++) {
            if (data_[i].size() != width) {
                throw std::runtime_error("Input breadth mismatch!");
            }
        }
        width_ = width;
    }

    [[nodiscard]] unsigned int width() const override {
        return width_;
    }

    [[nodiscard]] unsigned int height() const override {
        return height_;
    }

    [[nodiscard]] char get(int x, int y) const {
        return data_[y][x];
    }

    [[nodiscard]] bool blocked(const Coordinate &v) const override {
        const char c = get(v.x, v.y);
        const bool blocked = (c == '1');
        return blocked;
    }

    [[nodiscard]] float getCost(const Coordinate &start, const Coordinate &target) const override {
        // Calculate the absolute difference in x and y coordinates
        const auto dx = abs(target.x - start.x);
        const auto dy = abs(target.y - start.y);

        // Higher cost for diagonal movements
        float cost = 1.0f;
        if (dx > 0 && dy > 0) {
            cost = 1.5f; // Adjust this value as needed
        }

        return cost;
    }

    [[nodiscard]] std::vector<std::string> data() const {
        return data_;
    }

private:
    unsigned int width_, height_;
    std::vector<std::string> data_;
};


#endif//PATHFINDING_GAMEMAP_HPP
