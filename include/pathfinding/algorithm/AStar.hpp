
#ifndef PATHFINDING_ASTAR_HPP
#define PATHFINDING_ASTAR_HPP

#include "pathfinding/Heuristic.hpp"
#include "pathfinding/PathFinder.hpp"
#include "pathfinding/TileBasedMap.hpp"

#include <algorithm>
#include <memory>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pathfinding {
    class AStar: public Pathfinder {

    private:
        struct Node;// forward declaration

    public:
        explicit AStar(std::unique_ptr<TileBasedMap> map, std::unique_ptr<Heuristic> heuristic = nullptr)
            : map(std::move(map)), heuristic(std::move(heuristic)),
              nodes(this->map->width() * this->map->height()) {

            for (int x = 0; x < this->map->width(); x++) {
                for (int y = 0; y < this->map->height(); y++) {
                    nodes[index(x, y)] = Node{{x, y}};
                }
            }
        }

        AStar& setMaxSearchDistance(int distance) {
            maxSearchDistance = distance;
            return *this;
        }
        AStar& setAllowDiagMovement(bool allow) {
            allowDiagMovement = allow;
            return *this;
        }

        std::optional<Path> findPath(const Coordinate& s, const Coordinate& t) override {

            // easy first check, if the destination is blocked, we can't get there
            if (map->blocked(s) || map->blocked(t)) {
                return std::nullopt;
            }

            for (auto& node : nodes) {
                node.reset();
            }

            std::unordered_set<Node*> closed;
            std::priority_queue<Node*> open;

            // initial state for A*. The closed group is empty. Only the starting
            // tile is in the open list and it's cost is zero, i.e. we're already there
            Node& startNode = nodes[index(s.x, s.y)];
            Node& targetNode = nodes[index(t.x, t.y)];

            startNode.cost = 0;
            startNode.depth = 0;
            targetNode.parent = nullptr;

            open.emplace(&startNode);

            // while we haven't found the goal and haven't exceeded our max search depth
            int maxDepth = 0;
            while ((maxDepth < maxSearchDistance) && (!open.empty())) {
                // pull out the first node in our open list, this is determined to
                // be the most likely to be the next step based on our heuristic
                Node* current = open.top();
                open.pop();

                if (current == &targetNode) {
                    break;
                }

                closed.emplace(current);

                // search through all the neighbours of the current node evaluating
                // them as next steps
                for (int x = -1; x < 2; x++) {
                    for (int y = -1; y < 2; y++) {
                        // not a neighbour, it's the current tile
                        if ((x == 0) && (y == 0)) {
                            continue;
                        }

                        // if we're not allowing diagonal movement then only
                        // one of x or y can be set
                        if (!allowDiagMovement) {
                            if ((x != 0) && (y != 0)) {
                                continue;
                            }
                        }

                        // determine the location of the neighbour and evaluate it
                        Coordinate p{x + current->xy.x, y + current->xy.y};

                        if (isValidLocation(s, p)) {
                            // the cost to get to this node is cost the current plus the
                            // movement cost to reach this node. Note that the heuristic value
                            // is only used in the sorted open list
                            Node& neighbor = nodes[index(p.x, p.y)];
                            const float nextStepCost = current->cost + getMovementCost(current->xy, p);

                            if (nextStepCost < neighbor.cost || neighbor.parent == nullptr) {
                                if (!closed.contains(&neighbor)) {
                                    neighbor.cost = nextStepCost;
                                    neighbor.heuristic = getHeuristicCost(p, t);
                                    maxDepth = std::max(maxDepth, neighbor.setParent(current));
                                    open.push(&neighbor);
                                }
                            }
                        }
                    }
                }
            }

            // since we've got an empty open list, or we've run out of search
            // there was no path. Just return null
            if (targetNode.parent == nullptr) {
                return std::nullopt;
            }

            // At this point we've definitely found a path so we can uses the parent
            // references of the nodes to find out way from the target location back
            // to the start recording the nodes on the way.
            Path path;
            for (const Node* n = &targetNode; n != &startNode; n = n->parent) {
                path.prependStep(n->xy);
            }
            path.prependStep(s);

            // that's it, we have our path
            return path;
        }

    private:
        /**
         * A single node in the search graph
         */
        struct Node {

            // The coordinate of the node
            Coordinate xy;

            float cost = std::numeric_limits<float>::max();
            /** The parent of this node, how we reached it in the search */
            Node* parent = nullptr;
            /** The heuristic cost of this node */
            float heuristic = 0;
            /** The search depth of this node */
            int depth = 0;

            int setParent(Node* p) {
                depth = p->depth + 1;
                this->parent = p;

                return depth;
            }

            void reset() {
                cost = std::numeric_limits<float>::max();
                parent = nullptr;
                heuristic = 0;
                depth = 0;
            }

            // used for sorting
            bool operator<(const Node& other) const {
                const auto f = heuristic + cost;
                const auto of = other.heuristic + other.cost;

                return f < of;
            }

            ~Node() = default;
        };

        std::unique_ptr<TileBasedMap> map;
        std::unique_ptr<Heuristic> heuristic;

        std::vector<Node> nodes;

        int maxSearchDistance = 100;
        bool allowDiagMovement = true;

        [[nodiscard]] int index(int x, int y) const {
            return y * map->width() + x;
        }

        static bool contains(const std::vector<Node*>& list, Node* node) {

            return std::ranges::find(list, node) != std::end(list);
        }

        static void removeFrom(std::vector<Node*>& list, Node* node) {

            std::erase_if(list, [&](Node* n) { return n == node; });
        }

        /**
         * Check if a given location is valid for the supplied mover
         *
         * @param c1 The starting x coordinate
         * @param c2 The coordinate of the location to check
         *
         * @return True if the location is valid
         */
        [[nodiscard]] bool isValidLocation(const Coordinate& c1, const Coordinate& c2) const {

            bool invalid = (c2.x < 0) || (c2.y < 0) || (c2.x >= map->width()) ||
                           (c2.y >= map->height());

            if ((!invalid) && ((c1.x != c2.x) || (c1.y != c2.y))) {
                invalid = map->blocked(c2);
            }

            return !invalid;
        }

        /**
         * Get the cost to move through a given location
         *
         * @param s The coordinate of the tile whose cost is being determined
         * @param t The coordinate of the target location
         *
         * @return The cost of movement through the given tile
         */
        [[nodiscard]] float getMovementCost(const Coordinate& s, const Coordinate& t) const {

            return map->getCost(s, t);
        }

        /**
         * Get the heuristic cost for the given location. This determines in which
         * order the locations are processed.
         *
         * @param s The coordinate of the tile whose cost is being determined
         * @param t The coordinate of the target location
         *
         * @return The heuristic cost assigned to the tile
         */
        [[nodiscard]] float getHeuristicCost(const Coordinate& s, const Coordinate& t) const {

            if (!heuristic) return 0;// Dijkstra's

            return heuristic->getCost(*map, s, t);
        }
    };
}// namespace pathfinding

#endif// PATHFINDING_ASTAR_HPP
