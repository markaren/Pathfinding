# Pathfinding

C++ implementation of the A* search algorithm.

The code found here is a port of [this](https://cokeandcode.com/tutorials/tilemap2.html) Java implementation by Kevin Glass.


## Consuming package with CMake FetchContent

```cmake
include(FetchContent)
set(PATHFINDING_BUILD_EXAMPLES OFF)
FetchContent_Declare(
    pathfinding
    GIT_REPOSITORY https://github.com/markaren/Pathfinding.git
    GIT_TAG tag_or_commit_hash
)
FetchContent_MakeAvailable(pathfinding)
#...
target_link_libraries(mytarget PUBLIC pathfinding)
```
