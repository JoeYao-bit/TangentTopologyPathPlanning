#include <iostream>
#include "../Pathfinding/Grid.h"
#include "../Pathfinding/RandomNumberGenerator.h"

namespace Pathfinding {

Grid::Grid(int sizeX, int sizeY): sizeX(sizeX), sizeY(sizeY), totalSize(sizeX*sizeY) {
    blocked.resize(sizeX*sizeY);
}

void Grid::printGrid() const {
    for (int y=0;y<sizeY;++y) {
        for (int x=0;x<sizeX;++x) {
            std::cout << (isBlockedRaw(x, y) ? "X" : " ");
        }
        std::cout << std::endl;
    }
}

}