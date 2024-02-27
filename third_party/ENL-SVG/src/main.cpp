#include <iostream>
#include "../Pathfinding/Grid.h"
#include "../Pathfinding/Drawer.h"
#include "../Pathfinding/RandomGridGenerator.h"
#include "../Pathfinding/ENLSVGAlgorithm.h"
#include "../Pathfinding/PathfindingDataTypes.h"
double initial_time_ENLSVG, search_graph_time_ENLSVG;
int main() {
    int sizeX = 203;
    int sizeY = 154;

    Pathfinding::Grid grid(sizeX, sizeY);
    //Pathfinding::RandomGridGenerator::generateRandomGrid(grid, 0.5f);
    Pathfinding::RandomGridGenerator::generateAutomataGrid(grid, 0.5f, 5, 1.f);

    Pathfinding::ENLSVG::Algorithm enlsvg(grid);
    Pathfinding::ENLSVG::Memory memory(enlsvg);

    const int sx = 152;
    const int sy = 80;
    const int ex = 48;
    const int ey = 40;
    Pathfinding::ParentPtrs parentPtrs;

    for (size_t i=0;i<10000;++i) {
        //std::cout << "trial " << i << std::endl;
        enlsvg.computePath(memory, rand()%sizeX, rand()%sizeY, rand()%sizeX, rand()%sizeY, initial_time_ENLSVG, search_graph_time_ENLSVG);
    }
    Pathfinding::Path path = enlsvg.computePath(memory, sx, sy, ex, ey,
                                                initial_time_ENLSVG, search_graph_time_ENLSVG,&parentPtrs);

    //std::cout << "Path hops : " << path.size() << std::endl;
    Pathfinding::Drawer drawer(grid, 1);
    drawer.drawGrid(grid);
    //drawer.drawVisibilityGraph(enlsvg);
    drawer.drawParentPointers(parentPtrs);
    drawer.drawPath(path);
    drawer.drawPoint(sx, sy);
    drawer.drawPoint(ex, ey);

    drawer.save("outputImage.tga");
}