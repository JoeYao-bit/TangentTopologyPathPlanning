//
// Created by yaozhuo on 2022/3/25.
//

#include "2d_ENLSVG_grid.h"
#include "sys/time.h"

Pathfinding::Grid getENL_SVG_Module(freeNav::DimensionLength* dimension, freeNav::IS_OCCUPIED_FUNC<2> f1) {
    /* 1, set grid map */
    Pathfinding::Grid grid(dimension[0], dimension[1]);
    freeNav::Pointi<2> pt;
    for(int x=0; x<dimension[0]; x++) {
        for(int y=0; y<dimension[1]; y++) {
            pt[0] = x, pt[1] = y;
            if(f1(pt)) {
                grid.setBlocked(x, y, true);
            } else {
                grid.setBlocked(x, y, false);
            }
        }
    }

    // 3, return algorithm
    return grid;
}