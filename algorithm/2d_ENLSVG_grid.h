//
// Created by yaozhuo on 2022/3/25.
//

#ifndef FREENAV_TOPO_2D_ENLSVG_GRID_H
#define FREENAV_TOPO_2D_ENLSVG_GRID_H

#include "../third_party/ENL-SVG/Pathfinding/ENLSVG.h"
#include "../freeNav-base/basic_elements/point.h"

Pathfinding::Grid getENL_SVG_Module(freeNav::DimensionLength* dimension, freeNav::IS_OCCUPIED_FUNC<2> f1);

#endif //FREENAV_2D_ENLSVG_GRID_H
