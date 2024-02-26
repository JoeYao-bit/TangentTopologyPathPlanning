//
// Created by yaozhuo on 2022/2/19.
//

#ifndef FREENAV_TOPO_POINT_TO_POINT_CONSTRAINTS_H
#define FREENAV_TOPO_POINT_TO_POINT_CONSTRAINTS_H

#include "../../algorithm/graph_construction/tangent_graph.h"
#include "../../freeNav-base/basic_elements/surface_process.h"

namespace freeNav::Topo {

    /* PTC: point to point constraint
    * necessary constraint of tangent line, it must be collision free */
    // NOTICE: current PTC_DoNotCrossObstacle cause loss of useful line
    template <Dimension N>
    bool PTC_DoNotCrossObstacle(RoadMapGraphPtr<N>& tg, const GridPtr<N>& tc1, const GridPtr<N>& tc2, IS_OCCUPIED_FUNC<N> is_occupied, const Pointis<N - 1>& neightbor) {
        return !LineCrossObstacle(tc1->pt_, tc2->pt_, is_occupied, neightbor);
    }

    template <Dimension N>
    bool PTC_LocalCrossPartially(RoadMapGraphPtr<N>& tg, const GridPtr<N>& tc1, const GridPtr<N>& tc2, IS_OCCUPIED_FUNC<N> is_occupied, const Pointis<N - 1>& neightbor) {
        if(LineCrossObstacleLocal(tc1->pt_, tc2, is_occupied)
        //&& LineCrossObstacleLocal(tc2->pt_, tc1, is_occupied)
        ) { return true; }
        else return false;
    }

    bool PTC_CrosserToObstacle_2D(RoadMapGraphPtr<2>& tg, const GridPtr<2>& tc1, const GridPtr<2>& tc2, IS_OCCUPIED_FUNC<2> is_occupied, const Pointis<1>& neightbor);

}

#endif //FREENAV_POINT_TO_POINT_CONSTRAINTS_H
