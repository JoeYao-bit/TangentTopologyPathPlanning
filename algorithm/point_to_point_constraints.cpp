//
// Created by yaozhuo on 2022/2/19.
//

#include "constraints/point_to_point_constraints.h"

namespace freeNav::Topo {

    bool PTC_CrosserToObstacle_2D(RoadMapGraphPtr<2>& tg, const GridPtr<2>& tc1, const GridPtr<2>& tc2, IS_OCCUPIED_FUNC<2> is_occupied, const Pointis<1>& neightbor) {
        const auto& obst = tc2->nearby_obst_offsets_[0];
        if(obst[0] == 1) {
            if(obst[1] == 1) {
                if(tc1->pt_[0] < tc2->pt_[0] && tc1->pt_[1] < tc2->pt_[1]) {
                    return false;
                } else return true;
            } else {
                if(tc1->pt_[0] < tc2->pt_[0] && tc1->pt_[1] > tc2->pt_[1]) {
                    return false;
                } else return true;
            }
        } else {
            if(obst[1] == 1) {
                if(tc1->pt_[0] > tc2->pt_[0] && tc1->pt_[1] < tc2->pt_[1]) {
                    return false;
                } else return true;
            } else {
                if(tc1->pt_[0] > tc2->pt_[0] && tc1->pt_[1] > tc2->pt_[1]) {
                    return false;
                } else return true;
            }
        }
    }

}
