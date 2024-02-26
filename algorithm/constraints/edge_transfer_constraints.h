//
// Created by yaozhuo on 2022/2/19.
//

#ifndef FREENAV_TOPO_EDGE_TRANSFER_CONSTRAINTS_H
#define FREENAV_TOPO_EDGE_TRANSFER_CONSTRAINTS_H
#include "../../algorithm/graph_construction/tangent_graph.h"
#include "../../freeNav-base/basic_elements/surface_process.h"

namespace freeNav::Topo {

    /* edge transfer constraints
     * if a edge connect to no other edge, it and the two ends of it are both illegal
     * */

    // for 2d ENL-SVG
    bool ETC_IC_GetCloserToObstacle_ENLSVG(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3);


    template <Dimension N>
    bool ETC_NotLookBack(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitPtr<N>& edge1, const RoadMapEdgeTraitPtr<N>& edge2) {
        return angleNotMinorThan90(edge1->from_->sg_->pt_, edge2->from_->sg_->pt_, edge2->to_->sg_->pt_);
    }

    // cause missing of tangent line when use ENL-SVG style collision check and cubic line of sight check
    // because the turning angle may greater than 90 degree
    template <Dimension N>
    bool ETC_NotLookBack3(RoadMapGraphPtr<N>& tg, const RoadMapNodePtr<N>& node1, const RoadMapNodePtr<N>& node2, const RoadMapNodePtr<N>& node3) {
        return angleNotMinorThan90(node1->sg_->pt_, node2->sg_->pt_, node3->sg_->pt_);
    }

    bool ETC_Taut_2D(RoadMapGraphPtr<2>& tg, const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3);

    bool ETC_ENLSVG_Taut_2D(RoadMapGraphPtr<2>& tg, const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3);

    bool ETC_NotLookBack3_2D(RoadMapGraphPtr<2>& tg, const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3);

    template <Dimension N>
    bool ETC_NoUselessPoint(RoadMapGraph<N>& tg, const RoadMapEdgeTraitPtr<N>& edge1, const RoadMapEdgeTraitPtr<N>& edge2) {
        return edge1->from_->split_edges_.find(edge2->to_->sg_->id_) == edge1->from_->split_edges_.end();
    }

    template <Dimension N>
    bool ETC_NoUselessPoint3(RoadMapGraphPtr<N>& tg, const RoadMapNodePtr<N>& node1, const RoadMapNodePtr<N>& node2, const RoadMapNodePtr<N>& node3) {
        //return node1->split_edges_.find(node3->sg_->id_) == node1->split_edges_.end();
        for(const auto& edge_temp_id : node1->nextEdges(true)) {
            if(tg->nodes_[tg->edges_[edge_temp_id]->nextNode(true)]->sg_->id_ == node3->sg_->id_) {
                return false;
            }
        }
        return true;
    }

    template <Dimension N>
    bool ETC_IC_GetCloserToObstacle(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitPtr<N>& edge1, const RoadMapEdgeTraitPtr<N>& edge2) {
        Pointi<N> current = edge2->from_->sg_->pt_, next = edge2->to_->sg_->pt_,
                pre = edge1->from_->sg_->pt_;
        if(next == pre) return false;
        double angle1 = angleBetweenThreePoints(pre, current, next);
        Pointd<N> new_line = (next - current).Normalize();
        Pointd<N> pre_line = (pre  - current).Normalize();
        Pointd<N> medium = new_line + pre_line;
        for(const auto& obst : edge2->from_->sg_->nearby_obst_pts_) {
            Pointi<N> to_obst = obst - current;
            double val1 = medium * to_obst;
            if(val1 < 0) continue;
            double angle2 = acos(val1/(medium.Norm()*to_obst.Norm()));
            if(2*angle2 < angle1 + 1e-9) return true;
        }
        return false;
    }

    template <Dimension N>
    bool ETC_IC_GetCloserToObstacle3(RoadMapGraphPtr<N>& tg, const RoadMapNodePtr<N>& node1, const RoadMapNodePtr<N>& node2, const RoadMapNodePtr<N>& node3) {
        Pointi<N> current = node2->sg_->pt_, next = node3->sg_->pt_, pre = node1->sg_->pt_;
        if(next == pre) return false;
        double angle1 = angleBetweenThreePoints(pre, current, next);
        Pointd<N> new_line = (next - current).Normalize();
        Pointd<N> pre_line = (pre  - current).Normalize();
        Pointd<N> medium = new_line + pre_line;
        for(const auto& obst : node2->sg_->nearby_obst_pts_) {
            Pointi<N> to_obst = obst - current;
            double val1 = medium * to_obst;
            if(val1 < 0) continue;
            double angle2 = acos(val1/(medium.Norm()*to_obst.Norm()));
            if(2*angle2 < angle1 + 1e-9) return true;
        }
        return false;
    }

}

#endif //FREENAV_EDGE_TRANSFER_CONSTRAINTS_H
