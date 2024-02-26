//
// Created by yaozhuo on 2023/2/25.
//

#ifndef FREENAV_TOPO_NODE_ITERATION_CONSTRAINTS_H
#define FREENAV_TOPO_NODE_ITERATION_CONSTRAINTS_H

//#include "rim_jump/graph_construction/tangent_graph.h"
#include "../../algorithm/online_search/path_tree_with_node.h"

namespace freeNav::Topo {


    template<Dimension N, typename TARGET_TYPE>
    using NodeIterationConstraints = std::vector<bool (*)(RoadMapGraphPtr<N> &tg,
                                                          DynamicDataOfSearchWithNodePtr<N> &data,
                                                          const PathPointWithNodePtr<N> &current_path,
                                                          const RoadMapNodeTraitPtr<N> &candidate,
                                                          const TARGET_TYPE &target)>;

    int determinant(const int& v1, const int& v2, const int& v3, const int& v4);

    bool intersect3(const Pointi<2>& aa, const Pointi<2>& bb, const Pointi<2>& cc, const Pointi<2>& dd);

    /* a path shouldn't visit the same tangent point more than once
 * if there is IC_GlobalMinimumPathLength, no need for IC_NoLoop
 * TODO: there may be loop caused by initial start path and initial target path , when search distinctive paths
 * */
    template <typename TARGET_TYPE>
    bool NIC_NoLoop                (RoadMapGraphPtr<2> &tg,
                                    DynamicDataOfSearchWithNodePtr<2> &data,
                                    const PathPointWithNodePtr<2> &current_path,
                                    const RoadMapNodeTraitPtr<2> &candidate,
                                    const TARGET_TYPE &target) {
        PathPointWithNodePtr<2> pre = current_path;
        while(pre != nullptr) {
            if(pre->last_node_->sg_->id_ == candidate->sg_->id_) {
                return false;
            }
            pre = pre->from();
        }
        // check line cross
        //TODO:  transfer this to pre-computation, but this is very time consuming
        //     need n(n+1)/2  check, n = count of edges
        //     this reduce the cost of IC_GlobalMinimumPathLength
        //     but too slow
        //if(pre!=nullptr && IsEdgeCrossEdge(pre->last_edge_, candidate)) return false;
        if(current_path->from() == nullptr) return true;
        pre = current_path->from();
        auto pre_of_pre = pre->from();
        auto & node_1 = current_path->last_node_, node_2 = candidate;
        while(pre_of_pre != nullptr) {
            //if(IsEdgeCrossEdge(node_1, node_2, pre->last_node_, pre_of_pre->last_node_)) { return false; }
            if(intersect3(node_1->sg_->pt_, node_2->sg_->pt_, pre->last_node_->sg_->pt_, pre_of_pre->last_node_->sg_->pt_)) { return false; }
            pre = pre->from();
            if(pre != nullptr) pre_of_pre = pre->from();
            else break;
        }
        return true;
    }



    template <typename TARGET_TYPE>
    bool NIC_NoStraightExtend      (RoadMapGraphPtr<2> &tg,
                                    DynamicDataOfSearchWithNodePtr<2> &data,
                                    const PathPointWithNodePtr<2> &current_path,
                                    const RoadMapNodeTraitPtr<2> &candidate,
                                    const TARGET_TYPE &target) {
        if(current_path->from() == nullptr) return true;
        // as ENLSVG line scanner stop extend when hit obstacle
        if(current_path->from()->from() == nullptr) return true;

        PointF<2> k1 = PointiToPointF(candidate->sg_->pt_ - current_path->last_node_->sg_->pt_);
        PointF<2> k2 = PointiToPointF(current_path->last_node_->sg_->pt_ - current_path->from()->last_node_->sg_->pt_);
//        std::cout << " candidate->sg_->pt_ " << candidate->sg_->pt_ << std::endl;
//        std::cout << " current_path->last_node_->sg_->pt_ " << current_path->last_node_->sg_->pt_ << std::endl;
//        std::cout << " current_path->from()->last_node_->sg_->pt_ " << current_path->from()->last_node_->sg_->pt_ << std::endl;
//
//        std::cout << " k1 = " << k1 << " / " << k2 << std::endl;
        if(k1[1] != 0 && k2[1] != 0) {
            if((k1[0] / k1[1]) == (k2[0] / k2[1])) return false;
        } else if(k1[1] == 0 && k2[1] == 0) { return false; }
        return true;
    }

}
#endif //FREENAV_NODE_ITERATION_CONSTRAINTS_H
