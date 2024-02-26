//
// Created by yaozhuo on 2023/2/12.
//

#ifndef FREENAV_TOPO_CREATE_INITIAL_PATHS_WITHOUT_EDGE_H
#define FREENAV_TOPO_CREATE_INITIAL_PATHS_WITHOUT_EDGE_H

#include "search_path_with_node.h"

namespace freeNav::Topo {

    template <Dimension N>
    PathPointWithNodePtrs<N> GeneralGraphPathPlannerWithNode<N>::createInitPathPtrsWithNode(const Pointi<N>& start,
                                                                               bool is_from_start,
                                                                               const PointTransferConstraints<N>& ptcs,
                                                                               const EdgeTransferConstraints3<N>& etcs) {
        // 1, check whether start is an existing tangent node
        bool new_node = false;
        RoadMapEdgeTraitPtrs<N> init_edges;
        const auto& dim = tg_->surface_processor_->getDimensionInfo();
        Id start_id = PointiToId(start, dim);
        RoadMapNodeTraitPtr<N> current_node_ptr = nullptr;
        //RoadMapEdgeTraitPtrs<N> retv;
        PathPointWithNodePtrs<N> retv;
        int taut_count = 0;

        auto & ns = tg_->nodes_;
        auto & es = tg_->edges_;
        auto & hes = tg_->hyper_edges_;
        auto & dns = data_->node_dynamic_datas_;

        //std::cout << "flag 1 " << std::endl;

        // 3, create new node
        current_node_ptr = std::make_shared<RoadMapNodeTrait<N>>();
        if(tg_->surface_processor_->grid_map_[start_id] == nullptr || tg_->surface_processor_->grid_map_[start_id]->node_id_ == MAX<NodeId> ) {
            current_node_ptr->sg_ = std::make_shared<Grid<N> >();
            current_node_ptr->sg_->id_ = start_id;
            current_node_ptr->sg_->pt_ = start;
            current_node_ptr->sg_->node_id_ = tg_->nodes_.size(); // set new node id
            new_node = true;
            tg_->nodes_.push_back(current_node_ptr); // insert new node to all nodes
            dns.push_back(std::make_shared<NodeDynamicDataWithNode>());
            //std::cout << "flag 1.8 " << std::endl;
        } else {
            current_node_ptr->sg_ = tg_->surface_processor_->grid_map_[start_id];
        }
        //std::cout << " current_node_ptr " << current_node_ptr->sg_->node_id_ << std::endl;
        //std::cout << "flag 1.9 " << std::endl;
        data_->dist_to(current_node_ptr, is_from_start, currentTime()) = 0;
        data_->is_expanded(current_node_ptr, currentTime()) = true;

        //std::cout << "flag 2 " << std::endl;

        PathPointWithNodePtr<N> new_path_ptr = nullptr;
        new_path_ptr = std::make_shared<PathPointWithNode<N> >(tg_, nullptr, current_node_ptr);


        // 4, create new edges of new nodes
#if 1
        GridPtrs<N> tcs = tg_->surface_processor_->getVisibleTangentCandidates(start);
        bool is_legal = true;
        for(const auto& tc : tcs) {
            if (!isPointTransferLegal(tg_, current_node_ptr->sg_, tc, tg_->surface_processor_->is_occupied_,
                                      tg_->surface_processor_->getNeighbor(), ptcs)) { continue; }

            PathLen l1 = (current_node_ptr->sg_->pt_ - ns[tc->node_id_]->sg_->pt_).Norm();
            data_->dist_to(ns[tc->node_id_], is_from_start, currentTime()) = l1;
            data_->close_node_to(ns[tc->node_id_], is_from_start, currentTime()) = current_node_ptr->sg_->node_id_;

            auto next_path_ptr = std::make_shared<PathPointWithNode<N> >(tg_, new_path_ptr, ns[tc->node_id_]);
            retv.push_back(next_path_ptr);
        }
#else
        //std::cout << " flag 1 " << std::endl;
        for(const auto& tc : tg_->surface_processor_->surface_grids_) {

            if(tg_->surface_processor_->grid_map_[tc->id_] == nullptr) continue;
            //std::cout << " flag 2 " << std::endl;
            if(tg_->surface_processor_->grid_map_[tc->id_]->node_id_ == MAX<NodeId>) continue;
            //std::cout << " flag 3 " << std::endl;
            if(!isPointTransferLegal(tg_, current_node_ptr->sg_, tc, tg_->surface_processor_->is_occupied_,
                                      tg_->surface_processor_->getNeighbor(), ptcs)) { continue; }
            //std::cout << " flag 4 " << std::endl;
            if(tg_->surface_processor_->lineCrossObstacle(current_node_ptr->sg_->pt_, tc->pt_)) continue;
            //std::cout << " flag 5 " << std::endl;

            PathLen l1 = (current_node_ptr->sg_->pt_ - ns[tc->node_id_]->sg_->pt_).Norm();
            data_->dist_to(ns[tc->node_id_], is_from_start, currentTime()) = l1;
            data_->close_node_to(ns[tc->node_id_], is_from_start, currentTime()) = current_node_ptr->sg_->node_id_;

            auto next_path_ptr = std::make_shared<PathPointWithNode<N> >(tg_, new_path_ptr, ns[tc->node_id_]);
            retv.push_back(next_path_ptr);
        }
#endif
        return retv;
    }

}

#endif //FREENAV_CREATE_INITIAL_PATHS_WITHOUT_EDGE_H
