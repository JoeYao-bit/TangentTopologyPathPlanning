//
// Created by yaozhuo on 2023/2/6.
//

#ifndef FREENAV_TOPO_DEPTH_FIRST_SEARCH_WITH_EDGE_H
#define FREENAV_TOPO_DEPTH_FIRST_SEARCH_WITH_EDGE_H

#include "search_path_with_edge.h"

namespace freeNav::Topo {


    template <Dimension N>
    template<typename TARGET_TYPE>
    ExitCode GeneralGraphPathPlannerWithEdge<N>::DepthFirstSearchWithEdge(const PathPointWithEdgePtrs<N> &init_set,
                                                                          RoadMapEdgeTraitDeques<N> &final_set, // resulted path set
            //const IterationConstraints<N, TARGET_TYPE> &ics, // when dfs, only considering ics ?
                                           const TARGET_TYPE& target
    ) {
        auto & es = tg_->edges_;
        auto & ns = tg_->nodes_;
        auto & hes = tg_->hyper_edges_;
        global_minimum_path_length = MAX<PathLen>;

        if(init_set.empty()) {
            std::cerr << __FUNCTION__ << ": init_set.empty() " << std::endl;
            return ExitCode::ILLEGAL_INPUT_INITIAL_SET;
        }

        current_set_.reset();
        final_set.clear();
        /* 1, create initial set */
        //std::cout << " flag 0 " << std::endl;
        EdgeId init_shortest_path_id = MAX<EdgeId>; // the shortest path in the initial path
        PathLen init_shortest_len = MAX<PathLen>, temp_len;
        for(auto& init_path : init_set) {
            // check whether start set reach target, accelerate the iteration in some cases
            //std::cout << " flag 1 " << std::endl;
            if(isReachTarget(init_path->last_edge_, true, currentTime())) {
                //std::cout << " flag 2 " << std::endl;
                // TODO: ensure legal edge to edge transfer, current only use shortest to keep legal transfer
                //if(!is_edge && !isNodeReachTargetLegal(start_edge.second->current_on_edge(is_edge))) continue;
                temp_len = data_->dist_to(init_path->last_edge_, true, true, currentTime())
                           + data_->dist_to(init_path->last_edge_, false, true, currentTime());
                init_shortest_path_id = init_path->last_edge_->edgeId();
                break;
//                    if(temp_len < init_shortest_len) {
//                        // find the shortest connect in the init set
//                        init_shortest_path = start_edge;
//                        init_shortest_len = temp_len;
//                    }
            }
            //std::cout << " flag 3 " << std::endl;
            //if(!start_edge->isHyperLoopEdge()) continue;
            //std::cout << " flag 3.1 " << std::endl;
            closest_grid_ = findClosestFromSetToPoint<N>(target, ns[init_path->last_edge_->nextNode(true)]->sg_->pt_);
            //std::cout << " flag 3.2 " << std::endl;
            heuristic_length_ = data_->dist_to(init_path->last_edge_, true, true, currentTime()) +
                                //(closest_grid_->pt_ - init_path->last_edge_->next_node(true)->sg_->pt_).Norm();
                                tg_->surface_processor_->distBetween(closest_grid_->pt_, ns[init_path->last_edge_->nextNode(true)]->sg_->pt_);
            //std::cout << " flag 3.3 " << std::endl;
            auto & index_in_queue_ptr = data_->index_in_heap(init_path->last_edge_, currentTime());

            auto temp_pair = withIndex<RoadMapEdgeTraitPtr<N> >(init_path->last_edge_, index_in_queue_ptr);

            current_set_.HeapInsert({heuristic_length_, temp_pair});
        }
        //std::cout << " flag 4 " << std::endl;
        bool reach_target_hyper = false;
        if(init_shortest_path_id != MAX<EdgeId>)
        {
            //std::cout << "init reach target path length = " << init_shortest_len << " | " << init_shortest_path->printStr(true, true) << std::endl;
            final_set.push_back(NodeTransformToContinuousEdgesWithEdge(tg_, data_, es[init_shortest_path_id], true, currentTime()));
            return ExitCode::SUCCESS;
        }
        PathLen future_length = MAX<PathLen>, temp_future_length;
        int count = 0;
        EdgeId future_edge_id = MAX<EdgeId>;
        bool reach_target = false;
        while(!current_set_.isEmpty()) {
            //std::cout << count << " th, size = " << current_set_.size() << std::endl;
            count ++;
            future_edge_id = (current_set_.HeapPopMin().second.val_)->edgeId();

            if(es[future_edge_id]->isHyperLoopEdge() //&& !reach_target_hyper
                    ) {
//                            if(pre_tpt->last_edge_->is_marked(edge_mode_, true, tg_.currentTime()))
//                                std::cout << "first start hyper " << std::endl;
//                            if(!all_reach_start_hyper) {
//                                current_set.push_back(pre_tpt);
//                                continue;
//                            }
                if(!data_->is_marked(es[future_edge_id], true, false, currentTime())) {
                    for (const auto &candidate_edge_id : es[future_edge_id]->nextHyperLoopEdges(true)) {
                        //std::cout << " reach hyper without" << std::endl;
                        const auto& retv = getNextHyperEdge(es[future_edge_id], hes[candidate_edge_id], target);
                        if(retv != MAX<EdgeId>) { reach_target = true; break; }
                    }
                } else {
                    //reach_target_hyper = true;
                    //std::cout << " reach hyper target " << std::endl;
                    for (const auto &candidate_edge_id : tg_->nextEdges(es[future_edge_id], true, true)) {
                        if(!data_->is_marked(es[candidate_edge_id], true, false, currentTime())) continue;
                        const auto& retv = getNextEdge(es[future_edge_id], es[candidate_edge_id], target);
                        if(retv != MAX<EdgeId>) { reach_target = true; break; }
                    }
                }
            } else
            {
                for (const auto &candidate_edge_id : tg_->nextEdges(es[future_edge_id], true, true)) {
                    // only considering un-loop edges that mark from start or target
                    if(data_->is_marked(es[candidate_edge_id], true, true, currentTime())
                       || data_->is_marked(es[candidate_edge_id], true, false, currentTime()))
                    {
                        const auto& retv = getNextEdge(es[future_edge_id], es[candidate_edge_id], target);
                        if(retv != MAX<EdgeId>) { reach_target = true; break; }
                    }
                }
            }
            if(reach_target) {
                //std::cout << __FUNCTION__ << " reach target " << std::endl;
                final_set.clear();
                const auto& result_path = NodeTransformToContinuousEdgesWithEdge(tg_, data_, shortest_path_edge_, true, currentTime());
                final_set.push_back(result_path);
                reach_target = true;
                //printPathLevel(result_path);
                break;
            }
        }
        // get result path
        if(!final_set.empty()) { return ExitCode::SUCCESS; }
        else { return ExitCode::FAILED; }
    }

    template <Dimension N>
    template<typename TARGET_TYPE>
    EdgeId GeneralGraphPathPlannerWithEdge<N>::getNextHyperEdge(const RoadMapEdgeTraitPtr<N> &future_edge_ptr, const HyperLoopEdgeWithLengthPtr<N> & next_edge, const TARGET_TYPE& target) {
        PathLen temp_future_length = data_->dist_to(future_edge_ptr, true, true, currentTime()) + next_edge->length();


        auto & es = tg_->edges_;
        auto & ns = tg_->nodes_;
        auto & hes = tg_->hyper_edges_;
        // never visit each node more than onece
        if(data_->dist_to(es[next_edge->edgeHeadId()], true, edge_mode_, currentTime()) != MAX<PathLen>) { return MAX<EdgeId>; }

        data_->dist_to(es[next_edge->edgeHeadId()], true, true, currentTime()) = temp_future_length;
        data_->close_edge_to(es[next_edge->edgeHeadId()], true, true, currentTime()) = future_edge_ptr->edgeId();
        if(!edge_mode_) {
            data_->dist_to(es[next_edge->edgeHeadId()], true, false, currentTime()) = temp_future_length;
        }

        closest_grid_ = findClosestFromSetToPoint<N>(target, ns[es[next_edge->edgeHeadId()]->nextNode(true)]->sg_->pt_);
        heuristic_length_ = temp_future_length +
                            //(closest_grid_->pt_ - next_edge->edge_->next_node(true)->sg_->pt_).Norm();
                            tg_->surface_processor_->distBetween(closest_grid_->pt_, ns[es[next_edge->edgeHeadId()]->nextNode(true)]->sg_->pt_);
        auto & index_in_queue_ptr = data_->index_in_heap(es[next_edge->edgeHeadId()], currentTime());

        auto temp_pair = withIndex<RoadMapEdgeTraitPtr<N> >(es[next_edge->edgeHeadId()], index_in_queue_ptr);

        current_set_.HeapInsert({heuristic_length_, temp_pair});
        if(isReachTarget(es[next_edge->edgeHeadId()], true, currentTime())) {
            //std::cout << __FUNCTION__ << " reach target " << std::endl;
            shortest_path_edge_ = es[next_edge->edgeHeadId()];
            return next_edge->edgeHeadId();
        }
        return MAX<EdgeId>;
    }

    template <Dimension N>
    template<typename TARGET_TYPE>
    EdgeId GeneralGraphPathPlannerWithEdge<N>::getNextEdge(const RoadMapEdgeTraitPtr<N> &future_edge_ptr, const RoadMapEdgeTraitPtr<N> & next_edge, const TARGET_TYPE& target) {
        auto & es = tg_->edges_;
        auto & ns = tg_->nodes_;
        auto & hes = tg_->hyper_edges_;
        PathLen temp_future_length = data_->dist_to(future_edge_ptr, true, true, currentTime()) + next_edge->length();

        if(data_->dist_to(next_edge, true, edge_mode_, currentTime()) != MAX<PathLen>) { return MAX<EdgeId>; }

        data_->dist_to(next_edge, true, true, currentTime()) = temp_future_length;
        data_->close_edge_to(next_edge, true, true, currentTime()) = future_edge_ptr->edgeId();
        if(!edge_mode_) {
            data_->dist_to(next_edge, true, false, currentTime()) = temp_future_length;
//                next_edge->close_edge_to(true, false, tg_.currentTime()) = future_edge_ptr;
        }

        closest_grid_ = findClosestFromSetToPoint<N>(target, ns[next_edge->nextNode(true)]->sg_->pt_);
        heuristic_length_ = temp_future_length +
                            //(closest_grid_->pt_ - next_edge->next_node(true)->sg_->pt_).Norm();
                            tg_->surface_processor_->distBetween(closest_grid_->pt_, ns[next_edge->nextNode(true)]->sg_->pt_);
        auto & index_in_queue_ptr = data_->index_in_heap(next_edge, currentTime());

        auto temp_pair = withIndex<RoadMapEdgeTraitPtr<N> >(next_edge, index_in_queue_ptr);

        current_set_.HeapInsert({heuristic_length_, temp_pair});
        if(isReachTarget(next_edge, true, currentTime())) {
            //std::cout << __FUNCTION__ << " reach target " << std::endl;
            shortest_path_edge_ = next_edge;
            return next_edge->edgeId();
        }
        return MAX<EdgeId>;
    }

    // TODO: DFS without edge pre computation

};

#endif //FREENAV_DEPTH_FIRST_SEARCH_WITH_EDGE_H
