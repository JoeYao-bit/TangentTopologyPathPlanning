//
// Created by yaozhuo on 2023/2/12.
//

#ifndef FREENAV_TOPO_DEPTH_FIRST_SEARCH_WITHOUT_EDGE_H
#define FREENAV_TOPO_DEPTH_FIRST_SEARCH_WITHOUT_EDGE_H

#include "search_path_with_node.h"

namespace freeNav::Topo {


    template <Dimension N>
    template<typename TARGET_TYPE>
    ExitCode GeneralGraphPathPlannerWithNode<N>::DepthFirstSearchWithNode(const PathPointWithNodePtrs<N> &init_set,
                                                                          RoadMapNodeTraitDeques<N> &final_set, // resulted path set
            //const IterationConstraints<N, TARGET_TYPE> &ics, // when dfs, only considering ics ?
                                                                          const TARGET_TYPE& target
    ) {
        auto & es = tg_->edges_;
        auto & ns = tg_->nodes_;
        global_minimum_path_length = MAX<PathLen>;

        if(init_set.empty()) {
            std::cerr << __FUNCTION__ << ": init_set.empty() " << std::endl;
            return ExitCode::ILLEGAL_INPUT_INITIAL_SET;
        }

        current_set_.reset();
        final_set.clear();
        /* 1, create initial set */
        //std::cout << " flag 0 " << std::endl;
        NodeId init_shortest_path_id = MAX<EdgeId>; // the shortest path in the initial path
        PathLen init_shortest_len = MAX<PathLen>, temp_len;
        for(auto& init_path : init_set) {
            // check whether start set reach target, accelerate the iteration in some cases
            //std::cout << " flag 1 " << std::endl;
            if(data_->close_node_to(init_path->last_node_, false, currentTime()) != MAX<NodeId>) {
                //std::cout << " flag 2 " << std::endl;
                const NodeId& pre_id = data_->close_node_to(init_path->last_node_, true, currentTime());
                const NodeId& next_id = data_->close_node_to(init_path->last_node_, false, currentTime());

                if(!isEdgeTransferLegal(tg_, ns[pre_id], init_path->last_node_, ns[next_id], tg_->etcs_)) continue;

                temp_len = data_->dist_to(init_path->last_node_, true,  currentTime())
                           + data_->dist_to(init_path->last_node_, false, currentTime());
                init_shortest_path_id = init_path->last_node_->sg_->node_id_;
                break;
            }

            closest_grid_ = findClosestFromSetToPoint<N>(target, init_path->last_node_->sg_->pt_);

            heuristic_length_ = data_->dist_to(init_path->last_node_, true, currentTime()) +
                                tg_->surface_processor_->distBetween(closest_grid_->pt_, init_path->last_node_->sg_->pt_);
            //std::cout << " flag 3.3 " << std::endl;
            current_set_.HeapInsert({heuristic_length_, withIndex<RoadMapNodeTraitPtr<N> > (init_path->last_node_,
                        data_->index_in_heap(init_path->last_node_, currentTime()))});
        }
        //std::cout << " flag 4 " << std::endl;
        bool reach_target_hyper = false;
        if(init_shortest_path_id != MAX<NodeId>)
        {
            //std::cout << "init reach target path length = " << init_shortest_len << " | " << init_shortest_path->printStr(true, true) << std::endl;
            final_set.push_back(NodeTransformToContinuousEdgesWithNode(tg_, data_, ns[init_shortest_path_id], currentTime()));
            return ExitCode::SUCCESS;
        }


        PathLen future_length = MAX<PathLen>, temp_future_length;
        int count = 0;
        NodeId future_node_id = MAX<NodeId>;
        bool reach_target = false;
        while(!current_set_.isEmpty()) {
            //std::cout << count << " th, size = " << current_set_.size() << std::endl;
            count ++;
            future_node_id = current_set_.HeapPopMin().second.val_->sg_->node_id_;
            * data_->index_in_heap(ns[future_node_id], currentTime()) = MAX<NodeId>;

            if(data_->is_expanded(ns[future_node_id], currentTime())) continue;
            data_->is_expanded(ns[future_node_id], currentTime()) = true;
            //std::cout << " next size " << tg_->nextNodes(ns[future_node_id]).size() << std::endl;
            for (const auto &candidate_node_id : tg_->nextNodes(ns[future_node_id])) {
                if(!isEdgeTransferLegal(tg_,
                                        ns[data_->close_node_to(ns[future_node_id], true, currentTime())],
                                        ns[future_node_id],
                                        ns[candidate_node_id], tg_->etcs_)) continue;

                // do not visit again ? cause loss legal path when use ETC
                //if(data_->dist_to(ns[candidate_node_id], true, currentTime()) != MAX<PathLen>) continue;

                temp_future_length = data_->dist_to(ns[future_node_id], true, currentTime()) +
                                     (ns[future_node_id]->sg_->pt_ - ns[candidate_node_id]->sg_->pt_).Norm();

                // TODO: allow update value in heap, when dist change to smaller value
                if(temp_future_length >= data_->dist_to(ns[candidate_node_id], true, currentTime())) continue;


                data_->dist_to(ns[candidate_node_id], true, currentTime()) = temp_future_length;
                data_->close_node_to(ns[candidate_node_id], true, currentTime()) = future_node_id;

                closest_grid_ = findClosestFromSetToPoint<N>(target, ns[candidate_node_id]->sg_->pt_);
                heuristic_length_ = temp_future_length +
                                    tg_->surface_processor_->distBetween(closest_grid_->pt_,
                                                                         ns[candidate_node_id]->sg_->pt_);
                auto & index_in_queue_ptr = data_->index_in_heap(ns[candidate_node_id], currentTime());
                // if already in queue, update index
                if(*index_in_queue_ptr != MAX<NodeId>) {
                    current_set_.updateValue(*index_in_queue_ptr, temp_future_length);
                } else
                    {
                    // if not in queue, insert to queue
                    current_set_.HeapInsert({heuristic_length_,
                        withIndex<RoadMapNodeTraitPtr<N> >(ns[candidate_node_id], index_in_queue_ptr)});
                }

                //std::cout << " add  node " << candidate_node_id << std::endl;
                if (data_->close_node_to(ns[candidate_node_id], false, currentTime()) != MAX<NodeId>) {

                    if(!isEdgeTransferLegal(tg_,
                                            ns[data_->close_node_to(ns[candidate_node_id], true, currentTime())],
                                            ns[candidate_node_id],
                                            ns[data_->close_node_to(ns[candidate_node_id], false, currentTime())],
                                            tg_->etcs_)) continue;

                    //std::cout << __FUNCTION__ << " reach target " << std::endl;
                    shortest_path_node_ = ns[candidate_node_id];
                    reach_target = true;
                    break;
                }
            }
            if(reach_target) {
                //std::cout << __FUNCTION__ << " reach target " << std::endl;
                final_set.clear();
                const auto& result_path = NodeTransformToContinuousEdgesWithNode(tg_, data_, shortest_path_node_, currentTime());
//                for(int i=1; i<result_path.size()-1; i++) {
//                    if(!isEdgeTransferLegal(tg_,
//                        result_path[i-1], result_path[i], result_path[i+1],
//                        tg_->etcs_)) {
//                        std::cout << " illegal transfer " << result_path[i-1]->sg_->pt_
//                                << "->" << result_path[i]->sg_->pt_
//                                << "->" << result_path[i+1]->sg_->pt_
//                                << std::endl;
//                    }
//                }
                final_set.push_back(result_path);
                reach_target = true;
                break;
            }
        }
        // get result path
        if(!final_set.empty()) { return ExitCode::SUCCESS; }
        else { return ExitCode::FAILED; }
    }


}

#endif //FREENAV_DEPTH_FIRST_SEARCH_WITHOUT_EDGE_H
