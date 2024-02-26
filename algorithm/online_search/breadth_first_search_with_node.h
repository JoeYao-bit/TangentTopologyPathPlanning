//
// Created by yaozhuo on 2023/2/12.
//

#ifndef FREENAV_BREADTH_FIRST_SEARCH_WITHOUT_EDGE_H
#define FREENAV_BREADTH_FIRST_SEARCH_WITHOUT_EDGE_H

#include "search_path_with_node.h"

namespace freeNav::Topo {


    template<Dimension N>
    template<typename TARGET_TYPE>
    ExitCode GeneralGraphPathPlannerWithNode<N>::BreadthFirstSearchWithNode(const PathPointWithNodePtrs <N> &init_set,
                                                                            const NodeIterationConstraints <N, TARGET_TYPE> &ics, // when dfs, only considering ics ?
                                                                            RoadMapNodeTraitDeques <N> &final_set, // resulted path set
                                                                            const TARGET_TYPE &target
    ) {
        auto & es = tg_->edges_;
        auto & ns = tg_->nodes_;
        global_minimum_path_length = MAX<PathLen>;

        if(init_set.empty()) {
            std::cerr << __FUNCTION__ << ": init_set.empty() " << std::endl;
            return ExitCode::ILLEGAL_INPUT_INITIAL_SET;
        }

        current_set_.reset();
        std::priority_queue<PathPointWithNodePtr<N>, PathPointWithNodePtrs<N>, PathPointWithNodePtrCompare<N> > buffer_set;
        final_set.clear();
        /* 1, create initial set */
        //std::cout << " flag 0 " << std::endl;
        PathPointWithNodePtr<N> shortest_path = nullptr; // the shortest path in the initial path
        PathLen init_shortest_len = MAX<PathLen>, temp_len;
        PathPointWithNodePtrs<N> current_set = init_set, next_set;


        PathLen future_length = MAX<PathLen>, temp_future_length;
        int count = 0;
        NodeId future_node_id = MAX<NodeId>;
        bool reach_target = false;
        int reach_target_count = 0;
        struct timeval tv_pre_temp, tv_after_temp;
        gettimeofday(&tv_pre_temp, &tz);

        int current_set_size_sum = 0;
        while(!current_set.empty() && !isAllReachTarget(current_set, -1, currentTime())) {
//            std::cout << count << " th, size = " << current_set.size()
//            << ", reach " << reach_target_count << " neat " << current_set.size()-reach_target_count << std::endl;
//            std::cout << count << " " << current_set.size()
//                      << " " << reach_target_count << " " << current_set.size()-reach_target_count << std::endl;
            count ++;
            if(count % 100) {
                gettimeofday(&tv_after_temp, &tz);
                double search_path_cost = (tv_after_temp.tv_sec - tv_pre_temp.tv_sec) * 1e3 + (tv_after_temp.tv_usec - tv_pre_temp.tv_usec) / 1e3;
                if(search_path_cost > max_time_cost_) {
                    return ExitCode::FAILED;
                }
            }
            // break before finish all path, to keep unfinished path
//            if(count > 20)
//            {
//                break;
//            }
            current_set_size_sum += current_set.size();
            next_set.clear();
            reach_target_count = 0;
            for(auto & current_path : current_set) {
                future_node_id = current_path->last_node_->sg_->node_id_;


                //std::cout << "  current_path length " << current_path->length_ << std::endl;
                if(isReachTarget(current_path, currentTime())) {

                    //std::cout << " flag 1 " << std::endl;
                    //std::cout << " to target id " << data_->close_node_to(ns[future_node_id], false, currentTime()) << std::endl;
                    if (data_->close_node_to(ns[future_node_id], false, currentTime()) != MAX<NodeId> &&
                        !isEdgeTransferLegal(tg_,
                                         current_path->from()->last_node_,
                                         ns[future_node_id],
                                         ns[data_->close_node_to(ns[future_node_id], false, currentTime())], tg_->etcs_))
                        continue;

                    reach_target_count ++;
                    //std::cout << " reach target " << std::endl;
                    PathLen complete_length = current_path->length_ + data_->dist_to(current_path->last_node_, false, currentTime());
                    //std::cout << " complete_length = " << complete_length << std::endl;
                    if(global_minimum_path_length > complete_length) {
                        global_minimum_path_length = complete_length;
                        shortest_path = current_path;
                    }
                    next_set.push_back(current_path);
                    continue;
                }

                if(first_reach_count_ > 0 && reach_target_count >= first_reach_count_) {
                    break;
                }

                // cause not shortest path in 2d map aurora
                // index 666: (544, 223) -> (332, 295), shortest length = 267.605
                //statistics[0].front() > length : 289.889 > 267.605
                if(global_shortest_ && current_path->length_ > data_->dist_to(ns[future_node_id], true, currentTime()) + EPS_FR) continue;


                if(global_shortest_) {
                    //closest_grid_ = findClosestFromSetToPoint<N>(target, current_path->last_node_->sg_->pt_);
                    heuristic_length_ = current_path->length_;// +
                              //tg_->surface_processor_->distBetween(closest_grid_->pt_, current_path->last_node_->sg_->pt_);
                    if(heuristic_length_ > global_minimum_path_length) continue;
                }

//                if (data_->is_expanded(ns[future_node_id], currentTime())) continue;
//                data_->is_expanded(ns[future_node_id], currentTime()) = true;

                //std::cout << " next size " << tg_->nextNodes(ns[future_node_id]).size() << std::endl;
                for (const auto &candidate_node_id : tg_->nextNodes(ns[future_node_id])) {

                    //std::cout << " flag 2 " << std::endl;

                    if (!isEdgeTransferLegal(tg_,
                                             current_path->from()->last_node_,
                                             ns[future_node_id],
                                             ns[candidate_node_id], tg_->etcs_))
                        continue;

                    bool is_legal = true;
                    for (const auto &ic : ics) {
                        if (!ic(tg_, data_, current_path, ns[candidate_node_id], target)) {
                            is_legal = false;
                            break;
                        }
                    }
                    if(!is_legal) continue;
                    // do not visit again ? cause loss legal path when use ETC
                    //if(data_->dist_to(ns[candidate_node_id], true, currentTime()) != MAX<PathLen>) continue;

                    temp_future_length = current_path->length_ +
                                         (ns[future_node_id]->sg_->pt_ - ns[candidate_node_id]->sg_->pt_).Norm();

                    // TODO: allow update value in heap, when dist change to smaller value
                    if (global_shortest_) {

                        if(temp_future_length > data_->dist_to(ns[candidate_node_id], true, currentTime()) + EPS_FR)
                            continue;

                        data_->dist_to(ns[candidate_node_id], true, currentTime()) = temp_future_length;
                        data_->close_node_to(ns[candidate_node_id], true, currentTime()) = future_node_id;

                    }
//                    next_set.push_back(std::make_shared<PathPointWithNode<N> >(tg_, current_path, ns[candidate_node_id]));

                    if(first_reach_count_ <= 0)
                    {
                        next_set.push_back(std::make_shared<PathPointWithNode<N> >(tg_, current_path, ns[candidate_node_id]));
                    } else
                        {
                        auto future_path = std::make_shared<PathPointWithNode<N> >(tg_, current_path,
                                                                                   ns[candidate_node_id]);
                        heuristic_length_ = future_path->length_ +
                                tg_->surface_processor_->distBetween(target->pt_, ns[candidate_node_id]->sg_->pt_);

                            future_path->length_to_target_ = heuristic_length_;

                        buffer_set.push(future_path);
                    }
                }
            }

            if(first_reach_count_ > 0 && reach_target_count >= first_reach_count_) {
//                std::cout << count << " " << next_set.size()
//                          << " " << reach_target_count << " " << next_set.size()-reach_target_count << std::endl;
                break;
            }

            if(first_reach_count_ <= 0) {
                std::swap(next_set, current_set);
            } else {
                while(!buffer_set.empty()) {
                    auto future_path = buffer_set.top();
                    buffer_set.pop();
                    next_set.push_back(future_path);
                    if(next_set.size() >= first_reach_count_) { break; }
                }
                std::swap(next_set, current_set);
            }
            //break;
        }
        //std::cout << " current_set_size_sum " << current_set_size_sum + buffer_set.size() << std::endl;
        // get result path
        final_set.clear();
        if(!current_set.empty()) {
            //std::cout << __FUNCTION__ << " reach target " << std::endl;
            if(!global_shortest_) {
                float way_points_count = 0.;
                for (auto &final_path : current_set) {
                    // release if want output unfinished path
                    if(!isReachTarget(final_path, currentTime())) continue;

                    auto final_node = data_->close_node_to(final_path->last_node_, false, currentTime());

                    //std::cout << " flag 3 " << std::endl;

//                    if(final_node != MAX<NodeId> && !isEdgeTransferLegal(tg_, final_path->from()->last_node_,
//                                            final_path->last_node_,
//                                            ns[data_->close_node_to(final_path->last_node_, false, currentTime())],
//                                            tg_->etcs_)) continue;

                    const auto &result_path = PathTransformToContinuousEdgesWithNode(tg_, data_, final_path, currentTime());
                    final_set.push_back(result_path);
                    way_points_count += result_path.size();
                }
                std::cout << " mean way points in path = " << way_points_count/first_reach_count_ << std::endl;
            } else {
//                auto shortest_path = current_set[0];
//                PathLen minimum_length = MAX<PathLen>;
//                for (auto &final_path : current_set) {
//                    auto final_node = data_->close_node_to(final_path->last_node_, false, currentTime());
//                    if(final_node != MAX<NodeId> && !isEdgeTransferLegal(tg_, final_path->from()->last_node_,
//                                            final_path->last_node_, ns[final_node], tg_->etcs_)) continue;
//
//                    PathLen new_length = final_path->length_ + data_->dist_to(final_path->last_node_, false, currentTime());
////                    std::cout << "final_path->length_ " << final_path->length_ << " / add length = "
////                    << data_->dist_to(final_path->last_node_, false, currentTime()) << std::endl;
//                    if(minimum_length > new_length) {
//                        minimum_length = new_length;
//                        shortest_path = final_path;
//                    }
//                    //std::cout << " minimum_length = " << minimum_length << std::endl;
//                }
                const auto &result_path = PathTransformToContinuousEdgesWithNode(tg_, data_, shortest_path, currentTime());
                final_set.push_back(result_path);
            }
        } else {
            std::cout << " reach no target " << std::endl;
        }
//        if(first_reach_count_ > current_set.size()) {
//            std::cout << " first_reach_count_ " << first_reach_count_
//                      << " > current_set.size() " << current_set.size() << std::endl;
//        } else {
//            std::cout << " first_reach_count_ " << first_reach_count_
//                      << " <= current_set.size() " << current_set.size() << std::endl;
//        }
        if(!final_set.empty()) { return ExitCode::SUCCESS; }
        else { return ExitCode::FAILED; }
    }

}

#endif //FREENAV_BREADTH_FIRST_SEARCH_WITHOUT_EDGE_H
