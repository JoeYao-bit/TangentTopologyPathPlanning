//
// Created by yaozhuo on 2023/2/6.
//

#ifndef FREENAV_TOPO_BREADTH_FIRST_SEARCH_WITH_EDGE_H
#define FREENAV_TOPO_BREADTH_FIRST_SEARCH_WITH_EDGE_H

#include "search_path_with_edge.h"

namespace freeNav::Topo {

    template <Dimension N>
    template <typename TARGET_TYPE>
    ExitCode GeneralGraphPathPlannerWithEdge<N>::BreadthFirstSearchWithEdge(const PathPointWithEdgePtrs<N> &init_set,
                                                                            const IterationConstraints<N, TARGET_TYPE> &ics,
                                                                            RoadMapEdgeTraitDeques<N> &final_set, // resulted path set
                                             const TARGET_TYPE& target
    ) {
        auto & es = tg_->edges_;
        auto & ns = tg_->nodes_;
        auto & hes = tg_->hyper_edges_;

        //std::cout << __FUNCTION__ << " start " << std::endl;
        global_minimum_path_length = MAX<PathLen>;
        if(init_set.empty()) { return ExitCode::ILLEGAL_INPUT_INITIAL_SET; }
        //if(ics.empty() && !global_shortest_) { return ExitCode::ILLEGAL_INPUT_CONSTRAINT; }
        /* 1, create initial paths */
        PathPoints<N> current_set = init_set;
        shortest_path_edge_ = nullptr;
        PathPoints<N> pre_set = current_set;
        // check whether start set reach target, accelerate the iteration in some cases
        for (auto &pre_tpt : pre_set) {
            if(isReachTarget(pre_tpt->last_edge_, true, currentTime())) {
                PathLen finished_length = data_->dist_to(pre_tpt->last_edge_, true, true, currentTime())
                                          + data_->dist_to(pre_tpt->last_edge_, false, true, currentTime())
                                          - pre_tpt->last_edge_->length();
                // update the global minimum length of finished path, if the start set contain the target set
                if (global_minimum_path_length > finished_length) {
                    global_minimum_path_length = finished_length;
                    //std::cout << "reach target length = " << global_minimum_path_length << std::endl;
                    shortest_path_edge_ = pre_tpt->last_edge_;
                }
            }
        }
        //std::cout << " pre_set.size " << pre_set.size() << std::endl;
        bool all_reach_start_hyper = false;
        /* 2, iterative search */
        int iter_count = 0;
        while (!pre_set.empty() && !isAllReachTarget(pre_set, first_reach_count_, true, currentTime()))
            //for(int i=0 ; i<5; i++)
        {
            //all_reach_start_hyper = allReachHyper(pre_set);
            //std::cout << " is_all_hyper ? " << all_reach_start_hyper << std::endl;
            //std::cout << iter_count << " th with " << pre_set.size() << " path " << std::endl;
            current_set.clear();
//            for(const auto& pre_tpt : pre_set) {
//                data_->is_expanded(pre_tpt->last_edge_, true, currentTime()) = false;
//            }
            //std::cout << " flag 1" << std::endl;
            // for all current level path points
            for (auto &pre_tpt : pre_set) {

//                if(global_shortest_ && pre_tpt->length_ > data_->dist_to(pre_tpt->last_edge_, true, edge_mode_, currentTime())) {
//                    pre_tpt->setIllegal();
//                }

                if(!pre_tpt->is_legal_) continue;
                //std::cout << " flag 1" << std::endl;


                // if has reach target, do nothing to it and keep it
                if (isReachTarget(pre_tpt, true, currentTime())) {
                    //std::cout << "reach target 1" << std::endl;
                    //if(global_shortest_ && data_->is_expanded(pre_tpt->last_edge_, true, currentTime())) continue;
                    current_set.push_back(pre_tpt);
                    //data_->is_expanded(pre_tpt->last_edge_, true, currentTime()) = true;
                    continue;
                } else {
                    // expand hyper loop edge node
                    // for all candidates of current path point that not reach target
                    //std::cout << " flag 2" << std::endl;
                    if(pre_tpt->last_edge_->isHyperLoopEdge()) {
//                            if(pre_tpt->last_edge_->is_marked(edge_mode_, true, currentTime()))
//                                std::cout << "first start hyper " << std::endl;
//                            if(!all_reach_start_hyper) {
//                                current_set.push_back(pre_tpt);
//                                continue;
//                            }
                        if(!data_->is_marked(pre_tpt->last_edge_, true, false, currentTime()))
                        {
                            for (const auto &candidate_edge_id : tg_->edges_[pre_tpt->last_edge_->edgeId()]->nextHyperLoopEdges(true)) {
                                //std::cout << " reach hyper without" << std::endl;
                                const auto& retv = generateNextPathsHyper(pre_tpt, hes[candidate_edge_id], ics, target);
                                if(retv != nullptr) { pre_tpt->addNext(retv); current_set.push_back(retv); }
                            }
                        }
                        else
                        {
                            //std::cout << " reach hyper target " << std::endl;
                            for (const auto &candidate_edge_id : tg_->nextEdges(pre_tpt->last_edge_, true, true)) {
                                if(!data_->is_marked(es[candidate_edge_id], true, false, currentTime())) continue;
                                const auto& retv = generateNextPaths(pre_tpt, tg_->edges_[candidate_edge_id], ics, target);
                                if(retv != nullptr) { pre_tpt->addNext(retv); current_set.push_back(retv); }
                            }
                        }
                    } else
                    {
                        for (const auto &candidate_edge_id : tg_->nextEdges(pre_tpt->last_edge_, true, true)) {
                            // only considering un-loop edges that mark from start or target
//                            if(data_->is_marked(es[candidate_edge_id], true, true, currentTime())
//                               || data_->is_marked(es[candidate_edge_id], true, false, currentTime()) )
                            {
                                const auto &retv = generateNextPaths(pre_tpt, tg_->edges_[candidate_edge_id], ics, target);
                                if (retv != nullptr) { pre_tpt->addNext(retv); current_set.push_back(retv); }
                            }
                        }
                    }
                }
            }
            std::swap(pre_set, current_set);
            iter_count ++;
        }
        Paths<N> paths;
        final_set.clear();
        if(!global_shortest_) {
            for(auto& pre_tpt : pre_set) {
                if(isReachTarget(pre_tpt->last_edge_, true, currentTime()))
                {
                    //pre_tpt->last_edge_->print(true, true);
                    final_set.push_back(PathPointTransformToContinuousEdgesDeque(tg_, data_, pre_tpt));
                }
            }
        } else if(shortest_path_edge_ != nullptr) {
            //std::cout << "final: shortest_path_edge " << shortest_path_edge << std::endl;
            //std::cout << "final: global_minimum_path_length = " << global_minimum_path_length << std::endl;
            const auto& result_path = NodeTransformToContinuousEdgesWithEdge(tg_, data_, shortest_path_edge_, true, currentTime());
            //printPathDequeLevel(result_path);
            final_set.push_back(result_path);
        }
        if(!final_set.empty()) { return ExitCode::SUCCESS; }
        else { return ExitCode::FAILED; }
    }

    template <Dimension N>
    template <typename TARGET_TYPE>
    inline PathPointWithEdgePtr<N> GeneralGraphPathPlannerWithEdge<N>::generateNextPathsHyper(const PathPointWithEdgePtr<N>& pre_tpt,
                                                                                              const HyperLoopEdgeWithLengthPtr<N>& candidate_edge,
                                                                                              const IterationConstraints<N, TARGET_TYPE> &ics,
                                                                                              const TARGET_TYPE& target) {
        //std::cout << " edge " << pre_tpt->last_edge_->nextNode(false)->sg_->pt_ << "->" << pre_tpt->last_edge_->nextNode(true)->sg_->pt_
        //<< "/ split to " << candidate_edge.second->edge_->nextNode(false)->sg_->pt_ << "->" << candidate_edge.second->edge_->nextNode(true)->sg_->pt_ << std::endl;
        //std::cout << " is loop " << pre_tpt->last_edge_->isLoopEdge() << " / " << candidate_edge.second->edge_->isLoopEdge() << std::endl;
        //std::cout << " is hyper loop " << pre_tpt->last_edge_->is_hyper_edge_node_ << " / " << candidate_edge.second->edge_->is_hyper_edge_node_ << std::endl;
//            if(candidate_edge->edge_->is_marked(edge_mode_, false, currentTime())) {
//                std::cout << " reach target " << candidate_edge->edge_->printStr(edge_mode_, true) << std::endl;
//            }

        auto & ns = tg_->nodes_;
        auto & es = tg_->edges_;
        auto & hes = tg_->hyper_edges_;

        // when there is the global shortest constraint, if the future min length to start is longer than the now min length to start, it is illegal
        PathLen future_length_edge = data_->dist_to(pre_tpt->last_edge_, true, true, currentTime()) + hes[candidate_edge->edgeId()]->length(); // ?
        //std::cout << " future_length_edge " << future_length_edge << std::endl;
        if(global_shortest_ && future_length_edge > data_->dist_to(es[candidate_edge->edgeHeadId()], true, true, currentTime())) { return nullptr; }
        if(!edge_mode_) {
            if(global_shortest_ && future_length_edge > data_->dist_to(es[candidate_edge->edgeHeadId()], true, false, currentTime())) {
                return nullptr;
            }
        }
        // the candidate must satisfy all constraint otherwise it is illegal
        bool is_legal = true;
        for (const auto &ic : ics) {
            if (!ic(tg_, data_, pre_tpt, es[candidate_edge->edgeHeadId()], target)) {
                is_legal = false;
                break;
            }
        }
        // only legal candidates will join the next iteration, but need to satisfy the global shortest constraint is there is
        if (is_legal) {
            //return current_path_ptr;
            //std::cout << " is legal / future_length_edge = " << future_length_edge << std::endl;
            if(global_shortest_) {
                /* only use when no extra ics except global shortest, otherwise may cause illegal edge transfer */
                if(future_length_edge >= data_->dist_to(es[candidate_edge->edgeHeadId()], true, true, currentTime())) { return nullptr; }
                // TODO: following code cause error in node mode BFS, so limited to edge mode
                if (data_->close_edge_to(es[candidate_edge->edgeHeadId()], true, true, currentTime()) != MAX<EdgeId>) {
//                        PathLen shorter_dist = candidate_edge->dist_to(true, true, currentTime()) - future_length_edge;
//                        shorterHyperEdgeToStartLength(candidate_edge->edge_, shorter_dist, true, currentTime()); // may shrink one edge twice
                    //pre_tpt->is_legal_ = false;
                }
                data_->dist_to(es[candidate_edge->edgeHeadId()], true, true, currentTime()) = future_length_edge;
                data_->close_edge_to(es[candidate_edge->edgeHeadId()], true, true, currentTime()) = pre_tpt->last_edge_->edgeId();
                if(!edge_mode_) {
                    if(data_->dist_to(es[candidate_edge->edgeHeadId()], true, false, currentTime()) >= future_length_edge) {
                        data_->dist_to(es[candidate_edge->edgeHeadId()], true, false, currentTime()) = future_length_edge;
                    } else { return nullptr; }
                }
                // update the global minimum length of finished path, if the new legal path point reach target
                if (isReachTarget(es[candidate_edge->edgeHeadId()], true, currentTime())) {
                    //std::cout << " reach target " << candidate_edge->edge_->printStr(edge_mode_, true) << std::endl;
                    PathLen finished_length = data_->dist_to(es[candidate_edge->edgeHeadId()], true, true, currentTime()) +
                                              data_->dist_to(es[candidate_edge->edgeHeadId()], false, true, currentTime()) - es[candidate_edge->edgeHeadId()]->length();
                    if (global_minimum_path_length > finished_length) {
                        global_minimum_path_length = finished_length;
                        shortest_path_edge_ = es[candidate_edge->edgeHeadId()];
                    } else return nullptr;
                } else if(global_minimum_path_length != MAX<PathLen>) {
                    // longer than the global shortest path will not join next iteration
                    // unfinished path must be short than the global shortest finish length
                    if (future_length_edge > global_minimum_path_length) { return nullptr; }
                    closest_grid_ = findClosestFromSetToPoint<N>(target, ns[es[candidate_edge->edgeHeadId()]->nextNode(true)]->sg_->pt_);
                    if(future_length_edge +
                       //(closest_grid_->pt_ - candidate_edge->nextNode(true)->sg_->pt_).Norm()
                       tg_->surface_processor_->distBetween(closest_grid_->pt_, ns[es[candidate_edge->edgeHeadId()]->nextNode(true)]->sg_->pt_)
                       > global_minimum_path_length) {
                        return nullptr;
                    }
                }
            }
            // avoid add more than once during one iteration
//            if(global_shortest_ && data_->is_expanded(es[candidate_edge->edgeHeadId()], true, currentTime())) { return nullptr; }
//            data_->is_expanded(es[candidate_edge->edgeHeadId()], true, currentTime()) = true;
            return std::make_shared<PathPointWithEdge<N> >(tg_, pre_tpt, es[candidate_edge->edgeHeadId()]);
        } else { return nullptr; }
    }

    template <Dimension N>
    template <typename TARGET_TYPE>
    inline PathPointWithEdgePtr<N> GeneralGraphPathPlannerWithEdge<N>::generateNextPaths(const PathPointWithEdgePtr<N>& pre_tpt,
                                                                                         const RoadMapEdgeTraitPtr<N>& candidate_edge,
                                                                                         const IterationConstraints<N, TARGET_TYPE> &ics, const TARGET_TYPE& target) {
        //std::cout << " edge " << pre_tpt->last_edge_->nextNode(false)->sg_->pt_ << "->" << pre_tpt->last_edge_->nextNode(true)->sg_->pt_
        //<< "/ split to " << candidate_edge.second->edge_->nextNode(false)->sg_->pt_ << "->" << candidate_edge.second->edge_->nextNode(true)->sg_->pt_ << std::endl;
        //std::cout << " is loop " << pre_tpt->last_edge_->isLoopEdge() << " / " << candidate_edge.second->edge_->isLoopEdge() << std::endl;
        //std::cout << " is hyper loop " << pre_tpt->last_edge_->is_hyper_edge_node_ << " / " << candidate_edge.second->edge_->is_hyper_edge_node_ << std::endl;

        // when there is the global shortest constraint, if the future min length to start is longer than the now min length to start, it is illegal
        auto current_path_ptr = std::make_shared<PathPointWithEdge<N> >(tg_, pre_tpt, candidate_edge);
        auto & ns = tg_->nodes_;
        //return current_path_ptr;
        PathLen future_length_edge = data_->dist_to(pre_tpt->last_edge_, true, true, currentTime()) + candidate_edge->length();
        //std::cout << " future_length_edge " << future_length_edge << std::endl;
        if(global_shortest_ && future_length_edge > data_->dist_to(candidate_edge, true, true, currentTime())) { return nullptr; }
        if(!edge_mode_) {
            if(global_shortest_ && future_length_edge > data_->dist_to(candidate_edge, true, false, currentTime())) {
                return nullptr;
            }
        }
        //return current_path_ptr;
        // the candidate must satisfy all constraint otherwise it is illegal
        bool is_legal = true;
        for (const auto &ic : ics) {
            if (!ic(tg_, data_, pre_tpt, candidate_edge, target)) {
                is_legal = false;
                break;
            }
        }
        // only legal candidates will join the next iteration, but need to satisfy the global shortest constraint is there is
        if (is_legal) {
            //return current_path_ptr;
            //std::cout << " is legal / future_length_edge = " << future_length_edge << std::endl;
            if(global_shortest_) {
                /* only use when no extra ics except global shortest, otherwise may cause illegal edge transfer */
                if(future_length_edge >= data_->dist_to(candidate_edge, true, true, currentTime())) { return nullptr; }
                // TODO: following code cause error in node mode BFS, so limited to edge mode
                if (data_->close_edge_to(candidate_edge, true, true, currentTime()) != MAX<EdgeId>) {
                    //PathLen shorter_dist = data_->dist_to(candidate_edge, true, true, currentTime()) - future_length_edge;
                    //shorterEdgeToStartLength(tg_, data_, candidate_edge, shorter_dist, true, true, currentTime());
                    //pre_tpt->is_legal_ = false;
                }
                data_->dist_to(candidate_edge, true, true, currentTime()) = future_length_edge;
                data_->close_edge_to(candidate_edge, true, true, currentTime()) = pre_tpt->last_edge_->edgeId();
                if(!edge_mode_) {
                    if(data_->dist_to(candidate_edge, true, false, currentTime()) >= future_length_edge) {
                        data_->dist_to(candidate_edge, true, false, currentTime()) = future_length_edge;
                    } else { return nullptr; }
                }
                // update the global minimum length of finished path, if the new legal path point reach target
                if (isReachTarget(candidate_edge, true, currentTime())) {
                    //std::cout << " reach target " << candidate_edge->printStr(edge_mode_, true) << std::endl;
                    PathLen finished_length = data_->dist_to(candidate_edge, true, true, currentTime()) +
                                              data_->dist_to(candidate_edge, false, true, currentTime()) - candidate_edge->length();
                    if (global_minimum_path_length > finished_length) {
                        global_minimum_path_length = finished_length;
                        shortest_path_edge_ = candidate_edge;
                    } else return nullptr;
                } else if(global_minimum_path_length != MAX<PathLen>) {
                    // longer than the global shortest path will not join next iteration
                    // unfinished path must be short than the global shortest finish length
                    if (future_length_edge > global_minimum_path_length) { return nullptr; }
                    closest_grid_ = findClosestFromSetToPoint<N>(target, ns[candidate_edge->nextNode(true)]->sg_->pt_);
                    if(future_length_edge +
                       //(closest_grid_->pt_ - candidate_edge->nextNode(true)->sg_->pt_).Norm()
                       tg_->surface_processor_->distBetween(closest_grid_->pt_, ns[candidate_edge->nextNode(true)]->sg_->pt_)
                       > global_minimum_path_length) {
                        return nullptr;
                    }
                }
            }
            // avoid add more than once during one iteration
//            if(global_shortest_ && data_->is_expanded(candidate_edge, true, currentTime())) { return nullptr; }
//            data_->is_expanded(candidate_edge, true, currentTime()) = true;
            return current_path_ptr;
        } else { return nullptr; }
    }

};

#endif //FREENAV_BREADTH_FIRST_SEARCH_WITH_EDGE_H
