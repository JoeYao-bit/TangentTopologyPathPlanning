//
// Created by yaozhuo on 2022/1/3.
//

#ifndef FREENAV_TOPO_ITERATION_CONSTRAINTS_H
#define FREENAV_TOPO_ITERATION_CONSTRAINTS_H
#include "../../algorithm/online_search/path_tree_with_edge.h"

namespace freeNav::Topo {

    extern PathLen global_minimum_path_length;

    template <Dimension N>
    class GeneralGraphPathPlannerWithEdge;

    template <Dimension N>
    class GeneralGraphPathPlannerWithNode;

    template<Dimension N>
    using GeneralGraphPathPlannerPtr = std::shared_ptr<GeneralGraphPathPlannerWithEdge<N> >;

    // some constraints will change the candidate tangent point node's data
    //template <Dimension N, typename TARGET_TYPE>
    //using IterationConstraints = std::vector<bool (*)   (RoadMapEdgeTraitPtr<N> current_edge, HyperLoopEdgeWithLengthPtr<N> candidate, const TARGET_TYPE& target, bool is_dynamic)>;

    template <Dimension N, typename TARGET_TYPE>
    using IterationConstraints = std::vector<bool (*)   (RoadMapGraphPtr<N>& tg, DynamicDataOfSearchWithEdgePtr<N>& data, const PathPointWithEdgePtr<N>& current_path, const RoadMapEdgeTraitPtr<N>& candidate, const TARGET_TYPE& target)>;

    template <Dimension N>
    void shorterEdgeToStartLength(RoadMapGraphPtr<N>& tg, DynamicDataOfSearchWithEdgePtr<N>& data, const RoadMapEdgeTraitPtr<N>& current_edge, const PathLen& length, bool is_from_start, bool is_edge, Time current_time) {
        if(length < 0) {
            std::cerr << "sETSL: length = " << length << " < 0" << std::endl;
            return;
        }
        auto & es = tg->edges_;
        data->dist_to(current_edge, is_from_start, is_edge, current_time) = data->dist_to(current_edge, is_from_start, is_edge, current_time) - length;
        if(data->dist_to(current_edge, is_from_start, is_edge, current_time) < 0) {
            std::cerr << "sETSL: current_edge->min_length_to_start_ "
                      << data->dist_to(current_edge, is_from_start, is_edge, MIN_TIME) << " < 0" << std::endl;
        }
        for(auto& next_edge_id : es[data->current_on_edge(current_edge, is_edge, current_time)]->nextEdges(is_from_start)) {
            if(data->close_edge_to(es[next_edge_id], is_from_start, is_edge, current_time) == data->current_on_edge(current_edge, is_edge, current_time)) {
                shorterEdgeToStartLength(tg, data, es[next_edge_id], length, is_from_start, is_edge, current_time);
            }
        }
    }

    // may shorter twice ? TODO: to avoid twice, remember the count of shorter, and store the number in each edge/node, avoid shorter more than twice
    template <Dimension N>
    void shorterHyperEdgeToStartLength(RoadMapEdgeTraitPtr<N> current_edge, const PathLen &length, bool is_edge, const Time& current_time) {
        if(length < 0) {
            std::cerr << "sHETSL: length = " << length << " < 0" << std::endl;
            //exit(0);
            return;
        }
        PathLen future_length = current_edge->dist_to(true, is_edge, current_time) - length;
        if(future_length < 0) {
            std::cerr << "sHETSL: current_edge->min_length_to_start_ " << current_edge->dist_to(true, is_edge, MIN_TIME)
                      << " < 0" << std::endl;
            //exit(0);
        }
        current_edge->dist_to(true, is_edge, current_time) = future_length;
        for(auto& next_edge : current_edge->current_on_edge(is_edge, current_time)->nextHyperLoopEdges(is_edge)) {
            if(next_edge->edge_->close_edge_to(true, is_edge, current_time) == current_edge->current_on_edge(is_edge, current_time)) {
                shorterHyperEdgeToStartLength(next_edge->edge_, length, is_edge, current_time);
            }
        }
    }

    template <Dimension N>
    void shorterNodeToStartLength(RoadMapEdgeTraitPtr<N> current_edge, const PathLen& length, const Time& current_time) {
        if(length < 0) {
            std::cerr << "sETSL: length = " << length << " < 0" << std::endl;
            //exit(0);
        }
        current_edge->next_node(true)->dist_to(true, true, current_time) -= length;
        if(current_edge->next_node(true)->dist_to(true, true, current_time) < 0) {
            std::cerr << "sNTSL: current_edge->to_->min_length_to_start_ "
                      << current_edge->next_node(true)->dist_to(true, true, current_time) << " < 0" << std::endl;
            //exit(0);
        }
        for(auto& next_edge : current_edge->next_hyper_loop_edges(true)) {
            if(next_edge->edge_->next_node(true)->close_edge_to(true, true, MIN_TIME) == current_edge) {
                shorterNodeToStartLength(next_edge->edge_, length);
            }
        }
    }

    /* every unfinished path must be shorter than the shortest finished path */
    template <Dimension N, typename TARGET_TYPE>
    bool IC_OriginalBFS            (RoadMapGraphPtr<N>& tg, DynamicDataOfSearchWithEdgePtr<N>& data, const PathPointWithEdgePtr<N>& current_path, const RoadMapEdgeTraitPtr<N>& candidate, const TARGET_TYPE& target) {
        if(candidate->close_edge_to(true, true, MIN_TIME) != nullptr) {
            return false;
        }
        return true;
    }

    template <Dimension N>
    bool IsEdgeCrossEdge(RoadMapEdgeTraitPtr<N> edge1, RoadMapEdgeTraitPtr<N> edge2) {
        return IsLineCrossLine(edge1->next_node(false)->sg_->pt_, edge1->next_node(true)->sg_->pt_, edge2->next_node(false)->sg_->pt_, edge2->next_node(true)->sg_->pt_);
    }

    template <Dimension N>
    bool IsEdgeCrossEdge(RoadMapNodeTraitPtr<N>& node1,
                         RoadMapNodeTraitPtr<N>& node2,
                         RoadMapNodeTraitPtr<N>& node3,
                         RoadMapNodeTraitPtr<N>& node4) {
        return IsLineCrossLine(node1->sg_->pt_, node2->sg_->pt_, node3->sg_->pt_, node4->sg_->pt_);
    }

    /* a path shouldn't visit the same tangent point more than once
     * if there is IC_GlobalMinimumPathLength, no need for IC_NoLoop
     * TODO: there may be loop caused by initial start path and initial target path , when search distinctive paths
     * */
    template <Dimension N, typename TARGET_TYPE>
    bool IC_NoLoop                (RoadMapGraphPtr<N>& tg, DynamicDataOfSearchWithEdgePtr<N>& data, const PathPointWithEdgePtr<N>& current_path, const RoadMapEdgeTraitPtr<N>& candidate, const TARGET_TYPE& target) {
//        PathPointWithEdgePtr<N> pre = current_path;
//        auto & nodes = tg.nodes_;
//        while(pre != nullptr) {
//            if(pre->last_edge_->next_node(true)->sg_->id_ == candidate->next_node(true)->sg_->id_) {
//                return false;
//            }
//            pre = pre->from();
//            //TODO:  transfer this to pre-computation, but this is very time consuming
//            //     need n(n+1)/2 IsEdgeCrossEdge check, n = count of edges
//            //     this reduce the cost of IC_GlobalMinimumPathLength
//            //     but too slow
//            //if(pre!=nullptr && IsEdgeCrossEdge(pre->last_edge_, candidate)) return false;
//        }
        return true;
    }

    template <Dimension N, typename TARGET_TYPE>
    bool IC_EdgeNLevelLimit         (RoadMapGraphPtr<N>& tg, DynamicDataOfSearchWithEdgePtr<N>& data, const PathPointWithEdgePtr<N>& current_path, const RoadMapEdgeTraitPtr<N>& candidate, const TARGET_TYPE& target) {
        EdgeId pre_id = data->close_edge_to(current_path->last_edge_, true, true, MIN_TIME);
        EdgeId current_edge_id = current_path->last_edge_->edgeId();
        auto & es = tg->edges_;
        auto & ns = tg->nodes_;

        if(pre_id == MAX<EdgeId>) {
            if(candidate->level() >= es[current_edge_id]->level()) return true;
            else return false;
        } else {
            const Lv& current_level = es[current_edge_id]->level();
            const Lv& pre_level     = es[pre_id]->level();
            const Lv& next_level    = candidate->level();

//            if(current_level > pre_level) {
//                // if current level is INF, the next edge could be any level
//                if(current_edge->isLoopEdge()) return true; // running in loop edges
//                if(next_level > current_level) return true; // keep increasing
//                return false;
//            } else if (current_level < pre_level) {
//                // if is decreasing, the level must keep decreasing and can't be equal
//                if(next_level < current_level) return true;
//                else return false;
//            } else {
//                // if two level is equal, they must be both INF
//                if(!current_edge->isLoopEdge() || !pre->isLoopEdge()) {
//                    std::cout << "pre_level = " << pre_level << " / current_level = " << current_level << " / next_level = " << next_level << std::endl;
//                    std::cout << "shouldn't reach here" << std::endl;
//                    return false;
//                } else {
//                    // for two INF, the next could any level
//                    return true;
//                }
//            }

            if(es[current_edge_id]->isLoopEdge()) {
                // always legal
            } else {
                if(es[pre_id]->isLoopEdge()) {
                    if(current_level > next_level) return false;
                } else {
                    if(pre_level > current_level || current_level > next_level) return false;
                }
            }
            return true;
        }
    }

    template <Dimension N, typename TARGET_TYPE>
    bool IC_EdgeNLevelLimitUndirectedGraph(RoadMapGraphPtr<N>& tg, DynamicDataOfSearchWithEdgePtr<N>& data, const PathPointWithEdgePtr<N>& current_path, const RoadMapEdgeTraitPtr<N>& candidate, const TARGET_TYPE& target) {
        //RoadMapEdgeTraitPtr<N> pre = current_path->last_edge_->close_edge_to(true, true, MIN_TIME);
        if(current_path->from() == nullptr) { return true; }
        //         RoadMapEdgeTraitPtr<N> pre = current_path->last_edge_->close_edge_to(true, true, MIN_TIME);
        const RoadMapEdgeTraitPtr<N>& pre = current_path->from()->last_edge_;
        const RoadMapEdgeTraitPtr<N>& current_edge = current_path->last_edge_;
        if(pre == nullptr) {
            //if(candidate->level() > current_edge->level())
                return true;
            //else return false;
        } else {
            const Lv& current_level = current_edge->level();
            const Lv& pre_level     = pre->level();
            const Lv& next_level    = candidate->level();

            if(current_edge->isLoopEdge()) {
                // always legal
            } else {
                if(pre->isLoopEdge()) {
                    if(current_level <= next_level) return false;
                } else {
                    if(data->is_marked(current_edge, true, true, MIN_TIME) && data->is_marked(candidate, true, true, MIN_TIME)) {
                        if(current_level >= next_level) return false;
                    } else if(data->is_marked(current_edge, true, false, MIN_TIME) && data->is_marked(candidate, true, false, MIN_TIME)) {
                        if(current_level <= next_level) return false;
                    }
                }
            }
            return true;
        }
    }

}

#endif //FREENAV_ITERATION_CONSTRAINTS_H
