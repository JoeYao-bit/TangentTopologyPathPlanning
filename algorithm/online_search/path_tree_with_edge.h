//
// Created by yaozhuo on 2022/1/2.
//

#ifndef FREENAV_TOPO_PATH_TREE_WITH_EDGE_H
#define FREENAV_TOPO_PATH_TREE_WITH_EDGE_H
#include <iostream>
#include <memory>
#include <list>

#include "../../algorithm/graph_construction/tangent_graph.h"

namespace freeNav::Topo {

    template <Dimension N> class PathPointWithEdge;

    template <Dimension N>
    using PathPointWithEdgePtr = std::shared_ptr<PathPointWithEdge<N> >;

    template <Dimension N>
    using PathPointWithEdgePtrs = std::vector<PathPointWithEdgePtr<N> >;

    template <Dimension N>
    class PathPointWithEdge {
    public:

        explicit PathPointWithEdge(RoadMapGraphPtr<N>& tg, const PathPointWithEdgePtr<N>& from, const RoadMapEdgeTraitPtr<N>& edge)
                : from_(from), is_legal_(true), last_edge_(edge) {
            if(edge == nullptr) {
                exit(0);
            }
            if(from != nullptr) {
                length_ = edge->length() + from->length_;
            } else {
                length_ = edge->length();
            }
        }

        explicit PathPointWithEdge(RoadMapGraphPtr<N>& tg, const PathPointWithEdgePtr<N>& from, const HyperLoopEdgeWithLengthPtr<N>& hyper_edge)
                : from_(from), is_legal_(true), last_edge_(hyper_edge->edge_) {
            if(hyper_edge == nullptr) {
                exit(0);
            }
            if(from != nullptr) {
                length_ = hyper_edge->length_ + from->length_;
            } else {
                length_ = hyper_edge->length_;
            }
        }

        void addNext(const PathPointWithEdgePtr<N>& pt) {
            if(pt == nullptr) return;
            to_.push_back(pt);
        }

        void setIllegal() {
            is_legal_ = false;
            for(auto& leaf : to_) {
                leaf->setIllegal();
            }
        }

        void print(RoadMapGraphPtr<N>& tg, bool is_node = false) {
//            auto & nodes = tg->nodes_;
//            auto & edges = tg->edges_;
//            auto & hyper_edges = tg->hyper_edges_;
//            auto & edge_datas = tg->edge_dynamic_datas_;
//            PathPointWithEdgePtr<N> ppt = from();
//            std::cout << "id "<< nodes[last_edge_->to_]->sg_->id_ << " " << nodes[last_edge_->to_]->sg_->pt_;
//            if(ppt != nullptr) {
//                while(ppt != nullptr) {
//                    std::cout << "<-" << nodes[ppt->last_edge_->to_]->sg_->pt_;
//                    if(ppt->from_ == nullptr) {
//                        std::cout << "<-" << nodes[ppt->last_edge_->from_]->sg_->pt_;
//                    }
//                    ppt = ppt->from();
//                }
//            } else {
//                std::cout << "<-" << nodes[last_edge_->from_]->sg_->pt_;
//                if(edge_datas[last_edge_->edge_id_]->close_to_start_edge_ != nullptr) {
//                    edge_datas[last_edge_->edge_id_]->close_to_start_edge_->print();
//                }
//            }
//            std::cout << std::endl;
            // draw level
//            std::string level = last_edge_->level_ == std::numeric_limits<Id>::max() ? std::string("INF") : std::to_string(last_edge_->level_);
//            std::cout << std::endl << "level: " << level.c_str();
//            ppt = from();
//            if(ppt != nullptr) {
//                while(ppt != nullptr) {
//                    level = ppt->last_edge_->level_ == std::numeric_limits<Id>::max() ? std::string("INF") : std::to_string(ppt->last_edge_->level_);
//                    std::cout << "<-" << level.c_str();
//                    ppt = ppt->from();
//                }
//            }
//            std::cout << ", l = " << length_ << ", legal = " << is_legal_ << ", ptr = " << last_edge_->to_ << std::endl;
        }

        PathPointWithEdgePtr<N> from() {
            return from_;
        }

        PathPointWithEdgePtrs<N> to() {
            return to_;
        }

        RoadMapEdgeTraitPtr<N> last_edge_; // the tangent point that the path point corresponding to

        bool is_legal_; // used in lag delete

        double length_ = 0; // dist from start to here

    private:

        PathPointWithEdgePtr<N> from_ = nullptr; // the father node of the node

        PathPointWithEdgePtrs<N> to_; // the child nodes of the node

    };

    template <Dimension N>
    using PathPointWithEdgePtr = std::shared_ptr<PathPointWithEdge<N> >;

    template <Dimension N>
    using PathPoints = std::vector<PathPointWithEdgePtr<N>>;

    template <Dimension N>
    bool operator<(const PathPointWithEdge<N> ppt1, const PathPointWithEdge<N> ppt2);

    template <Dimension N>
    RoadMapEdgeTraitPtrs<N> edgesBetweenTwoHyperEdge(RoadMapGraphPtr<N>& tg,
                                                     DynamicDataOfSearchWithEdgePtr<N> data,
                                                     RoadMapEdgeTraitPtr<N> pre,
                                                     RoadMapEdgeTraitPtr<N> next) {
        if(!pre->is_hyper_edge_node_ || !next->is_hyper_edge_node_) {
            return {};
        }
//        if(pre->nextHyperLoopEdges(true).find(next->edge_id()) == pre->nextHyperLoopEdges(true).end()) {
//            return {};
//        }
//        if(next->nextHyperLoopEdges(false).find(pre->edge_id()) == next->nextHyperLoopEdges(false).end()) {
//            return {};
//        }
        RoadMapEdgeTraitPtrs<N> retv = {};
        EdgeId buffer_id = pre->edgeId();
        EdgeId next_buffer_id = MAX<EdgeId>;
        auto & ns = tg->nodes_;
        auto & es = tg->edges_;
        auto & hes = tg->hyper_edges_;

        while(buffer_id != MAX<EdgeId>) {
            next_buffer_id = MAX<EdgeId>;
            int count = 0;
            //std::cout << "flag 2 " << std::endl;
            for(const auto& candidate_id : es[buffer_id]->nextLoopEdges(true)) {
                //std::cout << "flag 3 " << std::endl;
                if(ns[es[candidate_id]->nextNode(false)]->sg_->id_ != ns[es[buffer_id]->nextNode(true)]->sg_->id_) {
                    std::cerr << "eBTHE: un match next loops " << std::endl;
                }
                // notice: non-hyper-loop edge have only one preHyperLoopEdges or one nextHyperLoopEdges
                if(!es[candidate_id]->isHyperLoopEdge()
                   && hes[es[candidate_id]->nextHyperLoopEdges(true).front()]->edgeHeadId() == next->edgeId()
                        ) {
                    // filter that two next loop edges reach the same hyper loop edge
                    if((!retv.empty() && ns[es[candidate_id]->nextNode(false)]->sg_->id_ != ns[retv.back()->nextNode(true)]->sg_->id_)) continue;
                    next_buffer_id = candidate_id;
                    retv.push_back(es[candidate_id]);
                    count ++;
                }
            }
            if(count > 1) { std::cerr << "eBTHE: " <<  count << " (> 1) un-hyper loop connect the same hyper edge" << std::endl; }
            //else { std::cout << "1 un-hyper loop connect the same hyper edge " << std::endl; }
            buffer_id = next_buffer_id;
        }
        return retv;
    }

    template <Dimension N>
    RoadMapEdgeTraitPtrs<N> edgesFromEdgeTo(RoadMapGraphPtr<N>& tg,
                                            DynamicDataOfSearchWithEdgePtr<N> data,
                                            const RoadMapEdgeTraitPtr<N>& edge,
                                            bool is_start, bool is_edge) {
        if(edge == nullptr) {
            return {};
        } else {
            RoadMapEdgeTraitPtrs<N> retv = {};
            EdgeId current_edge_id = edge->edgeId();

            auto & es = tg->edges_;
            auto & ns = tg->nodes_;

            while(current_edge_id != MAX<EdgeId>) {
                retv.push_back(es[current_edge_id]);
                current_edge_id = data->close_edge_to(es[current_edge_id], is_start, is_edge, MIN_TIME);
            }
            if(is_start) reverse(retv.begin(), retv.end());
            return retv;
        }
    }

    template <Dimension N>
    RoadMapEdgeTraitDeque<N> edgesDequeFromEdgeTo(RoadMapGraphPtr<N>& tg,
                                                  DynamicDataOfSearchWithEdgePtr<N> data,
                                                  const RoadMapEdgeTraitPtr<N>& edge,
                                                  bool is_start, bool is_edge) {
        if(edge == nullptr) {
            return {};
        } else {
            RoadMapEdgeTraitDeque<N> retv = {};
            EdgeId current_edge_id = edge->edgeId();

            auto & es = tg->edges_;
            auto & ns = tg->nodes_;

            while(current_edge_id != MAX<EdgeId>) {
                retv.push_back(es[current_edge_id]);
                current_edge_id = data->close_edge_to(es[current_edge_id], is_start, is_edge, MIN_TIME);
            }
            if(is_start) reverse(retv.begin(), retv.end());
            return retv;
        }
    }

    template<Dimension N>
    bool PathContinuousCheck(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitPtrs<N>& path) {
        auto & ns = tg->nodes_;
        if(path.empty()) return false;
        if(path.size() == 1) return true;
        for(int i=0; i<path.size()-1; i++) {
            if(ns[path[i]->nextNode(true)]->sg_->id_ != ns[path[i+1]->nextNode(false)]->sg_->id_) return false;
        }
        return true;
    }

    template<Dimension N>
    bool PathContinuousCheck(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitDeque<N>& path) {
        auto & ns = tg->nodes_;
        if(path.empty()) return false;
        if(path.size() == 1) return true;
        for(int i=0; i<path.size()-1; i++) {
            if(ns[path[i]->nextNode(true)]->sg_->id_ != ns[path[i+1]->nextNode(false)]->sg_->id_) return false;
        }
        return true;
    }

    template<Dimension N>
    Pointis<N> ContinuousEdgesToPath(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitPtrs<N>& path) {
        auto & ns = tg->nodes_;
        if(path.empty()) return {};
        Pointis<N> retv = {ns[path[0]->nextNode(false)]->sg_->pt_, ns[path[0]->nextNode(true)]->sg_->pt_};
        for(int i=1; i<path.size(); i++) {
            retv.push_back(ns[path[i]->nextNode(true)]->sg_->pt_);
        }
        return retv;
    }

    template<Dimension N>
    Pointis<N> ContinuousEdgesToPath(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitDeque<N>& path) {
        auto & ns = tg->nodes_;
        if(path.empty()) return {};
        Pointis<N> retv = {ns[path[0]->nextNode(false)]->sg_->pt_, ns[path[0]->nextNode(true)]->sg_->pt_};
        for(int i=1; i<path.size(); i++) {
            retv.push_back(ns[path[i]->nextNode(true)]->sg_->pt_);
        }
        return retv;
    }


    template <Dimension N>
    RoadMapEdgeTraitPtrs<N> PathPointTransformToContinuousEdges(RoadMapGraphPtr<N>& tg,
                                                                DynamicDataOfSearchWithEdgePtr<N> data,
                                                                const PathPointWithEdgePtr<N>& path_point,
                                                                bool is_edge) {
        if(path_point == nullptr || path_point->last_edge_ == nullptr) return {};
        auto & es = tg->edges_;
        auto & ns = tg->nodes_;
        // follower of final hyper edge
        RoadMapEdgeTraitPtrs<N> to_target_path = edgesFromEdgeTo(tg, data, path_point->last_edge_, false, true);
        // edges between intermediate hyper edge
        auto buffer = path_point;
        EdgeId start_edge_id = path_point->last_edge_->edgeId();
        if(buffer->from() != nullptr) {
            while(buffer->from()->last_edge_->isHyperLoopEdge()) {
                if(buffer->from()->last_edge_->isHyperLoopEdge()) {
                    RoadMapEdgeTraitPtrs<N> medium_edge = edgesBetweenTwoHyperEdge(tg, data, buffer->from()->last_edge_, buffer->last_edge_);
                    to_target_path.insert(to_target_path.begin(), medium_edge.begin(), medium_edge.end());
                    to_target_path.insert(to_target_path.begin(), buffer->from()->last_edge_);
                } else {
                    start_edge_id = data->close_edge_to(buffer->from()->last_edge_, true, true, MIN_TIME);
                    break;
                }
                start_edge_id = data->close_edge_to(buffer->from()->last_edge_, true, true, MIN_TIME);
                buffer = buffer->from();
                if(buffer == nullptr || buffer->last_edge_ == nullptr || buffer->from() == nullptr) break;
            }
        } else {
            // pop the begin to avoid replicate
            if(!to_target_path.empty()) {
                to_target_path.erase(to_target_path.begin());
            }
        }
        //return to_target_path;
        if(start_edge_id == MAX<EdgeId>) return to_target_path;
        RoadMapEdgeTraitPtrs<N> from_start_path = edgesFromEdgeTo(tg, data, es[start_edge_id], true, true);
        to_target_path.insert(to_target_path.begin(), from_start_path.begin(), from_start_path.end());
        if(!PathContinuousCheck(to_target_path)) {
            std::cerr << "PPTTCE: noncontinuous path" << std::endl;
        }
        return to_target_path;
    }



    template <Dimension N>
    RoadMapEdgeTraitPtrs<N> NodeTransformToContinuousEdges(RoadMapGraphPtr<N>& tg,
                                                           DynamicDataOfSearchWithEdgePtr<N> data,
                                                           RoadMapEdgeTraitPtr<N> edge,
                                                           bool is_edge, Time current_time) {
        if(edge == nullptr) return {};
        // follower of final hyper edge
        RoadMapEdgeTraitPtrs<N> to_target_path = edgesFromEdgeTo(tg, data, edge, false, true);
        //std::cout << "to_target_path = " << toStr(to_target_path) << std::endl;
        // edges between intermediate hyper edge
        auto buffer_id = edge->edgeId();
        auto & es = tg->edges_;
        auto & ns = tg->nodes_;
        while(es[buffer_id]->isHyperLoopEdge()) {
            if(data->close_edge_to(es[buffer_id], true, is_edge, current_time) != MAX<EdgeId> &&
              es[data->close_edge_to(es[buffer_id], true, is_edge, current_time)]->isHyperLoopEdge()
                    ) {
                if(data->close_edge_to(es[buffer_id], true, is_edge, current_time)->dist_to(true, true, is_edge)
                >= data->dist_to(es[buffer_id], true, true, is_edge))
                {
                    std::cerr << "closer edge far to start "
                              << "close dist = " << data->close_edge_to(es[buffer_id], true, is_edge, current_time)->dist_to(true, is_edge, current_time)
                              << " >= current dist = " << data->dist_to(es[buffer_id], true, is_edge, current_time)
                              << std::endl;
                    std::cerr << ns[es[data->close_edge_to(es[buffer_id], true, is_edge, current_time)]->nextNode(false)]->sg_->pt_ << "->"
                              << ns[es[data->close_edge_to(es[buffer_id], true, is_edge, current_time)]->nextNode(true)]->sg_->pt_ << std::endl;
                    std::cerr << "min_length_to_start "
                              << ns[es[data->close_edge_to(es[buffer_id], true, is_edge, current_time)]->nextNode(true)]->dist_to(true, current_time) << std::endl;
                    exit(0);
                }
                //std::cout << "current dist to start = " << buffer->min_length_to_start(is_edge) << std::endl;
                //std::cout << "closer  dist to start = " << buffer->close_edge_to(is_edge)->min_length_to_start(is_edge) << std::endl << std::endl;

                RoadMapEdgeTraitPtrs<N> medium_edge = edgesBetweenTwoHyperEdge(tg, data, data->close_edge_to(es[buffer_id], true, is_edge, current_time), es[buffer_id]);
                to_target_path.insert(to_target_path.begin(), medium_edge.begin(), medium_edge.end());
                to_target_path.insert(to_target_path.begin(), data->close_edge_to(es[buffer_id], true, is_edge, current_time));
                buffer_id = data->close_edge_to(es[buffer_id], true, is_edge, current_time);
            } else {
                break;
            }
            if(buffer_id == MAX<EdgeId>) break;
        }
        //std::cout << "to_target_path1 = " << toStr(to_target_path) << std::endl;
        // start to first hyper edge
        RoadMapEdgeTraitPtrs<N> from_start_path = edgesFromEdgeTo(tg, data, data->close_edge_to(es[buffer_id], true, is_edge, current_time), true, true);
        to_target_path.insert(to_target_path.begin(), from_start_path.begin(), from_start_path.end());
        //std::cout << "to_target_path2 = " << toStr(to_target_path) << std::endl;
        if(!PathContinuousCheck(to_target_path)) {
            std::cerr << "NTTCEP: noncontinuous path" << std::endl;
        }
        return to_target_path;
    }

    template <Dimension N>
    RoadMapEdgeTraitDeque<N> edgesDequeBetweenTwoHyperEdge(RoadMapGraphPtr<N>& tg,
                                                           DynamicDataOfSearchWithEdgePtr<N> data,
                                                           RoadMapEdgeTraitPtr<N> pre,
                                                           RoadMapEdgeTraitPtr<N> next) {
        if(!pre->isHyperLoopEdge() || !next->isHyperLoopEdge()) {
            return {};
        }
//        if(pre->nextHyperLoopEdges(true).find(next->edge_id()) == pre->nextHyperLoopEdges(true).end()) {
//            return {};
//        }
//        if(next->nextHyperLoopEdges(false).find(pre->edge_id()) == next->nextHyperLoopEdges(false).end()) {
//            return {};
//        }
        RoadMapEdgeTraitDeque<N> retv = {};
        EdgeId buffer_id = pre->edgeId();
        EdgeId next_buffer_id = MAX<EdgeId>;
        auto & es = tg->edges_;
        auto & ns = tg->nodes_;
        auto & hes = tg->hyper_edges_;

        while(buffer_id != MAX<EdgeId>) {
            next_buffer_id = MAX<EdgeId>;
            int count = 0;
            //std::cout << "flag 2 " << std::endl;
            for(const auto& candidate_id : es[buffer_id]->nextLoopEdges(true)) {
                //std::cout << "flag 3 " << std::endl;
//                if(candidate->nextNode(false)->sg_->id_ != buffer->nextNode(true)->sg_->id_) {
//                    std::cerr << "eBTHE: un match next loops " << std::endl;
//                }
                // notice: non-hyper-loop edge have only one preHyperLoopEdges or one nextHyperLoopEdges
                if(!es[candidate_id]->isHyperLoopEdge()
                   && hes[es[candidate_id]->nextHyperLoopEdges(true).front()]->edgeHeadId() == next->edgeId()
                        ) {
                    // filter that two next loop edges reach the same hyper loop edge
                    if((!retv.empty() && ns[es[candidate_id]->nextNode(false)]->sg_->id_ != ns[retv.back()->nextNode(true)]->sg_->id_)) continue;
                    next_buffer_id = candidate_id;
                    retv.push_back(es[candidate_id]);
                    count ++;
                }
            }
            //if(next_buffer_id == MAX<EdgeId>) std::cout << " find no buffer_id " << std::endl;
            //if(count > 1) { std::cerr << "eBTHE: " <<  count << " (> 1) un-hyper loop connect the same hyper edge" << std::endl; }
            //else { std::cout << "1 un-hyper loop connect the same hyper edge " << std::endl; }
            buffer_id = next_buffer_id;
        }
        return retv;
    }

    template <Dimension N>
    RoadMapEdgeTraitDeque<N> NodeTransformToContinuousEdgesWithEdge(RoadMapGraphPtr<N>& tg,
                                                                         DynamicDataOfSearchWithEdgePtr<N> data,
                                                                         const RoadMapEdgeTraitPtr<N>& edge,
                                                                         bool is_edge, Time current_time) {
        if(edge == nullptr) return {};
        RoadMapEdgeTraitDeque<N> to_target_path = edgesDequeFromEdgeTo(tg, data, edge, false, true);
        //to_target_path.resize(100);
        //std::cout << "to_target_path 1 = " << toStr(to_target_path) << std::endl;
        auto current_edge = edge->edgeId();
//        if(current_edge == nullptr) {
//            std::cerr << "NTTCEPWOL: shouldn't reach here" << std::endl;
//            return to_target_path;
//        }
        auto & es = tg->edges_;
        auto & ns = tg->nodes_;

        auto pre_edge = data->close_edge_to(es[current_edge], true, true, MIN_TIME);

        while(current_edge != MAX<EdgeId> && pre_edge != MAX<EdgeId>) {
            //std::cout << "flag 1 " << current_edge << " / " << pre_edge << std::endl;
            if(ns[es[current_edge]->nextNode(false)]->sg_->id_ != ns[es[pre_edge]->nextNode(true)]->sg_->id_) {
                const RoadMapEdgeTraitDeque<N>& medium_edge = edgesDequeBetweenTwoHyperEdge(tg, data, es[pre_edge], es[current_edge]);
                to_target_path.insert(to_target_path.begin(), medium_edge.begin(), medium_edge.end());
            }
            to_target_path.push_front(es[pre_edge]);
            //to_target_path.insert(to_target_path.begin(), pre_edge);
            current_edge = pre_edge;
            pre_edge = data->close_edge_to(es[pre_edge], true, true, MIN_TIME);
        }
        //std::cout << "to_target_path final = " << toStr(to_target_path) << std::endl;
        // test code
//        if(!PathContinuousCheck(to_target_path)) {
//            std::cerr << "NTTCEPWOL: noncontinuous path" << std::endl;
//        }
        return to_target_path;
    }

    template <Dimension N>
    RoadMapEdgeTraitDeque<N> PathPointTransformToContinuousEdgesDeque(RoadMapGraphPtr<N>& tg,
                                                                      DynamicDataOfSearchWithEdgePtr<N>& data,
                                                                      const PathPointWithEdgePtr<N>& path_point) {
        if(path_point == nullptr) return {};
        RoadMapEdgeTraitDeque<N> to_target_path = edgesDequeFromEdgeTo(tg, data, path_point->last_edge_, false, true);
        //to_target_path.resize(100);
        //std::cout << "to_target_path 1 = " << toStr(to_target_path) << std::endl;
        auto current_path = path_point;
        if(path_point == nullptr) {
            std::cerr << "PPTTCED: shouldn't reach here" << std::endl;
            return to_target_path;
        }
        auto pre_path = current_path->from();
        auto & ns = tg->nodes_;
        auto & hes = tg->hyper_edges_;
        while(current_path != nullptr && pre_path != nullptr) {
            //std::cout << "flag 1 " << current_edge << " / " << pre_edge << std::endl;
            if(ns[current_path->last_edge_->nextNode(false)]->sg_->id_ != ns[pre_path->last_edge_->nextNode(true)]->sg_->id_) {
                RoadMapEdgeTraitDeque<N> medium_edge = edgesDequeBetweenTwoHyperEdge(tg, data, pre_path->last_edge_, current_path->last_edge_);
                to_target_path.insert(to_target_path.begin(), medium_edge.begin(), medium_edge.end());
            }
            to_target_path.push_front(pre_path->last_edge_);
            //to_target_path.insert(to_target_path.begin(), pre_edge);
            current_path = pre_path;
            pre_path = pre_path->from();
        }

//        auto edge_from_start = edgesDequeFromEdgeTo(current_path->last_edge_, true, true);
//        edge_from_start.pop_back();
//        to_target_path.insert(to_target_path.begin(), edge_from_start.begin(), edge_from_start.end());
        //if(current_path != nullptr) to_target_path.push_front(current_path->last_edge_);
        if(!PathContinuousCheck(tg, to_target_path)) {
            std::cerr << "NTTCEPWOL: noncontinuous path : " << toStr(to_target_path) << std::endl;
        }
        return to_target_path;
    }

}
#endif //FREENAV_PATH_TREE_WITH_EDGE_H
