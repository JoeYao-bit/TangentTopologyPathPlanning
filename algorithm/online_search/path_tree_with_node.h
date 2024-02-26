//
// Created by yaozhuo on 2023/2/12.
//

#ifndef FREENAV_TOPO_PATH_TREE_WITHOUT_EDGE_H
#define FREENAV_TOPO_PATH_TREE_WITHOUT_EDGE_H

#include "search_path_with_edge.h"

namespace freeNav::Topo {

    template <Dimension N> class PathPointWithNode;

    template <Dimension N>
    using PathPointWithNodePtr = std::shared_ptr<PathPointWithNode<N> >;

    template <Dimension N>
    using PathPointWithNodePtrs = std::vector<PathPointWithNodePtr<N> >;

    template <Dimension N>
    class PathPointWithNode {
    public:

        explicit PathPointWithNode(RoadMapGraphPtr<N>& tg, const PathPointWithNodePtr<N>& from, const RoadMapNodeTraitPtr<N>& node)
                : from_(from), is_legal_(true), last_node_(node) {
            if(node == nullptr) {
                exit(0);
            }
            if(from != nullptr) {
                length_ = from_->length_ + (from->last_node_->sg_->pt_ - node->sg_->pt_).Norm();
            } else {
                length_ = 0;
            }
        }


        void addNext(const PathPointWithNodePtr<N>& pt) {
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
            //
        }

        PathPointWithNodePtr<N> from() {
            return from_;
        }

        PathPointWithNodePtr<N> to() {
            return to_;
        }

        RoadMapNodeTraitPtr<N> last_node_; // the tangent point that the path point corresponding to

        bool is_legal_; // used in lag delete

        PathLen length_ = 0; // dist from start to here

        PathLen length_to_target_ = 0;//

    private:

        PathPointWithNodePtr<N> from_ = nullptr; // the father node of the node

        PathPointWithNodePtrs<N> to_; // the child nodes of the node

    };

    template <Dimension N>
    struct PathPointWithNodePtrCompare
    {
        bool operator()(PathPointWithNodePtr<N> a, PathPointWithNodePtr<N> b){
            return a->length_to_target_ > b->length_to_target_;
        }
    };

    template <Dimension N>
    RoadMapNodeTraitDeque<N> NodeTransformToContinuousEdgesWithNode(RoadMapGraphPtr<N>& tg,
                                                                    DynamicDataOfSearchWithNodePtr<N> data,
                                                                    const RoadMapNodeTraitPtr<N>& node,
                                                                    const Time& current_time) {
        //std::cout << __FUNCTION__ << " start" << std::endl;
        auto & ns = tg->nodes_;
        NodeId current_node_id = node->sg_->node_id_;
        RoadMapNodeTraitDeque<N> retv;
        if(data->close_node_to(node, false, current_time) != MAX<NodeId>) {
            retv.push_back(ns[data->close_node_to(node, false, current_time)]);
        }
        while(current_node_id != MAX<NodeId>) {
            retv.push_back(ns[current_node_id]);
            if(current_node_id == data->close_node_to(ns[current_node_id], true, current_time)) {
                std::cout << " loop in path node id " << current_node_id << std::endl;
                exit(0);
            }
            //std::cout << " current_node_id " << current_node_id << std::endl;
            current_node_id = data->close_node_to(ns[current_node_id], true, current_time);
        }
        return retv;
    }

    template <Dimension N>
    RoadMapNodeTraitDeque<N> PathTransformToContinuousEdgesWithNode(RoadMapGraphPtr<N>& tg,
                                                                    DynamicDataOfSearchWithNodePtr<N> data,
                                                                    const PathPointWithNodePtr<N>& path,
                                                                    const Time& current_time) {
        //std::cout << __FUNCTION__ << " start" << std::endl;
        if(path == nullptr) return {};
        if(path->last_node_ == nullptr) return {};
        auto & ns = tg->nodes_;
        RoadMapNodeTraitDeque<N> retv;
        if(data->close_node_to(path->last_node_, false, current_time) != MAX<NodeId>) {
            retv.push_back(ns[data->close_node_to(path->last_node_, false, current_time)]);
        }
        auto current_path = path;
        while(current_path != nullptr) {
            retv.push_back(current_path->last_node_);
            //std::cout << " current_node_id " << current_node_id << std::endl;
            current_path = current_path->from();
        }
        return retv;
    }

    template<Dimension N>
    Pointis<N> ContinuousNodesToPath(const RoadMapNodeTraitDeque<N>& path) {
        if(path.empty()) return {};
        Pointis<N> retv;
        for(int i=0; i<path.size(); i++) {
            retv.push_back(path[i]->sg_->pt_);
        }
        return retv;
    }

};


#endif //FREENAV_PATH_TREE_WITHOUT_EDGE_H
