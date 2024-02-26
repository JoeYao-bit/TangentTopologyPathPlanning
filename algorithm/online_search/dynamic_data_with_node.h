//
// Created by yaozhuo on 2023/2/12.
//

#ifndef FREENAV_TOPO_DYNAMIC_DATA_WITH_NODE_H
#define FREENAV_TOPO_DYNAMIC_DATA_WITH_NODE_H

#include "../../algorithm/graph_construction/tangent_graph.h"

namespace freeNav::Topo {

    struct NodeDynamicDataWithNode {

        bool is_expanded_ = false; // used for accelerate

        Time used_ticket_ = 0;

        PathLen min_length_to_start_ = MAX<PathLen>; // the minimum of min_length_to_art_ of all edge that reach here
        NodeId close_to_start_node_ = MAX<NodeId>; // the shortest node from start to here

        PathLen min_length_to_target_ = MAX<PathLen>; // the minimum of min_length_to_target_ of all edge that reach here
        NodeId close_to_target_node_ = MAX<NodeId>; // the shortest node from target to here

        std::shared_ptr<NodeId> index_in_heap_ = std::make_shared<NodeId>(MAX<NodeId>);

    };



    typedef std::shared_ptr<NodeDynamicDataWithNode> NodeDynamicNodeDataPtr;

    template<Dimension N>
    struct DynamicDataOfSearchWithNode {

    public:

        explicit DynamicDataOfSearchWithNode(const RoadMapGraphPtr<N>& tg) {
            tg_ = tg;
            resetGraphInnerData();
        }

        void resetGraphInnerData() {
            node_dynamic_datas_.clear();
            for(int i=0; i<tg_->static_node_count_; i++) {
                node_dynamic_datas_.push_back(std::make_shared<NodeDynamicDataWithNode>());
            }
            //std::cout << " tg_->static_node_count_ " << tg_->static_node_count_ << std::endl;
            //std::cout << " node_dynamic_datas_ size " << node_dynamic_datas_.size() << std::endl;
            //std::cout << " tg_ node size " << tg_->nodes_.size() << std::endl;
        }

        // when use this edge, remove its previous data
        inline void updateUsedTicket(const RoadMapNodeTraitPtr <N> &node, const Time &used_ticket) {
            //std::cout << " node->sg_->node_id_ " << node->sg_->node_id_ << std::endl;
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            if (used_ticket > node_data->used_ticket_) {
                node_data->used_ticket_ = used_ticket;
                node_data->is_expanded_ = false;
                node_data->min_length_to_target_ = MAX<PathLen>;
                node_data->close_to_target_node_ = MAX<NodeId>;

                node_data->min_length_to_start_ = MAX<PathLen>;
                node_data->close_to_start_node_ = MAX<NodeId>;

                * node_data->index_in_heap_ = MAX<NodeId>;

            }
        }


        inline NodeId& close_node_to(const RoadMapNodeTraitPtr <N> &node,
                                     bool is_start,
                                     const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return is_start ? node_data->close_to_start_node_
                            : node_data->close_to_target_node_;
        }

        inline bool& is_expanded(const RoadMapNodeTraitPtr <N> &node,
                                   const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            //std::cout << __FUNCTION__ << " " << node->sg_->node_id_ << std::endl;
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return node_data->is_expanded_;
        }

        inline std::shared_ptr<NodeId>& index_in_heap(const RoadMapNodeTraitPtr <N> &node,
                                     const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            //std::cout << __FUNCTION__ << " " << node->sg_->node_id_ << std::endl;
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return node_data->index_in_heap_;
        }

        // no edge pre computation
        inline PathLen &dist_to(const RoadMapNodeTraitPtr <N> &node,
                                bool is_start,
                                const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return is_start ? node_data->min_length_to_start_ : node_data->min_length_to_target_;
        }

        void pruneToStaticNodeAndEdge() {
            node_dynamic_datas_.resize(tg_->static_node_count_);
        }

    private:

        std::vector<NodeDynamicNodeDataPtr> node_dynamic_datas_;


        RoadMapGraphPtr<N> tg_;

        friend class GeneralGraphPathPlannerWithNode<N>;


    };

};

#endif //FREENAV_DYNAMIC_DATA_WITH_NODE_H
