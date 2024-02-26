//
// Created by yaozhuo on 2023/2/6.
//

#ifndef FREENAV_TOPO_DYNAMIC_DATA_WITH_EDGE_H
#define FREENAV_TOPO_DYNAMIC_DATA_WITH_EDGE_H

#include "../../algorithm/graph_construction/tangent_graph.h"

namespace freeNav::Topo {

    struct EdgeDynamicDataWithEdge {

        /* dynamic updated information during path search */
        Time used_ticket_ = 0; // use time ticket so only reset when next time use it
        Time used_ticket_for_mark_ = 0; // use time ticket so only reset when next time use it

        bool is_expanded_ = false;

        bool marked_from_start_ = false;
        bool marked_from_target_ = false;


        // min_length_to_target_ is related to edge, it may infinite or finite when the same node on different edge
        // replace this with shortest_path_point_to_target_
        PathLen min_length_to_target_ = MAX<PathLen>; // the minimum length of path that reach here

        // if from_ to target dist is not infinite, close_to_target_edge_ != nullptr
        // current edge from current node to target
        EdgeId close_to_target_edge_ = MAX<EdgeId>; // the shortest edge from target to here

        PathLen min_length_to_start_ = MAX<PathLen>; // the minimum length of path that reach here
        EdgeId close_to_start_edge_ = MAX<EdgeId>; // the shortest edge from start to here

        // store the edge on which edge, of course the edge always on itself
        //EdgeId current_on_edge_ = MAX<EdgeId>;

        std::shared_ptr<NodeId> index_in_heap_ = std::make_shared<NodeId>(MAX<NodeId>);

    };

    struct NodeDynamicDataWithEdge {

        //private:

        bool marked_from_start_ = false;
        bool marked_from_target_ = false;

        bool is_expanded_ = false; // used for accelerate

        EdgeId init_edge_from_start_ = MAX<EdgeId>, init_edge_from_target_ = MAX<EdgeId>;

        Time used_ticket_ = 0;

        Time used_ticket_for_mark_ = 0;

        PathLen min_length_to_start_ = MAX<PathLen>; // the minimum of min_length_to_art_ of all edge that reach here
        EdgeId close_to_start_edge_ = MAX<EdgeId>; // the shortest edge from start to here


        // store the node on which edge
        EdgeId current_on_edge_ = MAX<EdgeId>;

        PathLen min_length_to_target_ = MAX<PathLen>; // the minimum of min_length_to_target_ of all edge that reach here
        EdgeId close_to_target_edge_ = MAX<EdgeId>; // the shortest edge from target to here

        std::shared_ptr<NodeId> index_in_heap_ = std::make_shared<NodeId>(MAX<NodeId>);

    };


    template<Dimension N>
    struct DynamicDataOfSearchWithEdge {

        explicit DynamicDataOfSearchWithEdge(const RoadMapGraphPtr<N>& tg) {
            tg_ = tg;
            resetGraphInnerData();
        }

        void resetGraphInnerData() {
            edge_dynamic_datas_.clear();
            node_dynamic_datas_.clear();
            for(int i=0; i<tg_->static_edge_count_; i++) {
                edge_dynamic_datas_.push_back(std::make_shared<EdgeDynamicDataWithEdge>());
            }
            for(int i=0; i<tg_->static_node_count_; i++) {
                node_dynamic_datas_.push_back(std::make_shared<NodeDynamicDataWithEdge>());
            }
        }

        void pruneToStaticNodeAndEdge() {
            node_dynamic_datas_.resize(tg_->static_node_count_);
            edge_dynamic_datas_.resize(tg_->static_edge_count_);
        }

        // no edge pre computation
        inline bool & is_marked(const RoadMapNodeTraitPtr <N> &node,
                                const bool &is_from_start,
                                const Time &used_ticket) {
            updateUsedTicketForMark(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return is_from_start ? node_data->marked_from_start_ : node_data->marked_from_target_;
        }

        // with edge pre computation
        inline bool &is_marked(const RoadMapEdgeTraitPtr <N> &edge,
                               const bool &edge_mode,
                               const bool &is_from_start,
                               const Time &used_ticket) {
            updateUsedTicketForMark(edge, used_ticket);
            const auto &ns = tg_->nodes_;
            auto &edge_data = edge_dynamic_datas_[edge->edgeId()];
            //std::cout << " edge->edgeId() " << edge->edgeId() << std::endl;
            return edge_mode ? (is_from_start ? edge_data->marked_from_start_ : edge_data->marked_from_target_)
                             : is_marked(ns[edge->nextNode(is_from_start)], is_from_start, used_ticket);
        }

        // no edge pre computation
        inline bool &is_expanded(const RoadMapNodeTraitPtr <N> &node,
                                 const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return node_data->is_expanded_;
        }

        // with edge pre computation
        inline bool &is_expanded(const RoadMapEdgeTraitPtr <N> &edge,
                                 const bool &is_edge,
                                 const Time &used_ticket) {
            updateUsedTicket(edge, used_ticket);
            auto &edge_data = edge_dynamic_datas_[edge->edge_id_];
            const auto &ns = tg_->nodes_;
            if (is_edge) {
                return edge_data->is_expanded_;
            } else {
                return is_expanded(ns[edge->to_], used_ticket);
            }
        }

        // TODO: set dist from start to to_ as min_length_to_start_
        //       and set dist from target to from_ as min_length_to_target_
        // with edge pre computation
        inline PathLen &dist_to(const RoadMapEdgeTraitPtr <N> &edge,
                                const bool &is_start,
                                const bool &is_edge,
                                const Time &used_ticket) {
            auto &edge_data = edge_dynamic_datas_[edge->edgeId()];
            const auto &ns = tg_->nodes_;
            if (is_edge) updateUsedTicket(edge, used_ticket);
            if (is_start) {
                if (is_edge) {
                    return edge_data->min_length_to_start_;
                } else {
                    return dist_to(ns[edge->nextNode(true)], true, used_ticket);
                }
            } else {
                if (is_edge) {
                    return edge_data->min_length_to_target_;
                } else {
                    return dist_to(ns[edge->nextNode(false)], false, used_ticket);
                }
            }
        }

        // no edge pre computation
        inline PathLen &dist_to(const RoadMapNodeTraitPtr <N> &node,
                                bool is_start,
                                const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return is_start ? node_data->min_length_to_start_ : node_data->min_length_to_target_;
        }

        // with edge pre computation
        inline EdgeId & current_on_edge(const RoadMapEdgeTraitPtr <N> &edge,
                                                         const bool &is_edge,
                                                         const Time &used_ticket) {
            updateUsedTicket(edge, used_ticket);
            const auto &ns = tg_->nodes_;
            auto &edge_data = edge_dynamic_datas_[edge->edge_id_];
            if (is_edge) { return edge->edgeId(); }
            else { return current_on_edge(ns[edge->to_], used_ticket); }
        }

        // with edge pre computation
        inline EdgeId& close_edge_to(const RoadMapEdgeTraitPtr <N> &edge,
                                                       const bool &is_start,
                                                       const bool &is_edge,
                                                       const Time &used_ticket) {
            updateUsedTicket(edge, used_ticket);
            if (is_start) {
                return close_to_start_edge(edge, is_edge, used_ticket);
            } else {
                return close_to_target_edge(edge, is_edge, used_ticket);
            }
        }

        inline EdgeId & init_edge_from(const RoadMapNodeTraitPtr <N> &node,
                                                        bool is_start,
                                                        const Time &used_ticket) {
            //std::cout << "flag1.1" << std::endl;
            updateUsedTicket(node, used_ticket);
            //std::cout << "flag1.2" << std::endl;
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            //std::cout << "flag1.3" << std::endl;
            //std::cout << " node_data->init_edge_from_start_ " << node_data->init_edge_from_start_ << std::endl;
            //std::cout << " node_data->init_edge_from_target_ " << node_data->init_edge_from_target_ << std::endl;
            return is_start ? node_data->init_edge_from_start_ : node_data->init_edge_from_target_;
        }

        inline std::shared_ptr<NodeId>& index_in_heap(const RoadMapNodeTraitPtr <N> &node,
                                                      const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            //std::cout << __FUNCTION__ << " " << node->sg_->node_id_ << std::endl;
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return node_data->index_in_heap_;
        }

        inline std::shared_ptr<NodeId>& index_in_heap(const RoadMapEdgeTraitPtr <N> &edge,
                                                      const Time &used_ticket) {
            updateUsedTicket(edge, used_ticket);
            //std::cout << __FUNCTION__ << " " << node->sg_->node_id_ << std::endl;
            auto &edge_data = edge_dynamic_datas_[edge->edge_id_];
            return edge_data->index_in_heap_;
        }

    private:

        // when use this edge, remove its previous data
        inline void updateUsedTicket(const RoadMapNodeTraitPtr <N> &node, const Time &used_ticket) {
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            if (used_ticket > node_data->used_ticket_) {

                node_data->used_ticket_ = used_ticket;

                node_data->is_expanded_ = false;

                node_data->min_length_to_target_ = MAX<PathLen>;
                node_data->close_to_target_edge_ = MAX<EdgeId>;

                node_data->min_length_to_start_ = MAX<PathLen>;
                node_data->close_to_start_edge_ = MAX<EdgeId>;

                node_data->current_on_edge_ = MAX<EdgeId>;

                node_data->init_edge_from_start_ = MAX<EdgeId>;
                node_data->init_edge_from_target_ = MAX<EdgeId>;

                * node_data->index_in_heap_ = MAX<NodeId>;

            }
        }

        // when use this edge, remove its previous data
        inline void updateUsedTicketForMark(const RoadMapNodeTraitPtr <N> &node, const Time &used_ticket) {
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            if (used_ticket > node_data->used_ticket_for_mark_) {
                node_data->used_ticket_for_mark_ = used_ticket;
                node_data->marked_from_start_ = false;
                node_data->marked_from_target_ = false;
            }
        }

        // when use this edge, remove its previous data
        inline void updateUsedTicket(const RoadMapEdgeTraitPtr <N> &edge, const Time &used_ticket) {
            auto &edge_data = edge_dynamic_datas_[edge->edgeId()];
            if (used_ticket > edge_data->used_ticket_) {
                edge_data->used_ticket_ = used_ticket;
                edge_data->is_expanded_ = false;
                edge_data->min_length_to_target_ = MAX<PathLen>;
                edge_data->close_to_target_edge_ = MAX<EdgeId>;

                edge_data->min_length_to_start_ = MAX<PathLen>;
                edge_data->close_to_start_edge_ = MAX<EdgeId>;

                * edge_data->index_in_heap_ = MAX<NodeId>;

            }
        }

        // when use this edge, remove its previous data
        inline void updateUsedTicketForMark(const RoadMapEdgeTraitPtr <N> &edge, const Time &used_ticket) {
            auto &edge_data = edge_dynamic_datas_[edge->edgeId()];
            if (used_ticket > edge_data->used_ticket_for_mark_) {
                edge_data->used_ticket_for_mark_ = used_ticket;
                edge_data->marked_from_start_ = false;
                edge_data->marked_from_target_ = false;
            }
        }

        inline EdgeId& close_edge_to(const RoadMapNodeTraitPtr <N> &node,
                                                       bool is_start,
                                                       const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return is_start ? node_data->close_to_start_edge_
                            : node_data->close_to_target_edge_;
        }

        inline EdgeId& close_to_start_edge(const RoadMapEdgeTraitPtr <N> &edge,
                                                             const bool &is_edge,
                                                             const Time &used_ticket) {
            updateUsedTicket(edge, used_ticket);
            auto &edge_data = edge_dynamic_datas_[edge->edgeId()];
            if (is_edge) {
                return edge_data->close_to_start_edge_;
            } else {
                return close_edge_to(tg_->nodes_[edge->nextNode(true)], true, used_ticket);
            }
        }

        inline EdgeId& close_to_target_edge(const RoadMapEdgeTraitPtr <N> &edge,
                                                              const bool &is_edge,
                                                              const Time &used_ticket) {
            updateUsedTicket(edge, used_ticket);
            auto &edge_data = edge_dynamic_datas_[edge->edgeId()];
            if (is_edge) {
                return edge_data->close_to_target_edge_;
            } else {
                return close_edge_to(tg_->nodes_[edge->nextNode(true)], false, used_ticket);
            }
        }

        // with edge pre computation
        inline EdgeId& current_on_edge(const RoadMapNodeTraitPtr <N> &node,
                                                        const Time &used_ticket) {
            updateUsedTicket(node, used_ticket);
            auto &node_data = node_dynamic_datas_[node->sg_->node_id_];
            return node_data->current_on_edge_;
        }

        typedef std::shared_ptr<NodeDynamicDataWithEdge> NodeDynamicEdgeDataPtr;
        typedef std::shared_ptr<EdgeDynamicDataWithEdge> EdgeDynamicEdgeDataPtr;

        std::vector<EdgeDynamicEdgeDataPtr> edge_dynamic_datas_;
        std::vector<NodeDynamicEdgeDataPtr> node_dynamic_datas_;

        RoadMapGraphPtr<N> tg_;

        friend class GeneralGraphPathPlannerWithEdge<N>;

    };
}
#endif //FREENAV_DYNAMIC_DATA_WITH_EDGE_H
