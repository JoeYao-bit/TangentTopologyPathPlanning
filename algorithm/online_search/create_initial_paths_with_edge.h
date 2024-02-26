//
// Created by yaozhuo on 2021/12/29.
//

#ifndef FREENAV_TOPO_CREATE_INITIAL_PATHS_WITH_EDGE_H
#define FREENAV_TOPO_CREATE_INITIAL_PATHS_WITH_EDGE_H

#include "search_path_with_edge.h"

namespace freeNav::Topo {

    template <Dimension N>
    void GeneralGraphPathPlannerWithEdge<N>::markEdge(const RoadMapEdgeTraitPtr<N>& edge_ptr, bool is_from_start) {
        //std::cout << edge_ptr << " 0 is hyper edge " << std::endl;
        if(data_->is_marked(edge_ptr, true, true, currentTime()) || data_->is_marked(edge_ptr, true, false, currentTime())) {
            return;
        } else {
            data_->is_marked(edge_ptr, true, is_from_start, currentTime()) = true;
        }

        if(edge_ptr->isHyperLoopEdge()) {
            //std::cout << edge_ptr << " 1 is hyper edge " << std::endl;
            return;
        }
        auto & ns = tg_->nodes_;
        auto & es = tg_->edges_;
        for(auto & next_edge_id : tg_->nextEdges(edge_ptr, is_from_start, true)) {
            if(is_from_start)
            {
                // 11.22 level cause failed in massive case
                if(edge_ptr->level() < es[next_edge_id]->level() || es[next_edge_id]->isLoopEdge())
                {
                    //std::cout << " start " << is_from_start << " / cur next " << edge_ptr->level() << " -> " << next_edge->level() << std::endl;
                    markEdge(es[next_edge_id], is_from_start);
                }
            }
            else {
                // 11.22 level cause failed in massive case
                bool legal_jump = tg_->isUndirectedGraph() ? edge_ptr->level() < es[next_edge_id]->level() : edge_ptr->level() > es[next_edge_id]->level();
                if(legal_jump || es[next_edge_id]->isLoopEdge())
                {
                    //std::cout << " start " << is_from_start << " / cur next " << edge_ptr->level() << " -> " << next_edge->level() << std::endl;
                    markEdge(es[next_edge_id], is_from_start);
                }
            }
        }
    }

    // not use tg->etcs_ for some acceleration
    // for example, ETC_NoUselessPoint3 is useless in this section
    template <Dimension N>
    PathPointWithEdgePtrs<N> GeneralGraphPathPlannerWithEdge<N>::createVisiblePathPointPtrsWithEdge(const Pointi<N>& start,
                                                                                                    const Pointi<N>& target,
                                                                                                    const PointTransferConstraints<N>& ptcs,
                                                                                                    const EdgeTransferConstraints3<N>& etcs) {
        auto retv_1 = createInitPathPtrsWithEdge(target, false, ptcs, etcs);
        auto retv_2 = createInitPathPtrsWithEdge(start, true, ptcs, etcs);
        //std::cout << "retv_1 size " << retv_1.size() << std::endl;
        //std::cout << "retv_2 size " << retv_2.size() << std::endl;
        if((depth_first_// || global_shortest_
           ) && retv_2.size() == 1 && isReachTarget(retv_2[0], true, currentTime())) {
            //std::cout << "reach target" << std::endl;
            return retv_2;
        }
        auto retv_3 = markInitEdges(retv_1, false, etcs);
        auto retv_4 = markInitEdges(retv_2, true, etcs);
        //std::cout << "retv_3 size " << retv_3.size() << std::endl;
        //std::cout << "retv_4 size " << retv_4.size() << std::endl;
        return retv_4;
    }

    template <Dimension N>
    std::pair<PathPointWithEdgePtrs < N>, PathPointWithEdgePtrs<N>>
    GeneralGraphPathPlannerWithEdge<N>::createVisiblePathPointPtrsPairWithEdge(
            const Pointi<N>& start,
            const Pointi<N>& target,
            const PointTransferConstraints<N>& ptcs,
            const EdgeTransferConstraints3<N>& etcs) {
        auto retv_1 = createInitPathPtrsWithEdge(target, false, ptcs, etcs);
        auto retv_2 = createInitPathPtrsWithEdge(start, true, ptcs, etcs);
        std::cout << "retv_1 size " << retv_1.size() << std::endl;
        std::cout << "retv_2 size " << retv_2.size() << std::endl;
        if((depth_first_// || global_shortest_
           ) && retv_2.size() == 1 && isReachTarget(retv_2[0], true, currentTime())) {
            std::cout << "reach target" << std::endl;
            return {retv_1, retv_2};
        }
        auto retv_3 = markInitEdges(retv_1, false, etcs);
        auto retv_4 = markInitEdges(retv_2, true, etcs);
        std::cout << "retv_3 size " << retv_3.size() << std::endl;
        std::cout << "retv_4 size " << retv_4.size() << std::endl;
        return {retv_4, retv_3};
    }

    // mark travelsable edge during, return which reach target
    template <Dimension N>
    PathPointWithEdgePtrs<N> GeneralGraphPathPlannerWithEdge<N>::createInitPathPtrsWithEdge(
                                                        const Pointi<N>& start,
                                                        bool is_from_start,
                                                        // not use tg->etcs_ for some acceleration
                                                        // for example, ETC_NoUselessPoint3 is useless in this section
                                                        const PointTransferConstraints<N>& ptcs,
                                                        const EdgeTransferConstraints3<N>& etcs) {
        // 1, check whether start is an existing tangent node
        bool new_node = false;
        RoadMapEdgeTraitPtrs<N> init_edges;
        const auto& dim = tg_->surface_processor_->getDimensionInfo();
        Id start_id = PointiToId(start, dim);
        RoadMapNodeTraitPtr<N> current_node_ptr = nullptr;
        //RoadMapEdgeTraitPtrs<N> retv;
        PathPointWithEdgePtrs<N> retv;
        int taut_count = 0;

        auto & ns = tg_->nodes_;
        auto & es = tg_->edges_;
        auto & hes = tg_->hyper_edges_;
        auto & dns = data_->node_dynamic_datas_;
        auto & des = data_->edge_dynamic_datas_;

        // 3, create new node
        current_node_ptr = std::make_shared<RoadMapNodeTrait<N>>();
        if(tg_->surface_processor_->grid_map_[start_id] == nullptr) {
            current_node_ptr->sg_ = std::make_shared<Grid<N> >();
            current_node_ptr->sg_->id_ = start_id;
            current_node_ptr->sg_->pt_ = start;
            current_node_ptr->sg_->node_id_ = tg_->nodes_.size(); // set new node id
            new_node = true;
            tg_->nodes_.push_back(current_node_ptr); // insert new node to all nodes
            dns.push_back(std::make_shared<NodeDynamicDataWithEdge>()); // allocate dynamic data of new node
        } else {
            current_node_ptr->sg_ = tg_->surface_processor_->grid_map_[start_id];
        }


        // 4, create new edges of new nodes
        GridPtrs<N> tcs = tg_->surface_processor_->getVisibleTangentCandidates(start);
        //std::cout << "is_from_start " << is_from_start << " tcs size " << tcs.size() << std::endl;
        bool is_legal = true;
        for(const auto& tc : tcs) {
            // TODO: considering from different orientation
            if (!isPointTransferLegal(tg_, current_node_ptr->sg_, tc, tg_->surface_processor_->is_occupied_,
                                      tg_->surface_processor_->getNeighbor(), ptcs)) { continue; }

            //std::cout << " tc: " << tc->pt_ << std::endl;
            //tg_->node_updateUsedTicket(tc->node_id_, currentTime());
            is_legal = true;
            const NodeId &pt_from = is_from_start ? current_node_ptr->sg_->node_id_ : ns[tc->node_id_]->sg_->node_id_;
            const NodeId &pt_to = is_from_start ? ns[tc->node_id_]->sg_->node_id_ : current_node_ptr->sg_->node_id_;

            //std::cout << " pt_from: " << pt_from << " / pt_to " << pt_to << std::endl;

            RoadMapEdgeTraitPtr<N> new_edge =
                    std::make_shared<RoadMapEdgeTrait<N> >(pt_from, pt_to, (current_node_ptr->sg_->pt_ - ns[tc->node_id_]->sg_->pt_).Norm());
            new_edge->edgeId() = tg_->edges_.size();

            tg_->edges_.push_back(new_edge);
            des.push_back(std::make_shared<EdgeDynamicDataWithEdge>());

//            std::cout << " flag 1 " << std::endl;
//            std::cout << " ns size " << ns.size() << std::endl;
//            std::cout << " es size " << es.size() << std::endl;
//            std::cout << " dns size " << dns.size() << std::endl;
//            std::cout << " des size " << des.size() << std::endl;
//            std::cout << " new_edge->nextNode(is_from_start) " << new_edge->nextNode(is_from_start) << std::endl;
            data_->init_edge_from(ns[new_edge->nextNode(is_from_start)], is_from_start, currentTime()) = new_edge->edgeId();
            //std::cout << " flag 2 " << std::endl;
            data_->dist_to(new_edge, is_from_start, true, currentTime()) = new_edge->length();
            data_->dist_to(new_edge, is_from_start, false, currentTime()) = new_edge->length();
            //std::cout << " flag 3 " << std::endl;
            // if both start and target visible to the same node, means three points may form a path
            EdgeId &another_close_to = data_->init_edge_from(ns[new_edge->nextNode(is_from_start)], !is_from_start, currentTime());
            //std::cout << " flag 4 " << std::endl;
            if (another_close_to != MAX<EdgeId>) {

                RoadMapNodeTraitPtr<N> p1, p2, p3;
                //std::cout << " flag 5 " << std::endl;
                p1 = ns[es[data_->init_edge_from(ns[new_edge->nextNode(is_from_start)], true, currentTime())]->nextNode(false)];
                //std::cout << " flag 6 " << std::endl;
                p2 = ns[new_edge->nextNode(is_from_start)];
                //std::cout << " flag 7 " << std::endl;
                p3 = ns[es[data_->init_edge_from(ns[new_edge->nextNode(is_from_start)], false, currentTime())]->nextNode(true)];
                //std::cout << " flag 8 " << std::endl;
                // if three point form a legal edge transfer, add it as path
                if (isEdgeTransferLegal(tg_, p1, p2, p3, tg_->etcs_)) {
                    //std::cout << "reach target" << std::endl;
                    data_->close_edge_to(new_edge, !is_from_start, true, currentTime()) = another_close_to;
                    data_->dist_to(new_edge, !is_from_start, true, currentTime())  = es[another_close_to]->length() + new_edge->length();
                    data_->dist_to(new_edge, !is_from_start, false, currentTime()) = es[another_close_to]->length() + new_edge->length();
                    if (depth_first_// || global_shortest_
                            ) {
                        retv.clear();
                        auto new_path_ptr = std::make_shared<PathPointWithEdge<N> >(tg_, nullptr, new_edge);
                        retv.push_back(new_path_ptr);
                        return retv;
                    }
                }
            }
            //std::cout << " flag 5 " << std::endl;
            new_edge->level() = is_from_start ? -1 : ( tg_->isUndirectedGraph() ? -1 : MAX<Lv> - 1); // start must increase level, target must decrease level
            auto new_path_ptr = std::make_shared<PathPointWithEdge<N> >(tg_, nullptr, new_edge);
            retv.push_back(new_path_ptr);
            //std::cout << " tc edge = " << std::endl;//new_edge->printStr(true, true) << std::endl;
        }
        //std::cout << " RJ taut_count " << taut_count << std::endl;
        return retv;
    }

    template <Dimension N>
    PathPointWithEdgePtrs<N> GeneralGraphPathPlannerWithEdge<N>::markInitEdges(const PathPointWithEdgePtrs<N>& init_paths,
                                                                               bool is_from_start,
                                                                               const EdgeTransferConstraints3<N>& etcs) {

        auto & ns  = tg_->nodes_;
        auto & es  = tg_->edges_;
        auto & hes = tg_->hyper_edges_;
        auto & dns = data_->node_dynamic_datas_;
        auto & des = data_->edge_dynamic_datas_;

        PathPointWithEdgePtrs<N> retv;

        for(const auto& init_path : init_paths) {
            if(is_from_start && isReachTarget(init_path->last_edge_, true, currentTime())) {
                retv.push_back(init_path);
            }
            for (const auto & next_edge_id : tg_->nextEdges(init_path->last_edge_, is_from_start, false)) {
                bool is_legal = is_from_start ?  isEdgeTransferLegal(tg_, ns[init_path->last_edge_->nextNode(false)],  ns[es[next_edge_id]->nextNode(false)], ns[es[next_edge_id]->nextNode(true)], etcs)
                                               : isEdgeTransferLegal(tg_, ns[es[next_edge_id]->nextNode(false)],       ns[es[next_edge_id]->nextNode(true)],  ns[es[init_path->last_edge_->edgeId()]->nextNode(true)],  etcs);
                if(is_legal) {
                    data_->dist_to(es[next_edge_id], is_from_start, true, currentTime())       = init_path->last_edge_->length() + es[next_edge_id]->length();
                    data_->dist_to(es[next_edge_id], is_from_start, false, currentTime())      = init_path->last_edge_->length() + es[next_edge_id]->length();
                    data_->close_edge_to(es[next_edge_id], is_from_start, true, currentTime()) = init_path->last_edge_->edgeId();

                    markEdge(es[next_edge_id], is_from_start);
                    auto new_path_ptr = std::make_shared<PathPointWithEdge<N> >(tg_, init_path, es[next_edge_id]);
                    retv.push_back(new_path_ptr);

                }
            }
        }
        return retv;
    }

}

#endif //FREENAV_CREATE_INITIAL_PATHS_WITH_EDGE_H
