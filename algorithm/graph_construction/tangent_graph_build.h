//
// Created by yaozhuo on 2022/1/5.
//

#ifndef FREENAV_TOPO_TANGENT_GRAPH_BUILD_H
#define FREENAV_TOPO_TANGENT_GRAPH_BUILD_H
#include <iomanip>
#include <fstream>
#include <climits>
#include <sys/time.h>
#include <unistd.h>

#include "../../freeNav-base/basic_elements/surface_process.h"
#include "../../freeNav-base/dependencies/thread_pool.h"
#include "../../algorithm/constraints/point_to_point_constraints.h"
#include "../../algorithm/graph_construction/tangent_graph.h"

namespace freeNav::Topo {


    // set from and to pointer of current graph, remember use this when you copy a tangent graph
    // otherwise these pointer will point to the element
    template<Dimension N>
    void updateRoadMapEdgePreAndNext(RoadMapGraphPtr<N>& tg) {
        for(int i=0; i<tg->nodes_.size(); i++) {
            auto& node = tg->nodes_[i];
            for(auto& edge_id :node->split_edges_) {
                if(tg->edges_[edge_id] != nullptr) {
                    tg->edges_[edge_id]->next_node(false) = i;
                }
                auto reverse_edge = reverse(edge_id);
                if(tg->edges_[reverse_edge] != nullptr) {
                    tg->edges_[reverse_edge]->next_node(true) = i;
                }
            }
        }
    }

    // whether edge meet all input ptcs
    template<Dimension N>
    bool isPointTransferLegal(RoadMapGraphPtr<N>& tg,
                              const GridPtr<N>& pt1,
                              const GridPtr<N>& pt2,
                              IS_OCCUPIED_FUNC<N> is_occupied,
                              const Pointis<N-1>& neightbor,
                              const PointTransferConstraints<N>& ptcs) {
        bool is_legal = true;
        for(const auto& ptc : ptcs) {
            if(!ptc(tg, pt1, pt2, is_occupied, neightbor)) {
                is_legal = false;
                break;
            }
        }
        return is_legal;
    }

    // whether edge meet all input ETC
    template<Dimension N>
    bool isEdgeTransferLegal(RoadMapGraphPtr<N>& tg,
                             const RoadMapEdgeTraitPtr<N>& edge1,
                             const RoadMapEdgeTraitPtr<N>& edge2,
                             const EdgeTransferConstraints<N>& etcs) {
        bool is_legal = true;
        for(const auto& etc : etcs) {
            if(!etc(tg, edge1, edge2)) {
                is_legal = false;
                break;
            }
        }
        return is_legal;
    }

    // whether edge meet all input ETC
    template<Dimension N>
    bool isEdgeTransferLegal(RoadMapGraphPtr<N>& tg,
                             const RoadMapNodeTraitPtr<N>& node1,
                             const RoadMapNodeTraitPtr<N>& node2,
                             const RoadMapNodeTraitPtr<N>& node3,
                             const EdgeTransferConstraints<N>& etcs) {
        bool is_legal = true;
        for(const auto& etc : etcs) {
            if(!etc(tg, node1, node2, node3)) {
                is_legal = false;
                break;
            }
        }
        return is_legal;
    }

    // whether edge meet all input ETC
    template<Dimension N>
    bool isEdgeTransferLegal(RoadMapGraphPtr<N>& tg,
                             const RoadMapEdgeTraitPtr<N>& edge1,
                             const RoadMapEdgeTraitPtr<N>& edge2,
                             const EdgeTransferConstraints3<N>& etcs) {
        bool is_legal = true;
        for(const auto& etc : etcs) {
            if(!etc(tg, tg->nodes_[edge1->nextNode(false)], tg->nodes_[edge2->nextNode(false)], tg->nodes_[edge2->nextNode(true)])) {
                is_legal = false;
                break;
            }
        }
        return is_legal;
    }

    // whether edge meet all input ETC
    template<Dimension N>
    bool isEdgeTransferLegal(RoadMapGraphPtr<N>& tg,
                             const RoadMapNodePtr<N>& node1,
                             const RoadMapNodePtr<N>& node2,
                             const RoadMapNodePtr<N>& node3,
                             const EdgeTransferConstraints3<N>& etcs) {
        bool is_legal = true;
        for(const auto& etc : etcs) {
            if(!etc(tg, node1, node2, node3)) {
                is_legal = false;
                break;
            }
        }
        return is_legal;
    }


template <Dimension N>
    class RoadMapGraphBuilder {
    public:

        // load the tangent graph from a binary file
        explicit RoadMapGraphBuilder(SurfaceProcessorPtr<N> surface_processor,
                                     const PointTransferConstraints<N>&  unordered_constraints,
                                     const PointTransferConstraints<N>&  ordered_constraints,
                                     const EdgeTransferConstraints3<N>&  etc_constraints,
                                     const std::string& file_path = "",
                                     bool force_update = false, // whether calculate graph again and overwrite the previous in file_path
                                     bool static_edge_connection = true, // whether calculate edge connection in advance
                                     int max_thread = 3
        ): surface_processor_(surface_processor),
           unordered_ptcs_(unordered_constraints),
           ordered_ptcs_(ordered_constraints),
           etcs_(etc_constraints),
           //file_path_(file_path),
           static_edge_connection_(static_edge_connection),
           //thread_pool_(max_thread), // main thread and other max_thread - 1 thread
           max_thread_(max_thread)
        {
            surface_processor_->initialize();
            road_map_graph_ = std::make_shared<RoadMapGraph<N> >(unordered_constraints, ordered_constraints, etc_constraints);
            road_map_graph_->surface_processor_ = surface_processor_;
            initNodes();
            if(!force_update && roadMapGraphDeserialize(file_path)) {
                std::cout << "-- load tangent graph from " << file_path << " success " << std::endl;
            } else
                {
                std::cout << "-- load tangent graph from " << file_path << " failed, try to build" << std::endl;
                if(!buildRoadMapGraphMultiThread()) { // faster but not exit normally, need restart program for path planning
                //if(!buildRoadMapGraph()) {
                    std::cout << "-- build tangent graph failed " << std::endl;
                    exit(0);
                }
                if(roadMapGraphSerialize(file_path)) {
                    std::cout << "-- save tangent graph success " << std::endl;
                } else {
                    std::cout << "-- save tangent graph failed " << std::endl;
                }
            }
            // update pre_edges_ and next_edges_ of edge
            if(static_edge_connection) {
                updateConnectionBetweenRoadMapEdges();
                if(ordered_ptcs_.empty()) {
                    if(!road_map_graph_->UndirectedGraphCheck()) {
                        std::cout << " FATAL: ordered_ptcs_.empty() but UndirectedGraphCheck failed " << std::endl;
                    }
                }
                // update edge LEVEL value
                computeAllEdgeLevels(); // ENLSVG style detect loop
                updateHyperNodeState();
            }
        }

        // set grid_map's road_map_node_ptr_ of surface_processor
        static void updateGridRoadMapNodePtr(const RoadMapGraphPtr<N>& roda_map_graph, GridPtrs<N>& grid_map_ptrs) {
            for(int i=0; i < roda_map_graph.nodes_.size(); i++) {
                roda_map_graph.nodes_[i]->sg_->node_id_ = i;
            }
        }

        void initNodes() {
            road_map_graph_->nodes_.clear();
            road_map_graph_->edges_.clear();
            const auto& candidates = surface_processor_->getTangentCandidates();

            for(int i=0; i<candidates.size(); i++) {
                candidates[i]->node_id_ = i;
                auto new_node_ptr = std::make_shared<RoadMapNodeTrait<N> >();
                new_node_ptr->sg_ = candidates[i];
                road_map_graph_->nodes_.push_back(new_node_ptr);
            }
            road_map_graph_->static_node_count_ = road_map_graph_->nodes_.size();
        }

        /* build the static tangent graph, establish connect between nodes */
        bool buildRoadMapGraph() {
            std::cout << "start build tangent graph " << std::endl;
            const auto& candidates = surface_processor_->getTangentCandidates();
            std::cout << " tangent candidates size = " << candidates.size() << std::endl;
            count_of_tangent_ = 0;
            road_map_graph_->edges_.clear();
            NodeId total_counts = candidates.size();
            NodeId count = 0;
            double time_interval = 0.f;
            gettimeofday (&tvpre , &tz);
            Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
            road_map_graph_->raw_edge_count_ = 0;
            auto& nodes = road_map_graph_->nodes_;

            for(int i=0; i<candidates.size(); i++) {
                // connection to the another_candidates is not strictly collision free
                GridPtrs<N> another_candidates;
                another_candidates = surface_processor_->getLocalVisibleSurfaceGrids(candidates[i]->pt_, true);
                road_map_graph_->raw_edge_count_ += another_candidates.size();
                count ++;
                if(count % 5 == 0) {
                    gettimeofday (&tvafter , &tz);
                    time_interval = (tvafter.tv_sec-tvpre.tv_sec)+(tvafter.tv_usec-tvpre.tv_usec)/10e6;
                    if(time_interval > .1) {
                        std::cout << "-- " << std::setprecision(5) <<  100*((double)count)/total_counts << "%"
                                  << ", count_of_surface_grid = " << i << ", count_of_tangent = " << count_of_tangent_
                                  << std::endl;
                        gettimeofday (&tvpre , &tz);
                    }
                }
                for(int j=0; j<another_candidates.size(); j++) {
                    // avoid repeat more than once
                    if (candidates[i]->id_ > another_candidates[j]->id_) continue;
                    InsertNodePairToGraph(candidates[i], another_candidates[j], neighbor);
                }
            }
            std::cout << "finish build tangent graph, get " << count_of_tangent_ << " tangents" << std::endl;
            road_map_graph_->static_edge_count_ = road_map_graph_->edges_.size();
            return true;
        }

        /* build the static tangent graph, establish connect between nodes */
        bool buildRoadMapGraphMultiThread() {
            std::cout << "start build tangent graph in " << max_thread_ << " thread " << std::endl;
            const auto& candidates = surface_processor_->getTangentCandidates();
            std::cout << " tangent candidates size = " << candidates.size() << std::endl;
            lock_.lock();
            road_map_graph_->edges_.clear();
            total_node_count_ = candidates.size();
            count_of_tangent_ = 0;
            node_count_ = 0;
            all_seg_finish_ = std::vector<bool>(max_thread_, false);
            lock_.unlock();
            double time_interval = 0.f;
            gettimeofday (&tvpre , &tz);
            gettimeofday (&tvinit , &tz);
            Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
            road_map_graph_->raw_edge_count_ = 0;
            auto& nodes = road_map_graph_->nodes_;
            EdgeId total_count_of_task = nodes.size() * (nodes.size()-1)/2;
            double mean_task_count = ((double)total_count_of_task) / max_thread_;
            segments_ = std::vector<NodeId>(max_thread_);
            for(int i=1; i<=max_thread_; i++) {
                segments_[i-1] = nodes.size() - sqrt(2 * ((double)max_thread_ - i) / max_thread_ * total_count_of_task) - 1;
                std::cout << " segments " << i << " is " << segments_[i-1]  << std::endl;
            }
            NodeId pre_final_index = 0;
            //ThreadPool thread_pool(max_thread_);
            for(int i=0; i<max_thread_; i++) {
                //thread_pool_.Schedule([&, pre_final_index, i] {
                std::thread t1([&, pre_final_index, i] {
                    gettimeofday(&tvpre, &tz);
                    //std::cout << "from " << pre_final_index << " to " << segments_[i] << std::endl;
                    addTangentBetweenNodeId(pre_final_index, segments_[i], neighbor);
                    //sleep(i);
                    gettimeofday(&tvafter, &tz);
                    time_interval = (tvafter.tv_sec - tvpre.tv_sec) + (tvafter.tv_usec - tvpre.tv_usec) / 10e6;
                    std::cout << "-- thread " << i << " finish in " << time_interval << " ms, count of edge = " << count_of_tangent_ << std::endl;
                    lock_.lock();
                    all_seg_finish_[i] = true;
                    lock_.unlock();
//                    std::cout << " -- all_seg_finish_ "
//                    << all_seg_finish_[0]
//                    << all_seg_finish_[1]
//                    << all_seg_finish_[2]
//                    << all_seg_finish_[3]
//                    << std::endl;
                });
                t1.detach();
//                } else {
//                    gettimeofday(&tvpre, &tz);
//                    std::cout << "from " << pre_final_index << " to " << segments[i] << std::endl;
//                    addTangentBetweenNodeId(pre_final_index, segments[i], neighbor);
//                    pre_final_index = segments[i] + 1;
//                    gettimeofday(&tvafter, &tz);
//                    time_interval = (tvafter.tv_sec - tvpre.tv_sec) + (tvafter.tv_usec - tvpre.tv_usec) / 10e6;
//                    std::cout << "-- main thread finish in " << time_interval << " ms " <<std::endl;
//                }
                pre_final_index = segments_[i] + 1;
            }

            // wait until finish
            //std::unique_lock<std::mutex> lck(lock_);
            // wait until start planner is prepared
            while(! (all_seg_finish_ == std::vector<bool>(max_thread_, true)) ) {
                //cv_.wait(lck);
                usleep(5e5); // 5ms TODO: use conditional variable is a better way, reduce unnecessary wait
            }
            gettimeofday(&tvafter, &tz);
            time_interval = (tvafter.tv_sec - tvinit.tv_sec) + (tvafter.tv_usec - tvinit.tv_usec) / 10e6;
            //thread_pool_.allDetach();
            //lck.unlock();
            road_map_graph_->static_edge_count_ = road_map_graph_->edges_.size();
            std::cout << " all thread finish build tangent graph, in " << time_interval << "ms, get "
            << count_of_tangent_ << " tangents" << std::endl;
            return true;
        }

        void addTangentBetweenNodeId(const NodeId& pre_final_index, const NodeId& final_index, const Pointis<N-1>& neighbor ) {
            //std::cout << "-- from " << pre_final_index << " to " << final_index << std::endl;
            const auto& candidates = surface_processor_->getTangentCandidates();
            for (NodeId id = pre_final_index; id <= final_index; id++) {
                // connection to the another_candidates is not strictly collision free
                //GridPtrs<N> another_candidates;
                //another_candidates = surface_processor_->getLocalVisibleSurfaceGrids(candidates[id]->pt_, true);
                lock_.lock();
                //road_map_graph_->raw_edge_count_ += another_candidates.size();
                node_count_++;
                lock_.unlock();
                if(node_count_ % 10 == 0) {
                    lock_.lock();
                    gettimeofday (&tvafter , &tz);
                    lock_.unlock();
                    double time_interval = (tvafter.tv_sec-tvpre.tv_sec)+(tvafter.tv_usec-tvpre.tv_usec)/10e6;
                    if(time_interval > .1) {
                        std::cout << "-- " << std::setprecision(5) <<  100*((double)node_count_)/total_node_count_ << "%"
                                  << ", count_of_surface_grid = " << node_count_ << ", count_of_tangent = " << count_of_tangent_
                                  << std::endl;
                        lock_.lock();
                        gettimeofday (&tvpre , &tz);
                        lock_.unlock();
                    }
                }
//                for (int jd = 0; jd < another_candidates.size(); jd++) {
//                    // avoid repeat more than once
//                    if (candidates[id]->id_ > another_candidates[jd]->id_) continue;
//                    InsertNodePairToGraph(candidates[id], another_candidates[jd], neighbor);
//                }
                for (int jd = id + 1; jd < candidates.size(); jd++) {
                    // avoid repeat more than once
                    //if (candidates[id]->id_ > another_candidates[jd]->id_) continue;
                    InsertNodePairToGraph(candidates[id], candidates[jd], neighbor);
                }
            }
        }

        void InsertNodePairToGraph(const GridPtr<N>& p1, const GridPtr<N>& p2, const Pointis<N-1>& neighbor) {
            auto& nodes = road_map_graph_->nodes_;

            if(p2->node_id_ == MAX<NodeId>) return;
            if(!isPointTransferLegal(road_map_graph_, p1, p2, surface_processor_->is_occupied_, neighbor, unordered_ptcs_)) { return; }
            if(!isPointTransferLegal(road_map_graph_, p2, p1, surface_processor_->is_occupied_, neighbor, unordered_ptcs_)) { return; }

            // do PTC is faster than LOS check, so do it earlier
            if(surface_processor_->lineCrossObstacle(p1->pt_, p2->pt_)) {
                return;
            }

            PathLen edge_length = (p1->pt_ - p2->pt_).Norm();

            // insert tangent p1 -> p2 to the tangent graph
            if(isPointTransferLegal(road_map_graph_, p1, p2, surface_processor_->is_occupied_, neighbor, ordered_ptcs_))
            {
                lock_.lock();
                if(static_edge_connection_) {
                    auto new_edge_ptr = std::make_shared<RoadMapEdgeTrait<N> >(p1->node_id_,
                                                                               p2->node_id_,
                                                                               edge_length);
                    new_edge_ptr->edge_id_ = road_map_graph_->edges_.size();
                    nodes[p1->node_id_]->nextEdges(true).push_back(road_map_graph_->edges_.size());
                    nodes[p2->node_id_]->nextEdges(false).push_back(road_map_graph_->edges_.size());
                    road_map_graph_->edges_.push_back(new_edge_ptr);
                } else {
                    nodes[p1->node_id_]->nextNodes().push_back(p2->node_id_);
                }
                count_of_tangent_ ++;
                lock_.unlock();
            } else {
                lock_.lock();
                road_map_graph_->edges_.push_back(nullptr);
                lock_.unlock();
            }

            // insert tangent p2 -> p1 to the tangent graph
            if(isPointTransferLegal(road_map_graph_, p2, p1, surface_processor_->is_occupied_, neighbor, ordered_ptcs_))
            {
                lock_.lock();
                if(static_edge_connection_) {
                    auto new_edge_ptr = std::make_shared<RoadMapEdgeTrait<N> >(p2->node_id_, p1->node_id_, edge_length);
                    new_edge_ptr->edge_id_ = road_map_graph_->edges_.size();
                    nodes[p2->node_id_]->nextEdges(true).push_back(road_map_graph_->edges_.size());
                    nodes[p1->node_id_]->nextEdges(false).push_back(road_map_graph_->edges_.size());
                    road_map_graph_->edges_.push_back(new_edge_ptr);
                } else {
                    nodes[p2->node_id_]->nextNodes().push_back(p1->node_id_);
                }
                count_of_tangent_ ++;
                lock_.unlock();
            } else {
                lock_.lock();
                road_map_graph_->edges_.push_back(nullptr);
                lock_.unlock();
            }
        }

        // load the tangent graph from a binary file
        bool roadMapGraphDeserialize(const std::string& file_path) {
            if(file_path == "") return false;
            std::ifstream vis_bin(file_path, std::ios_base::in | std::ios_base::binary);
            if(vis_bin.fail()) return false;
            NodeId buff_id, pre = MAX<NodeId>, cur_id;
            vis_bin.read((char*)&buff_id, sizeof (NodeId));
            std::cout << "-- load tangent graph from binary file " << file_path << std::endl;
            road_map_graph_->edges_.clear();
            count_of_tangent_ = 0;
            node_count_ = 0;
            NodeId count = 0;
            bool new_node = true;
            GridPtr<N> cur_ptr = nullptr;
            auto& nodes = road_map_graph_->nodes_;
            while(vis_bin.read((char*)&buff_id, sizeof (NodeId))) {
                if(buff_id==MAX<NodeId>) {
                    new_node = true;
                } else if (new_node) {
                    cur_id = buff_id;
                    //std::cout << "flag 0" << std::endl;
                    //std::cout << "cur_id = " << cur_id << " /  buff_id " << buff_id << std::endl;
                    if(cur_id > surface_processor_->grid_map_.size()-1) {
                        std::cout << " load failed because of oversize " << std::endl;
                        return false;
                    }
                    cur_ptr = surface_processor_->grid_map_[cur_id];
                    //std::cout << "cur_id pt = " << cur_ptr << std::endl;

                    if(cur_ptr == nullptr || cur_ptr->node_id_ == MAX<NodeId>) {
                        std::cout << " load failed because of new node " << std::endl;
                        return false;
                    }
                    new_node = false;
                } else {
                    // if there is not buff_id in tangent graph, insert it
                    if(ordered_ptcs_.empty() && cur_id > buff_id) continue;
                    if(buff_id > surface_processor_->grid_map_.size()-1) return false;
                    GridPtr<N> buff_ptr = surface_processor_->grid_map_[buff_id];

                    if(buff_ptr == nullptr || buff_ptr->node_id_ == MAX<NodeId>) { return false; }
                    //std::cout << "cur_id = " << cur_id << " /  buff_id " << buff_id << std::endl;
                    //std::cout << "cur_id pt = " << cur_ptr << " /  buff_id pt " << buff_ptr << std::endl;
                    //std::cout << "cur_id pt = " << cur_ptr->pt_ << " /  buff_id pt " << buff_ptr->pt_ << std::endl;

                    PathLen edge_length = (cur_ptr->pt_ - buff_ptr->pt_).Norm();
                    //std::cout << "flag 1" << std::endl;
                    if(static_edge_connection_) {
                        auto new_edge_ptr = std::make_shared<RoadMapEdgeTrait<N> >(cur_ptr->node_id_,
                                                                                   buff_ptr->node_id_, edge_length);
                        new_edge_ptr->edgeId() = road_map_graph_->edges_.size();
                        road_map_graph_->nodes_[cur_ptr->node_id_]->nextEdges(true).push_back(
                                road_map_graph_->edges_.size());
                        road_map_graph_->nodes_[buff_ptr->node_id_]->nextEdges(false).push_back(
                                road_map_graph_->edges_.size());
                        road_map_graph_->edges_.push_back(new_edge_ptr);
                    } else {
                        nodes[cur_ptr->node_id_]->nextNodes().push_back(buff_ptr->node_id_ );
                    }
                    count_of_tangent_ ++;
                    //std::cout << "flag 2" << std::endl;
                    // if is undirected graph, save the reverse version of current edge
                    if(ordered_ptcs_.empty()) {
                        if(static_edge_connection_) {
                            auto new_edge_ptr = std::make_shared<RoadMapEdgeTrait<N> >(buff_ptr->node_id_, cur_ptr->node_id_, edge_length);
                            new_edge_ptr->edgeId() = road_map_graph_->edges_.size();
                            road_map_graph_->nodes_[cur_ptr->node_id_]->nextEdges(false).push_back(road_map_graph_->edges_.size());
                            road_map_graph_->nodes_[buff_ptr->node_id_]->nextEdges(true).push_back(road_map_graph_->edges_.size());
                            road_map_graph_->edges_.push_back(new_edge_ptr);
                        } else {
                            nodes[buff_ptr->node_id_]->nextNodes().push_back(cur_ptr->node_id_);
                        }
                        count_of_tangent_ ++;
                    }
                    //std::cout << "flag 3" << std::endl;
                    count ++;
                }
            }
            vis_bin.close();
            std::cout << "-- load " << count_of_tangent_ << " tangents" << std::endl;
            road_map_graph_->static_edge_count_ = road_map_graph_->edges_.size();
            return true;
        }

        // return a reference version of tangent graph
        // if use a copy, all from_ and to_ pointer will failed
        RoadMapGraphPtr<N>& getRoadMapGraph() {
            return road_map_graph_;
        }

        // save the static tangent graph as a binary file
        bool roadMapGraphSerialize(const std::string& file_path) {
            if(file_path == "") return false;
            std::ofstream vis_bin(file_path, std::ios_base::out|std::ios_base::binary|std::ios_base::trunc);
            if(vis_bin.fail()) return false;
            NodeId end = MAX<NodeId>;
            std::cout << "-- save tangent graph to binary file " << file_path << std::endl;
            int count = 0;
            vis_bin.write((char *) &end, sizeof (NodeId));
            auto & nodes = road_map_graph_->nodes_;
            for(const RoadMapNodeTraitPtr<N>& node : road_map_graph_->nodes_) {
                if(static_edge_connection_) {
                    vis_bin.write((char *) &node->sg_->id_, sizeof(NodeId));
                    for (const auto &edge_id : node->nextEdges(true)) {
                        const auto &edge = road_map_graph_->edges_[edge_id];
                        if (ordered_ptcs_.empty()) {
                            // if is undirected graph, save half space
                            if (nodes[edge->nextNode(true)]->sg_->id_ > node->sg_->id_) {
                                vis_bin.write((char *) &nodes[edge->nextNode(true)]->sg_->id_, sizeof(NodeId));
                            }
                        } else {
                            vis_bin.write((char *) &nodes[edge->nextNode(true)]->sg_->id_, sizeof(NodeId));
                        }
                        count++;
                    }
                    vis_bin.write((char *) &end, sizeof(NodeId));
                } else {
                    vis_bin.write((char *) &node->sg_->id_, sizeof(NodeId));
                    for (const auto & node_id : node->nextNodes()) {
                        if (ordered_ptcs_.empty()) {
                            // if is undirected graph, save half space
                            if (nodes[node_id]->sg_->id_ > node->sg_->id_) {
                                vis_bin.write((char *) &nodes[node_id]->sg_->id_, sizeof(NodeId));
                            }
                        } else {
                            vis_bin.write((char *) &nodes[node_id]->sg_->id_, sizeof(NodeId));
                        }
                        count++;
                    }
                    vis_bin.write((char *) &end, sizeof(NodeId));
                }
            }
            vis_bin.close();
            std::cout << "-- save " << count << " tangents" << std::endl;
            return true;
        }

        /* surface processor, surface detection, surface inflation & tangent candidate detection */
        SurfaceProcessorPtr<N> surface_processor_;


        std::vector<RoadMapEdgeTraitPtrs<N> > getLoopEdges() {
            return loop_edges_;
        }

        bool isHyperEdgeNode(const RoadMapEdgeTraitPtr<N>& edge) {
            auto & edges = road_map_graph_->edges_;
            if(!edge->isLoopEdge()) return false;
            int count1 = 0;
            for (const auto &next_edge_id : road_map_graph_->nextEdges(edge, true, true)) {
                auto & next_edge = edges[next_edge_id];
                if (next_edge->isLoopEdge())
                    count1++;
                if (count1 >= 2) break;
            }
            int count2 = 0;
            for (const auto &next_edge_id : edge->pre_edges_) {
                auto & next_edge = edges[next_edge_id];
                if (next_edge->isLoopEdge())
                    count2++;
                if (count2 >= 2 ) break;
            }
            if(count1 + count2 >= 3 && count1 > 0 && count2 > 0) return true;
            return false;
        }

    private:

        // update edge connections of each edge with ETC (Edge Transfer Condition)
        void updateConnectionBetweenRoadMapEdges() {
            std::cout << "start update connection of tangent edges " << std::endl;
            // count total counts of edges connect check,
            // for each node in the graph
            long long int total_counts = 0, count = 0;
            for(auto & edge_ptr : road_map_graph_->edges_) {
                if(edge_ptr == nullptr) continue;
                // for each edge in the graph
                total_counts += road_map_graph_->nodes_[edge_ptr->nextNode(true)]->nextEdges(true).size();
            }
            std::cout << " total_counts " << total_counts << std::endl;
            double time_interval = 0.f;
            gettimeofday (&tvpre , &tz);
            int total_edge_degree = 0;
            auto & nodes = road_map_graph_->nodes_;

#if 0
            // edge transfer eligible check
            for(auto& edge : road_map_graph_->edges_) {

                for(const auto& next_candidate_id : nodes[edge->nextNode(true)]->nextEdges(true)) {
//                    if(edge->to_->split_edges_.empty()) std::cout << "split_edges_.empty() " << std::endl;

                    auto & next_candidate = road_map_graph_->edges_[next_candidate_id];
                    bool is_legal = isEdgeTransferLegal(road_map_graph_, edge, next_candidate, etcs_);
                    // do not add the reverse version of current edge
                    // add which edge can jump to and can jump to which edge

                    if(is_legal && nodes[edge->nextNode(false)]->sg_->id_ != nodes[next_candidate->nextNode(true)]->sg_->id_) {
                        road_map_graph_->nextEdges(edge, true, true).push_back(next_candidate->edgeId());
                        road_map_graph_->nextEdges(next_candidate, false, true).push_back(edge->edgeId());
                        total_edge_degree ++;
                    }
                    count ++;

                }

                // cost lots of time ...
                if(count % 1000 == 0) {
                    gettimeofday (&tvafter , &tz);
                    time_interval = (tvafter.tv_sec-tvpre.tv_sec)+(tvafter.tv_usec-tvpre.tv_usec)/10e6;
                    if(time_interval > 1.0) {
                        std::cout << "-- " << std::setprecision(5) <<  100*((double)count)/total_counts << "%" << std::endl;
                        gettimeofday (&tvpre , &tz);
                    }
                }
            }

#else
            all_seg_finish_ = std::vector<bool>(max_thread_, false);
            for(int i=0; i<max_thread_; i++) {
                EdgeId start_id = round((road_map_graph_->edges_.size() * (double) i) / max_thread_ ) + 1,
                       end_id   = round(((double)road_map_graph_->edges_.size() *  (i + 1)) / max_thread_);
                if(i == 0) {
                    start_id = 0;
                }
                if(i == max_thread_ - 1) {
                    end_id = road_map_graph_->edges_.size()-1;
                }
                std::cout << " thread " << i << ": " << start_id << " -> " << end_id << std::endl;
                std::thread t1([&, i, start_id, end_id] {
                    for(EdgeId j = start_id; j <= end_id; j++) {
                        auto & edge = road_map_graph_->edges_[j];
                        //std::cout << " edge id " << j << std::endl;
                        for(const auto& next_candidate_id : nodes[edge->nextNode(true)]->nextEdges(true)) {
//                    if(edge->to_->split_edges_.empty()) std::cout << "split_edges_.empty() " << std::endl;

                            auto & next_candidate = road_map_graph_->edges_[next_candidate_id];
                            bool is_legal = isEdgeTransferLegal(road_map_graph_, edge, next_candidate, etcs_);
                            // do not add the reverse version of current edge
                            // add which edge can jump to and can jump to which edge
                            if(is_legal && nodes[edge->nextNode(false)]->sg_->id_ != nodes[next_candidate->nextNode(true)]->sg_->id_) {
                                lock_.lock();
                                road_map_graph_->nextEdges(edge, true, true).push_back(next_candidate->edgeId());
                                road_map_graph_->nextEdges(next_candidate, false, true).push_back(edge->edgeId());
                                total_edge_degree ++;
                                lock_.unlock();
                            }
                            lock_.lock();
                            count ++;
                            lock_.unlock();
                        }
                        if(count % 1000 == 0) {
                            gettimeofday (&tvafter , &tz);
                            time_interval = (tvafter.tv_sec-tvpre.tv_sec)+(tvafter.tv_usec-tvpre.tv_usec)/10e6;
                            if(time_interval > 1.0) {
                                std::cout << "-- " << std::setprecision(5) <<  100*((double)count)/total_counts << "%" << std::endl;
                                gettimeofday (&tvpre , &tz);
                            }
                        }
                    }
                    lock_.lock();
                    all_seg_finish_[i] = true;
                    std::cout << "segment " << i << " finish: " << start_id << " -> " << end_id << std::endl;
                    lock_.unlock();
                });

                t1.detach();
            }
            while(! (all_seg_finish_ == std::vector<bool>(max_thread_, true)) ) {
                //cv_.wait(lck);
                usleep(5e5); // 5ms TODO: use conditional variable is a better way, reduce unnecessary wait
            }
#endif
            std::cout << "finish update connection of tangent edges " << std::endl;
            std::cout << " total_edge_degree " << total_edge_degree << std::endl;
            std::cout << " mean_edge_degree " << total_edge_degree/(double)road_map_graph_->edges_.size() << std::endl;
        }

        bool ifEdgeConnectEdge(const RoadMapEdgeTraitPtr<N>& pre, const RoadMapEdgeTraitPtr<N>& next) {
            if(pre->next_node(true)->sg_->id_ != next->next_node(false)->sg_->id_) { return false; }
            if(pre->next_edges(true, true).find(next->next_node(true)->sg_->id_) != pre->next_edges(true, true).end()) {
                return true;
            }
            return false;
        }

        bool allEdgesReachTheOneEdge(const std::vector<RoadMapEdgeTraitPtr<N>>& edges, const RoadMapEdgeTraitPtr<N>& loop_start) {
            for(const auto& edge : edges) {
                if(!ifEdgeConnectEdge(edge, loop_start)) return false;
            }
            return true;
        }

        int cur_tarjan_time_ = 0;

        RoadMapEdgeTraitPtrs<N> tarjan_stack_vector_;

        // use only when the graph is undirected edge
        inline int opposite(const int& edgeID) const {
            return (edgeID % 2 == 0) ? edgeID + 1 : edgeID - 1;
        }

        // update unloop edge's level in ENLSVG way
        void updateUnLoopEdgeLevel() {
            /* update non-loop level */
            Lv max_level = 0;
            // 1, reset all to level infinity, and get init set for iteration
            std::vector<RoadMapEdgeTraitPtr<N> > candidate_edges; // the indexes of current candidate edges
            for(int i = 0; i < road_map_graph_->original_edge_iters_.size(); i++) {
                const auto& edge_ptr = road_map_graph_->original_edge_iters_[i];
                if(edge_ptr->isLoopEdge()) continue;
                if(edge_ptr->next_edges(false, true).empty()) {
                    edge_ptr->level_ = 0;
                    candidate_edges.push_back(edge_ptr);
                } else {
                    // check whether all pre is loop
                    bool all_loop = true;
                    for(const auto& pre_edge : edge_ptr->next_edges(false, true)) {
                        if(!pre_edge->isLoopEdge()) { all_loop = false; break; }
                    }// all loop pre is all loop
                    if(all_loop) {
                        edge_ptr->level_ = 0;
                        candidate_edges.push_back(edge_ptr);
                    }
                }
            }
            RoadMapEdgeTraitPtrs<N> next_candidate_edges; // the indexes of current candidate edges
            // 2, to set all set that has only one next_edges's level
            while(!candidate_edges.empty()) {
                next_candidate_edges.clear();
                for(auto& candidate_edge : candidate_edges) {
                    if(candidate_edge->isLoopEdge()) continue;
                    for(auto& next_edge : candidate_edge->next_edges(true, true)) {
                        if(next_edge->isLoopEdge()) continue;
//                        if(candidate_edge->level_ < 0) {
//                            std::cout << " candidate_edge->level_ < 0 = " << candidate_edge->level_ << std::endl;
//                        }
                        Lv expected_edge_level = candidate_edge->level_ + 1;
                        if(expected_edge_level <= next_edge->level_) { continue; }
//                        if(expected_edge_level == 0) {
//                            std::cout << " set to zero illegal " << std::endl;
//                        }
                        next_edge->level_ = expected_edge_level;
                        max_level = max_level < expected_edge_level ? expected_edge_level : max_level;
                        next_candidate_edges.push_back(next_edge);
                    }
                }
                std::swap(candidate_edges, next_candidate_edges);
            }

            gettimeofday (&tvafter, &tz);
            double time_interval = (tvafter.tv_sec-tvpre.tv_sec)*1000.+(tvafter.tv_usec-tvpre.tv_usec)/1000.;
            std::cout << "finish updateLevelOfRoadMapEdges in " << time_interval << " ms" << std::endl;
            std::cout << " RJ style get max_level = " << max_level << std::endl;
        }

        // ordered graph and unordered graph should have different ways...
        // update unloop edge's level in ENLSVG way, when is undirected graph
        void computeAllEdgeLevels() {

            std::vector<int> loop_datas(road_map_graph_->edges_.size(), 0);

            loop_edges_.clear();
            for(int i = 0; i < road_map_graph_->edges_.size(); i++) {
                road_map_graph_->edges_[i]->edgeId() = i;
                //if(road_map_graph_->edges_[i]->isLoopEdge()) continue;
                auto& edge_ptr = road_map_graph_->edges_[i];
                edge_ptr->level() = MAX<Lv>;
            }

            std::vector<EdgeId> currentLevelEdges;
            std::vector<EdgeId> nextLevelEdges;

            const auto& all_edges = road_map_graph_->edges_;
            for (int i=0; i<all_edges.size(); ++i) {
                int n = road_map_graph_->nextEdges(all_edges[i], true, true).size();
                //nNeighbours[i] = n;
                //all_edges[i]->nNextEdges_ = n;
                loop_datas[i] = n;
                // yz: set edge that do not have next edges as level_0
                if (n == 0) currentLevelEdges.push_back(i);
            } // set which has no next as initial edges

            int currLevel = 1;
            while (currentLevelEdges.size() > 0) {
                for (size_t j=0;j<currentLevelEdges.size(); ++j) {
                    // yz: current level edge
                    RoadMapEdgeTraitPtr<N> curr_edge = all_edges[currentLevelEdges[j]];
                    // yz: the opposite current edge
                    //EdgeID opp = opposite(curr);

                    //edges[curr].level = currLevel;
                    //edges[opp].level = currLevel;
                    curr_edge->level() = currLevel;
                    // if is undirected graph, update level of both the edge and its reversed version
                    // TODO: what is has ordered_ptcs_, and the graph is a directed graph ?
                    if(ordered_ptcs_.empty()) {
                        auto oppo_edge_id = opposite(currentLevelEdges[j]);
                        all_edges[oppo_edge_id]->level() = currLevel;
                    }
                    // Curr side must have no neighbours.
                    // Opp side may have neighbours.

                    //const std::vector<EdgeID>& neighbours = edges[opp].tautOutgoingEdges; // yz: opp the reversed edge's next edges
                    //RoadMapEdgeTraitPtrs<N>& neighbours = curr_edge->next_edges(false, true);
                    EdgeIds& neighbour_ids = road_map_graph_->nextEdges(curr_edge, false, true);

                    for (size_t j=0; j<neighbour_ids.size(); ++j) {
                        //EdgeID neighbour = opposite(neighbours[j]); // yz: the edges that point to cur edge
                        // yz: if the edges that point to cur edge is not updated yet, may update
                        //if (edges[neighbour].level != LEVEL_W) continue;
                        if(all_edges[neighbour_ids[j]]->level() != MAX<Lv>) continue;
                        //if(neighbours[j]->level_ != -1) continue;
                        // yz: only the next edges numbers of the candidate edge is 1, it should be update ?
                        //--nNeighbours[neighbour];
                        //--all_edges[neighbour_ids[j]]->nNextEdges_;
                        --loop_datas[neighbour_ids[j]];
//                        if (nNeighbours[neighbour] == 0) {
//                            nextLevelEdges.push_back(neighbour);
//                        }
                        //if (all_edges[neighbour_ids[j]]->nNextEdges_ == 0) {
                        if (loop_datas[neighbour_ids[j]] == 0) {
                            nextLevelEdges.push_back(all_edges[neighbour_ids[j]]->edgeId());
                        }
                        // debugging: if (nNeighbours[neighbour] < 0) std::cout << "ERROR" << std::endl;
                    }

                }
                currentLevelEdges.clear();
                std::swap(currentLevelEdges, nextLevelEdges);
                ++currLevel;
            }
            std::cout << " RJ ENLSVG style get max level = " << currLevel - 1 << std::endl;
            int loop_edge_count = 0;
            EdgeIds loop_edge;
            loop_edge.reserve(road_map_graph_->edges_.size());
            for(int i = 0; i < road_map_graph_->edges_.size(); i++) {
                auto& edge_ptr = road_map_graph_->edges_[i];
                if(edge_ptr->level() == MAX<Lv>) {
                    loop_edge.push_back(i);
                    loop_edge_count ++;
                }
            }
            loop_edge.shrink_to_fit();
            loop_edges_ = loop_edge;
            std::cout << " get " << loop_edge_count << " loop edges " << std::endl;
        }

        void updateHyperNodeState() {
            /* 1, update pre and next loop edges for each loop edge */
            // check pre and next loop edge validity
            const auto& edges = road_map_graph_->edges_;
            const auto& nodes = road_map_graph_->nodes_;
            auto & hyper_edges = road_map_graph_->hyper_edges_;
            hyper_edges.clear();
            for(int i=0; i< edges.size(); i++) {
                auto& edge = edges[i];
                if(edge->isLoopEdge()) {
                    edge->next_loop_edge_nodes_.clear();
                    edge->pre_loop_edge_nodes_.clear();
                    edge->next_hyper_loop_edges_.clear();
                    edge->pre_hyper_loop_edges_.clear();
                    for(const auto& next_edge_id : road_map_graph_->nextEdges(edge, true, true)) {
                        if(edges[next_edge_id]->isLoopEdge()) {
                            edge->next_loop_edge_nodes_.push_back(next_edge_id);
                        }
                    }
                    // update pre loop edges
                    for(const auto& pre_edge_id : road_map_graph_->nextEdges(edge, false, true)) {
                        if(edges[pre_edge_id]->isLoopEdge()) {
                            edge->pre_loop_edge_nodes_.push_back(pre_edge_id);
                        }
                    }
                }
            }

            // check pre and next loop edge validity
            for(const auto& edge : road_map_graph_->edges_) {
                if(edge->isLoopEdge()) {
                    bool pre_empty = edge->pre_loop_edge_nodes_.empty(), next_empty = edge->next_loop_edge_nodes_.empty();
                    if(pre_empty || next_empty) {
                        //std::cout << " isolated loop edge " << nodes[edge->from_]->sg_->pt_ << "->"
                        //<< nodes[edge->to_]->sg_->pt_ << pre_empty << " / " << next_empty << std::endl;
                    }
                }
            }

            /* 2, check whether edge is hyper edge node */
            int count = 0;
            for(const auto& edge_id : loop_edges_) {
                auto& loop_edge = edges[edge_id];
                bool is_hyper = isHyperEdgeNode(loop_edge);
                loop_edge->is_hyper_edge_node_ = is_hyper;
                if(is_hyper) count++;
            }

            std::cout << " hyper graph node count = " << count << std::endl;

            int hyper_graph_edge_count = 0;
            // detect hyper graph contains node
            int node_in_hyper_graph_count = 0;
            /* 3, use BFS to update each node hyper config */
            for(const auto& loop_edge_id : loop_edges_) {
                auto & loop_edge = edges[loop_edge_id];
                if (!isHyperEdgeNode(loop_edge)) continue;
                // update it's pre_hyper_edge_nodes_
                for (auto &pre_edge_id : loop_edge->pre_loop_edge_nodes_) {
                    auto& pre_edge = edges[pre_edge_id];
                    double history_length = 0;
                    auto temp_pre = pre_edge;
                    /* if the first neighbor is a hyper edge */
                    if (temp_pre->is_hyper_edge_node_) {
                        continue;
                    } else {
                        history_length = loop_edge->length_;
                        while (!temp_pre->is_hyper_edge_node_) {
                            // only update un-hyper loop edge's next hyper edge
                            //auto hyper_loop_edge = RoadMapEdge2RoadMapLoopEdge(loop_edge);

                            auto hyper_loop_edge_with_length = std::make_shared<HyperLoopEdgeWithLength<N> >(loop_edge->edge_id_, history_length);
                            temp_pre->next_hyper_loop_edges_.push_back(hyper_edges.size());
                            hyper_loop_edge_with_length->hyper_id_ = hyper_edges.size();
                            hyper_edges.push_back(hyper_loop_edge_with_length);
                            history_length += temp_pre->length_;
                            //if(temp_pre->pre_loop_edge_nodes_.size() == 0) break;
                            if(temp_pre->pre_loop_edge_nodes_.size() != 1) {
                                //std::cout << "uHNS: shouldn't reach here, temp_pre->pre_loop_edge_nodes_.size() != 1, = " << temp_pre->pre_loop_edge_nodes_.size() << std::endl;
                                break;
                            }
                            const auto & temp_pre_id = temp_pre->pre_loop_edge_nodes_.front();
                            temp_pre = edges[temp_pre_id];
                            if (temp_pre->is_hyper_edge_node_) {
                                break;
                            }
                        }
                    }
                }
                // update it's next_hyper_edge_nodes_
                for (auto &next_edge_id : loop_edge->next_loop_edge_nodes_) {
                    auto & next_edge = edges[next_edge_id];
                    auto temp_next = next_edge;
                    double history_length = 0;
                    if (temp_next->is_hyper_edge_node_) {
                        if(!nodes[temp_next->to_]->isInHyperGraph()) {
                            nodes[temp_next->to_]->isInHyperGraph() = true;
                            node_in_hyper_graph_count ++;
                        }
                        history_length += temp_next->length_;
                        // update nearby hyper loop edge's pre hyper edge
                        //auto hyper_loop_edge1 = RoadMapEdge2RoadMapLoopEdge(loop_edge);
                        auto hyper_loop_edge_with_length1 = std::make_shared<HyperLoopEdgeWithLength<N>>(loop_edge->edge_id_, -history_length);
                        temp_next->pre_hyper_loop_edges_.push_back(hyper_edges.size());
                        hyper_loop_edge_with_length1->hyper_id_ = hyper_edges.size();
                        hyper_edges.push_back(hyper_loop_edge_with_length1);
                        hyper_graph_edge_count ++;

                        // update nearby hyper loop edge's next hyper edge
                        //auto hyper_loop_edge2 = RoadMapEdge2RoadMapLoopEdge(temp_next);
                        auto hyper_loop_edge_with_length2 = std::make_shared<HyperLoopEdgeWithLength<N>>(temp_next->edge_id_, history_length);
                        loop_edge->next_hyper_loop_edges_.push_back(hyper_edges.size());
                        hyper_loop_edge_with_length2->hyper_id_ = hyper_edges.size();
                        hyper_edges.push_back(hyper_loop_edge_with_length2);

                        continue;
                    } else {
                        while (!temp_next->is_hyper_edge_node_) {
                            history_length += temp_next->length_;

                            // update un-hyper loop edge's next hyper edge
                            //auto hyper_loop_edge0 = RoadMapEdge2RoadMapLoopEdge(loop_edge);
                            auto hyper_loop_edge_with_length0 = std::make_shared<HyperLoopEdgeWithLength<N>>(loop_edge->edge_id_, -history_length);
                            temp_next->pre_hyper_loop_edges_.push_back(hyper_edges.size());
                            hyper_loop_edge_with_length0->hyper_id_ = hyper_edges.size();
                            hyper_edges.push_back(hyper_loop_edge_with_length0);

                            if(temp_next->next_loop_edge_nodes_.empty()) break;
                           temp_next = edges[temp_next->next_loop_edge_nodes_.front()];
                            if (temp_next->is_hyper_edge_node_) {
                                history_length += temp_next->length_;

                                // update hyper loop edge's pre hyper edge
                                //auto hyper_loop_edge1 = RoadMapEdge2RoadMapLoopEdge(loop_edge);
                                auto hyper_loop_edge_with_length1 = std::make_shared<HyperLoopEdgeWithLength<N>>(loop_edge->edge_id_, -history_length);
                                temp_next->pre_hyper_loop_edges_.push_back(hyper_edges.size());
                                hyper_loop_edge_with_length1->hyper_id_ = hyper_edges.size();
                                hyper_edges.push_back(hyper_loop_edge_with_length1);

                                // update hyper loop edge's next hyper edge
                                //auto hyper_loop_edge2 = RoadMapEdge2RoadMapLoopEdge(temp_next);
                                auto hyper_loop_edge_with_length2 = std::make_shared<HyperLoopEdgeWithLength<N>>(temp_next->edge_id_, history_length);
                                loop_edge->next_hyper_loop_edges_.push_back(hyper_edges.size());
                                hyper_loop_edge_with_length2->hyper_id_ = hyper_edges.size();
                                hyper_edges.push_back(hyper_loop_edge_with_length2);
                                break;
                            }
                        }
                    }
                }
            }

            std::cout << " hyper graph edge count = " << hyper_graph_edge_count << std::endl;
            std::cout << " node_in_hyper_graph_count " << node_in_hyper_graph_count << std::endl;

            // check hyper pre and next loop edge validity
            for(auto& edge : road_map_graph_->edges_) {
                if(!edge->isLoopEdge()) continue;
                if(edge->next_hyper_loop_edges_.empty()) {
                    //std::cout << "no next isolated hyper " << edge->is_hyper_edge_node_ << " loop edge " << edge->from_->sg_->pt_ << "->" << edge->to_->sg_->pt_ << std::endl;
                }
                if(edge->pre_hyper_loop_edges_.empty()) {
                    //std::cout << "no pre isolated hyper " << edge->is_hyper_edge_node_ << " loop edge  " << edge->from_->sg_->pt_ << "->" << edge->to_->sg_->pt_ << std::endl;
                }
            }
        }

        bool static_edge_connection_ = false; // whether compute edge connection in advance, if in advance, take lots of space

        int max_thread_ = 1;

        RoadMapGraphPtr<N> road_map_graph_ = nullptr;

        std::vector<EdgeId> loop_edges_; // loop edges in the road_map_graph_

        PointTransferConstraints<N> unordered_ptcs_;

        PointTransferConstraints<N> ordered_ptcs_;

        EdgeTransferConstraints3<N> etcs_;

        //std::string file_path_ = ""; // default to do not save in file

        //ThreadPool thread_pool_; // IMPORTANT NOTE: source is not released after finish build graph

        struct timeval tvafter, tvpre, tvinit;
        struct timezone tz;

        // statistics during calculation

        EdgeId count_of_tangent_ = 0;
        std::mutex lock_;
        NodeId node_count_ = 0;
        NodeId total_node_count_ = 0;
        std::condition_variable cv_;
        std::vector<NodeId> segments_;
        std::vector<bool> all_seg_finish_;
    };

}

#endif //FREENAV_TANGENT_GRAPH_BUILD_H
