//
// Created by yaozhuo on 2021/9/20.
//

#include "canvas.h"
#include "../../algorithm/graph_construction/tangent_graph_build.h"
#include "iomanip"
namespace freeNav::Topo {

    CanvasTopo::CanvasTopo(std::string name, int size_x, int size_y, double resolution, int zoom_ratio) :
            Canvas(name, size_x, size_y, resolution, zoom_ratio)
    {}

    void CanvasTopo::drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph) {
        for (const auto &edge : graph.edges) {
            //if (edge.level % 2 == 1) continue;
            auto color = cv::Scalar::all(20 * (edge.level % 10));
            if (edge.level != Pathfinding::ENLSVG::VisibilityGraph::LEVEL_W) {
                color = 200;
                continue;
            }
            int sx = graph.vertices[edge.sourceVertex].x,
                    sy = graph.vertices[edge.sourceVertex].y,
                    ex = graph.vertices[edge.destVertex].x,
                    ey = graph.vertices[edge.destVertex].y;
            drawGridLine(sx, sy, ex, ey, 1, false, cv::Vec3b(0,255,0));
        }
    }

    void CanvasTopo::drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph, const Pathfinding::ENLSVG::Memory& memory) {
        for (int i=0;i<graph.edges.size();i++) {
            const auto& edge = graph.edges[i];
            //if (edge.level % 2 == 1) continue;
            auto color = cv::Scalar::all(20 * (edge.level % 10));
//            if (edge.level != freeNav::RimJump::VisibilityGraph<2>::LEVEL_W) {
//                continue;
//            }
            if (!memory.markedEdges.isMarked[i]) {
                continue;
            }
//            if(edge.level != Pathfinding::ENLSVG::VisibilityGraph::LEVEL_W) continue;
//            if (!graph.isHyperEdge_[i]) {
//                color = cv::Vec3b(0,255,0);
//                //continue;
//            } else {
//                color = cv::Vec3b(0,0,255);
//            }
            const auto& grid_start  = graph.vertices[edge.sourceVertex];
            const auto& grid_target = graph.vertices[edge.destVertex];

//            if(memory.visited(grid_start->id_))
//            {
//                drawCircleInt(grid_start->pt_[0], grid_start->pt_[1], 5, false, -1, cv::Vec3b(0,255,0));
//            }
//            if(memory.visited(grid_target->id_))
//            {
//                drawCircleInt(grid_target->pt_[0], grid_target->pt_[1], 5, false, -1, cv::Vec3b(0,255,0));
//            }

            int sx = grid_start.x,
                    sy = grid_start.y,
                    ex = grid_target.x,
                    ey = grid_target.y;
            // drawGridLine(sx, sy, ex, ey, 1, false, color);
            drawArrowInt(sx, sy, ex, ey, 1, false, color);

        }
    }


    void
    CanvasTopo::drawTangentGraphAllNodes(RoadMapGraphPtr<2> &tg, const cv::Vec3b &color1,
                                     const cv::Vec3b &color2) {
        int color_count = 0;
        for (const auto &edge : tg->nodes_) {
            drawGrid(edge->sg_->pt_[0], edge->sg_->pt_[1], color1);
        }
    }

    void CanvasTopo::drawMarkedInitialEdges(RoadMapGraphPtr<2> &tg, bool center_offset) {
//        for (int i=0; i<tg.edges_.size(); i++) {
//            if (tg.dist_to(i, true, true, MIN_TIME) != std::numeric_limits<double>::max())
//                drawEdge(tg, tg.edges_[i], false, false, cv::Scalar(0, 255, 0));
//            if (tg.dist_to(i, false, true, MIN_TIME) != std::numeric_limits<double>::max())
//                drawEdge(tg, tg.edges_[i], false, false, cv::Scalar(255, 0, 0));
//        }
    }

    void CanvasTopo::drawTangentGraphEdge(DimensionLength dimen[2],
                                      RoadMapGraphPtr<2> &tg,
                                      const Pointi<2> &pt1,
                                      const Pointi<2> &pt2,
                                      bool center_offset) {
        Id start_id = freeNav::PointiToId<2>(pt1, dimen), end_id = freeNav::PointiToId<2>(pt2, dimen);
        int color_count = 0;
        //drawGrid(pt1[0], pt1[1], COLOR_TABLE[0]);
        //drawGrid(pt2[0], pt2[1], COLOR_TABLE[1]);
        auto start_grid_ptr = tg->surface_processor_->grid_map_[start_id];
        if (start_grid_ptr != nullptr && start_grid_ptr->node_id_ != MAX<NodeId>) {
            RoadMapEdgeTraitPtr<2> edge_iter = nullptr;//start_iter->second->split_edges_.find(end_id);
            EdgeId edge_id;//start_iter->second->split_edges_.find(end_id);
            for(auto edge_temp_id : tg->nextEdges(tg->nodes_[start_grid_ptr->node_id_], true)) {
                auto edge_temp = tg->edges_[edge_temp_id];
                if(tg->nodes_[edge_temp->nextNode(true)]->sg_->id_ == end_id) {
                    edge_iter = edge_temp;
                    edge_id = edge_temp_id;
                }
            }
            if(edge_iter != nullptr) {
                drawEdge(tg, edge_iter, center_offset, false, false, cv::Scalar(0, 0, 255));

                // draw edge
                for (auto &next_edge_id : tg->nextEdges(tg->edges_[edge_id], true, true)) {
                    drawEdge(tg, tg->edges_[next_edge_id], center_offset, false, false, cv::Scalar(255, 0, 0));
                }
                for (auto &pre_edge_id : tg->nextEdges(tg->edges_[edge_id], false, true)) {
                    drawEdge(tg, tg->edges_[pre_edge_id], center_offset, false, false, cv::Scalar(0, 255, 0));
                }
            }

        }
    }

    void CanvasTopo::drawRoadMapEdges(RoadMapGraphPtr<2> &tg,
                                  const RoadMapEdgeTraitPtrs<2> &edges,
                                  bool center_offset, const cv::Scalar &color1) {
        for (const auto &edge : edges) {
            drawEdge(tg, edge, center_offset, false, false, color1);
        }
    }


    void CanvasTopo::drawRoadMapEdgeAsPath(GeneralGraphPathPlannerWithEdge<2>& g2p2, RoadMapEdgeTraitPtr<2> edge, bool center_offset, bool is_edge, bool is_start,
                                       const cv::Scalar &color) {
        if(edge == nullptr) return;
        //if (is_start) edge = edge->current_on_edge(is_edge, MIN_TIME);
        if(edge == nullptr) return;
        drawEdge(g2p2.tg_, edge, center_offset, false, false, color);
        auto& data = g2p2.data_;
        auto & ns = g2p2.tg_->nodes_;
        auto & es = g2p2.tg_->edges_;
        auto & hes = g2p2.tg_->hyper_edges_;

        if (data->dist_to(edge, true, is_edge, MIN_TIME) != freeNav::MAX<freeNav::PathLen>)
            drawTextInt((3 * ns[edge->nextNode(false)]->sg_->pt_[0] + ns[edge->nextNode(true)]->sg_->pt_[0]) / 4,
                        (3 * ns[edge->nextNode(false)]->sg_->pt_[1] + ns[edge->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(edge, true, true, MIN_TIME))).c_str(), cv::Scalar(0, 255, 0));
        if (data->dist_to(edge, false, is_edge, MIN_TIME) != freeNav::MAX<freeNav::PathLen>)
            drawTextInt((ns[edge->nextNode(false)]->sg_->pt_[0] + 3 * ns[edge->nextNode(true)]->sg_->pt_[0]) / 4,
                        (ns[edge->nextNode(false)]->sg_->pt_[1] + 3 * ns[edge->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(edge, false, true, MIN_TIME))).c_str(), cv::Scalar(255, 0, 0));

        auto to_start = data->close_edge_to(edge, true, is_edge, MIN_TIME);
        while (to_start != MAX<EdgeId>) {
            drawEdge(g2p2.tg_, es[to_start], center_offset, false, false, color);
            drawTextInt((3 * ns[es[to_start]->nextNode(false)]->sg_->pt_[0] + ns[es[to_start]->nextNode(true)]->sg_->pt_[0]) / 4,
                        (3 * ns[es[to_start]->nextNode(false)]->sg_->pt_[1] + ns[es[to_start]->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(es[to_start], true, true, MIN_TIME))).c_str(), cv::Scalar(0, 255, 0));
            to_start = data->close_edge_to(es[to_start], true, is_edge, MIN_TIME);
        }
        auto to_target = data->close_edge_to(edge, false, is_edge, MIN_TIME);
        while (to_target != MAX<EdgeId>) {
            drawEdge(g2p2.tg_, es[to_target], center_offset, false, false, color);
            drawTextInt((ns[es[to_target]->nextNode(false)]->sg_->pt_[0] + 3 * ns[es[to_target]->nextNode(true)]->sg_->pt_[0]) / 4,
                        (ns[es[to_target]->nextNode(false)]->sg_->pt_[1] + 3 * ns[es[to_target]->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(es[to_target], false, true, MIN_TIME))).c_str(), cv::Scalar(255, 0, 0));
            to_target = data->close_edge_to(es[to_target], false, is_edge, MIN_TIME);
        }
    }

    void CanvasTopo::drawEdges(RoadMapGraphPtr<2> &tg, const RoadMapEdgeTraitPtrs<2> &edges, bool center_offset,const cv::Scalar &color1) {
        for (int i=0;i<edges.size();i++) {
            const auto &edge = edges[i];
            drawEdge(tg, edge, center_offset, true, false, COLOR_TABLE[i%30]);
        }
    }

    void CanvasTopo::drawNode(RoadMapGraphPtr<2> &tg, NodeId & id, bool center_offset, const cv::Scalar &color1) {
        auto& ns = tg->nodes_;
        for(const auto& vid : tg->nextNodes(ns[id])) {
            const auto& pt1 = ns[id]->sg_->pt_;
            const auto& pt2 = ns[vid]->sg_->pt_;
            drawArrowInt(pt1[0], pt1[1], pt2[0], pt2[1],
                         1, center_offset, color1);
        }
    }

    void CanvasTopo::drawNodes(RoadMapGraphPtr<2> &tg, bool center_offset, const cv::Scalar &color1) {
        auto& ns = tg->nodes_;
        for(NodeId i=0; i<ns.size(); i++) {
            drawNode(tg, i, center_offset, color1);
        }
    }

    void CanvasTopo::drawNodes(RoadMapGraphPtr<2> &tg,
                           DynamicDataOfSearchWithNodePtr<2> data,
                           bool center_offset, const cv::Scalar &color1) {
        auto& ns = tg->nodes_;
        for(NodeId i=0; i<ns.size(); i++) {
            if(data->close_node_to(ns[i], false, MIN_TIME) != MAX<NodeId>) {
                const auto & pt1 = ns[i]->sg_->pt_;
                const auto & pt2 = ns[data->close_node_to(ns[i], false, MIN_TIME)]->sg_->pt_;

                drawCircleInt(pt1[0], pt1[1], 1, false, -1, cv::Vec3b(0,255,0));

                //drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], center_offset, 1, color1);

            }
            if(data->close_node_to(ns[i], true, MIN_TIME) != MAX<NodeId>) {
                const auto & pt1 = ns[i]->sg_->pt_;
                const auto & pt2 = ns[data->close_node_to(ns[i], true, MIN_TIME)]->sg_->pt_;
                drawArrowInt(pt2[0], pt2[1], pt1[0], pt1[1], 1, center_offset, cv::Vec3b(255,0,0));
            }
        }
    }

    void CanvasTopo::drawEdgess(RoadMapGraphPtr<2> &tg,
                            const RoadMapEdgeTraitPtrss<2> &edgess,
                            bool center_offset) {
        for (int i = 0; i < edgess.size(); i++) {
            drawEdges(tg, edgess[i], center_offset, COLOR_TABLE[i]);
        }
    }

    void CanvasTopo::drawEdge(RoadMapGraphPtr<2> &tg,
                          const RoadMapEdgeTraitPtr<2> &edge,
                          bool center_offset,
                          bool only_loop, bool draw_branch,
                          const cv::Scalar &color1) {
        if(edge == nullptr) return;
        if (only_loop) {
            if (!edge->isLoopEdge()) return;
            if(edge->nextEdges(false).empty() || edge->nextEdges(true).empty()) return;
            //if(!freeNav::RimJump::RoadMapGraphBuilder<2>::isHyperLoopEdge(edge)) return;
            //if (!edge->isHyperLoopEdge()) return;
        }
        //if(edge->isLoopEdge()) return;
        //if(!edge->is_hyper_edge_node_) return;
        // draw arrow cause error in big map
        auto node_pre_ptr  = tg->nodes_[edge->nextNode(false)];
        auto node_next_ptr = tg->nodes_[edge->nextNode(true)];

        drawArrowInt(node_pre_ptr->sg_->pt_[0], node_pre_ptr->sg_->pt_[1], node_next_ptr->sg_->pt_[0], node_next_ptr->sg_->pt_[1],
                     2, center_offset, color1);

        //drawGridLine(edge->nextNode(false)->sg_->pt_[0], edge->nextNode(false)->sg_->pt_[1], edge->nextNode(true)->sg_->pt_[0], edge->nextNode(true)->sg_->pt_[1], 1, false, color1);
        if (!edge->isLoopEdge()) {
            //drawTextInt((3*edge->nextNode(false)->sg_->pt_[0] + edge->nextNode(true)->sg_->pt_[0]) / 4, (3*edge->nextNode(false)->sg_->pt_[1] + edge->nextNode(true)->sg_->pt_[1]) / 4, std::to_string(edge->level_).c_str(), cv::Scalar(0,255, 0));
        } else {
            //drawTextInt((3*edge->nextNode(false)->sg_->pt_[0] + edge->nextNode(true)->sg_->pt_[0]) / 4, (3*edge->nextNode(false)->sg_->pt_[1] + edge->nextNode(true)->sg_->pt_[1]) / 4, "INF", cv::Scalar(0,0, 255));
        }
        //std::stringstream ss;
        //ss << edge->tarjan_time_ << "|" << edge->tarjan_earliest_time_;
        //drawTextInt((3*edge->from_->sg_->pt_[0] + edge->to_->sg_->pt_[0]) / 4, (3*edge->from_->sg_->pt_[1] + edge->to_->sg_->pt_[1]) / 4, ss.str().c_str(), color1);

        if (!edge->isLoopEdge()) {
            //drawTextInt((3*edge->from_->sg_->pt_[0] + edge->to_->sg_->pt_[0]) / 4, (3*edge->from_->sg_->pt_[1] + edge->to_->sg_->pt_[1]) / 4, std::to_string(edge->level_).c_str(), COLOR_TABLE[4]);
        }
        if (!edge->isLoopEdge()) {
            //drawTextInt((3*edge->from_->sg_->pt_[0] + edge->to_->sg_->pt_[0]) / 4, (3*edge->from_->sg_->pt_[1] + edge->to_->sg_->pt_[1]) / 4, std::to_string(edge->level_).c_str(), COLOR_TABLE[4]);
        }

        int color_count = 0;
        if (draw_branch) {
//            auto& grid_map = tg.surface_processor_;
//            auto& edges = tg.edges_;
//            auto& nodes = tg.nodes_;
//            for (const auto &next_edge_id : tg.edge_next_edges(edge->edge_id(), true, true)) {
//                drawArrowInt(nodes[edge->nextNode(true)]->sg_->pt_[0],
//                             nodes[edge->nextNode(true)]->sg_->pt_[1],
//                             nodes[edges[next_edge_id]->nextNode(true)]->sg_->pt_[0],
//                             nodes[edges[next_edge_id]->nextNode(true)]->sg_->pt_[1], 1, true, COLOR_TABLE[color_count % 30]);
//                color_count++;
//            }
//            for (const auto &pre_edge_id : tg.edge_next_edges(edge->edge_id(), false, true)) {
//                drawArrowInt(nodes[edges[pre_edge_id]->nextNode(false)]->sg_->pt_[0],
//                             nodes[edges[pre_edge_id]->nextNode(false)]->sg_->pt_[1],
//                             nodes[edge->nextNode(true)]->sg_->pt_[0],
//                             nodes[edge->nextNode(true)]->sg_->pt_[1], 1, true,
//                             COLOR_TABLE[color_count % 30]);
//                color_count++;
//            }
        }
    }

    void
    CanvasTopo::drawTangentGraphLegalEdges(RoadMapGraphPtr<2> &tg,
                                       bool center_offset,
                                       const cv::Vec3b &color1,
                                       const cv::Vec3b &color2) {
        freeNav::Pointi<2> pt1, pt2, pt3, pt4;
        auto & edges = tg->edges_;
        auto & nodes = tg->nodes_;
        for (auto &node : tg->nodes_) {
            // for each edge in the graph
            for (auto &edge_id : tg->nextEdges(node, true)) {
                auto &next_edges = tg->nextEdges(tg->edges_[edge_id], true, true);
                //if(!next_edges.empty()) continue;
                pt1 = nodes[edges[edge_id]->nextNode(false)]->sg_->pt_;
                pt2 = nodes[edges[edge_id]->nextNode(true)]->sg_->pt_;
                // draw current edge
                drawGrid(pt2[0], pt2[1], color1);
                drawGridLine(pt1[0], pt1[1], pt2[0], pt2[1], 1, true, color2);
                // draw each next edges
                for (auto next_edge_id : next_edges) {
                    pt3 = nodes[edges[next_edge_id]->nextNode(false)]->sg_->pt_;
                    pt4 = nodes[edges[next_edge_id]->nextNode(true)]->sg_->pt_;
                    // draw next edge
                    drawGridLine(pt3[0], pt3[1], pt4[0], pt4[1], 1, true, color2);
                }
            }
        }
    }

//     void Canvas::drawPath(const std::vector<freeNav::RimJump::GridNode> &path, const cv::Scalar &color) {
//        if (path.empty()) return;
//        for (int i = 0; i < path.size() - 1; i++) {
//            drawGridLine(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, 1, false, color);
//        }
//    }

    void CanvasTopo::drawTangentPoints(const Pointi<2> &start,
                                   const RoadMapNodePtrs<2> &tpts) {
        int color_count = 0;
        for (const auto &tpt : tpts) {
            freeNav::Pointi<2> end = tpt->sg_->pt_;
            drawGridLine(start[0], start[1], end[0], end[1], 1, true, COLOR_TABLE[color_count % 30]);
            color_count++;
        }
    }

    void CanvasTopo::draw_RimJump_Extent(const Extent &extents, DimensionLength dimen[2], double scale) {
        for (int i = 0; i < extents.size(); i++) {
            const freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(i, dimen);
            if (isOutOfBoundary(pt, dimen)) continue;
            drawTextInt(pt[0], pt[1], std::to_string(extents[i]).c_str(), {0, 0, 255}, scale);
        }
    }

    void CanvasTopo::draw_RimJump_Intervals(const FractionPair& start,const IntervalPtrs& prev_halfs, const cv::Scalar &color) {
        for(const auto& interval : prev_halfs) {
            drawLineInt(interval->xL_, interval->y_, interval->xR_, interval->y_, true, 4, color);
            drawLineInt(interval->xL_, interval->y_, start.first, start.second, true, 2, color);
            drawLineInt(start.first, start.second, interval->xR_, interval->y_, true, 2, color);

            drawCircleInt(interval->xL_, interval->y_, 5, true, 1, color);
            drawCircleInt(interval->xR_, interval->y_, 5, true, 1, color);
        }
    }

    void CanvasTopo::draw_RimJump_IntervalTree(const FractionPair& start,
                                           const IntervalTree &tree, DimensionLength dimen[2],
                                           const cv::Scalar &color, double scale) {
        for(const auto& intervals : tree) {
            for(const auto& interval : intervals) {

                if(interval == nullptr) continue;

                drawLineInt(interval->xL_, interval->y_, interval->xR_, interval->y_, true, 4, cv::Scalar(0, 255, 0));
                drawLineInt(interval->xL_, interval->y_, start.first, start.second, true, 2, color);
                drawLineInt(start.first, start.second, interval->xR_, interval->y_, true, 2, color);


                int left_width = 1, right_width = 1;

                drawCircleInt(interval->xL_, interval->y_, 5, true, left_width, color);
                drawCircleInt(interval->xR_, interval->y_, 5, true, right_width, color);

            }
        }
    }


    void CanvasTopo::draw_ENLSVG_Extent(const std::vector<int> &extents, freeNav::DimensionLength dimen[2],
                                    double scale) {
        freeNav::DimensionLength internal_dimen[2];
        internal_dimen[0] = dimen[0] + 1; // ENL_SVG internal setting
        internal_dimen[1] = dimen[1] + 2; // ENL_SVG internal setting
        for (int i = 0; i < extents.size(); i++) {
            const freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(i, internal_dimen);
            if (isOutOfBoundary(pt, dimen)) continue;
            drawTextInt(pt[0], pt[1], std::to_string(extents[i]).c_str(), {0, 0, 255}, scale);
        }
    }

    void CanvasTopo::draw_ENLSVG_ScannerStacks(const Pathfinding::ScannerStacks &scanner_stacks, const cv::Scalar &color) {
//        for(const auto& interval : scanner_stacks.intervalStack_backup) {
//            draw_ENLSVG_ScannerInterval(interval);
//        }
        for (const auto &neighbour : scanner_stacks.neighbours) {
            drawCircleInt(neighbour.x, neighbour.y, 5, false, 1, color);
        }
    }

    void CanvasTopo::draw_ENLSVG_ScannerInterval(const Pathfinding::ScanInterval &scanner_interval, const cv::Scalar &color) {

        drawLineInt(scanner_interval.xL.toFloat(), scanner_interval.y, scanner_interval.xR.toFloat(), scanner_interval.y, false);

        drawCircleInt(scanner_interval.xL.toFloat(), scanner_interval.y, 5, false, 1, color);
        drawCircleInt(scanner_interval.xR.toFloat(), scanner_interval.y, 5, false, 1, color);

    }

    void CanvasTopo::drawPathi(const std::vector<Pathfinding::GridVertex> &path, const cv::Scalar &color) {
        if (path.empty()) return;
        for (int i = 0; i < path.size() - 1; i++) {
            drawGridLine(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, 1, false, color);
        }
    }

}







