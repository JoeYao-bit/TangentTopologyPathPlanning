//
// Created by yaozhuo on 2021/9/20.
//

#ifndef FREENAV_TOPO_CANVAS_H
#define FREENAV_TOPO_CANVAS_H
#include <functional>

#include "../../freeNav-base/dependencies/color_table.h"
#include "../../freeNav-base/dependencies/2d_grid/picture_loader.h"
#include "../../freeNav-base/basic_elements/map_down_sampler.h"
#include "../../freeNav-base/visualization/canvas/canvas.h"

#include "../../algorithm/online_search/search_path_with_edge.h"
#include "../../algorithm/surface_processor_ENLSVG_LineScanner.h"
#include "../../algorithm/surface_processor_LineScanner.h"
#include "../../algorithm/online_search/dynamic_data_with_node.h"
#include "../../algorithm/online_search/dynamic_data_with_edge.h"
#include "../../third_party/ENL-SVG/Pathfinding/ENLSVG.h"

namespace freeNav::Topo {

    class CanvasTopo: public Canvas {
    public:

        // m ratio must be integer that greater than 1
        explicit CanvasTopo(std::string name, int size_x, int size_y, double resolution, int zoom_ratio = 1);

        void drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph);

        void drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph, const Pathfinding::ENLSVG::Memory& memory);

        template<typename T>
        void drawEdges(const T &edges, bool center_offset, bool only_loop = false, bool draw_branch = false) {
            if (edges.empty()) return;
            int color_count = 0;
            for (const auto &edge : edges) {
                drawEdge(edge, center_offset, only_loop, draw_branch, COLOR_TABLE[color_count]);
                //drawEdge(edge, only_loop, draw_branch, cv::Scalar());
                color_count++;
            }
        }

        void
        drawEdge(RoadMapGraphPtr<2> &tg,
                 const RoadMapEdgeTraitPtr<2> &edge,
                 bool center_offset,
                 bool only_loop = false, bool draw_branch = false,
                 const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawEdges(RoadMapGraphPtr<2> &tg, const RoadMapEdgeTraitPtrs<2> &edges, bool center_offset, const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawNodes(RoadMapGraphPtr<2> &tg,
                       DynamicDataOfSearchWithNodePtr<2> data,
                       bool center_offset, const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawNode(RoadMapGraphPtr<2> &tg, NodeId & id, bool center_offset, const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawNodes(RoadMapGraphPtr<2> &tg, bool center_offset, const cv::Scalar &color1);

        void drawEdgess(RoadMapGraphPtr<2> &tg, const RoadMapEdgeTraitPtrss<2> &edgess, bool center_offset);

        void drawTangentGraphAllNodes(RoadMapGraphPtr<2> &tg,
                                      const cv::Vec3b &color1 = cv::Vec3b(
                                              0, 0, 0), const cv::Vec3b &color2 = cv::Vec3b(0, 0, 0));

        void drawTangentGraphEdge(DimensionLength dimen[2],
                                  RoadMapGraphPtr<2> &tg,
                                  const Pointi<2> &pt1,
                                  const Pointi<2> &pt2,
                                  bool center_offset);

        void drawTangentGraphLegalEdges(RoadMapGraphPtr<2> &tg, bool center_offset,
                                        const cv::Vec3b &color1 = cv::Vec3b(0, 0, 0),
                                        const cv::Vec3b &color2 = cv::Vec3b(0, 0, 0));

        void drawMarkedInitialEdges(RoadMapGraphPtr<2> &tg, bool center_offset);


        void drawTangentPoints(const Pointi<2> &start, const RoadMapNodePtrs<2> &tpts);

        void drawTangentPathPoints(PathPointWithEdgePtrs<2> pps, const Pointi<2> &start,
                                   const Pointi<2> &target);

        void drawTangentPathPoint(PathPointWithEdgePtr<2> pps, const Pointi<2> &start,
                                  const freeNav::Pointi<2> &target, const cv::Scalar &color);

        void drawRoadMapEdgeAsPath(GeneralGraphPathPlannerWithEdge<2>& g2p2, RoadMapEdgeTraitPtr<2> edge,
                                   bool center_offset, bool is_edge, bool is_start,
                                   const cv::Scalar &color = cv::Scalar(1, 1, 1, 1));

        void drawRoadMapEdges(RoadMapGraphPtr<2> &tg,
                              const RoadMapEdgeTraitPtrs<2> &edges,
                              bool center_offset,
                              const cv::Scalar &color1);

        void draw_RimJump_Extent(const Extent &extent, DimensionLength dimen[2], double scale = .7);

        void draw_RimJump_IntervalTree(const FractionPair& start, const IntervalTree &tree, freeNav::DimensionLength dimen[2],  const cv::Scalar &color = cv::Scalar(0,0,255), double scale = .7);

        void draw_RimJump_Intervals(const FractionPair& start, const IntervalPtrs& prev_halfs, const cv::Scalar &color = cv::Scalar::all(0));

        void draw_ENLSVG_Extent(const std::vector<int> &extents, freeNav::DimensionLength dimen[2], double scale = .7);

        void draw_ENLSVG_ScannerInterval(const Pathfinding::ScanInterval &scanner_interval,
                                         const cv::Scalar &color = cv::Scalar::all(0));

        void draw_ENLSVG_ScannerStacks(const Pathfinding::ScannerStacks &scanner_stacks,
                                       const cv::Scalar &color = cv::Scalar::all(0));

        void drawPathi(const std::vector<Pathfinding::GridVertex> &path, const cv::Scalar &color = cv::Scalar(0, 0, 0));

    };

}
#endif //FREENAV_CANVAS_H
