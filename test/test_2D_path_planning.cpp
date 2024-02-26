//
// Created by yaozhuo on 2022/1/11.
//

#include "gtest/gtest.h"
#include "sys/time.h"
#include <limits>

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/thread_pool.h"
#include "../freeNav-base/visualization/canvas/canvas.h"

#include "../algorithm/surface_processor_LineScanner.h"

#include "../freeNav-base/dependencies/2d_grid/picture_loader.h"
#include "../algorithm/graph_construction/tangent_graph_build.h"

#include "../algorithm/online_search/create_initial_paths_with_edge.h"

#include "../algorithm/constraints/point_to_point_constraints.h"
#include "../algorithm/constraints/edge_transfer_constraints.h"

#include "../freeNav-base/dependencies/2d_grid/2d_ENLSVG_grid.h"

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"

#include "../algorithm/online_search/breadth_first_search_with_edge.h"
#include "../algorithm/online_search/depth_first_search_with_edge.h"
#include "../algorithm/online_search/search_path_with_node.h"
#include "../algorithm/online_search/breadth_first_search_with_node.h"
#include "../algorithm/online_search/depth_first_search_with_node.h"
#include "../algorithm/online_search/create_initial_paths_with_node.h"



//#include "topologyPathPlanning/harrt_test.h"


// https://github.com/PathPlanning/3D-AStar-ThetaStar

using namespace freeNav::Topo;
using namespace freeNav;

auto is_grid_occupied1 = [](const cv::Vec3b& color) -> bool {
    if (color != cv::Vec3b::all(255)) return true;
    return false;
};

auto is_grid_occupied2 = [](const cv::Vec3b& color) -> bool {
    if (color[0] <= 200 || color[1] <= 200 || color[2] <= 200) return true;
    return false;
};

auto is_char_occupied = [](const char& value) -> bool {
    if (value != '.' && value != 'G' && value != 'S') return true;
    return false;
};

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};
#define USE_PIC_MAP 0
#if USE_PIC_MAP
// tarjan search raise hardfault in map fr-campus-rectified
// corridor.bmp
std::string file_name = "jump_test1.bmp";//"map_001.png";
PictureLoader loader("/home/yaozhuo/code/free-nav/resource/map/" + file_name, is_grid_occupied2);
std::string vis_file_path = "/home/yaozhuo/code/free-nav/resource/binary/" + file_name + ".vis";

// MapTestConfig_Entanglement;
// MapTestConfig_FloodedPlains;
// MapTestConfig_dustwallowkeys;
// MapTestConfig_Boston_0_1024;
// MapTestConfig_TheFrozenSea;
// MapTestConfig_maze512_4_8
// MapTestConfig_8room_002
// MapTestConfig_Aurora
// MapTestConfig_Berlin_1_256
int zoom_rate = 1;

#else

//MapTestConfig_TheFrozenSea
//MapTestConfig_FloodedPlains
//MapTestConfig_maze512_4_8
//MapTestConfig_Berlin_1_256
//MapTestConfig_maze512_4_0
//MapTestConfig_Entanglement
//MapTestConfig_Aurora
//MapTestConfig_dustwallowkeys // with parse error
//MapTestConfig_8room_002
//MapTestConfig_Boston_0_1024
//MapTestConfig_random512_10_0 // RJ success
//MapTestConfig_random512_35_2
//MapTestConfig_Denver_2_512
// MapTestConfig_Milan_1_512
//MapTestConfig_Sydney_1_256
// MapTestConfig_Shanghai_0_512

//MapTestConfig_Berlin_1_256, // ok
//MapTestConfig_Denver_2_256, // ok
//MapTestConfig_Boston_2_256, // ok
//MapTestConfig_Milan_2_256, //
//MapTestConfig_Moscow_2_256, // 100 not pass
//MapTestConfig_London_2_256, // ok
//MapTestConfig_Sydney_1_256, // ok
//MapTestConfig_Paris_0_256 // ok

//  MapTestConfig_AR0013SR,
//  MapTestConfig_AR0014SR,
//  MapTestConfig_AR0018SR,
//  MapTestConfig_AR0205SR,
//  MapTestConfig_TheFrozenSea //
//  MapTestConfig_EbonLakes,
//  MapTestConfig_Enigma,
//  MapTestConfig_Entanglement

auto map_test_config = MapTestConfig_AR0205SR;//MapTestConfig_AR0205SR;
std::string vis_file_path    = map_test_config.at("vis_path");

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 4;

#endif

auto dimension = loader.getDimensionInfo();

std::shared_ptr<RoadMapGraphBuilder<2> > tgb = nullptr;

RoadMapEdgeTraitPtrs<2> init_start_edges;
RoadMapEdgeTraitPtrs<2> init_target_edges;

PathPointWithEdgePtrs<2> init_start_paths;
PathPointWithEdgePtrs<2> init_target_paths;

RoadMapNodes<2> initial_start_nodes;
RoadMapNodes<2> initial_target_nodes;

bool new_pair = false;
bool plan_finish = false;
bool draw_candidate = true;
bool show_start_now = true;
bool draw_all_path = true;

bool draw_dijk   = false;
bool draw_star   = false;
bool show_rj     = true;
bool draw_enlsvg = false;
bool draw_rrt    = false;
bool draw_jps    = false;
bool draw_dmp    = false;
bool draw_lazy_theta = false;

bool center_offset = false;
ThreadPool tp;

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

Pointis<1> neighbor = GetNeightborOffsetGrids<1>();

IS_LINE_COLLISION_FREE_FUNC<2> line_collision_check = [](const Pointi<2> & pt1, const Pointi<2> & pt2) -> bool {
    return LineCrossObstacle(pt1, pt2, is_occupied_func, neighbor);
};



bool show_graph = false;
bool set_pt1 = true;
int current_show_index = 0;

Pointi<2> pt1, pt2;
FractionPair pt1_f, pt2_f;
GridPtr<2> sg1 = std::make_shared<Grid<2>>(),
        sg2 = std::make_shared<Grid<2>>();

// node and edge constraints
PointTransferConstraints<2> ptcs({PTC_LocalCrossPartially // cause missing of edge in map corridor.png
                                         ,
                                  PTC_DoNotCrossObstacle // when use ENL-SVG style collision check, ignore this
                                 });

PointTransferConstraints<2> ptcs_ordered;
// edge transfer constraints
EdgeTransferConstraints3<2> etcs({//ETC_NotLookBack3 // cause missing of tangent line when use ENL-SVG style collision check and cubic line of sight check
                                         //,
                                         ETC_NoUselessPoint3 // the first and last node of the two edge shouldn't visible, cause loss of legal edge transfer in MapTestConfig_maze512_4_0
                                         //, ETC_IC_GetCloserToObstacle_ENLSVG
                                         ,
                                         ETC_IC_GetCloserToObstacle3 //, cause sHETSL error
                                 });

PointTransferConstraints<2> init_ptcs({PTC_CrosserToObstacle_2D});
EdgeTransferConstraints3<2> init_etcs({ETC_Taut_2D});

IterationConstraints<2, GridPtr<2>> distinctive_ics({
                                                                              IC_NoLoop, // if is global shortest, no need to add IC_NoLoop
                                                                              IC_EdgeNLevelLimitUndirectedGraph
                                                                      });

IterationConstraints<2, GridPtr<2>> ics({
                                                                  //IC_NoLoop, // if is global shortest, no need to add IC_NoLoop
                                                                  IC_EdgeNLevelLimit
                                                          });

NodeIterationConstraints<2, GridPtr<2>> ics_node_topology({
                                                                  NIC_NoLoop, // if is global shortest, no need to add IC_NoLoop
                                                                  //IC_EdgeNLevelLimit
                                                                  NIC_NoStraightExtend,
                                                                  //NIC_Taut_2D
                                                          });

TEST(RIMJUMP, LINE_DISCRETE) {

}

bool show_loop = false;

TEST(RIMJUMP, COLLISION_CHECK) {
    std::cout << "int64 " << sizeof(int64) << std::endl;
    std::cout << "char " << sizeof(char) << std::endl;
    std::cout << "bool " << sizeof(bool) << std::endl;
    std::cout << "int32_t " << sizeof(int32_t) << std::endl;
    std::cout << "int16_t " << sizeof(int16_t) << std::endl;
    std::cout << "int8_t " << sizeof(int8_t) << std::endl;
    return;
}

TEST(RIMJUMP, getPerpedicularPoint) {

}


TEST(RIMJUMP, LINE_CROSS_LINE_CHECK) {

}

TEST(RIMJUMP, PRE_NEXT_EDGES_VISUALIZE) {

}

TEST(RIMJUMP, TANGENT_GRAPH_BUILD) {
    Canvas canvas("tangent graph build", dimension[0], dimension[1], .05, zoom_rate);
    auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    //auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);
    //auto surface_processor = std::make_shared<SurfaceProcess_LineScanner>(dimension, is_occupied_func, set_occupied_func);

    tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, ptcs, ptcs_ordered, etcs, vis_file_path, false, true, 1);
    //tgb = new RoadMapGraphBuilder<2>(surface_processor, init_ptcs, ptcs_ordered, init_etcs, vis_file_path);//, true);

    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "the tangent graph has " << tg->edges_.size() << " tangent " << std::endl;
    auto callback = [](int event, int x, int y, int flags, void *){
        if(event == CV_EVENT_LBUTTONDOWN) {
            pt1[0] = x;
            pt1[1] = y;
            std::cout << "get point " << x << ", " << y << std::endl;
        }
    };

    canvas.setMouseCallBack(callback);
    bool is_split = true;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied_func);
        if(show_graph) {
            if(!tg->edges_.empty()) {
                canvas.drawEdges(tg, tg->edges_, center_offset, cv::Vec3b(0,255,0));
            } else {
                //canvas.drawNodes(tg, data, center_offset, cv::Vec3b(0,255,0));
            }
        }

        auto grid_ptr = surface_processor->grid_map_[PointiToId(pt1, dimension)];
        if(grid_ptr != nullptr && grid_ptr->node_id_!= MAX<NodeId>) {
            auto node_ptr = tg->nodes_[grid_ptr->node_id_];
            if(is_split) {
                if(!node_ptr->nextEdges(true).empty()) {
                    std::cout << " split size " << node_ptr->nextEdges(true).size() << std::endl;
                    for(const auto& edge : node_ptr->nextEdges(true)) {
                        //std::cout << edge.second->next_node(false)->sg_->pt_ << "->" << edge.second->next_node(true)->sg_->pt_ << std::endl;
                        //canvas.drawGridLine(pt1[0], pt1[1], edge.second->to_->sg_->pt_[0], edge.second->to_->sg_->pt_[1], 1, true, COLOR_TABLE[3]);
                        //canvas.drawArrowInt(pt1[0], pt1[1], edge.second->next_node(true)->sg_->pt_[0], edge.second->next_node(true)->sg_->pt_[1], 1, true, COLOR_TABLE[3]);
                        canvas.drawEdge(tg, tg->edges_[edge], center_offset, false, false, COLOR_TABLE[3]);
                    }
                } else {
                    canvas.drawNode(tg, grid_ptr->node_id_, center_offset, COLOR_TABLE[3]);
                }
            } else {
                std::cout << " converge size " << node_ptr->nextEdges(false).size() << std::endl;
                for(const auto& edge : node_ptr->nextEdges(false)) {
                    //canvas.drawGridLine(pt1[0], pt1[1], edge.second->to_->sg_->pt_[0], edge.second->to_->sg_->pt_[1], 1, true, COLOR_TABLE[3]);
                    //canvas.drawArrowInt(edge.second->next_node(false)->sg_->pt_[0], edge.second->next_node(false)->sg_->pt_[1], pt1[0], pt1[1], 1, true, COLOR_TABLE[3]);
                    canvas.drawEdge(tg, tg->edges_[edge], center_offset, false, false, COLOR_TABLE[3]);
                }
            }
            for(const auto& obst : node_ptr->sg_->nearby_obst_pts_) {
                canvas.drawCircleInt(obst[0], obst[1], 5, center_offset, 1, COLOR_TABLE[2]);
            }
        }
        char key = canvas.show(300);
        if(key == ' ') {
            show_graph = !show_graph;
        } else if(key == 's') {
            is_split = !is_split;
        }
    }
}

TEST(RIMJUMP, angleBetweenThreePoints) {

}

TEST(RIMJUMP, ZERO_PROJECT_TO_LINE) {

}


RoadMapEdgeTraitPtrss<2> rmetps;
std::vector<Path<2>> paths;
std::vector<Path<2>> external_paths{{}, {}, {}, {}, {}};

double initial_time, search_graph_time;

std::vector<Pointi<2> > all_points;

TEST(RIMJUMP, PathPlanning) {

    struct timezone tz;
    struct timeval  tv_pre;
    struct timeval  tv_after;


    gettimeofday(&tv_pre, &tz);

    auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);
    //auto surface_processor = std::make_shared<SurfaceProcess_LineScanner>(dimension, is_occupied_func, set_occupied_func);
    //
    //auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    surface_processor->surfaceGridsDetection(true);

//    BlockDetectorGreedyPtr<2> block_detect = std::make_shared<BlockDetectorGreedy<2> >(
//            dimension, is_occupied_func, surface_processor->getSurfacePts(), 10);
//
//
//    IS_LINE_COLLISION_FREE_FUNC<2> line_collision_check_block = [&](const Pointi<2> & pt1, const Pointi<2> & pt2) -> bool {
//        std::vector<Pointi<2> > visited_pts;
//        int count_of_block;
////        LineCrossObstacleWithBlockJump(pt1, pt2,
////                                       (BlockDetectorInterfacePtr<2>)block_detect,
////                                       visited_pts,
////                                       count_of_block);
//        return LineCrossObstacleWithBlockJump(pt1, pt2,
//                                              (BlockDetectorInterfacePtr<2>)block_detect,
//                                              visited_pts,
//                                              count_of_block);
//    };

    #define WITH_EDGE 0
#if WITH_EDGE
    //tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, ptcs, ptcs_ordered, etcs, vis_file_path, false, true, 4);
    tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, init_ptcs, ptcs_ordered, init_etcs, vis_file_path, true, true, 1)%;
#else
    tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, init_ptcs, ptcs_ordered, init_etcs, vis_file_path, true, false, 1);
#endif
    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "-- the tangent graph has " << tg->edges_.size() << " edges " << std::endl;
    std::cout << "-- the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl;
    std::cout << "-- is the graph undirected ? " << tg->UndirectedGraphCheck() << std::endl << std::endl;
    //updateRoadMapEdgePointer(tg);
#if WITH_EDGE
    GeneralGraphPathPlannerWithEdge<2> g2p2(tg);
#else
    GeneralGraphPathPlannerWithNode<2> g2p2(tg);
#endif
    /* init ENL-SVG map */
    gettimeofday(&tv_pre, &tz);
    auto ENL_SVG_grid = getENL_SVG_Module(dimension, is_occupied_func);
    Pathfinding::Path path_ENLSVG;
    /* 2, pre computation */

    // Preprocess the grid to build a visibility graph.
    Pathfinding::ENLSVG::Algorithm algo(ENL_SVG_grid);
    algo.printStatistics();

    // Initialise a memory object for the algorithm. The algorithm uses this memory object for its path computations.
    Pathfinding::ENLSVG::Memory memory(ENL_SVG_grid);
    gettimeofday(&tv_after, &tz);
    double cost_ms1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- ENL-SVG Preprocess end in " << cost_ms1 << " ms" << std::endl;
    /* end ENL-SVG map */

    // 290, 428) to (232, 445
    // (1, 137) -> (109, 176) moscow
    //  (67, 153) -> (166, 168) moscow
    Canvas canvas("RimJump::PathPlanning", dimension[0], dimension[1], .05, zoom_rate);
    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = 18;//x;
                pt1[1] = 95;//y;
                sg1->pt_ = pt1;
                sg1->id_ = PointiToId<2>(pt1, dimension);
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
                all_points.push_back(pt1);
            } else {
                pt2[0] = 186;//x;
                pt2[1] = 135;//y;
                sg2->pt_ = pt2;
                sg2->id_ = PointiToId<2>(pt2, dimension);
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                all_points.push_back(pt2);
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);
    bool draw_block_ptr = false;
    ExitCode ec;
    double sum_no_block = 0, sum_with_block = 0;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied_func);
        if(show_graph) {
            canvas.drawEdges(tg, tg->edges_, false, COLOR_TABLE[0]);
#if !WITH_EDGE
            canvas.drawNodes(tg, g2p2.data_, center_offset, COLOR_TABLE[0]);
#endif
        //canvas.drawTangentGraphLegalEdges(tg, !center_offset, COLOR_TABLE[0], COLOR_TABLE[1]); // RimJumpGraph
            //canvas.drawENLVisibilityGraph(algo.graph);
        }
        if(draw_block_ptr) {
            //canvas.draw_DistMap(block_detect.dimension_info_, block_detect.dist_map_);
//            int total_count = getTotalIndexOfSpace<2>(dimension);
//            for(int i=0; i<total_count; i++) {
//                if(block_detect->block_ptr_map_[i] != nullptr) {
//                    Pointi<2> pt = IdToPointi<2>(i, dimension);
//                    Pointi<2> center_pt = block_detect->block_ptr_map_[i]->max_;
//                    Id center_id = PointiToId(center_pt, dimension);
//                    canvas.drawGrid(pt[0], pt[1], COLOR_TABLE[center_id%30]);
//                }
//            }
        }
        if(new_pair) {
            new_pair = false;
            if(tp.pool_[0].joinable() && !plan_finish)
                tp.Schedule([&] {
                    current_show_index = 0;
                    for(int i=0; i<1; i++) {
                        //std::cout << "-- start RimJump " << std::endl;
                        external_paths = {{}, {}, {}, {}, {}};
                        plan_finish = false;
//                        ExitCode ec = g2p2.planning(pt1, pt2, ics, paths, true, true, false);
                        //ExitCode ec = g2p2.planning(pt1, pt2, ics, paths, true, true, false, 10);
//                        if(!paths.empty()) {
//                            for(int i=0; i<paths.size(); i++) {
//                                std::cout << " RJ get path: " << paths[i] << std::endl;
//                            }
//                        }
//                        auto statistic = g2p2.getStatistic();
//                        if (ec != ExitCode::SUCCESS) {
//                            std::cout << "-- RimJump failed with " << ec << " in " << statistic[2] << " + " << statistic[3]
//                                      << "ms" << std::endl;
//                        } else {
//                            std::cout << "-- RimJump success in " << statistic[2] << " + " << statistic[3] << "ms"
//                                      << std::endl;
//                        }

#if WITH_EDGE
                        //ec = g2p2.planningWithEdge(pt1, pt2, init_ptcs, init_etcs, ics, paths, false, true, true);
                        //ec = g2p2.planningWithEdge(pt1, pt2, init_ptcs, init_etcs, distinctive_ics, paths, true, false, false, 5);
                        //ec = g2p2.planningWithEdge(pt1, pt2, {}, etcs, ics, paths, false, true, true);
                        ec = g2p2.planningWithEdge(pt1, pt2, init_ptcs, init_etcs, ics, paths, true, true, true);
#else
                        ec = g2p2.planningWithNode(pt1, pt2, init_ptcs, init_etcs, ics_node_topology, paths,
                                                   false, false, false,
                                                   100);
                        std::cout << " path " << paths.size() << std::endl;
#endif

                        //std::cout << " RJ without get " << paths.size() << " path(s) " << std::endl;
                        if(!paths.empty()) {
//                            for(int i=0; i<paths.size(); i++) {
//                                std::cout << " RJ without get path " << paths[i] << std::endl;
//                                std::cout << " RJ without get path length " << calculatePathLength(paths[i]) << std::endl;
//                            }
                        }
                        auto statistic_without = g2p2.getStatistic();
                        if (ec != ExitCode::SUCCESS) {
                            std::cout << "-- RimJump WithOut failed with " << ec << " in " << statistic_without[2] << " + " << statistic_without[3]
                                      << "ms" << std::endl;
                        } else {
                            std::cout << "-- RimJump WithOut success in " << statistic_without[2] << " + " << statistic_without[3] << "ms"
                                      << std::endl;
                        }

                        /* try ENL-SVG */
//                        path_ENLSVG = algo.computePath(memory, pt1[0], pt1[1], pt2[0], pt2[1], initial_time, search_graph_time);
//                        if (path_ENLSVG.empty()) {
//                            std::cout << "-- ENL-SVG failed in " << initial_time << " + " << search_graph_time << "ms"
//                                      << std::endl;
//                        } else {
//                            std::cout << "-- ENL-SVG success in " << initial_time << " + " << search_graph_time << " ms"
//                                      << std::endl;
//                        }
                        //cout << "-*-*- ENL-SVG path length = " << calculatePathLength(path_ENLSVG) << endl;
                        /* end ENL-SVG */
//                        external_paths.clear();
//                        /* try ros Dijkstra */
//                        FloatPath path_dijkf = {};//DijkstraPlanner(cost_move_base_2d, pt1[0], pt1[1], pt2[0], pt2[1],
//                                                  //             dimension[0], dimension[1]);
//                        Path<2> path_dijk = ToRimJumpPath(path_dijkf);
//                        cout << "-*-*- Dijk path length = " << calculatePathLength(path_dijk) << endl;
//                        external_paths.push_back(path_dijk);
                        /* end ros Dijkstra */

                        /* try ros A* */
//                        FloatPath path_astarf = AstarPlanner(cost_move_base_2d, pt1[0], pt1[1], pt2[0], pt2[1],
//                                                          dimension[0], dimension[1]);
//                        Path<2> path_astar = ToRimJumpPath(path_astarf);
//                        cout << "-*-*- Astar path length = " << calculatePathLength(path_astar) << endl;
//                        external_paths.push_back(path_astar);
                        /* end ros A* */

                        /* try ros Theta* */
//                        FloatPath path_thetastarf = ThetaStarPlanner(cost_move_base_2d, pt1[0], pt1[1], pt2[0], pt2[1],
//                                  dimension[0], dimension[1]);
//                        Path<2> path_thetastar = ToRimJumpPath(path_thetastarf);
//                        cout << "-*-*- ThetaStar path length = " << calculatePathLength(path_thetastar) << endl;
//                        external_paths.push_back(path_thetastar);
                        /* end ros Theta* */

                        /* try ompl RRT */
//                        gettimeofday(&tv_pre, &tz);
//                        Path<2> path_rrt;// = ompl_path_planner.plan(pt1, pt2);
//                        external_paths.push_back(path_rrt);
//                        gettimeofday(&tv_after, &tz);
//                        double cost_ms_rrt = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
//                        cout << "-*-*- RRT cost " << cost_ms_rrt << "ms" << endl;
//                        cout << "-*-*- RRT path length = " << calculatePathLength(path_rrt) << endl;
                        /* end ompl RRT */

                        /* try JPS */
//                        Path<2> path_jps  = {};//JPS_planner_ptr->plan(pt1, pt2, 1, true); // Plan from start to goal using JPS
//                        cout << "-*-*- JPS path length = " << calculatePathLength(path_jps) << endl;
//                        external_paths.push_back(path_jps);
                        /* end JPS */

                        /* try DMP */
//                        dmp.setMap(map_util, pt1);
//                        Path<2> path_dmp  = {};//dmp.computePath(pt1, pt2, path_jps); // Plan from start to goal using JPS
//                        cout << "-*-*- DMP(APF) path length = " << calculatePathLength(path_dmp) << endl;
//                        external_paths.push_back(path_dmp);
                        /* end DMP */
                        //std::cout << " before external_paths size " << external_paths.size() << std::endl;

                        /* try Lazy Theta, AC */
//                        gettimeofday(&tv_pre, &tz);
//                        Path<2> path_lazy_theta = LazyThetaWrap(theta_star_map, pt1, pt2, line_collision_check);
//                        gettimeofday(&tv_after, &tz);
//                        double cost_ms_thetastar = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
//                        cout << "-*-*- Lazy Theta* path length = " << calculatePathLength(path_lazy_theta) << " in " << cost_ms_thetastar <<  " ms " << endl;
//                        external_paths.push_back(path_lazy_theta);
                        /* end Lazy Theta*/

                        /* try Theta from ros_motion_planner */
                        // Theta* LazyTheta* InformedRRT acc success, RRT and RRT* not well
                        gettimeofday(&tv_pre, &tz);
                        // line_collision_check_block
                        // line_collision_check
                        Path<2> path1;// = InformedRRTRimJump(cost_move_base_2d, dimension, .1, line_collision_check, pt1, pt2, 2000, dimension[0]*0.5, 2000);//1.5*dimension[0]);
                        //Path<2> path1 = LazyThetaStarRimJump(cost_move_base_2d, dimension, line_collision_check, pt1, pt2);
                        gettimeofday(&tv_after, &tz);
                        double cost_ms_1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                        //cout << "-*-*- no JOB from ros_motion_planner path length = " << calculatePathLength(path1) << " in " << cost_ms_thetastar_ros <<  " ms " << endl;
                        external_paths.push_back(path1);
                        sum_no_block += cost_ms_1;

                        gettimeofday(&tv_pre, &tz);
                        path1;// = InformedRRTRimJump(cost_move_base_2d, dimension, .1, line_collision_check_block, pt1, pt2, 2000, dimension[0]*0.5, 2000);//1.5*dimension[0]);
                        //path1 = LazyThetaStarRimJump(cost_move_base_2d, dimension, line_collision_check_block, pt1, pt2);
                        gettimeofday(&tv_after, &tz);
                        double cost_ms_2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
                        //cout << "-*-*- with JOB from ros_motion_planner path length = " << calculatePathLength(path1) << " in " << cost_ms_thetastar_ros <<  " ms " << endl;
                        external_paths.push_back(path1);
                        sum_with_block += cost_ms_2;

                        // exception about polygon self exception in init_regions in Worldmap.cpp
//                        auto harrt_path = HARRT_demo(is_occupied_func, dimension, pt1, pt2,
//                                                     10, 10);
//                        for(const auto& path : harrt_path) {
//                            std::cout << path << std::endl;
//                        }
                        /* end Theta from ros_motion_planner */

                        //std::cout << " final external_paths size " << external_paths.size() << std::endl;
                    }
                    //std::cout << "acc ratio " << sum_with_block/sum_no_block << std::endl;
                    plan_finish = true;
                });
        }
        canvas.drawCircleInt(pt1[0], pt1[1], 5, center_offset, -1, COLOR_TABLE[0]);
        canvas.drawCircleInt(pt2[0], pt2[1], 5, center_offset, -1, COLOR_TABLE[1]);
        if(show_rj && plan_finish && !paths.empty()) {
            if(draw_all_path) {
                for (int i=0; i<paths.size(); i++) {
                    canvas.drawPath(paths[i], center_offset, COLOR_TABLE[i%30]);
                }
            } else {
                canvas.drawPath(paths[current_show_index], center_offset, COLOR_TABLE[current_show_index]);
            }
        }
        if(draw_enlsvg) canvas.drawPathi(path_ENLSVG);
        if(draw_dijk) {
            if(external_paths.size() >= 1)
                canvas.drawPath(external_paths[0], center_offset, COLOR_TABLE[3]);
        }
        if(draw_star) {
            if(external_paths.size() >= 2)
                canvas.drawPath(external_paths[1], center_offset, COLOR_TABLE[4]); }
        if(draw_rrt)  {
            if(external_paths.size() >= 3)
                canvas.drawPath(external_paths[2], center_offset, COLOR_TABLE[5]); }
        if(draw_jps)  {
            if(external_paths.size() >= 4)
                canvas.drawPath(external_paths[3], center_offset, COLOR_TABLE[6]); }
        if(draw_dmp)  {
            if(external_paths.size() >= 5)
                canvas.drawPath(external_paths[4], center_offset, COLOR_TABLE[7]); }
        if(draw_lazy_theta)  {
            if(external_paths.size() >= 6)
                canvas.drawPath(external_paths[5], center_offset, COLOR_TABLE[8]); }
        char key_value = canvas.show(33);
        if(key_value == 'g') {
            show_graph = !show_graph;
        } else if(key_value == 'w' && plan_finish) {
            current_show_index ++;
            current_show_index = current_show_index % paths.size();
        } else if(key_value == 's' && plan_finish) {
            current_show_index --;
            current_show_index += paths.size();
            current_show_index = current_show_index % paths.size();
        } else if(key_value == 'r') {
            show_rj = !show_rj;
        } else if(key_value == 'a') {
            draw_all_path = !draw_all_path;
        } else if(key_value == 'd') {
            draw_dijk = !draw_dijk;
        } else if(key_value == 'h') {
            draw_star = !draw_star;
        } else if(key_value == 'e') {
            draw_enlsvg = !draw_enlsvg;
        } else if(key_value == 't') {
            draw_rrt = !draw_rrt;
        } else if(key_value == 'j') {
            draw_jps = !draw_jps;
        } else if(key_value == 'f') {
            draw_dmp = !draw_dmp;
        } else if(key_value == 'l') {
            draw_lazy_theta = !draw_lazy_theta;
        } else if(key_value == 'b') {
            draw_block_ptr = !draw_block_ptr;
        } else if(key_value == 32) {
            for(int i=0; i<all_points.size()/2; i++) {
                std::cout << "{{" << all_points[2*i][0] << "," << all_points[2*i][1] << "}, {" << all_points[2*i+1][0] << "," << all_points[2*i+1][1] << "}}," << std::endl;
            }
        }
    }
}

TEST(ENLSVG_Fraction, test) {
    std::cout << Pathfinding::Fraction::gcd(5, 8) << std::endl;
    std::cout << Pathfinding::Fraction::gcd(8, 3) << std::endl;
    std::cout << Pathfinding::Fraction::gcd(18, 15) << std::endl;

}

TEST(createVisiblePathPointPtrsWithEdge, test) {
    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;

    Canvas canvas("RimJump::createVisiblePathPointPtrsWithEdge", dimension[0], dimension[1], .05, zoom_rate);
    gettimeofday(&tv_pre, &tz);
    //auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);

    tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, init_ptcs, ptcs_ordered, init_etcs, vis_file_path, false);
    //tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, ptcs, ptcs_ordered, etcs, vis_file_path, true, true, 1);

    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "the tangent graph has " << tg->edges_.size() << " edges " << std::endl;
    std::cout << "the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl << std::endl;

    GeneralGraphPathPlannerWithEdge<2> g2p2(tg);
    //(462, 496) to (464, 509)
    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = 762;
                pt1[1] = 771;
                sg1->pt_ = pt1;
                sg1->id_ = PointiToId<2>(pt1, dimension);
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
            } else {
                pt2[0] = 18;
                pt2[1] = 14;
                sg2->pt_ = pt2;
                sg2->id_ = PointiToId<2>(pt2, dimension);
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);

    auto ENL_SVG_grid = getENL_SVG_Module(dimension, is_occupied_func);
    Pathfinding::Path path;

    /* 2, pre computation */
    gettimeofday(&tv_pre, &tz);
    // Preprocess the grid to build a visibility graph.
    Pathfinding::ENLSVG::Algorithm algo(ENL_SVG_grid);
    algo.printStatistics();

    // Initialise a memory object for the algorithm. The algorithm uses this memory object for its path computations.
    Pathfinding::ENLSVG::Memory memory(ENL_SVG_grid);
    gettimeofday(&tv_after, &tz);
    double cost_ms1 =(tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- ENL-SVG Preprocess end in " << cost_ms1 << " ms" << std::endl;

    bool draw_all_path = true;
    bool draw_edges = false;
    bool is_edge = true;
    RoadMapEdgeTraitPtrs<2> all_marked_edges_from_start;
    RoadMapEdgeTraitPtrs<2> all_marked_edges_from_target;
    show_start_now = false;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied_func);
        if(show_graph) {
            //canvas.drawTangentGraphAllNodes(tg, COLOR_TABLE[0], COLOR_TABLE[1]);
            //canvas.drawEdges(tg.original_edge_iters_);
            //canvas.drawTangentGraphLegalEdges(tg, COLOR_TABLE[0], COLOR_TABLE[1]);
            canvas.drawENLVisibilityGraph(algo.graph, memory);
        }
        if(new_pair) {
            new_pair = false;
            if(tp.pool_[0].joinable() && !plan_finish)
                tp.Schedule([&]{
                    int count = 1000;
                    while(count > 0) {
                        count --;
                        //g2p2.tg_.initialise();

                        std::cout << "-- start RimJump create initial path " << std::endl;
                        plan_finish = false;
                        global_minimum_path_length = std::numeric_limits<PathLen>::max();
                        gettimeofday(&tv_pre, &tz);

                        // 1, create target set
                        init_target_edges.clear();
                        g2p2.initialise();
                        g2p2.edge_mode_ = false;
                        g2p2.depth_first_ = true;
                        g2p2.global_shortest_ = true;
                        //init_target_edges = g2p2.createVisiblePathPointPtrsWithOutLength(sg2->pt_, false, init_ptcs, init_etcs);
                        //std::cout << "target edges: size = " << init_target_edges.size() << std::endl;
//                    for(auto target_edge : init_target_edges) {
//                        target_edge->print(is_edge, false);
//                    }

                        // 2, create start set
                        init_start_edges.clear();
                        //initial_start_nodes.clear();
                        auto retv = g2p2.createVisiblePathPointPtrsPairWithEdge(sg1->pt_, sg2->pt_, init_ptcs, init_etcs);
                        //auto retv = g2p2.createVisiblePathPointPtrsPairWithOutLength(sg1->pt_, sg2->pt_, ptcs, etcs);
                        //std::cout << "start edges: size = " << init_start_edges.size() << std::endl;
//                    for(auto start_edge : init_start_edges) {
//                        start_edge->print(is_edge, true);
//                    }

                        gettimeofday(&tv_after, &tz);
                        init_start_paths = retv.first;
                        init_target_paths = retv.second;
                        double cost_ms1 =
                                (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                        std::cout << "-- RimJump finish create path in " << cost_ms1 << " ms" << std::endl;

                        path = algo.computePath(memory, pt1[0], pt1[1], pt2[0], pt2[1], initial_time, search_graph_time);
                        std::cout << "-- ENLSVG create init path in " << initial_time << " ms" << std::endl;

                    }
                    all_marked_edges_from_start.clear();
                    for(auto& edge : g2p2.tg_->edges_) {
                        if(g2p2.data_->is_marked(edge, true, true, g2p2.currentTime())) {
                            //if(edge->isHyperLoopEdge()) std::cout << " start reach loop edge " << std::endl;
                            all_marked_edges_from_start.push_back(edge);
                        }
                    }
                    std::cout << " all_marked_edges_from_start size " << all_marked_edges_from_start.size() << std::endl;
                    all_marked_edges_from_target.clear();
                    for(auto& edge : g2p2.tg_->edges_) {
                        if(g2p2.data_->is_marked(edge, true, false, g2p2.currentTime())) {
                            //if(edge->isHyperLoopEdge()) std::cout << " target reach loop edge " << std::endl;
                            all_marked_edges_from_target.push_back(edge);
                        }
                    }
                    std::cout << " all_marked_edges_from_target size " << all_marked_edges_from_target.size() << std::endl;
                    plan_finish = true;
                });
        }
        canvas.drawCircleInt(pt1[0], pt1[1], 5, center_offset, -1, COLOR_TABLE[0]);
        canvas.drawCircleInt(pt2[0], pt2[1], 5, center_offset, -1, COLOR_TABLE[1]);
        if(plan_finish && !show_graph) {
            if(!draw_all_path) {
                if(show_start_now && !init_start_edges.empty()) {
                    auto current_edge = init_start_edges.begin();
                    for(int i=0; i<current_show_index; i++) {
                        current_edge ++;
                    }
                    canvas.drawRoadMapEdgeAsPath(g2p2, *current_edge, center_offset, true, true);
                    for(int i=0; i<init_start_paths.size(); i++) {
                        canvas.drawRoadMapEdgeAsPath(g2p2, init_start_paths[i]->last_edge_, center_offset, true, true, cv::Vec3b(255,0,0));
                    }
                } else if(!init_target_edges.empty()) {

                    auto current_edge = init_target_edges.begin();
                    for(int i=0; i<current_show_index; i++) {
                        current_edge ++;
                    }
                    canvas.drawRoadMapEdgeAsPath(g2p2, *current_edge, center_offset, true, false);

                    for(int i=0; i<init_target_paths.size(); i++) {
                        canvas.drawRoadMapEdgeAsPath(g2p2, init_target_paths[i]->last_edge_, center_offset, true, true, cv::Vec3b(0,0,255));
                    }

                }
            } else {
                // draw all marked edge
                if(show_start_now) {
                    for(int i=0; i<all_marked_edges_from_start.size(); i++) {
                        canvas.drawRoadMapEdgeAsPath(g2p2, all_marked_edges_from_start[i], center_offset, true, true, cv::Vec3b(0,255,0));
                    }
                }
                if(!show_start_now) {
                    for(int i=0; i<all_marked_edges_from_target.size(); i++) {
                        canvas.drawRoadMapEdgeAsPath(g2p2, all_marked_edges_from_target[i], center_offset, true, false, cv::Vec3b(0,255,0));
                    }
                }
            }
        }
        //canvas.drawPathi(path);
        char key_value = canvas.show();
        if(key_value == 't') {
            show_start_now = !show_start_now;
        }  else if(key_value == 'w' && plan_finish) {
            current_show_index ++;
            if(show_start_now) {
                current_show_index = current_show_index % init_start_edges.size();
            } else {
                current_show_index = current_show_index % init_target_edges.size();
            }
        }
//        else if(key_value == 's' && plan_finish) {
//            current_show_index --;
//            if(show_start_now) {
//                if(!init_start_edges.empty()) {
//                    current_show_index += init_start_edges.size();
//                    current_show_index = current_show_index % init_start_edges.size();
//                }
//            } else {
//                if(!init_target_edges.empty()) {
//                    current_show_index += init_target_edges.size();
//                    current_show_index = current_show_index % init_target_edges.size();
//                }
//            }
//        }
        else if(key_value == 'a') {
            draw_all_path = !draw_all_path;
        } else if(key_value == 'g') {
            show_graph = !show_graph;
        }
    }
}


TEST(createVisiblePathPointPtrsWithNode, test) {
    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;

    Canvas canvas("RimJump::createVisiblePathPointPtrsWithNode", dimension[0], dimension[1], .05, zoom_rate);
    gettimeofday(&tv_pre, &tz);
    //auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);

    tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, init_ptcs, ptcs_ordered, init_etcs, vis_file_path, false);
    //tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, ptcs, ptcs_ordered, etcs, vis_file_path, true, true, 1);

    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "the tangent graph has " << tg->edges_.size() << " edges " << std::endl;
    std::cout << "the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl << std::endl;

    GeneralGraphPathPlannerWithNode<2> g2p2(tg);
    //(462, 496) to (464, 509)
    // 365, 222) -> (341, 251
    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = 365;//762;
                pt1[1] = 222;//771;
                sg1->pt_ = pt1;
                sg1->id_ = PointiToId<2>(pt1, dimension);
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
            } else {
                pt2[0] = 341;//18;
                pt2[1] = 251;//14;
                sg2->pt_ = pt2;
                sg2->id_ = PointiToId<2>(pt2, dimension);
                set_pt1 = true;
                std::cout << "get point " << x << ", " << y << std::endl;
                new_pair = true;
                plan_finish = false;
            }
        }
    };

    canvas.setMouseCallBack(callback);

    auto ENL_SVG_grid = getENL_SVG_Module(dimension, is_occupied_func);
    Pathfinding::Path path;

    /* 2, pre computation */
    gettimeofday(&tv_pre, &tz);
    // Preprocess the grid to build a visibility graph.
    Pathfinding::ENLSVG::Algorithm algo(ENL_SVG_grid);
    algo.printStatistics();

    // Initialise a memory object for the algorithm. The algorithm uses this memory object for its path computations.
    Pathfinding::ENLSVG::Memory memory(ENL_SVG_grid);
    gettimeofday(&tv_after, &tz);
    double cost_ms1 =(tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- ENL-SVG Preprocess end in " << cost_ms1 << " ms" << std::endl;

    std::pair<PathPointWithNodePtrs<2>, PathPointWithNodePtrs<2>> init_paths;

    bool draw_all_path = true;
    while(1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied_func);
        if(show_graph) {
            //canvas.drawTangentGraphAllNodes(tg, COLOR_TABLE[0], COLOR_TABLE[1]);
            //canvas.drawEdges(tg.original_edge_iters_);
            //canvas.drawTangentGraphLegalEdges(tg, COLOR_TABLE[0], COLOR_TABLE[1]);
            canvas.drawENLVisibilityGraph(algo.graph, memory);
        }
        if(new_pair) {
            new_pair = false;
            if(tp.pool_[0].joinable() && !plan_finish)
                tp.Schedule([&]{
                    int count = 1;
                    while(count > 0) {
                        count --;
                        //g2p2.tg_.initialise();

                        std::cout << "-- start RimJump create initial path " << std::endl;
                        plan_finish = false;
                        global_minimum_path_length = std::numeric_limits<PathLen>::max();
                        gettimeofday(&tv_pre, &tz);

                        // 1, create target set
                        init_target_edges.clear();
                        g2p2.initialise();
                        g2p2.edge_mode_ = false;
                        g2p2.depth_first_ = true;
                        g2p2.global_shortest_ = true;
                        //init_target_edges = g2p2.createVisiblePathPointPtrsWithOutLength(sg2->pt_, false, init_ptcs, init_etcs);
                        //std::cout << "target edges: size = " << init_target_edges.size() << std::endl;
//                    for(auto target_edge : init_target_edges) {
//                        target_edge->print(is_edge, false);
//                    }

                        // 2, create start set
                        init_start_edges.clear();
                        //initial_start_nodes.clear();
                        init_paths = g2p2.createVisiblePathPointPtrsPairWithNode(sg1->pt_, sg2->pt_, init_ptcs, init_etcs);
                        //auto retv = g2p2.createVisiblePathPointPtrsPairWithOutLength(sg1->pt_, sg2->pt_, ptcs, etcs);
                        //std::cout << "start edges: size = " << init_start_edges.size() << std::endl;
//                    for(auto start_edge : init_start_edges) {
//                        start_edge->print(is_edge, true);
//                    }

                        gettimeofday(&tv_after, &tz);
                        double cost_ms1 =
                                (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                        std::cout << "-- RimJump finish create path in " << cost_ms1 << " ms" << std::endl;

                        path = algo.computePath(memory, pt1[0], pt1[1], pt2[0], pt2[1], initial_time, search_graph_time);
                        std::cout << "-- ENLSVG create init path in " << initial_time << " ms" << std::endl;

                    }

                    std::cout << "from start  size " << init_paths.first.size() << std::endl;
                    std::cout << "from target size " << init_paths.second.size() << std::endl;

                    plan_finish = true;
                });
        }
        canvas.drawCircleInt(pt1[0], pt1[1], 5, center_offset, -1, COLOR_TABLE[0]);
        canvas.drawCircleInt(pt2[0], pt2[1], 5, center_offset, -1, COLOR_TABLE[1]);
        if(plan_finish && !show_graph) {
            // draw all marked edge
            if(show_start_now) {
                for(const auto & path : init_paths.first) {
                    canvas.drawLineInt(pt2[0], pt2[1], path->last_node_->sg_->pt_[0], path->last_node_->sg_->pt_[1], center_offset, 2, cv::Vec3b(0,255,0));
                }
            }
            if(!show_start_now) {
                for(const auto & path : init_paths.second) {
                    canvas.drawLineInt(pt1[0], pt1[1], path->last_node_->sg_->pt_[0], path->last_node_->sg_->pt_[1], center_offset, 2, cv::Vec3b(0,255,0));
                }
            }

        }
        //canvas.drawPathi(path);
        char key_value = canvas.show();
        if(key_value == 't') {
            show_start_now = !show_start_now;
        } else if(key_value == 'a') {
            draw_all_path = !draw_all_path;
        } else if(key_value == 'g') {
            show_graph = !show_graph;
        }
    }
}
