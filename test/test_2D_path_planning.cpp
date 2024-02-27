//
// Created by yaozhuo on 2022/1/11.
//

#include "gtest/gtest.h"
#include "sys/time.h"
#include <limits>

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/thread_pool.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/2d_grid/picture_loader.h"

#include "../test/visualization/canvas.h"

#include "../algorithm/graph_construction/tangent_graph_build.h"
#include "../algorithm/online_search/create_initial_paths_with_edge.h"
#include "../algorithm/constraints/point_to_point_constraints.h"
#include "../algorithm/constraints/edge_transfer_constraints.h"

#include "../algorithm/2d_ENLSVG_grid.h"

#include "../algorithm/online_search/breadth_first_search_with_edge.h"
#include "../algorithm/online_search/depth_first_search_with_edge.h"
#include "../algorithm/online_search/search_path_with_node.h"
#include "../algorithm/online_search/breadth_first_search_with_node.h"
#include "../algorithm/online_search/depth_first_search_with_node.h"
#include "../algorithm/online_search/create_initial_paths_with_node.h"
#include "../algorithm/surface_processor_LineScanner.h"

#include "../test/test_data.h"

//#include "topologyPathPlanning/harrt_test.h"


// https://github.com/PathPlanning/3D-AStar-ThetaStar

using namespace freeNav::Topo;
using namespace freeNav;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

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


//MapTestConfig_Berlin_1_256, // ok
//MapTestConfig_Denver_2_256, // ok
//MapTestConfig_Boston_2_256, // ok
//MapTestConfig_Milan_2_256, //
//MapTestConfig_Moscow_2_256, // 100 not pass
//MapTestConfig_London_2_256, // ok
//MapTestConfig_Sydney_1_256, // ok
//MapTestConfig_Paris_0_256 // ok


auto map_test_config = MapTestConfig_Berlin_1_256;
std::string vis_file_path    = map_test_config.at("vis_path");

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
int zoom_rate = 4;


auto dimension = loader.getDimensionInfo();

std::shared_ptr<RoadMapGraphBuilder<2> > tgb = nullptr;

RoadMapNodes<2> initial_start_nodes;
RoadMapNodes<2> initial_target_nodes;

bool new_pair = false;
bool plan_finish = false;
bool draw_all_path = true;

bool show_rj     = true;

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


RoadMapEdgeTraitPtrss<2> rmetps;
std::vector<Path<2>> paths;


std::vector<Pointi<2> > all_points;

int main() {

    struct timezone tz;
    struct timeval  tv_pre;
    struct timeval  tv_after;


    gettimeofday(&tv_pre, &tz);

    auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);
    surface_processor->surfaceGridsDetection(true);



    tgb = std::make_shared<RoadMapGraphBuilder<2> >(surface_processor, init_ptcs, ptcs_ordered, init_etcs, vis_file_path, true, false, 1);
    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "-- the tangent graph has " << tg->edges_.size() << " edges " << std::endl;
    std::cout << "-- the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl;
    std::cout << "-- is the graph undirected ? " << tg->UndirectedGraphCheck() << std::endl << std::endl;
    GeneralGraphPathPlannerWithNode<2> g2p2(tg);
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

    CanvasTopo canvas("RimJump::PathPlanning", dimension[0], dimension[1], .05, zoom_rate);
    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            if(set_pt1) {
                pt1[0] = x;
                pt1[1] = y;
                sg1->pt_ = pt1;
                sg1->id_ = PointiToId<2>(pt1, dimension);
                set_pt1 = false;
                plan_finish = false;
                std::cout << "get point " << x << ", " << y << std::endl;
                all_points.push_back(pt1);
            } else {
                pt2[0] = x;
                pt2[1] = y;
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
        }

        if(new_pair) {
            new_pair = false;
            if(tp.pool_[0].joinable() && !plan_finish)
                tp.Schedule([&] {
                    current_show_index = 0;
                    for(int i=0; i<1; i++) {
                        plan_finish = false;

                        ec = g2p2.planningWithNode(pt1, pt2, init_ptcs, init_etcs, ics_node_topology, paths,
                                                   false, false, false,
                                                   100);
                        std::cout << " path " << paths.size() << std::endl;
                        auto statistic_without = g2p2.getStatistic();
                        if (ec != ExitCode::SUCCESS) {
                            std::cout << "-- RimJump WithOut failed with " << ec << " in " << statistic_without[2] << " + " << statistic_without[3]
                                      << "ms" << std::endl;
                        } else {
                            std::cout << "-- RimJump WithOut success in " << statistic_without[2] << " + " << statistic_without[3] << "ms"
                                      << std::endl;
                        }
                    }
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
        } else if(key_value == 'b') {
            draw_block_ptr = !draw_block_ptr;
        } else if(key_value == 32) {
            for(int i=0; i<all_points.size()/2; i++) {
                std::cout << "{{" << all_points[2*i][0] << "," << all_points[2*i][1] << "}, {" << all_points[2*i+1][0] << "," << all_points[2*i+1][1] << "}}," << std::endl;
            }
        }
    }
    return 0;
}
