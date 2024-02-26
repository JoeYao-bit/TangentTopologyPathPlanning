//
// Created by yaozhuo on 2022/7/8.
//
#include "gtest/gtest.h"

#include "../freeNav-base/dependencies/massive_scene_loader/ScenarioLoader2D.h"
#include "../freeNav-base/dependencies/path_planning_interface.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/2d_grid/picture_loader.h"
#include "../freeNav-base/visualization/canvas/canvas.h"
#include "../freeNav-base/dependencies/massive_test_interfaces.h"
#include "../freeNav-base/dependencies/directory_loader.h"

#include "../algorithm/constraints/edge_transfer_constraints.h"
#include "../algorithm/constraints/iteration_constraints.h"
#include "../algorithm/surface_processor_LineScanner.h"
#include "../algorithm/2d_ENLSVG_grid.h"
#include "../algorithm/surface_processor_ENLSVG_LineScanner.h"

#include "../algorithm/online_search/search_path_with_edge.h"
#include "../algorithm/online_search/breadth_first_search_with_edge.h"
#include "../algorithm/online_search/depth_first_search_with_edge.h"
#include "../algorithm/online_search/create_initial_paths_with_edge.h"
#include "../algorithm/online_search/search_path_with_node.h"
#include "../algorithm/online_search/breadth_first_search_with_node.h"
#include "../algorithm/online_search/depth_first_search_with_node.h"
#include "../algorithm/online_search/create_initial_paths_with_node.h"

#include "../third_party/DOSL/encapsulations/cvMulticlassPathPlanner.tcc"
#include "../third_party/dynamicvoronoi/rhcf.h"

#include "../test/test_data.h"

using namespace freeNav::Topo;
using namespace freeNav;


struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

// determine whether a pixel in picture is occupied
auto is_grid_occupied = [](const cv::Vec3b& color) -> bool {
    if (color[0] == 0 && color[1] == 0 && color[2] == 0) return true;
    return false;
};

auto is_char_occupied = [](const char& value) -> bool {
    if (value != '.' && value != 'G' && value != 'S') return true;
    return false;
};

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value != '.') return false;
    return true;
};

// node and edge constraints
PointTransferConstraints<2> ptcs({PTC_LocalCrossPartially,
                                  PTC_DoNotCrossObstacle
                                 }
);

PointTransferConstraints<2> ptcs_ordered;

// edge transfer constraints
EdgeTransferConstraints3<2> etcs({//ETC_NotLookBack3
                                         ETC_NoUselessPoint3
                                         ,ETC_IC_GetCloserToObstacle3
                                         //,ETC_IC_GetCloserToObstacle_ENLSVG
                                         //,ETC_Taut_2D
                                 });

IterationConstraints<2, GridPtr<2>> ics({
                                                                  //IC_EdgeNLevelLimitUndirectedGraph
                                                          });

IterationConstraints<2, GridPtr<2>> distinctive_ics({IC_NoLoop, IC_EdgeNLevelLimit});

PointTransferConstraints<2> init_ptcs({PTC_CrosserToObstacle_2D});
EdgeTransferConstraints3<2> init_etcs({ETC_Taut_2D});

IterationConstraints<2, GridPtr<2>> distinctive_without_ics({
                                                              IC_EdgeNLevelLimitUndirectedGraph,
                                                              IC_NoLoop
                                                            });


#define WITH_EDGE_M2 1



//MapTestConfig_Berlin_1_256;
//MapTestConfig_FloodedPlains;
//MapTestConfig_Boston_0_1024;
//MapTestConfig_TheFrozenSea;
//MapTestConfig_maze512_4_8;
//MapTestConfig_maze512_4_0;
//MapTestConfig_Boston_0_1024,
//MapTestConfig_Aurora
//MapTestConfig_8room_002
//MapTestConfig_random512_10_0  //run out of memory, ENLSVG take 1G memory, but rj failed, because of too many edge ?
SingleMapTestConfigs<2> configs = {
        MapTestConfig_Berlin_1_256,  // pass
        MapTestConfig_Boston_0_1024,
        MapTestConfig_Denver_2_512,
        MapTestConfig_London_0_256,

};

TEST(RIMJUMP, DATA_PROCESS) {

    for(const auto& config : configs) {
        std::cout << config.at("map_name") << ":" << std::endl;
        SingleMapTestDataAnalysis<2>(config);
        std::cout << std::endl;
    }

}

bool SingleMapStatistic2D(const SingleMapTestConfig <2> &map_test_config) {

    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto dimension = tl.getDimensionInfo();

    IS_OCCUPIED_FUNC<2> is_occupied_func;

    SET_OCCUPIED_FUNC<2> set_occupied_func;

    auto is_occupied = [&tl](const Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    is_occupied_func = is_occupied;

    auto set_occupied = [&tl](const Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    /* initialize rim jump start */
    gettimeofday(&tv_pre, &tz);
    auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    RoadMapGraphBuilder<2> *tgb = new RoadMapGraphBuilder<2>(surface_processor,
                                                             ptcs, ptcs_ordered, etcs,
                                                             map_test_config.at("vis_path")
                                                             , true
                                                             , true
                                                             , 1
    );

    //auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);
//    RoadMapGraphBuilder<2> *tgb = new RoadMapGraphBuilder<2>(surface_processor,
//                                                             init_ptcs, ptcs_ordered, init_etcs,
//                                                             map_test_config.at("vis_path"), true
//    );

    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    std::cout << "the tangent graph has " << tg->edges_.size() << " edges " << std::endl;

    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl;

    tg->getStatistics();
    delete tgb;
    return true;
}

TEST(RIMJUMP, MASSIVE_STATISTIC_2D) {
//    configs = {//MapTestConfig_TheFrozenSea,
//               //MapTestConfig_FloodedPlains, // pass
//               //MapTestConfig_Entanglement,// pass, all distinctive test failed, after 2400, run out of storage space
//               MapTestConfig_Aurora,
//               MapTestConfig_8room_002
//    }; // single test
    for(const auto& config : configs) {
        SingleMapStatistic2D(config);
    }
}

NodeIterationConstraints<2, GridPtr<2>> ics_node_topology({
                                                                                    NIC_NoLoop, // if is global shortest, no need to add IC_NoLoop
                                                                                    //IC_EdgeNLevelLimit
                                                                                    NIC_NoStraightExtend
                                                                            });

Point2PointPathPlannings<2, Pointi<2>, Pointi<2> > distinctive_path_plannings;

double max_time_cost = 10e3;

#define RJ_TOPO(num_of_path)  [&](const Pointi<2> &start,        \
                                  const Pointi<2> &target,       \
                                  Pointis<2> &path,              \
                                  Statistic &statistic,          \
                                  OutputStream &output_stream) { \
                                  std::vector<Path<2> > paths;   \
                                  for(int i=0; i<repeat_times; i++) { \
                                      ec = g2p2.planningWithNode(start, target, init_ptcs, init_etcs, ics_node_topology, paths, false, false, false, num_of_path); \
                                  }                              \
                                  if (!paths.empty()) path = paths.front(); \
                                  else path.clear();             \
                                  statistic = g2p2.getStatistic(); \
                                  output_stream = g2p2.getOutputStream(); \
                              }                                 \

#define RHCF_TOPO(num_of_path)  [&](const Pointi<2> &start, \
                                    const Pointi<2> &target, \
                                    Pointis<2> &path, \
                                    Statistic &statistic, \
                                    OutputStream &output_stream) { \
                                    xrobot[0] = start[0], xrobot[1] = start[1]; \
                                    xgoal[0]  = target[0], xgoal[1]  = target[1]; \
                                    rhcf.scene_->setRobot(start[0], start[1], 0); \
                                    rhcf.scene_->setGoal(target[0], target[1]); \
                                    rhcf.buildVoronoiDiagram(); \
                                    gettimeofday(&tv_pre, &tz); \
                                    rhcf.buildNavigationGraph(); \
                                    gettimeofday(&tv_after, &tz); \
                                    double cost_ms1 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3; \
                                    gettimeofday(&tv_pre, &tz); \
                                    rhcf.K_ = num_of_path; \
                                    rhcf.findHomotopyClasses(); \
                                    gettimeofday(&tv_after, &tz); \
                                    double cost_ms2 = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3; \
                                    statistic.clear(); \
                                    if(rhcf.all_result_path_.empty()) { \
                                    statistic.push_back(MAX<PathLen>); \
                                    path.clear(); \
                                    } else { \
                                        PathLen total_path_length;                     \
                                        for(const auto& path : rhcf.all_result_path_) {              \
                                            total_path_length += calculatePathLength(path);                 \
                                        }                     \
                                        statistic.push_back(total_path_length/rhcf.all_result_path_.size()); \
                                        path = rhcf.all_result_path_.front();                         \
                                    } \
                                    statistic.push_back(0); \
                                    statistic.push_back(cost_ms1); \
                                    statistic.push_back(cost_ms2); \
                                    statistic.push_back(rhcf.all_result_path_.size()); \
                                    std::stringstream ss; \
                                    ss << "RHCF" << "_" << num_of_path << " " << start.toStr() << " " << target.toStr() << " "; \
                                    for(const auto & data : statistic) { \
                                    ss << data << " "; \
                                    } \
                                    output_stream = ss.str(); \
                                    }                      \

#define HAStar_TOPO(num_of_path) [&](const Pointi<2> &start, \
                                    const Pointi<2> &target, \
                                    Pointis<2> &path, \
                                    Statistic &statistic, \
                                    OutputStream &output_stream) { \
                                    gettimeofday(&tv_pre, &tz); \
                                    cv::Point start_cv(start[0], start[1]), target_cv(target[0], target[1]); \
                                    cvMulticlassPathPlanner<AStar> path_planner_a (background, is_occupied_func); \
                                    path_planner_a.max_time_cost = max_time_cost;                         \
                                    path_planner_a.find_paths (start_cv, target_cv, num_of_path, false); \
                                    const auto& raw_hstar_paths = path_planner_a.paths; \
                                    gettimeofday(&tv_after, &tz); \
                                    double cost_hstar = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3; \
                                    Paths<2> hstar_paths; \
                                    for(const auto& hstar_path : raw_hstar_paths) { \
                                    Path<2> path; \
                                    for(const auto& pt : hstar_path) { \
                                        Pointi<2> rj_pt({pt.x, pt.y}); \
                                    path.push_back(rj_pt); \
                                    } \
                                    hstar_paths.push_back(path); \
                                    } \
                                    statistic.clear(); \
                                    if(raw_hstar_paths.empty()) { \
                                        statistic.push_back(MAX<PathLen>); \
                                        path.clear(); \
                                    } else { \
                                        PathLen total_path_length;                     \
                                        for(const auto& path : hstar_paths) {              \
                                            total_path_length += calculatePathLength(path);                 \
                                        }                     \
                                        statistic.push_back(total_path_length/hstar_paths.size()); \
                                        path = hstar_paths.front(); \
                                    } \
                                    statistic.push_back(0); \
                                    statistic.push_back(0); \
                                    statistic.push_back(cost_hstar); \
                                    statistic.push_back(raw_hstar_paths.size()); \
                                    std::stringstream ss; \
                                    ss << "HsAs" << "_" << num_of_path << " " << start.toStr() << " " << target.toStr() << " "; \
                                    for(const auto & data : statistic) { \
                                        ss << data << " "; \
                                    } \
                                    output_stream = ss.str(); \
                                    }                       \

#define HTStar_TOPO(num_of_path) [&](const Pointi<2> &start, \
                                    const Pointi<2> &target, \
                                    Pointis<2> &path, \
                                    Statistic &statistic, \
                                    OutputStream &output_stream) { \
                                    gettimeofday(&tv_pre, &tz); \
                                    cv::Point start_cv(start[0], start[1]), target_cv(target[0], target[1]); \
                                    cvMulticlassPathPlanner<ThetaStar> path_planner_theta (background, is_occupied_func); \
                                    path_planner_theta.max_time_cost = max_time_cost;                         \
                                    path_planner_theta.find_paths (start_cv, target_cv, num_of_path, false); \
                                    const auto& raw_hstar_paths = path_planner_theta.paths; \
                                    gettimeofday(&tv_after, &tz); \
                                    double cost_hstar = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3; \
                                    Paths<2> hstar_paths; \
                                    for(const auto& hstar_path : raw_hstar_paths) { \
                                        Path<2> path; \
                                        for(const auto& pt : hstar_path) { \
                                            Pointi<2> rj_pt({pt.x, pt.y}); \
                                            path.push_back(rj_pt); \
                                        } \
                                        hstar_paths.push_back(path); \
                                    } \
                                    statistic.clear(); \
                                    if(raw_hstar_paths.empty()) { \
                                        statistic.push_back(MAX<PathLen>); \
                                        path.clear(); \
                                    } else {                 \
                                        PathLen total_path_length;                     \
                                        for(const auto& path : hstar_paths) {              \
                                            total_path_length += calculatePathLength(path);                 \
                                        }                     \
                                        statistic.push_back(total_path_length/hstar_paths.size()); \
                                        path = hstar_paths.front(); \
                                    } \
                                    statistic.push_back(0); \
                                    statistic.push_back(0); \
                                    statistic.push_back(cost_hstar); \
                                    statistic.push_back(hstar_paths.size()); \
                                    std::stringstream ss; \
                                    ss << "HsTs"  << "_" << num_of_path << " " << start.toStr() << " " << target.toStr() << " "; \
                                    for(const auto & data : statistic) { \
                                        ss << data << " "; \
                                    } \
                                    output_stream = ss.str(); \
                                    } \


bool SingleMapTestDistinctiveTopology2D(const SingleMapTestConfig <2> &map_test_config,
                                        double inflation_ratio,
                                        const StartAndTargets<2>& start_and_targets = {}) {

    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto temp_dimension = tl.getDimensionInfo();
    DimensionLength* dimension = new DimensionLength[2];
    dimension[0] = floor(temp_dimension[0]*inflation_ratio);
    dimension[1] = floor(temp_dimension[1]*inflation_ratio);

    IS_OCCUPIED_FUNC<2> is_occupied_func;

    SET_OCCUPIED_FUNC<2> set_occupied_func;

    auto is_occupied = [&tl, &inflation_ratio](const Pointi<2> &pt) -> bool {
        //if(pt[0] <= 0 || pt[0] >= tl.getDimensionInfo()[0]-1 || pt[1] <= 0 || pt[1] >= tl.getDimensionInfo()[1]-1 ) { return true; }
        Pointi<2> temp_pt;
        temp_pt[0] = floor(pt[0]/inflation_ratio);
        temp_pt[1] = floor(pt[1]/inflation_ratio);
        return tl.isOccupied(temp_pt);
    };
    is_occupied_func = is_occupied;

    auto set_occupied = [&tl](const Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    /* initialize rim jump start */
    gettimeofday(&tv_pre, &tz);
    //SurfaceProcess_ENLSVG_LineScanner scanner(dimension, is_occupied_func, set_occupied_func);
    //auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);


    RoadMapGraphBuilder<2> *tgb = new RoadMapGraphBuilder<2>(surface_processor,
                                                             init_ptcs, ptcs_ordered, init_etcs,
                                                             map_test_config.at("vis_path")
                                                             , true, false, 1
    );

    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    std::cout << "the tangent graph has " << tg->edges_.size() << " edges " << std::endl;

    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl << std::endl;

    GeneralGraphPathPlannerWithNode<2> g2p2(tg);
    /* initialize rim jump end */

    /* initialize RHCF */
#define RHCF_MACRO
#ifdef RHCF_MACRO
    gettimeofday(&tv_pre, &tz);
    freeNav::RimJump::RHCF rhcf(dimension, is_occupied_func);

    double xrobot[3];
    double xgoal[2];
    xrobot[0] = 0, xrobot[1] = 0, xrobot[2] = 0;
    xgoal[0]  = 0, xgoal[1]  = 0;

    rhcf.readScenario(xrobot, xgoal,
                      0, 1, 1, dimension[0], 0, dimension[1], 0,
                      max_time_cost); // wait at most 10 seconds
    gettimeofday(&tv_after, &tz);
    build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- load RHCF end in " << build_cost << "ms" << std::endl;
#endif
    /* initialize RHCF end */

    /* initialize H-star Astar */
    gettimeofday(&tv_pre, &tz);
    Canvas canvas("RimJump::DistinctiveTopologyPathPlanning", dimension[0], dimension[1], .05, 1);
    canvas.drawGridMap(dimension, is_occupied_func);
    cv::Mat background = canvas.getCanvas();
    cvMulticlassPathPlanner<AStar> path_planner_a (background, is_occupied_func);
    gettimeofday(&tv_after, &tz);

    build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- load H-star end in " << build_cost << "ms" << std::endl;
    /* initialize H-star Astar end */

    /* initialize H-starhetaStar */
    gettimeofday(&tv_pre, &tz);
    canvas.drawGridMap(dimension, is_occupied_func);
    gettimeofday(&tv_after, &tz);

    build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << "-- load H-star end in " << build_cost << "ms" << std::endl;
    /* initialize H-star ThetaStar end */

    /* construct the planning interfaces start */

    bool is_edge = true, is_dynamic = false, is_depth_first = false, global_shortest = true;
    int minimum_reach_count = -1; // exit only when all reach target
    ExitCode ec;

    int repeat_times = 1; // NOTICE: the more times repeat, the less time cost

    StatisticSS statisticss;
    OutputStreamSS output_streamss;
    // rim_jump_path_planning
    // ENL_SVG_path_planning
    distinctive_path_plannings = {
                                 RJ_TOPO(10),
//                                 RJ_TOPO(20),
//                                 RJ_TOPO(30),
//                                 RJ_TOPO(40),
//                                 RJ_TOPO(60),
//                                 RJ_TOPO(80),
//                                 RJ_TOPO(120),
//                                 RJ_TOPO(160),
//                                 RJ_TOPO(200),
#ifdef RHCF_MACRO
                                 RHCF_TOPO(10),
//                                 RHCF_TOPO(20),
//                                 RHCF_TOPO(30),
//                                 RHCF_TOPO(40),
//                                 RHCF_TOPO(60),
//                                 RHCF_TOPO(80),
//                                 RHCF_TOPO(120),
//                                 RHCF_TOPO(160),
//                                 RHCF_TOPO(200),
#endif

                                 HAStar_TOPO(10),
//                                 HAStar_TOPO(20),
//                                 HAStar_TOPO(30),
//                                 HAStar_TOPO(40),
//                                 HAStar_TOPO(60),
//                                 HAStar_TOPO(80),
//                                 HAStar_TOPO(120),
//                                 HAStar_TOPO(160),
//                                 HAStar_TOPO(200),
//
                                 HTStar_TOPO(10),
//                                 HTStar_TOPO(20),
//                                 HTStar_TOPO(30),
//                                 HTStar_TOPO(40),
//                                 HTStar_TOPO(60),
//                                 HTStar_TOPO(80),
//                                 HTStar_TOPO(120),
//                                 HTStar_TOPO(160),
//                                 HTStar_TOPO(200),

                                };
//    delete tgb;
//    delete[] cost_move_base_2d;

    if(start_and_targets.empty()) {
// use start and target from config file, but may not provide enough distinctive topology path
//        SceneTest2D(map_test_config.at("config_path"),
//                    distinctive_path_plannings, statisticss, output_streamss, 10);

    // random sample un visible start and target
    auto temp_random_pairs = generateRandomPointPair<2>(is_occupied, dimension, 10);
    SceneTest2DWithStartAndTargets(temp_random_pairs, distinctive_path_plannings, statisticss, output_streamss);

    } else {
        SceneTest2DWithStartAndTargets(expandSAT(start_and_targets, inflation_ratio), distinctive_path_plannings, statisticss, output_streamss,
                                       1, 10);
    }

    delete tgb;
    delete dimension;
    //SceneTest2DIndividual(map_test_config.at("config_path"), path_plannings, statisticss, output_streamss);
    std::ofstream os(map_test_config.at("output_path"));
    //os << "TYPE START_X START_Y TARGET_X TARGET_Y PATH_LENGTH RESET_TIME INITIAL_TIME SEARCH_TIME" << std::endl;
    for (const auto &multi_method_output : output_streamss) {
        for (const auto method_output : multi_method_output) {
            os << method_output << std::endl;
        }
    }
    os.close();
    return true;
}



template<Dimension N>
StartAndTargets<N> generateRandomStartAndTarget(const SingleMapTestConfig<2> map_test_config,
                                                double inflation_ratio,
                                                int num_of_pairs,
                                                int up_bound_of_sample = 1000,
                                                int up_bound_of_paths = 400) {
    TextMapLoader tl(map_test_config.at("map_path"), is_char_occupied);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto temp_dimension = tl.getDimensionInfo();
    DimensionLength dimension[2];
    dimension[0] = floor(temp_dimension[0]*inflation_ratio);
    dimension[1] = floor(temp_dimension[1]*inflation_ratio);

    IS_OCCUPIED_FUNC<2> is_occupied_func;

    SET_OCCUPIED_FUNC<2> set_occupied_func;

    auto is_occupied = [&tl, &inflation_ratio](const Pointi<2> &pt) -> bool {
        //if(pt[0] <= 0 || pt[0] >= tl.getDimensionInfo()[0]-1 || pt[1] <= 0 || pt[1] >= tl.getDimensionInfo()[1]-1 ) { return true; }
        Pointi<2> temp_pt;
        temp_pt[0] = floor(pt[0]/inflation_ratio);
        temp_pt[1] = floor(pt[1]/inflation_ratio);
        return tl.isOccupied(temp_pt);
    };
    is_occupied_func = is_occupied;

    auto set_occupied = [&tl](const Pointi<2> &pt) { tl.setOccupied(pt); };
    set_occupied_func = set_occupied;

    /* initialize rim jump start */
    gettimeofday(&tv_pre, &tz);
    //auto surface_processor = std::make_shared<SurfaceProcessor<2> >(dimension, is_occupied_func, set_occupied_func);
    auto surface_processor = std::make_shared<SurfaceProcess_ENLSVG_LineScanner>(dimension, is_occupied_func, set_occupied_func);


    RoadMapGraphBuilder<2> *tgb = new RoadMapGraphBuilder<2>(surface_processor,
                                                             init_ptcs, ptcs_ordered, init_etcs,
                                                             map_test_config.at("vis_path")
            , true, false, 1
    );

    gettimeofday(&tv_after, &tz);
    RoadMapGraphPtr<2> tg = tgb->getRoadMapGraph();
    std::cout << "the tangent graph has " << tg->nodes_.size() << " nodes " << std::endl;
    std::cout << "the tangent graph has " << tg->edges_.size() << " edges " << std::endl;

    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "-- build graph end in " << build_cost << "ms" << std::endl << std::endl;

    GeneralGraphPathPlannerWithNode<2> g2p2(tg);


    Pointis<N> pts;
    Pointi<N> pt;
    Id total_index = getTotalIndexOfSpace<N>(dimension), temp_id;
    StartAndTargets<N> retv;
    Pointis<N-1> neighbor = GetNeightborOffsetGrids<N-1>();
    for(int i=0; i<num_of_pairs; i++) {
        pts.clear();
        for (int j = 0; j < up_bound_of_sample; j++) {
            temp_id = (Id) (total_index * rand() / double(RAND_MAX));
            //std::cout << "temp_id " << temp_id << std::endl;
            pt = IdToPointi<N>(temp_id, dimension);
            if (is_occupied(pt)) { continue; }
            else {
                if(pts.size() == 1) {
                    if(!LineCrossObstacle<N>(pts[0], pt, is_occupied, neighbor)) {
                        continue;
                    }
                    Paths<N> paths;
                    std::cout << " from " << pts[0] << " to " << pt << std::endl;
                    ExitCode ec = g2p2.planningWithNode(pts[0], pt, init_ptcs, init_etcs, ics_node_topology,
                                                        paths, false, false, false, up_bound_of_paths);
                    if(ec != SUCCESS || paths.size() < up_bound_of_paths) {
                        continue;
                    }
                }
                pts.push_back(pt);
                if (pts.size() == 2) {
                    if (pts[0] == pts[1]) {
                        pts.pop_back();
                        continue;
                    }
                    break;
                }
            }
        }
        if (pts.size() != 2) {
            std::cout << " index " << i << " find no free start/target pair" << std::endl;
            continue;
        }
        retv.push_back({pts[0], pts[1]});
    }
    if(retv.size() != num_of_pairs) {
        std::cout << " find no sufficient pair: " << retv.size() << " < " << num_of_pairs << std::endl;
    } else {
        std::cout << " find sufficient pair " << num_of_pairs << std::endl;
    }
    return retv;
}

template<Dimension N>
StartAndTargets<N> loadStartAndTargetPairsFromFile(const std::string& file_path) {
    std::ifstream fin(file_path);
    std::string line;
    StartAndTargets<N> retv;
    Pointi<N> start, target;
    while (getline(fin, line)) {
        stringstream ss; ss << line;
        std::string buff_int;
        ss >> buff_int; start[0] = atoi(buff_int.c_str());
        ss >> buff_int; start[1] = atoi(buff_int.c_str());
        ss >> buff_int; target[0] = atoi(buff_int.c_str());
        ss >> buff_int; target[1] = atoi(buff_int.c_str());
        retv.push_back({start, target});
    }
    return retv;
}

template<Dimension N>
void saveStartAndTargetPairsFromFile(const StartAndTargets<N> sats, const std::string& file_path) {
    ofstream os(file_path, std::ios_base::out|std::ios_base::binary|std::ios_base::trunc);
    for(const auto& sat : sats) {
        os << sat.first[0] << " " << sat.first[1] << " " << sat.second[0] << " " << sat.second[1] << std::endl;
    }
    os.close();
}

int main() {
    configs = {
            // -- real world city map
            MapTestConfig_Berlin_1_256, // ok
            MapTestConfig_Denver_2_256, // ok
            MapTestConfig_Boston_2_256, // ok
            MapTestConfig_Milan_2_256, // ok
            MapTestConfig_Moscow_2_256, // ok
            MapTestConfig_London_2_256, // ok
            MapTestConfig_Sydney_1_256, // ok
            MapTestConfig_Paris_0_256, // ok
            // -- computer game map
//              MapTestConfig_AR0013SR, // RJ not ok
//              MapTestConfig_AR0014SR, // RJ ok, RHCF not ok
//              MapTestConfig_AR0018SR, // RJ ok,
//              MapTestConfig_AR0205SR,
//            MapTestConfig_TheFrozenSea //
//            MapTestConfig_EbonLakes,
//            MapTestConfig_Enigma,
//              MapTestConfig_Entanglement
            // --
    }; // single test

    std::vector<StartAndTargets<2> > SATs;
//    = {
//            Berlin_1_256_SAT,
//            Denver_2_256_SAT,
//            Boston_2_256_SAT,
//            Milan_2_256_SAT,
//            Moscow_2_256_SAT,
//            London_2_256_SAT,
//            Sydney_1_256_SAT,
//            Paris_0_256_SAT
//    };

    for(const auto& config : configs) {
        auto start_and_targets = loadStartAndTargetPairsFromFile<2>(config.at("sat_path"));
        if(start_and_targets.empty()) {
            start_and_targets = generateRandomStartAndTarget<2>(config, 1.0, 10);
            saveStartAndTargetPairsFromFile(start_and_targets, config.at("sat_path"));
        }
        SATs.push_back(start_and_targets);
    }

    for(int i=0; i<configs.size(); i++) {
        SingleMapTestDistinctiveTopology2D(configs[i], 1.0, SATs[i]);
    }
    for(const auto& config : configs) {
        std::cout << config.at("map_name") << ":   \t" << std::endl;
        SingleMapTestDataAnalysis<2>(config, true);
        std::cout << std::endl;
    }
}