//
// Created by yaozhuo on 2022/9/6.
//

#ifndef FREENAV_TEST_DATA_H
#define FREENAV_TEST_DATA_H
#include <iostream>
#include <map>
#include "../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/dependencies/massive_test_interfaces.h"


namespace freeNav {

// full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_Boston_0_1024 =

            {
                    {"map_name",    "Boston-0-1024"},
                    {"map_path",    "../test/test_data/Boston_0_1024.map"},
                    {"vis_path",    "../test/test_data/Boston_0_1024_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Boston_0_1024.map.scen"},
                    {"output_path", "../test/test_data/Boston_0_1024.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Boston_2_256 =

            {
                    {"map_name",    "Boston_2_256"},
                    {"map_path",    "../test/test_data/Boston_2_256.map"},
                    {"vis_path",    "../test/test_data/Boston_2_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Boston_2_256.map.scen"},
                    {"output_path", "../test/test_data/Boston_2_256.txt"},
                    {"sat_path", "../test/test_data/Boston_2_256.sat"}

            };

    // Milan_1_512.map
    SingleMapTestConfig<2> MapTestConfig_Milan_1_512 =
            {
                    {"map_name",    "Milan_1_512"},
                    {"map_path",    "../test/test_data/Milan_1_512.map"},
                    {"vis_path",    "../test/test_data/Milan_1_512_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Milan_1_512.map.scen"},
                    {"output_path", "../test/test_data/Milan_1_512.txt"}
            };

    // Milan_2_256.map
    SingleMapTestConfig<2> MapTestConfig_Milan_2_256 =
            {
                    {"map_name",    "Milan_1_512"},
                    {"map_path",    "../test/test_data/Milan_2_256.map"},
                    {"vis_path",    "../test/test_data/Milan_2_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Milan_2_256.map.scen"},
                    {"output_path", "../test/test_data/Milan_2_256.txt"},
                    {"sat_path", "../test/test_data/Milan_2_256.sat"}
            };
    // Moscow_2_256.map
    SingleMapTestConfig<2> MapTestConfig_Moscow_2_256 =
            {
                    {"map_name",    "Moscow_2_256"},
                    {"map_path",    "../test/test_data/Moscow_2_256.map"},
                    {"vis_path",    "../test/test_data/Moscow_2_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Moscow_2_256.map.scen"},
                    {"output_path", "../test/test_data/Moscow_2_256.txt"},
                    {"sat_path", "../test/test_data/Moscow_2_256.sat"}
            };

    SingleMapTestConfig<2> MapTestConfig_London_2_256 =
            {
                    {"map_name",    "London_2_256"},
                    {"map_path",    "../test/test_data/London_2_256.map"},
                    {"vis_path",    "../test/test_data/London_2_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/London_2_256.map.scen"},
                    {"output_path", "../test/test_data/London_2_256.txt"},
                    {"sat_path", "../test/test_data/London_2_256.sat"}
            };

    // Sydney_1_256.map
    SingleMapTestConfig<2> MapTestConfig_Sydney_1_256 =
            {
                    {"map_name",    "Sydney_1_256"},
                    {"map_path",    "../test/test_data/Sydney_1_256.map"},
                    {"vis_path",    "../test/test_data/Sydney_1_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Sydney_1_256.map.scen"},
                    {"output_path", "../test/test_data/Sydney_1_256.txt"},
                    {"sat_path", "../test/test_data/Sydney_1_256.sat"}
            };

    SingleMapTestConfig<2> MapTestConfig_Paris_0_256 =
            {
                    {"map_name",    "Paris_0_256"},
                    {"map_path",    "../test/test_data/Paris_0_256.map"},
                    {"vis_path",    "../test/test_data/Paris_0_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Paris_0_256.map.scen"},
                    {"output_path", "../test/test_data/Paris_0_256.txt"},
                    {"sat_path", "../test/test_data/Paris_0_256.sat"}

            };

    // Paris_0_512
    SingleMapTestConfig<2> MapTestConfig_Paris_0_512 =
            {
                    {"map_name",    "Paris_0_512"},
                    {"map_path",    "../test/test_data/Paris_0_512.map"},
                    {"vis_path",    "../test/test_data/Paris_0_512_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Paris_0_512.map.scen"},
                    {"output_path", "../test/test_data/Paris_0_512.txt"}
            };

    SingleMapTestConfig<2> MapTestConfig_Denver_2_256 =

            {
                    {"map_name",    "Denver_2_256"},
                    {"map_path",    "../test/test_data/Denver_2_256.map"},
                    {"vis_path",    "../test/test_data/Denver_2_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Denver_2_256.map.scen"},
                    {"output_path", "../test/test_data/Denver_2_256.txt"},
                    {"sat_path", "../test/test_data/Denver_2_256.sat"}

            };

    //Denver_2_512.map
    SingleMapTestConfig<2> MapTestConfig_Denver_2_512 =

            {
                    {"map_name",    "Denver_2_512"},
                    {"map_path",    "../test/test_data/Denver_2_512.map"},
                    {"vis_path",    "../test/test_data/Denver_2_512_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Denver_2_512.map.scen"},
                    {"output_path", "../test/test_data/Denver_2_512.txt"}
            };

    //full BFS edge pass
    SingleMapTestConfig<2> MapTestConfig_Berlin_1_256 =

            {
                    {"map_name",    "Berlin_1_256"},
                    {"map_path",    "../test/test_data/Berlin_1_256.map"},
                    {"vis_path",    "../test/test_data/Berlin_1_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Berlin_1_256.map.scen"},
                    {"output_path", "../test/test_data/Berlin_1_256.txt"},
                    {"sat_path", "../test/test_data/Berlin_1_256.sat"}

            };

    // London_0_256.map
    SingleMapTestConfig<2> MapTestConfig_London_0_256 =

            {
                    {"map_name",    "London_0_256"},
                    {"map_path",    "../test/test_data/London_0_256.map"},
                    {"vis_path",    "../test/test_data/London_0_256_ENLSVG.vis"},
                    {"config_path", "../test/test_data/London_0_256.map.scen"},
                    {"output_path", "../test/test_data/London_0_256.txt"}
            };

    SingleMapTestConfig<2> MAPFTestConfig_Berlin_1_256 =

            {
                    {"map_name",     "Berlin_1_256"},
                    {"map_path",     "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-map/Berlin_1_256.map"},
                    {"scene_path",   "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/Multi-Agent Path-Finding (MAPF) Benchmarks/mapf-scen-random/scen-random/Berlin_1_256-random-1.scen"},
                    {"ct_path",   "/home/yaozhuo/code/free-nav/third_party/EECBS/Berlin_1_256.ct"},
                    {"output_path", "../test/test_data/MAPF/Berlin_1_256-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/MAPF/Berlin_1_256-random-1_de.txt"},
                    {"agent_num",    "400"}, // 600
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

}
#endif //FREENAV_TEST_DATA_H
