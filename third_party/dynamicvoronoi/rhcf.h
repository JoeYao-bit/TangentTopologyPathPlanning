#ifndef _RHCF_H_
#define _RHCF_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <vector>
#include <time.h>
#include <cmath>
#include "dynamicweightedvoronoi.h"
#include <limits>
#include "../../freeNav-base/basic_elements/point.h"
#include "realpoint.h"
#include "gridUtils.h"
#include "scene.h"

namespace freeNav::RimJump {

    class RHCF {

    public:


        explicit RHCF(DimensionLength* dimension_info,
             const IS_OCCUPIED_FUNC<2>& is_occupied);

        ~RHCF();

        void readScenarioFromFile(std::string file_name);

        void readScenario(double *bot, double *goal, int Nagents,
                          double CW, double CH, double MAX_X, double MIN_X, double MAX_Y, double MIN_Y,
                          double time_upper_bound = 5000); // 5s

        void getBestPath(Pointis<3> & best_path);

        void getNavGraph(Pointis<3> & graph);

        void buildVoronoiDiagram();

        void buildNavigationGraph();

        void findHomotopyClasses();

        double discreteFrechet(float x1[], float y1[], float x2[], float y2[], int p, int q);

        double *measureDiversity(int *HCpathsCellLengths, int *HCpathsLengths, int **HCpaths);

        bool setDiscountingFactor(double d);


    public:

        double social_gain_;

        int border_gain_;

        /// discounting factor, used in the Random Walk
        double df_;
        /// number of homotopy class desired
        int K_;
        /// number of found homotopy classes
        int numHC_;

        // Visuals on or off?
        bool VISUALIZATION_;

        // timeout value in milliseconds
        int timeout_;

        // Define how fine is the border grid
        double borderInterval_;

        // save original border values (just in case)
        double XMINo_;
        double XMAXo_;
        double YMINo_;
        double YMAXo_;

        // Utils to compute quantities related to the grid
        GridUtils *grid_utils_;

        // Scene object that collects info about the scenario (robot, goal, agents, scene dimensions.. )
        Scene *scene_;

        // Voronoi Diagram Object
        DynamicVoronoiRRT *voronoi_;

        // binary map
        std::vector<std::vector<bool>> map_;


        // weighting map
        int **weighting_map_ = nullptr;

        // people directions
        std::vector<DynamicVoronoiRRT::humanCell> human_info;
        /// internal voronoi diagram
        int **voronoiArray_ = nullptr;

        int **V_ = nullptr;

        // size in terms of number of cells
        int sizeX_;

        // size in terms of number of cells
        int sizeY_;

        // Number of vertices in the navigation graph
        int verticesCounter_;

        // Matrix with costs
        double **costMatrix_ = nullptr;

        // keeping edges of the paths
        float ****edgePathsCells_  = nullptr;

        // path lengths
        int **edgePathsLengths_  = nullptr;

        /// edges costs matrix;
        double **edgeCostMatrix_  = nullptr;

        /// type of weighting to use
        int type_weighting_;

        /// gain for the weighting
        double weighting_gain_;

        /// show or not show the terminal info
        bool SHOW_LOG_INFO_ = false;

        int robotV_;
        int goalV_;

        int *posR_ = nullptr;
        int *posG_ = nullptr;
        int *posGv_ = nullptr;
        int *posRV_ = nullptr;

        double **socialCostMatrix_ = nullptr;
        double minSF_;
        double maxSF_;
        double alpha_;

        vector <vector<RealPoint>> paths_;

        Pointis<3> best_path_;

        Paths<2> all_result_path_;

        Pointis<3> nav_graph_;

        bool no_path_found_;

        double *res_diversity = nullptr;

        DimensionLength* dimension_info_;

        IS_OCCUPIED_FUNC<2> is_occupied_;

    };
}
#endif
