//
// Created by yaozhuo on 2022/10/16.
//

#ifndef FREENAV_TOPO_SURFACE_PROCESSOR_ENLSVG_LINESCANNER_H
#define FREENAV_TOPO_SURFACE_PROCESSOR_ENLSVG_LINESCANNER_H

#include "../freeNav-base/basic_elements/surface_process.h"
#include "../freeNav-base/dependencies/path_planning_interface.h"
#include "../algorithm/2d_ENLSVG_grid.h"

namespace freeNav::Topo {

    typedef std::shared_ptr<Pathfinding::LineOfSightScanner> LineOfSightScannerPtr;

    typedef std::shared_ptr<Pathfinding::ENLSVG::Memory> MemoryPtr;

    typedef std::shared_ptr<Pathfinding::Grid> PF_GridPtr;


    class SurfaceProcess_ENLSVG_LineScanner : public SurfaceProcessor<2> {

    public:

        explicit SurfaceProcess_ENLSVG_LineScanner(DimensionLength *dimension_info,
                                        IS_OCCUPIED_FUNC<2> is_occupied,
                                        SET_OCCUPIED_FUNC<2> set_occupied) :
                SurfaceProcessor<2>(dimension_info, is_occupied, set_occupied) {

            grid_ptr_ = std::make_shared<Pathfinding::Grid>(getENL_SVG_Module(dimension_info, is_occupied_));
            line_scanner_ptr_ = std::make_shared<Pathfinding::LineOfSightScanner>(*grid_ptr_);
            //memory_ptr_ = std::make_shared<Pathfinding::ENLSVG::Memory>(*grid_ptr_);
            surfaceGridsDetection();
        }


        inline bool lineCrossObstacle(const Pointi<2>& pt1, const Pointi<2>& pt2) override {
            return !grid_ptr_->lineOfSight(pt1[0], pt1[1], pt2[0], pt2[1]);
        }

        inline bool isOnSurface(const Pointi<2>& pt, const Pointis<2>& nearby_offset) override {
            Fraction x(2*pt[0]-1, 2), y(2*pt[1]-1, 2);
            if(isOneInFourCornerOccupied({x, y}, is_occupied_)) { return true; }
            return false;
        }

        inline bool isTangentPointCandidate(const GridPtr<2>&  surface_grid) override {
            Fraction x(2*surface_grid->pt_[0]-1, 2), y(2*surface_grid->pt_[1]-1, 2);
            if(isOneInFourCornerOccupied({x, y}, is_occupied_)) {
                return true;
            }
            else return false;
        }

        // check nearby four corner of current grid
        inline void updateNearbyGrids(const GridPtr<2>& surface_grid) override {
            Pointi<2> current = surface_grid->pt_;
            const auto& dim = getDimensionInfo();
            surface_grid->nearby_obst_pts_.clear();
            surface_grid->nearby_obst_offsets_.clear();
            if(is_occupied_(current)) {
                surface_grid->nearby_obst_pts_.push_back(current);
                Pointi<2> pt;
                pt[0] = pt[1] = 1;
                surface_grid->nearby_obst_offsets_.push_back(pt);
            }
            current[0] --;
            if(is_occupied_(current)) {
                surface_grid->nearby_obst_pts_.push_back(current);
                Pointi<2> pt;
                pt[0] = -1; pt[1] = 1;
                surface_grid->nearby_obst_offsets_.push_back(pt);
            }
            current[1] --;
            if(is_occupied_(current))  {
                surface_grid->nearby_obst_pts_.push_back(current);
                Pointi<2> pt;
                pt[0] = -1; pt[1] = -1;
                surface_grid->nearby_obst_offsets_.push_back(pt);
            }
            current[0] ++;
            if(is_occupied_(current))  {
                surface_grid->nearby_obst_pts_.push_back(current);
                Pointi<2> pt;
                pt[0] = 1; pt[1] = -1;
                surface_grid->nearby_obst_offsets_.push_back(pt);
            }
            surface_grid->nearby_obst_pts_.shrink_to_fit();
            surface_grid->nearby_obst_offsets_.shrink_to_fit();
        }

        PathLen distBetween(const Pointi<2>& pt1, const Pointi<2>& pt2) const override {
            int x = pt1[0]-pt2[0], y = pt1[1]-pt2[1];
            return sqrt(x*x + y*y);
        }

        // use ENL-SVG style line of sight check
        inline GridPtrs<2> getLocalVisibleSurfaceGrids(const Pointi<2>& pt, bool half = false) override {

            GridPtrs<2> retv;

            //Pathfinding::ScannerStacks& data = memory_ptr_->scannerStacks;

            line_scanner_ptr_->computeAllDirNeighbours(scanner_stacks_, pt[0], pt[1]);
            //std::cout << " data.neighbours size " << data.neighbours.size() << std::endl;
            Pointi<2> temp_pt;
            Id id_pt;
            for (const auto &neighbour : scanner_stacks_.neighbours) {
                temp_pt[0] = neighbour.x, temp_pt[1] = neighbour.y;
                id_pt = PointiToId(temp_pt, getDimensionInfo());
                if(grid_map_[id_pt] != nullptr) {
                    retv.push_back(grid_map_[id_pt]);
                }
            }
            retv.shrink_to_fit();
            return retv;
        }

        inline GridPtrs<2> getVisibleTangentCandidates(const Pointi<2>& pt) override {
            //gettimeofday(&tvpre, &tz);

            GridPtrs<2> surface_grids = getLocalVisibleSurfaceGrids(pt);
            //std::cout << " surface_grids size = " << surface_grids.size() << std::endl;
            GridPtrs<2> retv;
            for(const auto& surface_grid : surface_grids) {
                // use pre-calculated value to avoid online tangent check
                if(grid_map_[surface_grid->id_]->node_id_ != MAX<NodeId>) {
                    retv.push_back(grid_map_[surface_grid->id_]);
                }
            }
            //gettimeofday(&tvafter, &tz);
            //double cost_ms =(tvafter.tv_sec - tvpre.tv_sec)*1e3 + (tvafter.tv_usec - tvpre.tv_usec)/1e3;
            //std::cout << "-- getVisibleTangentCandidates " << cost_ms << "ms" << std::endl;
            retv.shrink_to_fit();
            return retv;
        }

    private:

        LineOfSightScannerPtr line_scanner_ptr_;

        //MemoryPtr memory_ptr_;

        Pathfinding::ScannerStacks scanner_stacks_;

        PF_GridPtr grid_ptr_;
    };

}
#endif //FREENAV_SURFACE_PROCESSOR_ENLSVG_LINESCANNER_H
