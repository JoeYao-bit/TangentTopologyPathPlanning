//
// Created by yaozhuo on 2022/7/26.
//

#ifndef FREENAV_TOPO_SURFACE_PROCESSOR_LINESCANNER_H
#define FREENAV_TOPO_SURFACE_PROCESSOR_LINESCANNER_H

#include <algorithm>

#include "../freeNav-base/basic_elements/surface_process.h"
#include "../freeNav-base/dependencies/path_planning_interface.h"
#include "../freeNav-base/basic_elements/fraction_imported.h"

namespace freeNav::Topo {

    typedef std::vector<int> Extent;

//    struct Interval {
//
//        Interval(int y, const Fraction xL, const Fraction xR, bool leftInclusive = true, bool rightInclusive = true)
//                : y_(y), xL_(xL), xR_(xR), leftInclusive_(leftInclusive), rightInclusive_(rightInclusive) {}
//
//
//        const int y_;
//
//        //const Fraction y_; // floor y as the previous int y
//
//        const Fraction xL_;
//        const Fraction xR_;
//
//        const bool leftInclusive_; // considering whether the left wing occupied to determine the way of calculate next interval
//        const bool rightInclusive_; // considering whether the right wing occupied to determine the way of calculate next interval
//        // TODO: if is occupied, y only delta 0.5 to get next interval, otherwise delta get 1
//    };

    struct Interval {

        Interval(const Fraction& y, const Fraction& xL, const Fraction& xR, bool is_up)
                : y_(y), xL_(xL), xR_(xR), is_up_(is_up) {}

        Interval(const int& y, const Fraction& xL, const Fraction& xR, bool is_up)
                : y_(Fraction(y)), xL_(xL), xR_(xR), is_up_(is_up) {}

        int rounded_y() {
            return is_up_ ? y_.round(false) : y_.round(true);
        }

        Fraction y() const {
            return y_;
        }

        const Fraction y_;

        //const Fraction y_; // floor y as the previous int y

        const Fraction xL_;
        const Fraction xR_;

        const bool is_up_;

        // TODO: if is occupied, y only delta 0.5 to get next interval, otherwise delta get 1
    };

    typedef std::shared_ptr<Interval> IntervalPtr;

    typedef std::vector<IntervalPtr> IntervalPtrs;

    typedef std::vector<IntervalPtrs> IntervalTree;

    typedef std::pair<IntervalTree, IntervalTree> IntervalTreePair;

    Fraction projFromPoint(const FractionPair& start_pt,
                           const Fraction& current_y, const Fraction& current_x,
                           const Fraction& next_y);

    FractionPair GridToFractionPair(const Pointi<2>& pt);


    class SurfaceProcess_LineScanner : public SurfaceProcessor<2> {

    public:

        explicit SurfaceProcess_LineScanner(DimensionLength* dimension_info,
                                            IS_OCCUPIED_FUNC<2> is_occupied,
                                            SET_OCCUPIED_FUNC<2> set_occupied) :
                SurfaceProcessor(dimension_info, is_occupied, set_occupied) {
            updateExtent();
            x_min_ = Fraction(-1, 2), x_max_ = Fraction(2*dimension_info[0] - 1, 2),
            y_min_ = Fraction(-1, 2), y_max_ = Fraction(2*dimension_info[1] - 1, 2);
        }

        /*
         *         bool expandFromStartToNextLevel(const FractionPair& start_pt,
                                        const Fraction& current_y, const Fraction& current_x,
                                        const Fraction& next_y, Fraction& next_x)
         * */

        bool lineCrossObstacle(const Pointi<2>& pt1, const Pointi<2>& pt2) override {
            if(pt1[1] == pt2[1]) {
                if(is_occupied_(pt1) || is_occupied_(pt2)) return true;
                const auto& dim = getDimensionInfo();
                Id id = PointiToId(pt1, dim);
                if(pt1[0] > pt2[0]) {
                    if(left_extent_[id] >= pt2[0]) return true;
                } else {
                    if(right_extent_[id] <= pt2[0]) return true;
                }
            } else {
                if(pt1 == pt2) return true;
                Line<2> line(pt1, pt2);
                int check_step = line.step;
                int base_dimension = line.dimension_index;
                Fraction half(1, 2);
                PointF<2> ptf; Pointi<2> pti1, pti2;
                for(int i=1; i<check_step; i++) {
                    ptf = line.GetPointF(i);
                    // TODO: should expand to N dimension
                    if(base_dimension == 0) {
                        pti1[0] = ptf[0].floor(); // int doesn't change
                        pti2[0] = ptf[0].floor();
                        pti1[1] = (ptf[1] - half).round(true);
                        pti2[1] = (ptf[1] - half).round(false);
                        if(is_occupied_(pti1) && is_occupied_(pti2)) return true;
                    } else {
                        pti1[1] = ptf[1].floor(); // int doesn't change
                        pti2[1] = ptf[1].floor();
                        pti1[0] = (ptf[0] - half).round(true);
                        pti2[0] = (ptf[0] - half).round(false);
                        if(is_occupied_(pti1) && is_occupied_(pti2)) return true;
                    }
                }
                return false;
            }
            return false;
        }

        bool isOnSurface(const Pointi<2>& pt, const Pointis<2>& nearby_offset) override {
            Fraction x(2*pt[0]-1, 2), y(2*pt[1]-1, 2);
            if(isOneInFourCornerOccupied({x, y}, is_occupied_)) { return true; }
            return false;
        }

        bool isTangentPointCandidate(const GridPtr<2>& surface_grid) override {
            Fraction x(2*surface_grid->pt_[0]-1, 2), y(2*surface_grid->pt_[1]-1, 2);
            if(isOneInFourCornerOccupied({x, y}, is_occupied_)) {
                return true;
            }
            else return false;
        }

        // check nearby four corner of current grid
        void updateNearbyGrids(const GridPtr<2>& surface_grid) override {
            Pointi<2> current = surface_grid->pt_;
            const auto& dim = getDimensionInfo();
            if(is_occupied_(current)) surface_grid->nearby_obst_pts_.push_back(current);
            current[0] --;
            if(is_occupied_(current)) surface_grid->nearby_obst_pts_.push_back(current);
            current[1] --;
            if(is_occupied_(current)) surface_grid->nearby_obst_pts_.push_back(current);
            current[0] ++;
            if(is_occupied_(current)) surface_grid->nearby_obst_pts_.push_back(current);
            surface_grid->nearby_obst_pts_.shrink_to_fit();
        }

        // use ENL-SVG style line of sight check
        GridPtrs<2> getLocalVisibleSurfaceGrids(const Pointi<2>& pt, bool half = false) override {
            //std::cout << " use " << __FUNCTION__ << std::endl;
            //std::cout << " ENL-SVG getLocalVisibleSurfaceGrids " << std::endl;
            IntervalPtrs up_prev_halfs, down_prev_halfs;
            IntervalTree tree_up   = searchIntervalFromPt(pt, true);
            IntervalTree tree_down = searchIntervalFromPt(pt, false);
            //std::cout << " get visible_up " << std::endl;
            FractionPairs visible_up   = getVisibleGridFromInterval(pt, tree_up);
            //std::cout << " get tree_down " << std::endl;
            FractionPairs visible_down = getVisibleGridFromInterval(pt, tree_down);
            const auto& dim = getDimensionInfo();
            GridPtrs<2> retv;
            Pointi<2> temp_pt;
            bool first_time = true;
            for(const auto& visible_f : visible_up) {
                temp_pt[0] = visible_f.first.ceil();
                temp_pt[1] = visible_f.second.ceil();
                if(first_time && temp_pt == pt) continue;
//                if(temp_pt[0] < 0 || temp_pt[0] > dim[0]-1 || temp_pt[1] < 0 || temp_pt[1] > dim[1]-1) {
//                    std::cout << "pt " << pt << " / " << "up illegal pt "<< temp_pt << " / visible_f " << visible_f.second << std::endl;
//                    exit(0);
//                }
                retv.push_back(grid_map_[PointiToId(temp_pt, dim)]);
            }
            retv.shrink_to_fit();
            first_time = true;
            for(const auto& visible_f : visible_down) {
                temp_pt[0] = visible_f.first.ceil();
                temp_pt[1] = visible_f.second.ceil();
                if(first_time && temp_pt == pt) continue;
//                if(temp_pt[0] < 0 || temp_pt[0] > dim[0]-1 || temp_pt[1] < 0 || temp_pt[1] > dim[1]-1) {
//                    std::cout << "down illegal pt "<< temp_pt << " / visible_f " << visible_f.second << std::endl;
//                    exit(0);
//                }
                retv.push_back(grid_map_[PointiToId(temp_pt, dim)]);
            }
            retv.shrink_to_fit();
            return retv;
        }

        GridPtrs<2> getVisibleTangentCandidates(const Pointi<2>& pt) override {
            //gettimeofday(&tvpre, &tz);

            GridPtrs<2> surface_grids = getLocalVisibleSurfaceGrids(pt);
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

        int countOfFourCornerOccupied(const FractionPair& start_pt) {
            Pointi<2> buffer;
            int occ_count;

            buffer[0] = start_pt.first.ceil(), buffer[1] = start_pt.second.ceil();
            if(is_occupied_(buffer)) occ_count ++;

            buffer[0] = start_pt.first.ceil(), buffer[1] = start_pt.second.floor();
            if(is_occupied_(buffer)) occ_count ++;

            buffer[0] = start_pt.first.floor(), buffer[1] = start_pt.second.ceil();
            if(is_occupied_(buffer)) occ_count ++;

            buffer[0] = start_pt.first.floor(), buffer[1] = start_pt.second.floor();
            if(is_occupied_(buffer)) occ_count ++;
            return occ_count == 1;
        }

        // visible grids should get from each prev-half interval
        // NOTICE: ENL_SVG style see more visible grids than RJ style
        FractionPairs getVisibleGridFromInterval(const Pointi<2>& start_pt, const IntervalTree& tree) {
            FractionPairs retv;
            Fraction left_fraction, right_fraction, y;
            FractionPair temp_pt;
            Fraction half(1, 2);
            FractionPair start_f = GridToFractionPair(start_pt);
            for(const auto& intervals : tree) {
                for(const auto& interval : intervals) {
                    if(interval->xL_ > interval->xR_) {
                        std::cout << __FUNCTION__ << ": interval->xL_ > interval->xR_ => " << interval->xL_ << " > " << interval->xR_ << std::endl;
                        continue;
                    }

                    //int x_i_left = interval->xL_.ceil(), x_i_right = interval->xR_.floor();
                    //if(x_i_left > x_i_right) return {};

                    left_fraction = (interval->xL_ + half).ceil() - half, right_fraction = (interval->xR_ - half).floor() + half;
                    if(left_fraction > right_fraction) continue;
                    y = interval->y_;

                    // TODO: speed up!
//                    temp_pt.second = interval->y_;
//                    for(Fraction i = left_fraction; i <= right_fraction; i += 1) {
//                        temp_pt.first = i;
//                        if(isOneInFourCornerOccupied(temp_pt))
//                        {
//                            retv.push_back(temp_pt); // first: x, second: y
//                        }
//                    }
                    Fractions vfs = getCornerBetween(interval->y_, left_fraction, right_fraction);
                    bool first_time = interval->y_ == start_f.second;
                    FractionPairs level_0_vf;
                    for(const auto& vf : vfs) {
                        if(!first_time) {
                            retv.push_back({vf, interval->y_}); // first: x, second: y
                        } else {
                            // when first interval, stop when there are more than one grid in nearby four corner
                            if(isOneInFourCornerOccupied({vf, interval->y_}, is_occupied_)) {
                                level_0_vf.push_back({vf, interval->y_});
                            } else {
                                if(vf > start_f.first) break;
                                else if(vf < start_f.first) level_0_vf.clear();
                            }
                        }
                    }
                    retv.shrink_to_fit();
                    retv.insert(retv.end(), level_0_vf.begin(), level_0_vf.end());
                }
            }
            return retv;
        }

        Fractions sortAndRemoveReplicate(const Fractions& frs) {
            Fractions retv = frs;
            sort(retv.begin(), retv.end());
            for(auto iter = retv.begin(); iter != retv.end()-1 && iter != retv.end();) {
                if(*iter == *(iter++)) {
                    iter = retv.erase(iter);
                } else {
                    iter = iter ++;
                }
            }
            return retv;
        }

        // ensure output in increase order
        Fractions getCornerBetween(const Fraction& y_f, const Fraction& x_left_f, const Fraction& x_right_f) {
            //std::cout << " y_f = " << y_f << " / x_left_f " << x_left_f << " / x_right_f " << x_right_f << std::endl;
            const auto& dim = getDimensionInfo();
            if(x_left_f > x_right_f) std::cout << " FATAL: getCornerBetween x_left_f > x_right_f" << std::endl;
            Fractions retv;
            if(isOneInFourCornerOccupied({x_left_f,  y_f}, is_occupied_)) { retv.push_back(x_left_f); }
            int upper_y = y_f.round(true), down_y = y_f.round(false);
            int left_x = x_left_f.round(true), right_x = x_right_f.round(false);
            if(left_x > right_x) {
                if(isOneInFourCornerOccupied({x_right_f, y_f}, is_occupied_)) { retv.push_back(x_right_f); }
                retv = sortAndRemoveReplicate(retv);
                return retv;
            }
            Pointi<2> buff;
            bool pre_is_free = true;
            if(upper_y <= dim[1]-1 && upper_y >= 0) {
                buff[1] = upper_y;
                buff[0] = left_x;
                buff[0] = right_extent_[PointiToId(buff, dim)];
                while( (buff[0] >= 0 && buff[0] <= dim[0]-1) && buff[0] <= right_x) {
                    retv.push_back(Fraction(2*buff[0]-1, 2));
                    buff[0] = right_extent_[PointiToId(buff, dim)];
                }
            }
            if(down_y <= dim[1] -1 && down_y >= 0) {
                buff[1] = down_y;
                buff[0] = left_x;
                buff[0] = right_extent_[PointiToId(buff, dim)];
                while( (buff[0] >= 0 && buff[0] <= dim[0]-1) && buff[0] <= right_x) {
                    retv.push_back(Fraction(2*buff[0]-1, 2));
                    buff[0] = right_extent_[PointiToId(buff, dim)];
                }
            }
            if(isOneInFourCornerOccupied({x_right_f, y_f}, is_occupied_)) { retv.push_back(x_right_f); }
            retv = sortAndRemoveReplicate(retv);
//            std::cout << "retv ";
//            for(const auto& pair_temp : retv) {
//                std::cout << pair_temp << " ";
//            }
//            std::cout << std::endl;
            retv.shrink_to_fit();
            return retv;
        }


        bool isTwoCornerOccupied(const Pointi<2>& start_pt, bool is_up) {
            return isTwoCornerBothOccupied({Fraction(2 * start_pt[0] - 1, 2), Fraction(2 * start_pt[1] - 1, 2)}, is_up);
        }

        bool isTwoCornerBothOccupied(const FractionPair& start_pt, bool is_up) {
            Pointi<2> buffer;
            buffer[1] = start_pt.second.round(is_up);
            buffer[0] = start_pt.first.round(true);
            if(!is_occupied_(buffer)) return false;
            buffer[0] = start_pt.first.round(false);
            if(!is_occupied_(buffer)) return false;
            return true;
        }

        bool isOneInTwoCornerOccupied(const FractionPair& start_pt, bool is_up) {
            Pointi<2> buffer;
            buffer[1] = start_pt.second.round(is_up);
            buffer[0] = start_pt.first.round(true);
            bool occ1 = is_occupied_(buffer);
            buffer[0] = start_pt.first.round(false);
            bool occ2 = is_occupied_(buffer);
            return occ1 ^ occ2; // one is occupied while another is not
        }

        // determine how far can start extent till reach both up and down occupied
        int findFirstTwoOccupiedExtent(const Pointi<2>& start_pt, bool is_left) {
            const auto& dim = getDimensionInfo();
            Fraction y(2*start_pt[1]-1, 2);
            Pointi<2> pt_up, pt_down;
            const auto& extent = is_left ? left_extent_ : right_extent_;
            pt_up[1] = y.round(true), pt_down[1] = y.round(false);
            pt_up[0] = start_pt[0], pt_down[0] = start_pt[0];
            // when is_left, should expand to left to get all visible
            if(!is_left && is_occupied_(pt_up) && is_occupied_(pt_down)) {
                return start_pt[0];
            }
            int bound, temp_bound, id;
            bool up_in_range = false, down_in_range = false;
            if(pt_up[1] <= dim[1]-1 && pt_up[1] >= 0) {
                up_in_range = true;
                while (1) {
                    id = PointiToId(pt_up, dim);
                    temp_bound = extent[id];
                    pt_up[0] = temp_bound;
                    pt_down[0] = temp_bound;
                    if (is_occupied_(pt_up) && is_occupied_(pt_down)) {
                        bound = temp_bound;
                        break;
                    }
                }
            }
            pt_up[0] = start_pt[0], pt_down[0] = start_pt[0];
            if(pt_down[1] <= dim[1]-1 && pt_down[1] >= 0) {
                down_in_range = true;
                while(1) {
                    id = PointiToId(pt_down, dim);
                    temp_bound = extent[id];
                    pt_up[0] = temp_bound; pt_down[0] = temp_bound;
                    if(is_occupied_(pt_up) && is_occupied_(pt_down)) {
                        if(is_left && bound < temp_bound) bound = temp_bound;
                        else if(!is_left && bound > temp_bound) bound = temp_bound;
                        break;
                    }
                }
            }
            if(!down_in_range && !up_in_range) {
                std::cout << " !down_in_range && !up_in_range " << std::endl;
            }
            return bound;
        }

        IntervalTree searchIntervalFromPt(const Pointi<2>& start_pt, bool is_up) {
            //std::cout << " start " << start_pt << " is free / is_up " << is_up << std::endl;
            if(left_extent_.empty() || right_extent_.empty()) {
                std::cout << " un-initialized extents: left " << left_extent_.empty() << ", " << right_extent_.empty() << std::endl;
                exit(0);
            }
            const auto& dim = getDimensionInfo();
            if(start_pt[1] == dim[1]-1 && is_up) return {};
            if(start_pt[1] == 0        && !is_up) return {};

            Fraction x(2*start_pt[0]-1, 2), y(2*start_pt[1]-1, 2); // transform to the coordinate system that used for search interval
            Pointi<2> center; center[1] = y.round(is_up);

            center[0] = x.round(false);
            if(center[0] < 0) center[0] = 0;
            if(center[1] < 0) return {};

            Id left_id = PointiToId(center, dim);
            bool left_occupied = is_occupied_(center);

            center[0] = x.round(true);
            bool right_occupied = is_occupied_(center);
            Id right_id = PointiToId(center, dim);

            int left_bound;
            if(left_occupied) left_bound = x.round(false); // filter cross left obstacle
            else { left_bound = left_extent_[left_id]; }
            int right_bound;
            if(right_occupied) right_bound = x.round(true); // filter cross right obstacle
            else { right_bound = right_extent_[right_id]; }

            // TODO: expand interval when both reach end

            //std::cout << "start_pt " << start_pt << "1 left_bound " << left_bound << " / right_bound " << right_bound << " / up " << is_up << std::endl;
            // considering the visible grid in the zero level
            int left_zero_expand = findFirstTwoOccupiedExtent(start_pt, true),
                right_zero_expand = findFirstTwoOccupiedExtent(start_pt, false);

            //std::cout << "start_pt " << start_pt << "2 left_zero_expand " << left_zero_expand << " / right_zero_expand " << right_zero_expand << " / up " << is_up << std::endl;

            IntervalPtr zero_interval    = std::make_shared<Interval>(y, Fraction(2*left_zero_expand + 1, 2), Fraction(2*right_zero_expand - 1, 2), is_up);
            IntervalTree retv = {{zero_interval}};
            if(isTwoCornerOccupied(start_pt, is_up)) return retv;
            //std::cout << " start " << start_pt << " is free / is_up " << is_up << std::endl;

            // now expand from level 1
            IntervalPtr level_1_interval = std::make_shared<Interval>(y + (is_up ? 1 : -1), Fraction(2*left_bound + 1, 2), Fraction(2*right_bound - 1, 2), is_up);

            IntervalTree next_intervals = ExpandIntervalTillEnd({x, y}, level_1_interval, is_up);
            retv.insert(retv.end(), next_intervals.begin(), next_intervals.end());
            return retv;
        }

        IntervalTree ExpandIntervalTillEnd(const FractionPair& start_pt, const IntervalPtr& zero, bool is_up) {
            IntervalTree retv;
            IntervalPtrs next_intervals = {zero};
            retv.push_back(next_intervals);
            int count = 0;
            while(!next_intervals.empty()) {
                //break;
                IntervalPtrs cur_intervals = next_intervals;
                next_intervals.clear();
                //std::cout << " cur_intervals size " << cur_intervals.size() << std::endl;
                for(const auto& cur_interval : cur_intervals) {
                    //std::cout << "cur_interval = " << cur_interval->y() << " / " << cur_interval->xL_ << " / " << cur_interval->xR_ << std::endl;
                    IntervalPtrs temp_intervals = ExpandInterval(start_pt, cur_interval, is_up);
                    next_intervals.insert(next_intervals.end(), temp_intervals.begin(), temp_intervals.end());
                }
                //std::cout << "next_intervals .size() " << next_intervals.size() << std::endl;
                retv.push_back(next_intervals);
                count ++;
                //if(count == 10) break;
            }
            retv.shrink_to_fit();
            return retv;
        }

        // find where to project when from start, six point
        /*
         *     __ __
         *    |     |
         *    |__ __|
         *
         *
         *
         */
        bool findStartProjectToGridPoint(const FractionPair& start_pt, const Pointi<2>& grid, bool is_up, bool is_left, Fraction& grid_proj_x, Fraction& grid_proj_y) {
            // check whether ob the same level, avoid multi case
            if(start_pt.second.round(!is_up) == grid[1]) return false;

            int delta_y = is_up ? 1 : -1;
            Fraction up_y(2*grid[1] + delta_y, 2), down_y(2*grid[1] - delta_y, 2);

            int delta_x = is_left ? -1 : 1;

            Fraction x(2*grid[0] + delta_x, 2);
            grid_proj_x = x; // left side
            Fraction x_offset = x - start_pt.first; // can never be zero
            if(x_offset * delta_x < 0) {
                grid_proj_y = up_y;
            } else {
                grid_proj_y = down_y;
            }

            //std::cout << "grid = " << grid << " / grid_proj_x " << grid_proj_x << " / grid_proj_y = " << grid_proj_y << std::endl;
            return true;
        }

        // is_left: whether start_pt jump to the left or right side of the grid
        bool expandFromStartToGrid(const FractionPair& start_pt, const Pointi<2>& grid, const Fraction& next_y, Fraction& next_x, bool is_up, bool is_left) {
            if(start_pt.second.round(!is_up) == grid[1]) return false;

            const auto& dim = getDimensionInfo();
            const Fraction y_min(-1, 2), y_max(2*dim[1] - 1, 2);

            if(next_y < y_min || next_y > y_max) {
                //std::cout << " return 1" << std::endl;
                return false;
            }
            Fraction corner_y, corner_x;
            if(findStartProjectToGridPoint(start_pt, grid, is_up, is_left, corner_x, corner_y)) {
                return expandFromStartToNextLevel(start_pt, corner_y, corner_x, next_y, next_x);
            } else {
                return false;
            }
        }

        // determine all surface grid on a line ( y is int ), need more test
        // search upward and downward occupied grids, and get nearby interval in current interval
        Pointis<2> getSurfacePointInInterval(int y, const int& left, const int& right) {
            //if(y == 19) return {};
            //std::cout << " y " << y << " / left " << left << " / " << right << std::endl;
            //std::cout << "flag 1" << std::endl;
            if(right < left) {
                //std::cout << " right < left " << std::endl;
                return {};
            }
            // determine surface grid between current level and next level
            const auto& dim = getDimensionInfo();
            if(y > dim[0] -1 || y < 0) {
                return {};
            }
            Pointis<2> retv;
            std::vector<int> nearby_y = {y - 1, y + 1};
            Pointi<2> check_occupied_pt, check_surface_pt;
            check_surface_pt[1] = y;
            int left_x = left - 1, right_x = right + 1;
            if(left_x < 0) left_x = 0;
            if(right_x > dim[0] - 1) right_x = dim[0] - 1;
            int current_left_x = left_x;
            //bool prev_grid_free;
            //std::cout << "flag 2" << std::endl;
            for(const int& current_y : nearby_y) {
                if(current_y < 0 || current_y > dim[1] - 1) continue;
                check_occupied_pt[1] = current_y;
                current_left_x = left_x;
                //std::cout << " left_x  = " << left_x << std::endl;
                //std::cout << " right_x = " << right_x << std::endl;

                while(current_left_x <= right_x) {

                    if(current_left_x < 0 || current_left_x >= dim[0]) break;

                    check_occupied_pt[0] = current_left_x;
                    //std::cout << "y = " << y << ", current_y " << current_y << ", pre current_left_x = " << current_left_x << std::endl;
                    bool current_occupied = false;
                    if(is_occupied_(check_occupied_pt))
                    {
                        current_occupied = true;
                        check_surface_pt[0] = current_left_x - 1;
                        if(check_surface_pt[0] >= left && check_surface_pt[0] <= right && isOnSurface(check_surface_pt, nearby_offsets_))
                            retv.push_back(check_surface_pt);

                        check_surface_pt[0] = current_left_x + 1;
                        if(check_surface_pt[0] >= left && check_surface_pt[0] <= right && isOnSurface(check_surface_pt, nearby_offsets_))
                            retv.push_back(check_surface_pt);
                    }
                    if(!current_occupied) { current_left_x = right_extent_[PointiToId(check_occupied_pt, dim)]; }
                    else {
                        if(current_left_x == right_extent_[PointiToId(check_occupied_pt, dim)] - 1) {
                            current_left_x = right_extent_[PointiToId(check_occupied_pt, dim)];
                        } else {
                            current_left_x = right_extent_[PointiToId(check_occupied_pt, dim)] - 1;
                        }
                    }
                    //std::cout << "y = " << y << ", current_y " << current_y << ", after current_left_x = " << current_left_x << std::endl;
                }

            }
            retv.shrink_to_fit();
            //std::cout << "flag 3" << std::endl;
            return retv;
        }

        Pointis<2> getSurfacePointInInterval(int y, const Fraction& left, const Fraction& right) {
            return getSurfacePointInInterval(y, left.round(true), right.round(false));
        }

        Fractions getIntervalOfGrid(const Pointi<2>& pt) {
            Pointi<2> current_grid; current_grid[1] = pt[1];
            bool pre_state, cur_state, next_state;

            current_grid[0] = pt[0] - 1;
            pre_state  = is_occupied_(current_grid);

            current_grid[0] = pt[0];
            cur_state  = is_occupied_(current_grid);

            current_grid[0] = pt[0] + 1;
            next_state = is_occupied_(current_grid);

            Fractions retv;
            if(pre_state ^ cur_state)  { retv.push_back(Fraction(2*pt[0] - 1, 2)); }
            if(cur_state ^ next_state) { retv.push_back(Fraction(2*pt[0] + 1, 2)); }
            retv.shrink_to_fit();
            return retv;
        }


        Fractions getIntervalBetween(const Fraction& y_f, bool is_up, const Fraction& left_x_f, const Fraction& right_x_f) {
            const auto& dim = getDimensionInfo();
            // limit range
            if(y_f < y_min_ || y_f > y_max_) return {};
            if(left_x_f > right_x_f) return {};
            if(left_x_f >= x_max_) return {};
            if(right_x_f <= x_min_) return {};

            // limit left < right
            if(left_x_f > right_x_f) return {};

            Fraction new_left_x_f  = left_x_f  >= x_min_ ? left_x_f  : x_min_;
            Fraction new_right_x_f = right_x_f <= x_max_ ? right_x_f : x_max_;

            // may be redundant
            if(new_left_x_f > new_right_x_f) return {};

            const int y = y_f.round(!is_up);
            Pointi<2> current_pt, right_pt;
            current_pt [1] = y,
                    right_pt[1] = y;
            current_pt [0] =  new_left_x_f.round(true),
                    right_pt[0] = new_right_x_f.round(false);

            if(current_pt[0] == right_pt[0]) {
                return {new_left_x_f, new_right_x_f};
            } else if(current_pt[0] > right_pt[0]) {
                return {new_left_x_f, new_right_x_f};
            } else {

                Fractions retv = {};
                Id current_id;
                int next_x, pre_x;
                while(1) {
                    current_id = PointiToId(current_pt, dim);
                    if(current_pt[0] <= right_pt[0]) {

//                        auto revs = getIntervalOfGrid(current_pt);
//                        for(const auto& rev : revs) {
//                            if(rev >= new_left_x_f && rev <= new_right_x_f) {
//                                if(retv.empty()) {
//                                    retv.push_back(rev);
//                                } else if(retv.back() != rev) {
//                                    retv.push_back(rev);
//                                }
//                            }
//                        }
                        pre_x = left_extent_[current_id];
                        next_x = right_extent_[current_id];
                        if(pre_x == current_pt[0]-1) {
                            retv.push_back(Fraction(2*current_pt[0]-1, 2));
                        }
                        current_pt[0] = next_x;
                    } else {
                        break;
                    }
                }
                if(!retv.empty()) {
                    if(retv.front() < new_left_x_f) retv.erase(retv.begin());
                }
                if(!retv.empty()) {
                    if(retv.back() > new_right_x_f) retv.erase(retv.end()-1);
                }
                retv.insert(retv.begin(), new_left_x_f);
                // special case
                if(new_right_x_f.d == 2 && is_occupied_(right_pt)) { retv.push_back(new_right_x_f); }
                retv.push_back(new_right_x_f);
                retv.shrink_to_fit();
                return retv;
            }
        }

//        Fractions getIntervalBetween(const Fraction& y_f, bool is_up, const Fraction& left_x_f, const Fraction& right_x_f) {
//            const auto& dim = getDimensionInfo();
//            // limit range
//            if(y_f < y_min_ || y_f > y_max_) return {};
//            if(left_x_f > right_x_f) return {};
//            if(left_x_f >= x_max_) return {};
//            if(right_x_f <= x_min_) return {};
//
//            // limit left < right
//            if(left_x_f > right_x_f) return {};
//
//            Fraction new_left_x_f  = left_x_f  >= x_min_ ? left_x_f  : x_min_;
//            Fraction new_right_x_f = right_x_f <= x_max_ ? right_x_f : x_max_;
//
//            // may be redundant
//            if(new_left_x_f > new_right_x_f) return {};
//
//            const int y = y_f.round(!is_up);
//            Pointi<2> current_pt, right_pt;
//            current_pt [1] = y,
//            right_pt[1] = y;
//            current_pt [0] =  new_left_x_f.round(true),
//            right_pt[0] = new_right_x_f.round(false);
//
//            if(current_pt[0] == right_pt[0]) {
//                return {new_left_x_f, new_right_x_f};
//            } else if(current_pt[0] > right_pt[0]) {
//                return {new_left_x_f, new_right_x_f};
//            } else {
//                Fractions retv = {};
//                while(1) {
//                    if(current_pt[0] <= right_pt[0]) {
//                        auto revs = getIntervalOfGrid(current_pt);
//                        for(const auto& rev : revs) {
//                            if(rev >= new_left_x_f && rev <= new_right_x_f) {
//                                if(retv.empty()) {
//                                    retv.push_back(rev);
//                                } else if(retv.back() != rev) {
//                                    retv.push_back(rev);
//                                }
//                            }
//                        }
//                        current_pt[0] = right_extent_[PointiToId(current_pt, dim)];
//                    } else {
//                        break;
//                    }
//                }
//                retv.insert(retv.begin(), new_left_x_f);
//                retv.push_back(new_right_x_f);
//                return retv;
//            }
//        }



        FractionPairs splitIntervalNew(const IntervalPtr& interval, const FractionPair& start_pt, const Fraction& y_f, bool is_up, const Fraction& left_x_f, const Fraction& right_x_f) {
            const auto& dim = getDimensionInfo();

            // 1, get all interest interval
            auto all_intervals = getIntervalBetween(y_f, is_up, left_x_f, right_x_f);

//            std::cout << "y_f: " << y_f << " / is_up " << is_up << " / left_x_f " << left_x_f << " / right_x_f " << right_x_f << " / " << " all_intervals: ";
//            for(const auto& vf : all_intervals) {
//                std::cout << vf.toFloat() << " ";
//            }
//            std::cout << std::endl;
            if(all_intervals.size() < 2) return {};
            // 2, traversal each interval and check whether legal to split
            FractionPairs retv;
            Fraction left, right;
            Fraction new_left, new_right;
            const int y = y_f.round(!is_up);
            Fraction prev_y_f = y_f + (is_up ? -1 : 1);
            Pointi<2> left_occupied;  left_occupied[1] = y;
            Pointi<2> right_occupied; right_occupied[1] = y;
            Pointi<2> left_pt;  left_pt[1] = y;
            Pointi<2> right_pt; right_pt[1] = y;
            Fraction bounded_x_f, temp_x_f;


            bool left_gd_occupied, right_gd_occupied;
            Id left_gd_id, right_gd_id;
            for(int i=0; i<all_intervals.size()-1; i++) {
                left = all_intervals[i], right = all_intervals[i+1];
                left_pt[0] = left.round(true);
                right_pt[0] = right.round(false);
                left_gd_occupied = is_occupied_(left_pt), right_gd_occupied = is_occupied_(right_pt);
                left_gd_id = PointiToId(left_pt, dim), right_gd_id = PointiToId(right_pt, dim);
                if(left < right) {
                    //std::cout << "left < right: ";
                    // check whether left->right is free, otherwise, continue
                    if(left_gd_occupied || right_gd_occupied) continue;
                    //std::cout << "both free /";
                    new_left = left;
                    if(right_pt[0] >= right_extent_[left_gd_id]) continue;
                    // considering change direction by grid
                    if (left > start_pt.first) {
                        left_occupied[0] = left_extent_[left_gd_id];
                        temp_x_f = Fraction(1, 2) + left_occupied[0];
                        if(temp_x_f > start_pt.first) {
                            if(!expandFromStartToNextLevel(start_pt, prev_y_f, temp_x_f, is_up,bounded_x_f)) return retv;

//                        if (!expandFromStartToGrid(start_pt, left_occupied, y_f, bounded_x_f, is_up, false))
//                            return retv;

                            if (bounded_x_f > left) { new_left = bounded_x_f; }
                        }
                    }

                    new_right = right;
                    if (right < start_pt.first) {
                        right_occupied[0] = right_extent_[right_gd_id];
                        temp_x_f = Fraction(-1, 2) + right_occupied[0];
                        if(temp_x_f < start_pt.first) {

//                        if (!expandFromStartToGrid(start_pt, right_occupied, y_f, bounded_x_f, is_up,true))
//                            return retv;

                            if(!expandFromStartToNextLevel(start_pt, prev_y_f, temp_x_f, is_up, bounded_x_f)) return retv;
                            if (bounded_x_f < right) { new_right = bounded_x_f; }
                        }
                    }
                    //std::cout << "new_left / new_right = " << new_left << " / " << new_right;
                    if(new_left > new_right) continue;
                    retv.push_back({new_left, new_right});

                } else if(left > right) {
                    std::cout << __FUNCTION__ << ", FATAL: left > right " << std::endl;
                    continue;
                } else {
                    // check whether left == right is free, otherwise, continue
                    // if one in left ot right is free, the interval left->right is free
                    //std::cout << "left = right: ";
                    if(left_gd_occupied && right_gd_occupied) continue;
                    //std::cout << "both free /";
                    new_left = left;
                    new_right = right;
                    if(!isBoundCrossObstacleNew(start_pt,interval, y_f, new_left, is_up, true))
                    { retv.push_back({new_left, new_right}); }

                }
            }
            retv.shrink_to_fit();
            return retv;
        }

        bool expandFromStartToNextLevel(const FractionPair& start_pt,
                                        const Fraction& current_y, const Fraction& current_x,
                                        const Fraction& next_y, Fraction& next_x) {
            //std::cout << " start_pt " << start_pt.first << " / " << start_pt.second << std::endl;
            //std::cout << "current_x" << current_x << " / current_y = " << current_y.toFloat() << " / next_y " << next_y.toFloat() << std::endl;

            if(next_y < y_min_ || next_y > y_max_) {
                //std::cout << " return 4" << std::endl;
                return false;
            }
            Fraction bottom_y = current_y - start_pt.second;

            Fraction k_ratio = (current_x - start_pt.first)/bottom_y;

            next_x = k_ratio*(next_y - start_pt.second) + start_pt.first;
            //if(next_y - current_y != 1) std::cout << "next_y - current_y != 1, = " << next_y - current_y << std::endl;
            //std::cout << "next_x " << next_x << std::endl;
            return true;
        }

        // jump with delta y = 1
        bool expandFromStartToNextLevel(const FractionPair& start_pt,
                                        const Fraction& current_y, const Fraction& current_x,
                                        bool is_up, Fraction& next_x) {
            //std::cout << " start_pt " << start_pt.first << " / " << start_pt.second << std::endl;
            //std::cout << "current_x" << current_x << " / current_y = " << current_y.toFloat() << " / next_y " << next_y.toFloat() << std::endl;


            Fraction k_ratio = (current_x - start_pt.first)/(current_y - start_pt.second);

            next_x = is_up ? current_x + k_ratio : current_x - k_ratio;
            //std::cout << "next_x " << next_x << std::endl;
            return true;
        }

        IntervalPtrs ExpandInterval(const FractionPair& start_pt, const IntervalPtr& interval, bool is_up) {
            const auto& dim = getDimensionInfo();
            //std::cout << "interval: y = " << interval->y().toFloat() << " / xL = " << interval->xL_.toFloat() << " / xR = " << interval->xR_.toFloat() << " / is_up " << is_up << std::endl;
            if(!is_up && interval->y_ < y_min_) {
                return {};
            }
            if( is_up && interval->y_ >= y_max_) {
                return {};
            }
            if(interval->xL_ == interval->xR_) return {};
            IntervalPtrs next_intervals;
            Fraction delta_y = is_up ? Fraction(1) : Fraction(-1);
            Fraction next_y = interval->y() + delta_y;
            if(!is_up && next_y < y_min_) {
                return {};
            }
            if( is_up && next_y >= y_max_) {
                return {};
            }
//            Pointi<2> left_grid, right_grid;
//            left_grid[1]  = interval->rounded_y(), right_grid[1] = interval->rounded_y();
//            left_grid[0]  = interval->xL_.round(true);
//            right_grid[0] = interval->xR_.round(false);

            // 1, determine the maximum left and right range of next interval
            //std::cout << " check loop " << std::endl;
            if(interval->y_ == start_pt.second) {
                // 2, determine the first interval's range
                std::cout << __FUNCTION__ << " : FATAL: interval->y_ == start_pt.second" << std::endl;
                exit(0);
            }

            // 1, determine the maximum left and right range of next interval


            // 3, project to next level with the same k_ratio, y = k*x+b
            Fraction left_proj_bound, right_proj_bound;

            // can only jump to next level, avoid horizontal line
            if(!expandFromStartToNextLevel(start_pt, interval->y_, interval->xL_, is_up, left_proj_bound))  {
                return {};
            }

            if(!expandFromStartToNextLevel(start_pt, interval->y_, interval->xR_, is_up, right_proj_bound))  {
                return {};
            }

            //std::cout << "next_y = " << next_y << " / left_proj = " <<  left_proj_bound.toFloat() << " / right_proj = " << right_proj_bound.toFloat() << std::endl;

            // 4, select the grid that close to when project to next level

//            Pointi<2> left_proj_pt, right_proj_pt;
//            left_proj_pt[1] = next_y.round(!is_up), right_proj_pt[1] = next_y.round(!is_up);
//            left_proj_pt[0] = left_proj_bound.round(true), right_proj_pt[0] = right_proj_bound.round(false);

            Fraction left_bound, right_bound;

//            if(!is_occupied_(left_proj_pt)) {
//                Fraction left_grid_bound;
//                //bool expandFromStartToGrid(const Pointi<2>& start_pt, const Pointi<2>& grid, const Fraction& next_y, Fraction& next_x, bool is_left)
//                left_proj_pt[0] = left_extent_[PointiToId(left_proj_pt, dim)];
//                if(!expandFromStartToGrid(start_pt, left_proj_pt, next_y, left_grid_bound, is_up, false)) return {};
//                //std::cout << "left_grid_bound = " <<  left_grid_bound.toFloat() << std::endl;
//                if(left_grid_bound > left_proj_bound) {
//                    left_bound = left_grid_bound;
//                } else {
//                    left_bound = left_proj_bound;
//                }
//            } else
                {
                left_bound = left_proj_bound;
            }

//            if(!is_occupied_(right_proj_pt)) {
//                Fraction right_grid_bound;
//                right_proj_pt[0] = right_extent_[PointiToId(right_proj_pt, dim)];
//                if(!expandFromStartToGrid(start_pt, right_proj_pt, next_y, right_grid_bound, is_up, true)) return {};
//                //std::cout << "right_grid_bound = " <<  right_grid_bound.toFloat() << std::endl;
//                if(right_grid_bound < right_proj_bound) {
//                    right_bound = right_grid_bound;
//                } else {
//                    right_bound = right_proj_bound;
//                }
//            } else
                {
                right_bound = right_proj_bound;
            }
            //std::cout << "left_bound " << left_bound << " / right_bound " << right_bound << std::endl;
            //auto new_intervals = splitInterval(interval, start_pt, next_y, is_up, left_bound, right_bound);
            const auto& new_intervals = splitIntervalNew(interval, start_pt, next_y, is_up, left_bound, right_bound);
            IntervalPtrs retv;
            for(const auto& new_interval : new_intervals) {
                const IntervalPtr& next_interval = std::make_shared<Interval>(next_y,
                                                                       new_interval.first, new_interval.second,
                                                                       is_up);
                retv.push_back(next_interval);
            }
            //std::cout << " when finish expand interval, next_y = " << next_y << std::endl;
            //std::cout << "new_intervals size " << new_intervals.size() << std::endl;
            return retv;
        }

        bool isBoundCrossObstacleNew(const FractionPair& start_pt, const IntervalPtr& interval, const Fraction& y, const Fraction& cur_x_fraction, bool is_up, bool is_left) {
            // assume interval is collision free and left < right
            const auto& dim = getDimensionInfo();
            if((interval->y() - y).toAbs() != 1) {
                std::cout << " FATAL: interval->y() - y != 1, = " << (interval->y() - y).toAbs() << std::endl;
            }
            if(y > Fraction(2*dim[1]-1, 2) || y < Fraction(-1, 2)) return true;
            if(cur_x_fraction > Fraction(2*dim[0]-1, 2) || cur_x_fraction < Fraction(-1, 2)) return true;
            Fraction delta_y = is_up ? Fraction(1) : Fraction(-1);
            Fraction next_y = interval->y() + delta_y;
            Pointi<2> next_pt; next_pt[1] = next_y.round(!is_up);

            Pointi<2> next_pt_l = next_pt, next_pt_r = next_pt;
            next_pt_l[0] = cur_x_fraction.round(true);
            next_pt_r[0] = cur_x_fraction.round(false);
            // NOTICE: when left proj = right proj, the left of right extent may be occupied, but it's legal
            bool left_occ = is_occupied_(next_pt_l), right_occ = is_occupied_(next_pt_r);
            if(left_occ && right_occ) return true;
            else if(!left_occ) {
                next_pt = next_pt_l;
            } else {
                next_pt = next_pt_r;
            }

            Pointi<2> left_grid_temp, right_grid_temp;
            left_grid_temp[1] = next_pt[1], right_grid_temp[1] = next_pt[1];

            int left_offset, right_offset;
            right_offset = right_extent_[PointiToId(next_pt, dim)];
            left_offset  = left_extent_ [PointiToId(next_pt, dim)];
            left_grid_temp[0] = left_offset, right_grid_temp[0] = right_offset;
            //std::cout << "left_offset = " << left_offset << " / right_offset = " << right_offset << std::endl;
            Fraction left_grid_proj, right_grid_proj;
            if(!expandFromStartToGrid(start_pt, left_grid_temp,  y, left_grid_proj,  is_up, false)) return true;
            if(!expandFromStartToGrid(start_pt, right_grid_temp, y, right_grid_proj, is_up, true)) return true;
            //std::cout << "cur_x_fraction = " << cur_x_fraction.toFloat() << " / left_grid_proj " << left_grid_proj.toFloat() << " / right_grid_proj = " << right_grid_proj.toFloat() << std::endl;
            //if(left_grid_proj > right_grid_proj) return true;
            //if(cur_x_fraction < left_grid_proj || cur_x_fraction > right_grid_proj) return true;
            if(cur_x_fraction > start_pt.first) {
                if(cur_x_fraction < left_grid_proj) return true;
            } else if(cur_x_fraction < start_pt.first) {
                if(cur_x_fraction > right_grid_proj) return true;
            }

            return false;
        }

        Extent left_extent_;

        Extent right_extent_;

    private:

        Fraction x_min_, x_max_, y_min_, y_max_;


        void updateExtent() {
            const auto& dim = getDimensionInfo();

            left_extent_.resize(getTotalIndex(), -1);
            right_extent_.resize(getTotalIndex(), -1);
            for(int y=0; y<dim[1]; y++) {
                Pointi<2> pre_pt, cur_pt;
                bool pre_oc, cur_oc;
                Id cur_id;
                // extent to left
                {
                    int start_x_left = -1;
                    pre_pt[1] = y, cur_pt[1] = y, cur_pt[0] = 0;
                    cur_id = PointiToId(cur_pt, dim);
                    if (!is_occupied_(cur_pt)) { left_extent_[cur_id] = start_x_left; }
                    else { left_extent_[cur_id] = -1; }
                    for (int x = 1; x < dim[0]; x++) {
                        pre_pt[0] = x - 1, cur_pt[0] = x;
                        cur_id = PointiToId(cur_pt, dim);
                        pre_oc = is_occupied_(pre_pt), cur_oc = is_occupied_(cur_pt);
                        if (pre_oc != cur_oc) { start_x_left = pre_pt[0]; }
                        left_extent_[cur_id] = start_x_left;
                    }
                }
                // extent to right
                {
                    int start_x_right = dim[0];
                    pre_pt[1] = y, cur_pt[1] = y, cur_pt[0] = dim[0] - 1;
                    cur_id = PointiToId(cur_pt, dim);
                    if(!is_occupied_(cur_pt)) { right_extent_[cur_id] = dim[0]; }
                    else { right_extent_[cur_id] = dim[0]; }
                    for(int x=dim[0]-2; x>=0; x--) {
                        pre_pt[0] = x + 1, cur_pt[0] = x;
                        cur_id = PointiToId(cur_pt, dim);
                        pre_oc = is_occupied_(pre_pt),    cur_oc = is_occupied_(cur_pt);
                        if(pre_oc != cur_oc) { start_x_right = pre_pt[0]; }
                        right_extent_[cur_id] = start_x_right;
                    }
                }
            }
        }

    };



}

#endif //FREENAV_SURFACE_PROCESSOR_LINESCANNER_H
