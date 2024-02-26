//
// Created by yaozhuo on 2023/2/12.
//

#ifndef FREENAV_TOPO_SEARCH_PATH_WITH_NODE_H
#define FREENAV_TOPO_SEARCH_PATH_WITH_NODE_H

#include "search_path_with_edge.h"
#include "dynamic_data_with_node.h"
//#include "node_iteration_constraits.h"
//#include "constraints/iteration_constraits.h"
#include "../../algorithm/constraints/node_iteration_constraints.h"
#include <queue>

namespace freeNav::Topo {


    template <Dimension N>
    class GeneralGraphPathPlannerWithNode {

    public:

        explicit GeneralGraphPathPlannerWithNode(RoadMapGraphPtr<N>& tg) : tg_(tg),
                                                                           current_set_(tg->nodes_.size()) {
            data_ = std::make_shared<DynamicDataOfSearchWithNode<N> >(tg);
            gettimeofday(&tv_pre, &tz);
            /* remember what inserted to the graph, to avoid traversal the whole graph to remove dynamic nodes */
            /* reset graph */
            gettimeofday(&tv_after, &tz);

            double reset_cost_ms = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
            std::cout << " G2P cost " << reset_cost_ms << " to init " << std::endl;
        }

        // ready for next path planning
        void initialise() {
            current_time_ ++;
            tg_->pruneToStaticNodeAndEdge();
            data_->pruneToStaticNodeAndEdge();
        }

        PathPointWithNodePtrs<N> createVisiblePathPointPtrsWithNode(const Pointi<N>& start,
                                                                    const Pointi<N>& target,
                                                                    const PointTransferConstraints<N>& ptcs,
                                                                    const EdgeTransferConstraints3<N>& etcs) {
            auto retv_1 = createInitPathPtrsWithNode(target, false, ptcs, etcs);
            auto retv_2 = createInitPathPtrsWithNode(start, true, ptcs, etcs);
            return retv_2;
        }

        std::pair<PathPointWithNodePtrs<N>, PathPointWithNodePtrs<N>> createVisiblePathPointPtrsPairWithNode(
                const Pointi<N>& start, const Pointi<N>& target,
                const PointTransferConstraints<N>& ptcs, const EdgeTransferConstraints3<N>& etcs) {
            auto retv_1 = createInitPathPtrsWithNode(target, false, ptcs, etcs);
            auto retv_2 = createInitPathPtrsWithNode(start, true, ptcs, etcs);
            return {retv_1, retv_2};
        }

//        bool isAllReachTarget(const PathPointWithNodePtrs<N> & paths, DynamicDataOfSearchWithNodePtr<N>& data, const Time& current_time) {
//            for(auto & path : paths) {
//                if(data->dist_to(path->last_node_, false, current_time) == MAX<PathLen>) return false;
//            }
//            return true;
//        }

        // mark travelsable edge during, return which reach target
        PathPointWithNodePtrs<N> createInitPathPtrsWithNode(const Pointi<N>& start,
                                                            bool is_from_start,
                                                            // not use tg.etcs_ for some acceleration
                                                            // for example, ETC_NoUselessPoint3 is useless in this section
                                                            const PointTransferConstraints<N>& ptcs,
                                                            const EdgeTransferConstraints3<N>& etcs);


        bool isReachTarget(const RoadMapNodeTraitPtr<N> & node, const Time& current_time) {
            if(data_->close_node_to(node, false, currentTime()) != MAX<NodeId> || data_->dist_to(node, false, currentTime()) == 0) {
                return true;
            }
            return false;
        }

        bool isReachTarget(const PathPointWithNodePtr<N> & path, const Time& current_time) {
            return isReachTarget(path->last_node_, current_time);
        }

        // if minimum_reach_count > 0, return true when minimum_reach_count paths reach target
        bool isAllReachTarget(const RoadMapNodeTraitPtrs<N>& nodes, int minimum_reach_count, const Time& current_time) {
            int reach_count = 0;
            for (const auto &node : nodes) {
                if(!isReachTarget(node, current_time)) { if(minimum_reach_count < 0) return false; }
                else { reach_count++; }
                if(minimum_reach_count > 0 && reach_count >= minimum_reach_count) { return true; }
            }
            // if not all reach target and reach count < minimum_reach_count, return false
            if(minimum_reach_count > 0 && reach_count < nodes.size() && reach_count < minimum_reach_count) { return false; }
            return true;
        }

        // if minimum_reach_count > 0, return true when minimum_reach_count paths reach target
        bool isAllReachTarget(const PathPointWithNodePtrs<N>& paths, int minimum_reach_count, const Time& current_time) {
            int reach_count = 0;
            for (const auto &path : paths) {
                if(!isReachTarget(path->last_node_, current_time)) {
                    if(minimum_reach_count < 0) {
                        //std::cout << " reach_count 1 = " << reach_count << std::endl;
                        return false;
                    }
                }
                else {
                    reach_count++;
                    //std::cout << " reach_count++ when " << path->last_edge_->printStr(true, true) << std::endl;
                }
                if(minimum_reach_count > 0 && reach_count >= minimum_reach_count) {
//                std::cout << " AllReachTarget 1 " << std::endl;
//                std::cout << " minimum_reach_count " << minimum_reach_count << std::endl;
//                std::cout << " reach_count " << reach_count << std::endl;

                    return true;
                }
            }
            //std::cout << " reach_count 2 = " << reach_count << std::endl;
            // if not all reach target and reach count < minimum_reach_count, return false
            if(minimum_reach_count > 0 && reach_count < paths.size() && reach_count < minimum_reach_count) {
                return false;
            }
            //std::cout << " AllReachTarget 2 " << std::endl;
            return true;
        }

        // path search must follow ptcs and etcs, because the tangent graph is defined by them
        // but ics is arbitrary/
        // 11.08: this way may be more suit for depth first mode ?
        // will search more hyper edge than raw BFS, reach target too late
        // TODO: add considering start/target is occupied
        ExitCode planningWithNode(const Pointi<N> &start,
                                  const Pointi<N> &target,
                                  const PointTransferConstraints<N>& ptcs,
                                  const EdgeTransferConstraints3<N>& etcs,
                                  const NodeIterationConstraints<N, GridPtr<N> >& ics,
                                  std::vector<Pointis<N> > &paths,
                                  bool edge_mode = true,
                                  bool global_shortest = true,
                                  bool depth_first = false,
                                  int first_reach_count = -1) {
            statistic_.clear();
            output_stream_.clear();
            paths.clear();
            edge_mode_         = edge_mode,
            global_shortest_   = global_shortest ,
            depth_first_       = depth_first,
            first_reach_count_ = first_reach_count;
            shortest_path_node_ = nullptr;
            global_minimum_path_length = std::numeric_limits<PathLen>::max();

            ExitCode ec = ExitCode::FAILED;
            auto dimension = tg_->surface_processor_->getDimensionInfo();
            initialise();
            std::stringstream ss;
            ss << "RJ_" << (first_reach_count > 0 ? first_reach_count : 0) << " ";
            for(int i=0; i<N; i++) {
                ss << start[i] << " ";
            }
            for(int i=0; i<N; i++) {
                ss << target[i] << " ";
            }
            /* create initial path */
            gettimeofday(&tv_pre, &tz);
            /* check direct continuity */
            Pointis<N-1> neightbor = GetNeightborOffsetGrids<N-1>();
            bool direct_visible = false;
            if(tg_->surface_processor_->is_occupied_(start) || tg_->surface_processor_->is_occupied_(target)) {
                statistic_.clear();
                statistic_.push_back(MAX<PathLen>);
                statistic_.push_back(0);
                statistic_.push_back(0);
                statistic_.push_back(0);
                statistic_.push_back(0);
                for(const auto & data : statistic_) {
                    ss << data << " ";
                }
                output_stream_ = ss.str();
                return ExitCode::FAILED;
            }
            if(start == target || !tg_->surface_processor_->lineCrossObstacle(start, target)) {
                //if(0) {
                //std::cout << " start " <<  start << " visible to target " << target << std::endl;
                bool is_legal = true;
                // TODO: should not add general tangent constraint, just point to point constraint
                if(is_legal) {
                    gettimeofday(&tv_after, &tz);
                    Path<N> path;
                    path.clear();
                    path.push_back(start);
                    path.push_back(target);
                    statistic_.clear();
                    statistic_.push_back(calculatePathLength(path));
                    paths.push_back(path);
                    double initial_path_cost =
                            (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                    statistic_.push_back(0);
                    statistic_.push_back(initial_path_cost);
                    statistic_.push_back(0);
                    statistic_.push_back(1);
                    ec = ExitCode::SUCCESS;
                }
                direct_visible = true;
            }
            if(!direct_visible// || !global_shortest_
            ) {

                GridPtr<N> start_ptr  = std::make_shared<Grid<N>>(PointiToId(start, dimension), start, false),
                        target_ptr = std::make_shared<Grid<N>>(PointiToId(target, dimension), target,false);

                const auto& initial_paths = createVisiblePathPointPtrsWithNode(start, target, ptcs, etcs);
                gettimeofday(&tv_after, &tz);
                //std::cout << " after create init path" << std::endl;
                double initial_path_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //std::cout << " initial_path_cost = " << initial_path_cost << std::endl;
                /* search path */
                RoadMapNodeTraitDeques<N> final_paths;
                gettimeofday(&tv_pre, &tz);
                if(!initial_paths.empty()) {
                    if (depth_first_) {
                        ec = DepthFirstSearchWithNode(initial_paths, final_paths, target_ptr);
                    } else {
                        ec = BreadthFirstSearchWithNode(initial_paths, ics, final_paths, target_ptr);
                    }
                } else {
                    ec == ExitCode::FAILED;
                }
                gettimeofday(&tv_after, &tz);
                double search_path_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //std::cout << " search_path_cost = " << search_path_cost << std::endl;
                /* fill statistic_ */
                statistic_.clear();
                Path <N> path;
                if(direct_visible) {
                    path.clear();
                    path.push_back(start);
                    path.push_back(target);
                    paths.push_back(path);
                }
                if (ec == ExitCode::SUCCESS && !final_paths.empty()) {
                    /* fill path */
                    PathLen total_path_len;
                    for(const auto& final_path : final_paths) {
                        // save time
//                        if(!PathContinuousCheck(final_path)) {
//                            std::cout << " FATAL: un-continue path " << final_path[0]->printStr(true, true) << std::endl;
//                        }
                        //printPathDequeLevel(final_path);
                        path = ContinuousNodesToPath(final_path);
                        total_path_len += calculatePathLength(path);
                        paths.push_back(path);
                    }
                    path = ContinuousNodesToPath(final_paths[0]);
                    statistic_.push_back(total_path_len/final_paths.size());//calculatePathLength(path));
                    //std::cout << " final path len = " << statistic_.back() << std::endl;
                } else if(!direct_visible) {
                    std::cout << " FATAL: RJ without path planning failed " << std::endl;
                    // infinity path length means search path failed
                    statistic_.push_back(MAX<PathLen>);
                } else {
                    statistic_.push_back(calculatePathLength(path));
                }
                statistic_.push_back(0);
                statistic_.push_back(initial_path_cost);
                statistic_.push_back(search_path_cost);
                statistic_.push_back(paths.size()); // path count
            }
            for(const auto & data : statistic_) {
                ss << data << " ";
            }
            output_stream_ = ss.str();
            //std::cout << "RJ polygon length  = " << statistic_[0] << std::endl;
            //std::cout << "reset graph  cost  = " << statistic_[1] << "ms" << std::endl;
            //std::cout << "initialize   cost  = " << statistic_[2] << "ms" << std::endl;
            //std::cout << "search graph cost  = " << statistic_[3] << "ms" << std::endl;
            return ec;
        };


        const Statistic& getStatistic() const { return statistic_; }

        const OutputStream& getOutputStream() const { return output_stream_; }


        template <typename TARGET_TYPE>
        ExitCode BreadthFirstSearchWithNode(const PathPointWithNodePtrs<N> &init_set,
                                            const NodeIterationConstraints<N, TARGET_TYPE> &ics,
                                            RoadMapNodeTraitDeques<N> &final_set, // resulted path set
                                            const TARGET_TYPE& target
        );


        template<typename TARGET_TYPE>
        ExitCode DepthFirstSearchWithNode(const PathPointWithNodePtrs<N> &init_set,
                                          RoadMapNodeTraitDeques<N> &final_set, // resulted path set
                //const IterationConstraints<N, TARGET_TYPE> &ics, // when dfs, only considering ics ?
                                          const TARGET_TYPE& target
        );

        RoadMapGraphPtr<N> tg_;

        bool edge_mode_; // search in edge mode

        bool global_shortest_; // only need global shortest path

        bool depth_first_; // in BFS mode or DFS mode

        int first_reach_count_; //

        double max_time_cost_ = 5e3;

        AutoSortHeap<PathLen, RoadMapNodeTraitPtr<N> > current_set_; // priority queue for DFS
        RoadMapNodeTraitPtr<N> shortest_path_node_ = nullptr;

        DynamicDataOfSearchWithNodePtr<N> data_;

        const Time & currentTime() {
            return current_time_;
        }

    private:

        GridPtr<N> closest_grid_;

        PathLen heuristic_length_;

        /* time cost statistic_ */
        struct timezone tz;
        struct timeval tv_pre, tv_pre1;
        struct timeval tv_after, tv_after1;

        Statistic statistic_;
        OutputStream output_stream_;

        Time current_time_ = 0;

    };


}

#endif //FREENAV_SEARCH_PATH_WITH_NODE_H
