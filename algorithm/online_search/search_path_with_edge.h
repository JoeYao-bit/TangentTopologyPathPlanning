//
// Created by yaozhuo on 2022/6/16.
//

#ifndef FREENAV_TOPO_SEARCH_PATH_WITH_EDGE_H
#define FREENAV_TOPO_SEARCH_PATH_WITH_EDGE_H

#include "../../algorithm/graph_construction/tangent_graph_build.h"
#include "../../algorithm/constraints/iteration_constraints.h"
//#include "path_tree.h"
#include "../../freeNav-base/dependencies/self_sorted_queue.h"
#include <iomanip>
#include <limits>
#include "dynamic_data_with_edge.h"
#include "path_tree_with_node.h"

namespace freeNav::Topo {

    /*
    * define the exit state of RimJump
    * */
    enum ExitCode {
        SUCCESS                   = 0, // find at least one legal path before times out
        SUCCESS_TIMES_OUT         = 1, // find at least one legal path when reach maximum times
        FAILED                    = 2, // find no legal result before time is out
        FAILED_TIMES_OUT          = 3, // reach maximum times of iteration and find no legal result
        ILLEGAL_INPUT_GRAPH       = 4, // illegal input
        ILLEGAL_INPUT_INITIAL_SET = 5, // illegal state input
        ILLEGAL_INPUT_TARGET_SET  = 6, // illegal target input
        ILLEGAL_INPUT_CONSTRAINT  = 7, // ???
    };

    typedef std::vector<double> Statistic; // a method's Statistic

    typedef std::vector<Statistic> StatisticS;  // multiple method's Statistic

    typedef std::vector<StatisticS> StatisticSS;  // multiple experiment 's multiple method's Statistic

    typedef std::string OutputStream;  // a method's output stream

    typedef std::vector<OutputStream> OutputStreamS; // multiple method's output stream

    typedef std::vector<OutputStreamS> OutputStreamSS; // multiple experiment 's multiple method's output stream

    template <Dimension N>
    class GeneralGraphPathPlannerWithEdge {

    public:

        explicit GeneralGraphPathPlannerWithEdge(RoadMapGraphPtr<N>& tg) : tg_(tg),
                                                                           current_set_(tg->edges_.size()) {
            data_ = std::make_shared<DynamicDataOfSearchWithEdge<N> >(tg);
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
            //data_ = std::make_shared<DynamicDataOfSearchWithEdge<N> >(tg_);
        }

        void markEdge(const RoadMapEdgeTraitPtr<N>& edge_ptr, bool is_from_start);

        PathPointWithEdgePtrs<N> createVisiblePathPointPtrsWithEdge(const Pointi<N>& start,
                                                                    const Pointi<N>& target,
                                                                    const PointTransferConstraints<N>& ptcs,
                                                                    const EdgeTransferConstraints3<N>& etcs);

        std::pair<PathPointWithEdgePtrs<N>, PathPointWithEdgePtrs<N>> createVisiblePathPointPtrsPairWithEdge(const Pointi<N>& start,
                                                                                                             const Pointi<N>& target,
                                                                                                             const PointTransferConstraints<N>& ptcs,
                                                                                                             const EdgeTransferConstraints3<N>& etcs);

        // mark travelsable edge during, return which reach target
        PathPointWithEdgePtrs<N> createInitPathPtrsWithEdge(const Pointi<N>& start,
                                                            bool is_from_start,
                // not use tg.etcs_ for some acceleration
                // for example, ETC_NoUselessPoint3 is useless in this section
                                                         const PointTransferConstraints<N>& ptcs,
                                                            const EdgeTransferConstraints3<N>& etcs);

        PathPointWithEdgePtrs<N> markInitEdges(const PathPointWithEdgePtrs<N>& init_paths,
                                               bool is_from_start,
                                               const EdgeTransferConstraints3<N>& etcs);

        bool isReachTarget(const RoadMapEdgeTraitPtr<N> & edge, bool is_edge, const Time& current_time) {
            // as the final edge dist to target is 0, but close_to_target_edge == nullptr
//        if(edge->close_to_target_edge(is_edge) != nullptr || edge->min_length_to_target(is_edge) == 0) {
//            return true;
//        }
            if(data_->dist_to(edge, false, is_edge, current_time) != MAX<PathLen>) {
                return true;
            }
            return false;
        }

        bool isReachTarget(const PathPointWithEdgePtr<N> & path, bool is_edge, const Time& current_time) {
            return isReachTarget(path->last_edge_, is_edge, current_time);
        }

        // if minimum_reach_count > 0, return true when minimum_reach_count paths reach target
        bool isAllReachTarget(const RoadMapEdgeTraitPtrs<N>& edges, int minimum_reach_count, bool is_edge, const Time& current_time) {
            int reach_count = 0;
            for (const auto &edge : edges) {
                if(!isReachTarget(edge, is_edge, current_time)) { if(minimum_reach_count < 0) return false; }
                else { reach_count++; }
                if(minimum_reach_count > 0 && reach_count >= minimum_reach_count) { return true; }
            }
            // if not all reach target and reach count < minimum_reach_count, return false
            if(minimum_reach_count > 0 && reach_count < edges.size() && reach_count < minimum_reach_count) { return false; }
            return true;
        }

        // if minimum_reach_count > 0, return true when minimum_reach_count paths reach target
        bool isAllReachTarget(const PathPointWithEdgePtrs<N>& paths, int minimum_reach_count, bool is_edge, Time current_time) {
            int reach_count = 0;
            for (const auto &path : paths) {
                if(!isReachTarget(path->last_edge_, is_edge, current_time)) { if(minimum_reach_count < 0) return false; }
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
            //std::cout << " reach_count = " << reach_count << std::endl;
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
        ExitCode planningWithEdge(const Pointi<N> &start,
                                       const Pointi<N> &target,
                                       const PointTransferConstraints<N>& ptcs,
                                       const EdgeTransferConstraints3<N>& etcs,
                                       const IterationConstraints<N, GridPtr<N> >& ics,
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
            global_minimum_path_length = std::numeric_limits<PathLen>::max();

            ExitCode ec = ExitCode::FAILED;
            auto dimension = tg_->surface_processor_->getDimensionInfo();
            initialise();
            std::stringstream ss;
            ss << "RJ ";
            for(int i=0; i<N; i++) {
                ss << start[i] << " ";
            }
            for(int i=0; i<N; i++) {
                ss << target[i] << " ";
            }
            /* create initial path */
            gettimeofday(&tv_pre, &tz);
            /* check direct continuity */
            freeNav::Pointis<N-1> neightbor = GetNeightborOffsetGrids<N-1>();
            if(tg_->surface_processor_->is_occupied_(start) || tg_->surface_processor_->is_occupied_(target)) {
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
            } else {

                GridPtr<N> start_ptr  = std::make_shared<Grid<N>>(PointiToId(start, dimension), start, false),
                        target_ptr = std::make_shared<Grid<N>>(PointiToId(target, dimension), target,false);

                const auto& initial_paths = createVisiblePathPointPtrsWithEdge(start, target, ptcs, etcs);
                //std::cout << " initial_start_edges size " << initial_paths.size() << std::endl;
                gettimeofday(&tv_after, &tz);
                //std::cout << " after create init path" << std::endl;
                double initial_path_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //std::cout << " initial_path_cost = " << initial_path_cost << std::endl;
                /* search path */
                RoadMapEdgeTraitDeques<N> final_paths;
                gettimeofday(&tv_pre, &tz);
                if(!initial_paths.empty()) {
                    if (depth_first_) {
                        ec = DepthFirstSearchWithEdge(initial_paths, final_paths, target_ptr);
                    } else
                    {
                        ec = BreadthFirstSearchWithEdge(initial_paths, ics, final_paths, target_ptr);
                    }
                } else {
                    ec == ExitCode::FAILED;
                }
                gettimeofday(&tv_after, &tz);
                double search_path_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
                //std::cout << " search_path_cost = " << search_path_cost << std::endl;
                /* fill statistic_ */
                statistic_.clear();
                if (ec == ExitCode::SUCCESS && !final_paths.empty()) {
                    /* fill path */
                    Path <N> path;
                    for(const auto& final_path : final_paths) {
                        // save time
//                        if(!PathContinuousCheck(final_path)) {
//                            std::cout << " FATAL: un-continue path " << final_path[0]->printStr(true, true) << std::endl;
//                        }
                        //printPathDequeLevel(final_path);
                        path = ContinuousEdgesToPath(tg_, final_path);
                        paths.push_back(path);
                    }
                    path = ContinuousEdgesToPath(tg_, final_paths[0]);
                    statistic_.push_back(calculatePathLength(path));
                    //std::cout << " final path len = " << statistic_.back() << std::endl;
                } else {
                    std::cout << " FATAL: RJ without path planning failed " << std::endl;
                    // infinity path length means search path failed
                    statistic_.push_back(MAX<PathLen>);
                }
                statistic_.push_back(0);
                statistic_.push_back(initial_path_cost);
                statistic_.push_back(search_path_cost);
                statistic_.push_back(final_paths.size()); // path count
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
        inline PathPointWithEdgePtr<N> generateNextPathsHyper(const PathPointWithEdgePtr<N>& pre_tpt,
                                                              const HyperLoopEdgeWithLengthPtr<N>& candidate_edge,
                                                              const IterationConstraints<N, TARGET_TYPE> &ics,
                                                              const TARGET_TYPE& target);

        template <typename TARGET_TYPE>
        inline PathPointWithEdgePtr<N> generateNextPaths(const PathPointWithEdgePtr<N>& pre_tpt,
                                                         const RoadMapEdgeTraitPtr<N>& candidate_edge,
                                                         const IterationConstraints<N, TARGET_TYPE> &ics,
                                                         const TARGET_TYPE& target);

        inline bool allReachHyper(const PathPoints<N>& pre_set) {
            for(const auto& path_ptr : pre_set) {
                if(!path_ptr->last_edge_->isHyperLoopEdge()) return false;
            }
            return true;
        }

        template <typename TARGET_TYPE>
        ExitCode BreadthFirstSearchWithEdge(const PathPointWithEdgePtrs<N> &init_set,
                                                 const IterationConstraints<N, TARGET_TYPE> &ics,
                                                 RoadMapEdgeTraitDeques<N> &final_set, // resulted path set
                                                 const TARGET_TYPE& target
        );


        template<typename TARGET_TYPE>
        ExitCode DepthFirstSearchWithEdge(const PathPointWithEdgePtrs<N> &init_set,
                                               RoadMapEdgeTraitDeques<N> &final_set, // resulted path set
                                               //const IterationConstraints<N, TARGET_TYPE> &ics, // when dfs, only considering ics ?
                                               const TARGET_TYPE& target
        );

        template<typename TARGET_TYPE>
        EdgeId getNextEdge(const RoadMapEdgeTraitPtr<N> &future_edge_ptr, const RoadMapEdgeTraitPtr<N> & next_edge, const TARGET_TYPE& target);

        template<typename TARGET_TYPE>
        EdgeId getNextHyperEdge(const RoadMapEdgeTraitPtr<N> &future_edge_ptr,
                                                const HyperLoopEdgeWithLengthPtr<N> & next_edge,
                                                const TARGET_TYPE& target);

        RoadMapGraphPtr<N> tg_;

        bool edge_mode_; // search in edge mode

        bool global_shortest_; // only need global shortest path

        bool depth_first_; // in BFS mode or DFS mode

        int first_reach_count_; //

        double max_time_cost_ = 5e3;

        AutoSortHeap<PathLen, RoadMapEdgeTraitPtr<N> > current_set_; // priority queue for DFS

        RoadMapEdgeTraitPtr<N> shortest_path_edge_ = nullptr;

        DynamicDataOfSearchWithEdgePtr<N> data_;

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

#endif //FREENAV_SEARCH_PATH_WITH_EDGE_H
