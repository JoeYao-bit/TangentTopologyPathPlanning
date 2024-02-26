//
// Created by yaozhuo on 2022/1/2.
//

#ifndef FREENAV_TOPO_TANGENT_GRAPH_H
#define FREENAV_TOPO_TANGENT_GRAPH_H

#include <map>
#include <limits>
#include <memory>
#include <deque>

#include "../../freeNav-base/basic_elements/point.h"
#include "../../freeNav-base/basic_elements/surface_process.h"

namespace freeNav::Topo {

    template <Dimension N>
    struct RoadMapNodeTrait;

    template <Dimension N> class RoadMapGraphBuilder;

    template <Dimension N>
    using RoadMapNodeTraitPtr = std::shared_ptr<RoadMapNodeTrait<N> >;

    template <Dimension N>
    using RoadMapNodeTraitPtrs = std::vector<RoadMapNodeTraitPtr<N> >;

    /* edge definition */
    template <Dimension N>
    struct RoadMapEdgeTrait;

    template <Dimension N>
    struct RoadMapGraph;

    template <Dimension N>
    using RoadMapGraphPtr = std::shared_ptr<RoadMapGraph<N> >;

    template <Dimension N>
    struct HyperLoopEdgeWithLength;

    template <Dimension N>
    using RoadMapEdgeTraits = std::vector<RoadMapEdgeTrait<N> >;

    template <Dimension N>
    using RoadMapEdgeTraitPtr = std::shared_ptr<RoadMapEdgeTrait<N> >;

    template <Dimension N>
    using RoadMapEdgeTraitPtrs = std::vector<RoadMapEdgeTraitPtr<N> >;

    template <Dimension N>
    using RoadMapEdgeTraitDeque = std::deque<RoadMapEdgeTraitPtr<N> >;

    template <Dimension N>
    using RoadMapNodeTraitDeque = std::deque<RoadMapNodeTraitPtr<N> >;

    template <Dimension N>
    using RoadMapEdgeTraitDeques = std::vector<RoadMapEdgeTraitDeque<N> >;

    template <Dimension N>
    using RoadMapNodeTraitDeques = std::vector<RoadMapNodeTraitDeque<N> >;

    template <Dimension N>
    using RoadMapEdgeTraitPtrss = std::vector<RoadMapEdgeTraitPtrs<N> >;

    /* node definition */

    template <Dimension N>
    using RoadMapNodes = std::vector<RoadMapNodeTrait<N> >;

    template <Dimension N>
    using RoadMapNodePtr = RoadMapNodeTraitPtr<N>;

    template <Dimension N>
    using RoadMapNodePtrs = std::vector<RoadMapNodePtr<N> >;

    /* hyper edge definition */

    template <Dimension N> struct HyperLoopEdgeWithLength;

    template <Dimension N>
    struct DynamicDataOfSearchWithEdge;

    template<Dimension N>
    using DynamicDataOfSearchWithEdgePtr = std::shared_ptr<DynamicDataOfSearchWithEdge<N> >;

    template <Dimension N>
    struct DynamicDataOfSearchWithNode;

    template<Dimension N>
    using DynamicDataOfSearchWithNodePtr = std::shared_ptr<DynamicDataOfSearchWithNode<N> >;

    template <Dimension N>
    using HyperLoopEdgeWithLengthPtr = std::shared_ptr<HyperLoopEdgeWithLength<N> >;

    template <Dimension N>
    using HyperLoopEdgeWithLengths = std::vector<HyperLoopEdgeWithLength<N> >;

    template <Dimension N>
    using HyperLoopEdgeWithLengthPtrs = std::vector<HyperLoopEdgeWithLengthPtr<N> >;

    /* constraints definition */

    /* the constraint to determine whether there is tangent point in two surface grid */
    template <Dimension N>
    using PointTransferConstraints = std::vector<bool (*)(RoadMapGraphPtr<N>& tg, const GridPtr<N>& tc1, const GridPtr<N>& tc2, IS_OCCUPIED_FUNC<N> is_occupied, const Pointis<N - 1>& neightbor)>;

    /* the constraint to determine whether we cna transfer from a tangent line to another  */
    template <Dimension N>
    using EdgeTransferConstraints = std::vector<bool (*)(RoadMapGraphPtr<N>& tg, const RoadMapEdgeTraitPtr<N>& edge1, const RoadMapEdgeTraitPtr<N>& edge2)>;

    template <Dimension N>
    using EdgeTransferConstraints3 = std::vector<bool (*)(RoadMapGraphPtr<N>& tg, const RoadMapNodePtr<N>& node1, const RoadMapNodePtr<N>& node2, const RoadMapNodePtr<N>& node3)>;

    template <Dimension N> class PathPointWithEdge;

    template <Dimension N>
    using PathPointWithEdgePtr = std::shared_ptr<PathPointWithEdge<N> >;


    template <Dimension N>
    std::string toStr(const RoadMapEdgeTraitPtrs<N>& edges, bool verbose = true) {
        std::stringstream ssr;
        for(const auto& edge : edges) {
            ssr << "{" << edge->next_node(false)->sg_->pt_ << "->" << edge->next_node(true)->sg_->pt_ << "}->";
        }
        return ssr.str();
    }

    template <Dimension N>
    std::string toStr(const RoadMapEdgeTraitDeque<N>& edges, bool verbose = true) {
        std::stringstream ssr;
//        for(const auto& edge : edges) {
//            ssr << "{" << edge->nextNode(false)->sg_->pt_ << "->" << edge->next_node(true)->sg_->pt_ << "}->";
//        }
        return ssr.str();
    }

    template<Dimension N>
    void printPathLevel(const RoadMapEdgeTraitPtrs<N>& path) {
        std::cout << " path level = ";
        for(const auto& edge : path) {
            if(edge->isLoopEdge()) { std::cout << "INF "; }
            else { std::cout << edge->level_ << " "; }
            if(!edge->isLoopEdge() &&
               !edge->is_marked(true, true, 0) && !edge->is_marked(true, false, 0)
                    ) {
                std::cout << " UM / ";
            } else {
                std::cout << "/ ";
            }
        }
        std::cout << std::endl;
    }

    template<Dimension N>
    void printPathDequeLevel(const RoadMapEdgeTraitDeque<N>& path) {
        std::cout << " path level = ";
        for(const auto& edge : path) {
            if(edge->isLoopEdge()) { std::cout << "INF "; }
            else { std::cout << edge->level_ << " "; }
            if(!edge->isLoopEdge() &&
               !edge->is_marked(true, true, 0) && !edge->is_marked(true, false, 0)
                    ) {
                std::cout << " UM / ";
            } else {
                std::cout << "/ ";
            }
        }
        std::cout << std::endl;
    }

    // TODO: hyper-graph that consists of edges that has more than one branch,
    //       edges that has the same topology should be merge in to the shortest one
    template <Dimension N>
    struct RoadMapGraph {
    public:

        RoadMapGraph(const PointTransferConstraints<N>& unordered_ptcs,
                     const PointTransferConstraints<N>& ordered_ptcs,
                     const EdgeTransferConstraints3<N>& etcs) {
            unordered_ptcs_ = unordered_ptcs;
            ordered_ptcs_   = ordered_ptcs;
            etcs_ = etcs;
        }

        static EdgeId reverse(const EdgeId& edge_id) {
            return edge_id % 2 == 1 ? edge_id - 1 : edge_id + 1;
        }

        bool isUndirectedGraph() const {
            return ordered_ptcs_.empty();
        }

        bool UndirectedGraphCheck() {
            if(!isUndirectedGraph()) return false;
            if(edges_.size() % 2 != 0) return false;
            for(int i=0; i < edges_.size(); i += 2) {
                if(edges_[i] == nullptr) { return false; }
            }
            return true;
        }

        RoadMapGraph() = delete;

        void pruneToStaticNodeAndEdge() {
            nodes_.resize(static_node_count_);
            edges_.resize(static_edge_count_);
        }

        void getStatistics() {
            std::cout << " surface grid size " << surface_processor_->surface_grids_.size() << std::endl
                      << " node size " << surface_processor_->tangent_candidates_.size() << std::endl
                      << " raw edge size " << raw_edge_count_ << std::endl
                      << " edge size " << edges_.size() << std::endl
            ;
        }

        /* interfaces used when edge connection is precomputed */
        inline EdgeIds & nextEdges(const RoadMapNodeTraitPtr<N>& node, const bool& is_start) {
            return node->nextEdges(is_start);
        }

        inline EdgeIds & nextEdgesOfNode(const NodeId& node_id, const bool& is_start) {
            return nextEdges(nodes_[node_id], is_start);
        }

        inline EdgeIds & nextEdges(const RoadMapEdgeTraitPtr<N>& edge, const bool& is_start, const bool& on_edge) {
            if(on_edge) {
                return edge->nextEdges(is_start);
            } else {
                return nextEdges(nodes_[edge->nextNode(is_start)], is_start);
            }
        }

        inline EdgeIds & nextEdgesOfEdge(const EdgeId& edge_id, const bool& is_start, const bool& on_edge) {
            if(on_edge) {
                return edges_[edge_id]->nextEdges(is_start);
            } else {
                return nextEdges(nodes_[edges_[edge_id]->nextNode(is_start)], is_start);
            }
        }

        inline EdgeIds & nextEdges(const HyperLoopEdgeWithLengthPtr<N>& hyper_edge, const bool& is_start, const bool& on_edge) {
            if(on_edge) {
                return nextEdges(edges_[hyper_edge->nextEdge(is_start)], is_start);
            } else {
                return nextEdges(nodes_[edges_[hyper_edge->nextEdge(is_start)]->nextNode(is_start)], is_start);
            }
        }

        /* interfaces used when edge connection is precomputed */
        inline NodeIds & nextNodes(const RoadMapNodeTraitPtr<N>& node) {
            return node->nextNodes();
        }

        inline NodeId& nextNode(const HyperLoopEdgeWithLengthPtr<N>& hyper_edge, bool is_from_start) {
            return hyper_edge->nextEdge()->nextNode(is_from_start);
        }

        inline NodeId& nextNode(const RoadMapEdgeTraitPtr<N>& edge, bool is_from_start) {
            return edge->nextNode(is_from_start);
        }

        inline NodeId& nextNodeOfEdge(const EdgeId& edge_id, bool is_from_start) {
            return edges_[edge_id]->nextNode(is_from_start);
        }

        PointTransferConstraints<N> unordered_ptcs_;
        PointTransferConstraints<N> ordered_ptcs_;
        EdgeTransferConstraints3<N> etcs_;

        int raw_edge_count_ = 0;

        // dynamic updated edge in the tangent graph, for timely graph node reset

        SurfaceProcessorPtr<N> surface_processor_;

        NodeId static_node_count_ = 0;

        RoadMapNodePtrs<N> nodes_; // original edges

        NodeId static_edge_count_ = 0;

        RoadMapEdgeTraitPtrs<N> edges_; // edges, nullptr if existing ordered ptcs

        HyperLoopEdgeWithLengthPtrs<N> hyper_edges_;

    };

    template <Dimension N>
    struct HyperLoopEdgeWithLength {
    public:

        explicit HyperLoopEdgeWithLength(const EdgeId& edge_head, const PathLen& length)
        : edge_head_(edge_head), length_(length) {}

        const EdgeId& edgeHeadId() const {
            return edge_head_;
        }

        const EdgeId& edgeId() const {
            return hyper_id_;
        }

        const PathLen& length() const {
            return length_;
        }

    private:

        EdgeId edge_head_;
        EdgeId hyper_id_;
        PathLen length_;

        friend class RoadMapGraph<N>;
        friend class RoadMapGraphBuilder<N>;
        friend class DynamicDataOfSearchWithEdge<N>;


    };

    // Id means the id the unique id of edge, combination of its start and target
    //template <Dimension N>
    //using HyperLoopEdgeNodes = std::map<Id, HyperLoopEdgeWithLengthPtr<N> >;

    // use when there is precomputation of edge connection
    template <Dimension N>
    struct RoadMapEdgeTrait {

    public:

        /* do not change this once it was generated */
        explicit RoadMapEdgeTrait(const NodeId& from,
                                  const NodeId& to,
                                  const PathLen& length)
                                  : from_(from), to_(to) {
            length_ = length;
        }

        inline bool isLoopEdge() const {
            return level_ == MAX<Lv>;
        }

//        inline EdgeId& hyperEdgeId() {
//            return hyper_edge_id_;
//        }

        inline bool& isHyperLoopEdge() {
            return is_hyper_edge_node_;
        }


        inline Lv& level() {
            return level_;
        }

        inline const PathLen& length() const {
            return length_;
        }

        // which node visible to start/target, if current edge contain start/target
        inline const NodeId& nextNode(const bool& is_start) const {
            return is_start ? to_ : from_;
        }

        inline EdgeIds& nextEdges(const bool& is_start) {
            return is_start ? next_edges_ : pre_edges_;
        }

        inline EdgeIds& nextHyperLoopEdges(const bool& is_start) {
            return is_start ? next_hyper_loop_edges_ : pre_hyper_loop_edges_;
        }

        inline EdgeIds& nextLoopEdges(const bool& is_start) {
            return is_start ? next_loop_edge_nodes_ : pre_loop_edge_nodes_;
        }

        inline EdgeId& edgeId() {
            return edge_id_;
        }

    private:

        /* Edge-N-Level constraint information for a edge */
        Lv level_ = -1; // Edge-N-Level from ENL-SVG, -1 for unvisited, MAX<Lv> for loop edge

        // hyper graph config
        // if current is not a hyper node, there is only next hyper node AND one pre hyper node
        // if current is a hyper node, there is more than one next hyper node OR more than one pre hyper node
        bool is_hyper_edge_node_ = false; // hyper means W >= 3, got more than one pre or next loop edge

        //EdgeId hyper_edge_id_ = MAX<EdgeId>;

        /* basic information for a edge */
        NodeId from_; // which vertex that edge connect

        NodeId to_; // which vertex that edge connect

        PathLen length_; // the length of current tangent line

        EdgeId edge_id_  = MAX<EdgeId>;

        /* all the legal edge that the edge can jump as next edges */
        //RoadMapEdges<N> next_edges_; // first is the to_ node id
        EdgeIds next_edges_;

        /* all the edges that the edge can jump to current edge as legal next edge */
        //RoadMapEdges<N> pre_edges_; // first is the compound of from_ and to_ node id
        EdgeIds pre_edges_;

        // the following four could be removed if use more calculations
        EdgeIds next_loop_edge_nodes_;
        EdgeIds pre_loop_edge_nodes_;

        EdgeIds next_hyper_loop_edges_;
        EdgeIds pre_hyper_loop_edges_;

        friend class RoadMapGraph<N>;
        friend class RoadMapGraphBuilder<N>;
        friend class DynamicDataOfSearchWithEdge<N>;

    };


    template <Dimension N>
    struct RoadMapNodeTrait{

    public:

        GridPtr<N> sg_;

        inline EdgeIds& nextEdges(const bool& is_start) {
            return is_start ? split_edges_ : converge_edges_;
        }

        // equal split edges
        inline NodeIds& nextNodes() {
            return visible_nodes_;
        }

        inline bool & isInHyperGraph() {
            return is_in_hyper_graph_;
        }

    private:

        // edges that use current node as start
        EdgeIds split_edges_ = {}; // use when there is precomputed edge connection

        // edges that set current node as target
        EdgeIds converge_edges_ = {}; // use when there is precomputed edge connection

        // edges from current to visible nodes that satisfied PTCs
        NodeIds visible_nodes_ = {}; // use when no precomputed edge connection

        bool is_in_hyper_graph_ = false;

        friend class RoadMapGraph<N>;
        friend class RoadMapGraphBuilder<N>;
        friend class DynamicDataOfSearchWithEdge<N>;
    };

}

#endif //FREENAV_TANGENT_GRAPH_H
