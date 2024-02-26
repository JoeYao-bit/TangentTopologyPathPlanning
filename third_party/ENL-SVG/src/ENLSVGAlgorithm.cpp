#include "ENLSVGAlgorithm.h"
#include "ENLSVGGraph.h"
#include "LineOfSightScanner.h"
#include "Grid.h"
#include "IndirectHeap.h"
#include <algorithm>
#include "sys/time.h"

namespace Pathfinding { namespace ENLSVG {

    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;

    // negatePar makes parent pointers negative to indicate that the parent is a skip-edge.
    // note that negatePar(negatePar(k)) == k.
    inline VertexID negatePar(VertexID k) {return -k-1;}
    // restorePar returns the parent pointer to its original value.
    inline VertexID restorePar(VertexID k) {return k < 0 ? -k-1 : k;}

    Memory::Memory(const Algorithm& algo):
        nEdges(algo.nEdges()),
        nNodes(algo.nVertices()),
        markedEdges(nEdges),
        pq(nNodes) {
        const size_t nNodes = algo.nVertices();
        nodes.resize(nNodes);
        ticketCheck.resize(nNodes, 0);
        ticketNumber = 1;
    }


    Algorithm::Algorithm(const Grid& grid):
        grid(grid), scanner(grid), graph(grid, scanner) {
    }


    Path Algorithm::computeSVGPath(Memory& memory, const int sx, const int sy, const int ex, const int ey, ParentPtrs* parentPtrs) const {
        if (!memory.validate(graph)) std::cout << "VALIDATION ERROR: MEMORY USED IS INVALID" << std::endl;
        
        // START: SPECIAL CASES - Handle special cases first.
        if (sx == ex && sy == ey) {
            Path path;
            path.push_back(GridVertex(sx, sy));
            return path;
        } else if (grid.lineOfSight(sx, sy, ex, ey)) {
            Path path;
            path.push_back(GridVertex(sx, sy));
            path.push_back(GridVertex(ex, ey));
            return path;
        }
        // END: SPECIAL CASES


        const size_t nNodes = graph.vertices.size();
        memory.initialise();
        IndirectHeap& pq = memory.pq;
        pq.reinitialise();
        gettimeofday(&tv_pre, &tz);

        double goalDistance = POS_INF;
        VertexID goalParent = NO_PARENT;
        { // START: INITIALISATION
            ScannerStacks& data = memory.scannerStacks;
            {
                const VertexID startIndex = graph.nodeIndex(sx, sy);
                // Add all visible neighbours of start vertex
                scanner.computeAllDirNeighbours(data, sx, sy);
                const std::vector<GridVertex>& neighbours = data.neighbours;
                for (size_t i=0; i<neighbours.size(); ++i) {
                    const GridVertex& vn = neighbours[i];
                    VertexID neighbour = graph.nodeIndex(vn.x, vn.y);

                    double dist = grid.euclideanDistance(sx, sy, vn.x, vn.y);
                    memory.setDistance(neighbour, dist);
                    pq.decreaseKey(neighbour, dist + heuristic(neighbour, ex, ey));
                }
                if (startIndex != -1) {
                    // If start vertex is a VG node, mark it as visited so we don't waste time on it.
                    memory.setVisited(startIndex, true);
                }
            }
            {
                const VertexID goalIndex = graph.nodeIndex(ex, ey);
                // Add all visible neighbours of goal vertex
                scanner.computeAllDirNeighbours(data, ex, ey);
                const std::vector<GridVertex>& neighbours = data.neighbours;
                for (size_t i=0; i<neighbours.size(); ++i) {
                    const GridVertex& vn = neighbours[i];
                    VertexID neighbour = graph.nodeIndex(vn.x, vn.y);
                    memory.setEdgeWeightToGoal(neighbour, grid.euclideanDistance(vn.x, vn.y, ex, ey));
                }
                if (goalIndex != -1) {
                    // If goal vertex is a VG node, mark it as visited so we don't waste time on it.
                    memory.setVisited(goalIndex, true);
                }
            }
        } // END: INITIALISATION
        gettimeofday(&tv_after, &tz);
        double INITIALISATION_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        //std::cout << "-- INITIALISATION end in " << INITIALISATION_cost << "ms" << std::endl << std::endl;
        gettimeofday(&tv_pre, &tz);
        while (pq.size() > 0) {
            if (goalDistance <= pq.getMinValue()) {
                break; // Reached Goal.
            }
            VertexID curr = pq.popMinIndex();
            const double currDistance = memory.distance(curr);
            const VertexID currParent = restorePar(memory.parent(curr));
            memory.setVisited(curr, true);

            const std::vector<EdgeID>& neighbours = graph.edgeLists[curr];
            for (size_t i=0;i<neighbours.size();++i) {
                const auto& edge = graph.edges[neighbours[i]];
                VertexID dest = edge.destVertex;
                if (memory.visited(dest)) continue;
                double weight = graph.weight(edge);

                double destDistance = currDistance + weight;
                if (destDistance < memory.distance(dest) && isTaut(currParent, curr, dest)) {
                    memory.setParent(dest, curr);
                    memory.setDistance(dest, destDistance);
                    pq.decreaseKey(dest, destDistance + heuristic(dest, ex, ey));
                }
            }

            if (memory.edgeWeightToGoal(curr) != -1.0) {
                double destDistance = currDistance + memory.edgeWeightToGoal(curr);
                if (destDistance < goalDistance) {
                    goalDistance = destDistance; // heuristic of goal should be 0
                    goalParent = curr;
                }
            }
        }
        gettimeofday(&tv_after, &tz);
        double computeSVGPath_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        //std::cout << "-- computeSVGPath end in " << computeSVGPath_cost << "ms" << std::endl << std::endl;
        if (parentPtrs != nullptr) setParentPointers(memory, goalParent, sx, sy, ex, ey, parentPtrs);
        return getPath(memory, goalParent, sx, sy, ex, ey);
    }



    Path Algorithm::computePath(Memory& memory, const int sx, const int sy, const int ex, const int ey,
                                double& initial_time, double& search_graph_time,
                                ParentPtrs* parentPtrs) const {
        gettimeofday(&tv_pre, &tz);
        if (!memory.validate(graph)) std::cout << "VALIDATION ERROR: MEMORY USED IS INVALID" << std::endl;

        // START: SPECIAL CASES - Handle special cases first.
        if (sx == ex && sy == ey) {
            Path path;
            path.push_back(GridVertex(sx, sy));
            return path;
        } else if (grid.lineOfSight(sx, sy, ex, ey)) {
            Path path;
            path.push_back(GridVertex(sx, sy));
            path.push_back(GridVertex(ex, ey));
            return path;
        }
        // END: SPECIAL CASES

        const size_t nNodes = graph.vertices.size();
        memory.initialise();
        IndirectHeap& pq = memory.pq;
        pq.reinitialise();

        // yz: before initialization, there is no marked edges
        MarkedEdges& markedEdges = memory.markedEdges;
        markedEdges.clear();

        double goalDistance = POS_INF;
        VertexID goalParent = NO_PARENT;
        { // START: INITIALISATION
            ScannerStacks& data = memory.scannerStacks;
            {
                // transfer coordinate to id index
                const VertexID startIndex = graph.nodeIndex(sx, sy);
                // Add all visible neighbours of start vertex
                scanner.computeAllDirNeighbours(data, sx, sy);
                const std::vector<GridVertex>& neighbours = data.neighbours;
                //std::cout << " ENLSVG start neighbours.size() " << neighbours.size() << std::endl;
                for (size_t i=0; i<neighbours.size(); ++i) {
                    const GridVertex& vn = neighbours[i];
                    VertexID neighbour = graph.nodeIndex(vn.x, vn.y);

                    double dist = grid.euclideanDistance(sx, sy, vn.x, vn.y);
                    memory.setDistance(neighbour, dist);
                    // yz: insert init edges to the queue, by bubble up sort
                    pq.decreaseKey(neighbour, dist + heuristic(neighbour, ex, ey));
                }
                if (startIndex != -1) {
                    // If start vertex is a VG node, mark it as visited so we don't waste time on it.
                    memory.setVisited(startIndex, true);
                }
                graph.markEdgesFrom(markedEdges, sx, sy, neighbours);
            }
            {
                const VertexID goalIndex = graph.nodeIndex(ex, ey);
                // Add all visible neighbours of goal vertex
                scanner.computeAllDirNeighbours(data, ex, ey);
                const std::vector<GridVertex>& neighbours = data.neighbours;
                //std::cout << " ENLSVG target neighbours.size() " << neighbours.size() << std::endl;
                for (size_t i=0; i<neighbours.size(); ++i) {
                    const GridVertex& vn = neighbours[i];
                    VertexID neighbour = graph.nodeIndex(vn.x, vn.y);
                    memory.setEdgeWeightToGoal(neighbour, grid.euclideanDistance(vn.x, vn.y, ex, ey));
                }
                if (goalIndex != -1) {
                    // If goal vertex is a VG node, mark it as visited so we don't waste time on it.
                    memory.setVisited(goalIndex, true);
                }
                graph.markEdgesFrom(markedEdges, ex, ey, neighbours);
            }
        } // END: INITIALISATION
        graph.markBothWays(markedEdges);
        gettimeofday(&tv_after, &tz);
        double INITIALISATION_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        initial_time = INITIALISATION_cost;
        //std::cout << "-- INITIALISATION end in " << INITIALISATION_cost << "ms" << std::endl;
        gettimeofday(&tv_pre, &tz);
        int count = 0;
        while (pq.size() > 0) {
            //std::cout << count << " th, size() = " << pq.size() << "\n";
            count ++;
            if (goalDistance <= pq.getMinValue()) {
                break; // Reached Goal, or min value = POS_INF (can't find goal)
            }
            VertexID curr = pq.popMinIndex();

            const double currDistance = memory.distance(curr);
            const VertexID currParent = restorePar(memory.parent(curr));
            memory.setVisited(curr, true); // yz: an vertex shall be expand only ONCE, make the method incomplete to find all possible existing path ?

            // Traverse marked edges
            const std::vector<EdgeID>& neighbours = graph.edgeLists[curr];
            const VertexID goalIndex = graph.nodeIndex(ex, ey);
            //std::cout << "-> (" << graph.vertices[curr].x << ", " << graph.vertices[curr].y << ")" << ", next " << neighbours.size() << "|";
            //if(count%5 == 0) std::cout << std::endl;

            for (size_t i=0;i<neighbours.size();++i) {
                const EdgeID edgeId = neighbours[i]; // yz: in the neighbor, the method considering all possible legal edge ?
                // yz: without following code, there almost the same number of edge during iteration as RimJump DFS
                if (!markedEdges.isMarked[edgeId]) continue;
                const auto& edge = graph.edges[edgeId];
                VertexID dest = edge.destVertex;
                if (memory.visited(dest)) continue;
                double weight = graph.weight(edge);

                double destDistance = currDistance + weight;
                if (destDistance < memory.distance(dest) && isTaut(currParent, curr, dest)) {
                    memory.setParent(dest, curr);
                    memory.setDistance(dest, destDistance);
                    pq.decreaseKey(dest, destDistance + heuristic(dest, ex, ey));
                }
            }

            // Traverse skip edges
            const std::vector<SkipEdge>& skipEdges = graph.skipEdges[curr];
            for (size_t i=0;i<skipEdges.size();++i) {
                const SkipEdge& edge = skipEdges[i];
                VertexID dest = edge.next;
                if (memory.visited(dest)) continue;
                double weight = edge.weight;

                double destDistance = currDistance + weight;
                if (destDistance < memory.distance(dest) && isTaut(currParent, curr, edge.immediateNext)) {
                    memory.setParent(dest, negatePar(edge.immediateLast));
                    memory.setDistance(dest, destDistance);
                    pq.decreaseKey(dest, destDistance + heuristic(dest, ex, ey));
                }
            }

            if (memory.edgeWeightToGoal(curr) != -1.0) {
                double destDistance = currDistance + memory.edgeWeightToGoal(curr);
                if (destDistance < goalDistance) {
                    goalDistance = destDistance; // heuristic of goal should be 0
                    goalParent = curr;
                }
            }
        }
//        markedEdges.clear();
        if (parentPtrs != nullptr) setParentPointers(memory, goalParent, sx, sy, ex, ey, parentPtrs);
        auto path = getPath(memory, goalParent, sx, sy, ex, ey);
        double path_length = 0;
        if(path.empty()) { path_length = std::numeric_limits<double>::max(); }
        else {
            for(int i=0; i<path.size() - 1; i++) {
                path_length += sqrt(pow(path[i].x - path[i+1].x, 2) + pow(path[i].y - path[i+1].y, 2));
            }
        }
        gettimeofday(&tv_after, &tz);
        double search_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
        search_graph_time = search_cost;
        //std::cout << "-- ENL-SVG search end in " << search_cost << "ms, with path length = " << path_length << std::endl;
        return path;
    }

    Path Algorithm::getPath(const Memory& memory, VertexID goalParent,
    const int sx, const int sy, const int ex, const int ey) const {
        Path path;

        // no path found.
        if (goalParent == NO_PARENT) return path;

        // If the last vertex is not equal to the goal vertex.
        if (graph.nodeIndex(ex, ey) != goalParent) {
            path.push_back(GridVertex(ex,ey));
        }

        const int LEVEL_W = graph.LEVEL_W;
        VertexID prev = NO_PARENT;
        VertexID curr = goalParent;
        // Assumption: first edge from goal is not a skip-edge.
        // Loop invariant: prev is nonnegative.
        while (curr != NO_PARENT) {
            if (curr >= 0) {
                // Normal parent
                path.push_back(graph.vertices[curr]);
                prev = curr;
                curr = memory.parent(curr);
            } else {
                // Skip-edge parent
                curr = negatePar(curr);
                const auto& edges = graph.edges;

                // Set first edge.
                EdgeID currEdge;
                {
                    const auto& edgeList = graph.edgeLists[prev];
                    for (size_t i=0; i<edgeList.size(); ++i) {
                        const EdgeID id = edgeList[i];
                        if (edges[id].destVertex != curr) continue;
                        currEdge = id;
                        break;
                    }
                }

                // Follow level-W edges until you reach then ext skip-vertex.
                path.push_back(graph.vertices[curr]);
                while (!graph.isSkipVertex(edges[currEdge].destVertex)) {
                    const auto& tautOutgoingEdges = edges[currEdge].tautOutgoingEdges;

                    // Find next outgoing level-W edge.
                    for (size_t i=0; i<tautOutgoingEdges.size(); ++i) {
                        EdgeID nextEdge = tautOutgoingEdges[i];
                        if (edges[nextEdge].level == LEVEL_W) {
                            currEdge = nextEdge;
                            break;
                        }
                    }
                    path.push_back(graph.vertices[edges[currEdge].destVertex]);
                    // This should not infinite loop done correctly.
                }

                prev = edges[currEdge].destVertex;
                curr = memory.parent(prev);
            }
        }

        // If Start vertex is not equal to the first vertex.
        if (sx != path.back().x || sy != path.back().y) {
            path.push_back(GridVertex(sx,sy));
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void Algorithm::setParentPointers(const Memory& memory,
    VertexID goalParent, int sx, int sy, int ex, int ey, ParentPtrs* parentPtrs) const {

        parentPtrs->goal = GridVertex(ex, ey);
        parentPtrs->goalParent = (goalParent == NO_PARENT) ? parentPtrs->goal : graph.vertices[restorePar(goalParent)];

        std::vector<GridVertex>& current = parentPtrs->current;
        std::vector<GridVertex>& parent = parentPtrs->parent;

        current.clear();
        parent.clear();
        for (size_t i=0; i<memory.nNodes; ++i) {
            if (memory.distance(i) == POS_INF) continue;

            current.push_back(graph.vertices[i]);
            if (memory.parent(i) != NO_PARENT) {
                parent.push_back(graph.vertices[restorePar(memory.parent(i))]);
            } else {
                parent.push_back(GridVertex(sx, sy));
            }
        }
    }

    size_t Algorithm::nVertices() const {
        return graph.vertices.size();
    }

    size_t Algorithm::nEdges() const {
        return graph.edges.size();
    }
}}