#include "../Pathfinding/ENLSVGGraph.h"
#include "../Pathfinding/LineOfSightScanner.h"
#include "../Pathfinding/Grid.h"
#include <algorithm>

namespace Pathfinding {

namespace ENLSVG {
    VisibilityGraph::VisibilityGraph(const Grid& grid, const LineOfSightScanner& scanner):
        grid(grid), sizeX(grid.sizeX), sizeY(grid.sizeY), nodeIndexesSizeX(grid.sizeX+1), scanner(scanner) {

        // yz: expand x,y coordinate by 1
        // Initialise vertices (outer corners).
        nodeIndexes.resize(nodeIndexesSizeX*(sizeY+1), -1);
        for (int y=0;y<=sizeY;++y) {
            for (int x=0;x<=sizeX;++x) {
                if (grid.isOuterCorner(x,y)) {
                    nodeIndexes[y*nodeIndexesSizeX + x] = vertices.size();
                    vertices.push_back(GridVertex(x,y));
                }
            }
        }
        vertices.shrink_to_fit();

        // Initialise SVG edges
        edgeLists.resize(vertices.size());
        ScannerStacks scannerStacks;
        // yz: for each candidate vertex
        for (size_t i=0;i<vertices.size();++i) {
            int cx = vertices[i].x;
            int cy = vertices[i].y;
            // yz: use Line-Of-Sight Scan to find all visible angle in current vertex's sight
            scanner.computeTautDirNeighbours(scannerStacks, cx, cy);
            std::vector<GridVertex>& neighbours = scannerStacks.neighbours;

            for (size_t j=0;j<neighbours.size();++j) {
                int nx = neighbours[j].x;
                int ny = neighbours[j].y;
                // yz: only connect to vertex that has small id than current vertex, for avoid add twice ?
                VertexID dest = nodeIndexes[ny*nodeIndexesSizeX + nx];
                if (dest >= i) continue;
                // yz: insert two edge with reverse direction and default level-W
                // yz: store the available next edges of in the vertex i in edgeLists
                connectEdge(i, dest, cx, cy, nx, ny);
            }
        }
        edges.shrink_to_fit();
        for (size_t i=0; i<edgeLists.size(); ++i) {
            edgeLists[i].shrink_to_fit();
        }

        // Connect Taut Neighbours of Edges
        for (size_t i=0; i<edges.size(); ++i) {
            EdgeData& edge = edges[i];
            std::vector<EdgeID>& tautOutgoingEdges = edge.tautOutgoingEdges;
            VertexID src = edge.sourceVertex;
            VertexID dest = edge.destVertex;
            int sx = vertices[src].x;
            int sy = vertices[src].y;
            int ex = vertices[dest].x;
            int ey = vertices[dest].y;
            // yz: find all vertex that has the same vertex that are taut
            const std::vector<EdgeID>& outgoingEdges = edgeLists[dest];
            for (size_t j=0; j<outgoingEdges.size(); ++j) {
                EdgeID succ = outgoingEdges[j];
                // assert edges[succ].sourceVertex == dest
                VertexID next = edges[succ].destVertex;
                if (next == src) continue;

                int nx = vertices[next].x;
                int ny = vertices[next].y;
                // yz: if is taut, insert to current edge's next edges
                if (grid.isTaut(sx, sy, ex, ey, nx, ny)) {
                    tautOutgoingEdges.push_back(succ);
                }
            }
            tautOutgoingEdges.shrink_to_fit();
        }

        // yz: skip edge means compress a series of continues vertex, so the number of skip edges must be small than total number of vertex
        skipEdges.resize(vertices.size());
        buildHierarchy();
    }

    void VisibilityGraph::connectEdge(int i, int j, int xi, int yi, int xj, int yj) {
        EdgeID edge_ij = edges.size();
        edges.push_back(EdgeData(i, j, LEVEL_W));

        EdgeID edge_ji = edges.size();
        edges.push_back(EdgeData(j, i, LEVEL_W));

        // Note: The following must be true:
        // 1. edge_ij % 2 == 0
        // 2. edge_ji % 2 == 1
        // 3. edge_ij + 1 = edge_ji
        // We do this so that we can use the opposite(edgeId) function to get the opposite edge.

        //if (! ((edge_ij % 2 == 0) && (edge_ji % 2 == 1) && (edge_ji == edge_ij + 1)) ) std::cout << "error!" << std::endl;

        edgeLists[i].push_back(edge_ij);
        edgeLists[j].push_back(edge_ji);
    }

    void VisibilityGraph::buildHierarchy() {
        computeAllEdgeLevels();
        setupSkipEdges();
    }

    // ordered graph and unordered graph should have different ways...
    void VisibilityGraph::computeAllEdgeLevels() {

        std::vector<EdgeID> currentLevelEdges;
        std::vector<EdgeID> nextLevelEdges;
        
        std::vector<int> nNeighbours;
        nNeighbours.resize(edges.size());
        for (EdgeID i=0; i<edges.size(); ++i) {
            int n = edges[i].tautOutgoingEdges.size();
            nNeighbours[i] = n;
            // yz: set edge that do not have next edges as level_0
            if (n == 0) currentLevelEdges.push_back(i);
        } // set which has no next as initial edges
        
        int currLevel = 1;
        while (currentLevelEdges.size() > 0) {
            for (size_t i=0;i<currentLevelEdges.size(); ++i) {
                // yz: current level edge
                EdgeID curr = currentLevelEdges[i];
                // yz: the opposite current edge
                EdgeID opp = opposite(curr);
                
                edges[curr].level = currLevel;
                edges[opp].level = currLevel;

                // Curr side must have no neighbours.
                // Opp side may have neighbours.

                const std::vector<EdgeID>& neighbours = edges[opp].tautOutgoingEdges; // yz: opp the reversed edge's next edges
                for (size_t j=0; j<neighbours.size(); ++j) {
                    EdgeID neighbour = opposite(neighbours[j]); // yz: the edges that point to cur edge
                    // yz: if the edges that point to cur edge is not updated yet, may update
                    if (edges[neighbour].level != LEVEL_W) continue;

                    // yz: only the next edges numbers of the candidate edge is 1, it should be update ?
                    --nNeighbours[neighbour];
                    if (nNeighbours[neighbour] == 0) {
                        nextLevelEdges.push_back(neighbour);
                    }
                    // debugging: if (nNeighbours[neighbour] < 0) std::cout << "ERROR" << std::endl;
                }

            }
            currentLevelEdges.clear();
            std::swap(currentLevelEdges, nextLevelEdges);
            ++currLevel;
        }
        std::cout << " ENLSVG max level = " << currLevel - 1 << std::endl;
    }

    void VisibilityGraph::setupSkipEdges() {
        std::vector<VertexID> skipVertices;
        std::vector<bool> isSkipVertex;
        isSkipVertex.resize(vertices.size(), false);

        for (VertexID i=0; i<vertices.size(); ++i) {
            const auto& edgeList = edgeLists[i];
            int nLevelWEdges = 0;
            for (size_t j=0; j<edgeList.size(); ++j) {
                if (edges[edgeList[j]].level == LEVEL_W) {
                    ++nLevelWEdges;
                    if (nLevelWEdges >= 3) {
                        isSkipVertex[i] = true;
                        skipVertices.push_back(i);
                        break;
                    }
                }
            }
        }

        for (size_t i=0; i<skipVertices.size(); ++i) {
            VertexID curr = skipVertices[i];
            const auto& edgeList = edgeLists[curr];

            for (size_t j=0; j<edgeList.size(); ++j) {
                if (edges[edgeList[j]].level != LEVEL_W) continue;
                // follow edge till next skip vertex.
                double totalWeight;
                VertexID nextVertex;
                VertexID immediateNext;
                VertexID immediateLast;
                followLevelWPathToNextSkipVertex(edgeList[j], totalWeight, nextVertex, immediateNext, immediateLast, isSkipVertex);
                skipEdges[curr].push_back(SkipEdge(nextVertex, totalWeight, immediateNext, immediateLast));
            }
        }

        for (size_t i=0; i<skipEdges.size(); ++i) {
            skipEdges[i].shrink_to_fit();
        }
    }

    void VisibilityGraph::followLevelWPathToNextSkipVertex(EdgeID firstEdge,
        double& totalWeight, VertexID& nextVertex, VertexID& immediateNext,
        VertexID& immediateLast, const std::vector<bool>& isSkipVertex) const {
        EdgeID currEdge = firstEdge;
        totalWeight = weight(currEdge); // RETURN VALUE
        immediateNext = edges[currEdge].destVertex; // RETURN VALUE

        while (!isSkipVertex[edges[currEdge].destVertex]) {
            const auto& tautOutgoingEdges = edges[currEdge].tautOutgoingEdges;

            // Find next outgoing level-W edge.
            for (size_t i=0; i<tautOutgoingEdges.size(); ++i) {
                EdgeID nextEdge = tautOutgoingEdges[i];
                if (edges[nextEdge].level == LEVEL_W) {
                    currEdge = nextEdge;
                    break;
                }
            }
            totalWeight += weight(currEdge);
            // This should not infinite loop done correctly.
        }
        immediateLast = edges[currEdge].sourceVertex; // RETURN VALUE
        nextVertex = edges[currEdge].destVertex; // RETURN VALUE
    }

    void VisibilityGraph::markEdgesFrom(MarkedEdges& markedEdges, const int sx, const int sy, const std::vector<GridVertex>& neighbours) const {
        std::vector<EdgeID> edgeQueue;
        int taut_count = 0;
        for (size_t i=0; i<neighbours.size(); ++i) {
            const GridVertex& u = neighbours[i];
            const VertexID neighbour = nodeIndexes[u.y*nodeIndexesSizeX + u.x];
            const std::vector<EdgeID>& edgeList = edgeLists[neighbour];
            //std::cout << " start from " <<  u.x << u.y << std::endl;
            for (size_t j=0; j<edgeList.size(); ++j) {
                const EdgeID id = edgeList[j];
                const EdgeData& edge = edges[id];
                //const GridVertex& u = vertices[edge.sourceVertex];
                const GridVertex& v = vertices[edge.destVertex];
                if (grid.isTaut(sx, sy, u.x, u.y, v.x, v.y)) {
                    // yz: mark all taut edge to start as visited, equal to create initial path in RimJump
                    taut_count ++;
                    markedEdges.mark(id);
                    edgeQueue.push_back(id);
                }
            }
        }
        //std::cout << " taut_count " << taut_count << std::endl;
        size_t head = 0;
        while (head < edgeQueue.size()) {
            EdgeID curr = edgeQueue[head];

            const int currLevel = edges[curr].level;
            const auto& tautOutgoingEdges = edges[curr].tautOutgoingEdges;
            bool skipVertex = isSkipVertex(edges[curr].destVertex);

            for (size_t j=0; j<tautOutgoingEdges.size(); ++j) {
                EdgeID next = tautOutgoingEdges[j];
                if (markedEdges.isMarked[next]) continue;

                // Should be (currLevel != LEVEL_W && edges[next].level > currLevel) || (!skipVertex && edges[next].level == LEVEL_W)
                // But we set LEVEL_W = INT_MAX, so, "currLevel != LEVEL_W" is no longer needed.
                if ((edges[next].level > currLevel) || (!skipVertex && edges[next].level == LEVEL_W)) {
                    //std::cout << " cur -> next level " << currLevel << "->" << edges[next].level << std::endl;
                    taut_count ++;
                    markedEdges.mark(next);
                    edgeQueue.push_back(next);
                }
            }

            ++head;
        }
        //std::cout << " mark " << taut_count << " edges " << std::endl;
        //std::cout << " ENLSVG::edgeQueue.size() = " << edgeQueue.size() << std::endl;
    }

    // Duplicate all markings. Call this after calling markEdgesFrom from both start and goal.
    void VisibilityGraph::markBothWays(MarkedEdges& markedEdges) const {
        std::vector<EdgeID>& markedIndexes = markedEdges.markedIndexes;
        size_t nMarked = markedIndexes.size(); // freeze size.
        for (size_t i=0;i<nMarked;++i) {
            EdgeID opp = opposite(markedIndexes[i]);
            if (!markedEdges.isMarked[opp]) markedEdges.mark(opp);
        }
    }


    void VisibilityGraph::printStatistics() const {
        int nVertices = vertices.size();
        int nEdges = edges.size();

        int totalEdgeDegree = 0;
        for (auto& edge : edges) {
            totalEdgeDegree += edge.tautOutgoingEdges.size();
        }
        float averageEdgeDegree = (float)totalEdgeDegree / nEdges;

        int totalVertexDegree = 0;
        for (auto& edgeList : edgeLists) {
            totalVertexDegree += edgeList.size();
        }
        float averageVertexDegree = (float)totalVertexDegree / nVertices;

        int nSkipEdges = 0;
        for (auto& skipEdge : skipEdges) {
            nSkipEdges += skipEdge.size();
        }

        std::cout << "nVertices: " << nVertices << std::endl;
        std::cout << "nEdges: " << nEdges << std::endl;
        std::cout << "nSkipEdges: " << nSkipEdges << std::endl;
        std::cout << "totalEdgeDegree: " << totalEdgeDegree << std::endl;
        std::cout << "totalVertexDegree: " << totalVertexDegree << std::endl;
        std::cout << "averageEdgeDegree: " << averageEdgeDegree << std::endl;
        std::cout << "averageVertexDegree: " << averageVertexDegree << std::endl;
    }
}}