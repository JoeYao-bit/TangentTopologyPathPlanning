//
// Created by yaozhuo on 2022/1/2.
//

#include "path_tree_with_edge.h"

namespace freeNav::Topo {

    template <int N>
    bool operator<(const PathPointWithEdge<N> ppt1, const PathPointWithEdge<N> ppt2) {
        return ppt1.last_edge_->to_->sg_->id_ < ppt2.last_edge_->to_->sg_->id_;
    }

}