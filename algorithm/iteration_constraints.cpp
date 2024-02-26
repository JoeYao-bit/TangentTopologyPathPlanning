//
// Created by yaozhuo on 2022/1/3.
//

#include <cfloat>
#include "constraints/iteration_constraints.h"
#include "constraints/node_iteration_constraints.h"

namespace freeNav::Topo {

    PathLen global_minimum_path_length = MAX<PathLen>;

    int determinant(const int& v1, const int& v2, const int& v3, const int& v4)  // 行列式
    {
        return (v1*v3-v2*v4);
    }

    bool intersect3(const Pointi<2>& a, const Pointi<2>& b, const Pointi<2>& c, const Pointi<2>& d)
    {
//        double delta = determinant(bb[0]-aa[0], cc[0]-dd[0], bb[1]-aa[1], cc[1]-dd[1]);
//        if ( delta == 0)  // delta=0，表示两线段重合或平行
//        {
//            return true;
//        }
//        double namenda = determinant(cc[0]-aa[0], cc[0]-dd[0], cc[1]-aa[1], cc[1]-dd[1]) / delta;
//        if ( namenda>1 || namenda<0 )
//        {
//            return true;
//        }
//        double miu = determinant(bb[0]-aa[0], cc[0]-aa[0], bb[1]-aa[1], cc[1]-aa[1]) / delta;
//        if ( miu>1 || miu<0 )
//        {
//            return true;
//        }
//        return false;

        if(std::max(c[0],d[0]) < std::min(a[0],b[0]) ||
           std::max(a[0],b[0]) < std::min(c[0],d[0]) ||
           std::max(c[1],d[1]) < std::min(a[1],b[1]) ||
           std::max(a[1],b[1]) < std::min(c[1],d[1])) {
            return false;
        }

        if ((((a[0] - c[0])*(d[1] - c[1]) - (a[1] - c[1])*(d[0] - c[0]))*
             ((b[0] - c[0])*(d[1] - c[1]) - (b[1] - c[1])*(d[0] - c[0]))) > 0 ||
            (((c[0] - a[0])*(b[1] - a[1]) - (c[1] - a[1])*(b[0] - a[0]))*
             ((d[0] - a[0])*(b[1] - a[1]) - (d[1] - a[1])*(b[0] - a[0]))) > 0)
        {
            return false;
        }
        return true;

    }

}