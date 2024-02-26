//
// Created by yaozhuo on 2022/2/19.
//

#include "constraints/edge_transfer_constraints.h"
#include "surface_processor_LineScanner.h"
namespace freeNav::Topo {

    bool ETC_IC_GetCloserToObstacle_ENLSVG(const RoadMapNodePtr<2> &node1,
                                           const RoadMapNodePtr<2> &node2,
                                           const RoadMapNodePtr<2> &node3) {
        if(node1->sg_->pt_ == node3->sg_->pt_) return false;
//        std::cout << p1.first << ", " << p1.second << " / "
//                  << p2.first << ", " << p2.second << " / "
//                  << p3.first << ", " << p3.second << std::endl;

        Pointi<2> vec1 = node1->sg_->pt_ - node2->sg_->pt_, vec2 = node2->sg_->pt_ - node3->sg_->pt_;
        // vector on the same line
        if(vec1[0]*vec2[1] - vec1[1]*vec2[0] == 0) return true;

        FractionPair p1 = GridToFractionPair(node1->sg_->pt_);
        FractionPair p2 = GridToFractionPair(node2->sg_->pt_);
        FractionPair p3 = GridToFractionPair(node3->sg_->pt_);
        Pointd<2> current, next, pre;
        pre[0]     = p1.first.toFloat(), pre[1]     = p1.second.toFloat();
        current[0] = p2.first.toFloat(), current[1] = p2.second.toFloat();
        next[0]    = p3.first.toFloat(), next[1]    = p3.second.toFloat();

        double angle1 = angleBetweenThreePoints(pre, current, next);
        //std::cout << " angle1 = " << angle1*180.0/M_PI << std::endl;
        Pointd<2> new_line = (next - current).Normalize();
        Pointd<2> pre_line = (pre  - current).Normalize();
        Pointd<2> medium = new_line + pre_line;
        //if(node2->sg_->nearby_obst_pts_.size() != 1)
        //    std::cout << "node2->sg_->nearby_obst_pts_.size() = " << node2->sg_->nearby_obst_pts_.size() << std::endl;
        for(const auto& obst : node2->sg_->nearby_obst_pts_) {
            Pointd<2> obst_f; obst_f[0] = obst[0], obst_f[1] = obst[1];
            Pointd<2> to_obst = obst_f - current;
            double val1 = medium * to_obst;
            if(val1 < -1e-3) continue;
            double angle2 = acos(val1/(medium.Norm()*to_obst.Norm()));
            //std::cout << " angle2 = " << angle2*180.0/M_PI << std::endl;
            if(2*angle2 < angle1 + 1e-3) return true;
        }
        return false;
    }

    // TODO: add test for it
    bool ETC_Taut_2D(RoadMapGraphPtr<2>& tg, const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {

        const auto &flag_x1 = node2->sg_->pt_[0] - node1->sg_->pt_[0];
        const auto &flag_x2 = node3->sg_->pt_[0] - node2->sg_->pt_[0];
        const auto &flag_y1 = node2->sg_->pt_[1] - node1->sg_->pt_[1];
        const auto &flag_y2 = node3->sg_->pt_[1] - node2->sg_->pt_[1];

        // if on the same side, is illegal
        if(flag_x1*flag_x2 < 0 || flag_y1*flag_y2 < 0) return false;
        // three point on one line, must be legal, when use ENLSVG line scanner
        if(flag_x1 == 0 && flag_x2 == 0) return true;
        if(flag_y1 == 0 && flag_y2 == 0) return true;

        bool left_to_right = flag_x1 > 0 || flag_x2 > 0;

        const auto & left_x  = left_to_right ? flag_x1 : -flag_x2;
        const auto & right_x = left_to_right ? flag_x2 : -flag_x1;

        const auto & left_y  = left_to_right ? flag_y1 : -flag_y2;
        const auto & right_y = left_to_right ? flag_y2 : -flag_y1;

        const auto &obst = node2->sg_->nearby_obst_offsets_[0];

        if(left_x != 0 && right_x != 0) {
            if (obst[1] == 1) {
                // left k < right k
                if (left_y * right_x > right_y * left_x) return false;
            } else {
                // left k > right k
                if (left_y * right_x < right_y * left_x) return false;
            }
        } else if(left_x == 0) {
            if(obst[0] != 1) return false;
            if(obst[1] == 1) {
                if(left_y > 0) return false;
            } else {
                if(left_y < 0) return false;
            }
        } else { // right_x == 0
            if(obst[0] == 1) return false;
            if(obst[1] == 1) {
                if(right_y < 0) return false;
            } else {
                if(right_y > 0) return false;
            }
        }

        return true;
    }

    bool ETC_NotLookBack3_2D(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        int flag_x1 = node2->sg_->pt_[0] - node1->sg_->pt_[0];
        int flag_x2 = node3->sg_->pt_[0] - node2->sg_->pt_[0];
        int flag_y1 = node2->sg_->pt_[1] - node1->sg_->pt_[1];
        int flag_y2 = node3->sg_->pt_[1] - node2->sg_->pt_[1];

        return (flag_x1 * flag_x2 >= 0) && (flag_y1 * flag_y2 >= 0);
    }

    bool topRightOfBlockedTile(const RoadMapNodePtr<2>& node2) {
        const auto & offset = node2->sg_->nearby_obst_offsets_[0];
        return offset[0] == -1 && offset[1] == -1;
    }

    bool topLeftOfBlockedTile(const RoadMapNodePtr<2>& node2) {
        const auto & offset = node2->sg_->nearby_obst_offsets_[0];
        return offset[0] == 1 && offset[1] == -1;
    }

    bool bottomRightOfBlockedTile(const RoadMapNodePtr<2>& node2) {
        const auto & offset = node2->sg_->nearby_obst_offsets_[0];
        return offset[0] == -1 && offset[1] == 1;
    }

    bool bottomLeftOfBlockedTile(const RoadMapNodePtr<2>& node2) {
        const auto & offset = node2->sg_->nearby_obst_offsets_[0];
        return offset[0] == 1 && offset[1] == 1;
    }

    bool isTautFromBottomLeft(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (x3 < x2 || y3 < y2) return false;

        int compareGradients = (y2-y1)*(x3-x2) - (y3-y2)*(x2-x1); // m1 - m2
        if (compareGradients < 0) { // m1 < m2
            return bottomRightOfBlockedTile(node2);
        } else if (compareGradients > 0) { // m1 > m2
            return topLeftOfBlockedTile(node2);
        } else { // m1 == m2
            return true;
        }
    }


    bool isTautFromTopLeft(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (x3 < x2 || y3 > y2) return false;

        int compareGradients = (y2-y1)*(x3-x2) - (y3-y2)*(x2-x1); // m1 - m2
        if (compareGradients < 0) { // m1 < m2
            return bottomLeftOfBlockedTile(node2);
        } else if (compareGradients > 0) { // m1 > m2
            return topRightOfBlockedTile(node2);
        } else { // m1 == m2
            return true;
        }
    }

    bool isTautFromBottomRight(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (x3 > x2 || y3 < y2) return false;

        int compareGradients = (y2-y1)*(x3-x2) - (y3-y2)*(x2-x1); // m1 - m2
        if (compareGradients < 0) { // m1 < m2
            return topRightOfBlockedTile(node2);
        } else if (compareGradients > 0) { // m1 > m2
            return bottomLeftOfBlockedTile(node2);
        } else { // m1 == m2
            return true;
        }
    }

    bool isTautFromTopRight(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (x3 > x2 || y3 > y2) return false;

        int compareGradients = (y2-y1)*(x3-x2) - (y3-y2)*(x2-x1); // m1 - m2
        if (compareGradients < 0) { // m1 < m2
            return topLeftOfBlockedTile(node2);
        } else if (compareGradients > 0) { // m1 > m2
            return bottomRightOfBlockedTile(node2);
        } else { // m1 == m2
            return true;
        }
    }


    bool isTautFromLeft(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (x3 < x2) return false;

        int dy = y3 - y2;
        if (dy < 0) { // y3 < y2
            return topRightOfBlockedTile(node2);
        } else if (dy > 0) { // y3 > y2
            return bottomRightOfBlockedTile(node2);
        } else { // y3 == y2
            return true;
        }
    }

    bool isTautFromRight(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (x3 > x2) return false;

        int dy = y3 - y2;
        if (dy < 0) { // y3 < y2
            return topLeftOfBlockedTile(node2);
        } else if (dy > 0) { // y3 > y2
            return bottomLeftOfBlockedTile(node2);
        } else { // y3 == y2
            return true;
        }
    }

    bool isTautFromBottom(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (y3 < y2) return false;

        int dx = x3 - x2;
        if (dx < 0) { // x3 < x2
            return topRightOfBlockedTile(node2);
        } else if (dx > 0) { // x3 > x2
            return topLeftOfBlockedTile(node2);
        } else { // x3 == x2
            return true;
        }
    }

    bool isTautFromTop(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {
        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];
        if (y3 > y2) return false;

        int dx = x3 - x2;
        if (dx < 0) { // x3 < x2
            return bottomRightOfBlockedTile(node2);
        } else if (dx > 0) { // x3 > x2
            return bottomLeftOfBlockedTile(node2);
        } else { // x3 == x2
            return true;
        }
    }

    bool ETC_ENLSVG_Taut_2D(const RoadMapNodePtr<2>& node1, const RoadMapNodePtr<2>& node2, const RoadMapNodePtr<2>& node3) {

        const auto& x1 = node1->sg_->pt_[0];
        const auto& y1 = node1->sg_->pt_[1];
        const auto& x2 = node2->sg_->pt_[0];
        const auto& y2 = node2->sg_->pt_[1];
        const auto& x3 = node3->sg_->pt_[0];
        const auto& y3 = node3->sg_->pt_[1];

        if (x1 < x2) {
            if (y1 < y2) {
                return isTautFromBottomLeft(node1, node2, node3);
            } else if (y2 < y1) {
                return isTautFromTopLeft(node1, node2, node3);
            } else { // y1 == y2
                return isTautFromLeft(node1, node2, node3);
            }
        } else if (x2 < x1) {
            if (y1 < y2) {
                return isTautFromBottomRight(node1, node2, node3);
            } else if (y2 < y1) {
                return isTautFromTopRight(node1, node2, node3);
            } else { // y1 == y2
                return isTautFromRight(node1, node2, node3);
            }
        } else { // x2 == x1
            if (y1 < y2) {
                return isTautFromBottom(node1, node2, node3);
            } else if (y2 < y1) {
                return isTautFromTop(node1, node2, node3);
            } else { // y1 == y2
                std::cout << "ERROR: v == u?" << std::endl;
                return true;
            }
        }
    }

}