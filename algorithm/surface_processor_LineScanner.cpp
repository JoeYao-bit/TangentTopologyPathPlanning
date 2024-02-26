//
// Created by yaozhuo on 2022/9/29.
//

#include "surface_processor_LineScanner.h"
namespace freeNav::Topo {

    Fraction projFromPoint(const FractionPair& start_pt,
                           const Fraction& current_y, const Fraction& current_x,
                           const Fraction& next_y) {
        Fraction next_x;
        Fraction bottom_y = current_y - start_pt.second;

        Fraction k_ratio = (current_x - start_pt.first)/bottom_y;

        next_x = k_ratio*(next_y - start_pt.second) + start_pt.first;
        return next_x;
    }

    FractionPair GridToFractionPair(const Pointi<2>& pt) {
        return {Fraction(2*pt[0]-1, 2), Fraction(2*pt[1]-1, 2)};
    }



}