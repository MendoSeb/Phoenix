#pragma once
#include <gdstk/library.hpp>
#include "vector_types.h"
#include <gdstk/gdstk.hpp>
#include <Eigen/Dense>
#include "GdstkUtils.h"


using namespace gdstk;


namespace Warping
{
    // calcule la matrice de transformation homographique pour passer des points de src à dst
    Eigen::Matrix3d getPerspectiveMatrixTransform(const float2 src[4], const float2 dst[4]);

    // boite représentant la boite cible et sa destination réelle sur le plateau
    struct Boxes {
        float2 src[4];
        float2 dst[4];
    };
}


static std::vector<Warping::Boxes> src_dst_boxes =
{
    // 1ere transformation
    Warping::Boxes{
        float2{0, 0},
        float2{0, 10020},
        float2{10020, 10020},
        float2{10020, 0},

        float2{1000, 1000},
        float2{1000, 9000},
        float2{9000, 9000},
        float2{9000, 1000}
    }
};