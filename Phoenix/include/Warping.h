#pragma once
#include <gdstk/library.hpp>
#include "vector_types.h"
#include <gdstk/gdstk.hpp>
#include <Eigen/Dense>
#include "GdstkUtils.h"


using namespace gdstk;


namespace Warping
{
    std::vector<earcutPoly> getPolygonsInBox(const Library& lib, const Polygon* box);

    Eigen::Matrix3d getPerspectiveMatrixTransform(const float2 src[4], const float2 dst[4]);

    void applyMatrixToPolygons(Eigen::Matrix3d& m, std::vector<earcutPoly>& polys);

    // boite représentant la boite cible et sa destination réelle
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

        float2{300, 300},
        float2{300, 9700},
        float2{9700, 9700},
        float2{9700, 300}
    }
};