#pragma once
#include <opencv2/opencv.hpp>
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
        float2{0, 20000},
        float2{20000, 20000},
        float2{20000, 0},

        float2{1000, 500},
        float2{300, 10000},
        float2{15000, 15000},
        float2{18000, 1000}
    }
};