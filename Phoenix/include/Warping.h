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

    Eigen::Matrix3d getPerspectiveMatrixTransform(const double2 src[4], const double2 dst[4]);

    void applyMatrixToPolygons(Eigen::Matrix3d& m, std::vector<earcutPoly>& polys);

    // boite représentant la boite cible et sa destination réelle
    struct Boxes {
        double2 src[4];
        double2 dst[4];
    };
}


static std::vector<Warping::Boxes> src_dst_boxes =
{
    // 1ere transformation
    Warping::Boxes{
        double2{0.0, 0.0},
        double2{0.0, 20000.0},
        double2{20000.0, 20000.0},
        double2{20000.0, 0.0},

        double2{1000.0, 500.0},
        double2{300.0, 10000.0},
        double2{15000.0, 15000.0},
        double2{18000.0, 1000.0}
    }
};