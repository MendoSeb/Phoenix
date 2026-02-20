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
        double2{0.0, 10.0},
        double2{10.0, 10.0},
        double2{10.0, 0.0},

        double2{3.0, 3.0},
        double2{3.0, 13.0},
        double2{13.0, 13.0},
        double2{13.0, 3.0}
    }
};