#include "Warping.h"


using namespace gdstk;



std::vector<earcutPoly> Warping::getPolygonsInBox(const Library& lib, const Polygon* box)
{
    std::vector<earcutPoly> polys_in_box;

    for (size_t i = 0; i < lib.cell_array[0]->polygon_array.count; i++)
        if (box->contain_any(lib.cell_array[0]->polygon_array[i]->point_array))
        {
            earcutPoly poly;
            poly.push_back({});

            for (size_t k = 0; k < lib.cell_array[0]->polygon_array[i]->point_array.count; k++)
            {
                earcutPoint point;
                point.at(0) = lib.cell_array[0]->polygon_array[i]->point_array[k].x;
                point.at(1) = lib.cell_array[0]->polygon_array[i]->point_array[k].y;
                poly[0].push_back(point);
            }

            polys_in_box.push_back(poly);
        }

    return polys_in_box;
}


Eigen::Matrix3d Warping::getPerspectiveMatrixTransform(const double2 src[4], const double2 dst[4])
{
    Eigen::Matrix<double, 8, 8> a;
    Eigen::Matrix<double, 8, 1> b;

    for (size_t i = 0; i < 4; i++)
    {
        a.row(i*2) << src[i].x, src[i].y, 1, 0, 0, 0, -src[i].x * dst[i].x, -src[i].y * dst[i].x;
        a.row(i*2+1) << 0, 0, 0, src[i].x, src[i].y, 1, -src[i].x * dst[i].y, -src[i].y * dst[i].y;

        b.row(i*2) << dst[i].x;
        b.row(i*2+1) << dst[i].y;
    }

    Eigen::Matrix<double, 8, 1> x = a.colPivHouseholderQr().solve(b);

    Eigen::Matrix3d sol;
    sol << x(0), x(1), x(2),
           x(3), x(4), x(5),
           x(6), x(7), 1;

    return sol;
}


void Warping::applyMatrixToPolygons(Eigen::Matrix3d& m, std::vector<earcutPoly>& polys)
{
    for (earcutPoly& p : polys)
    {
        for (earcutPoint& point : p[0])
        {
            Eigen::Vector3d v;
            v << point.at(0), point.at(1), 1;

            Eigen::Vector3d res = m * v;
            res /= res(2); // divise par la coordonnée homogène

            point.at(0) = res(0);
            point.at(1) = res(1);
        }
    }
}