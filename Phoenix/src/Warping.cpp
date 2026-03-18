#include "Warping.h"


using namespace gdstk;


Eigen::Matrix3d Warping::getPerspectiveMatrixTransform(const float2 src[4], const float2 dst[4])
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