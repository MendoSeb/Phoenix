#include "warning.h"
#include <vector>
#include <clipper2/clipper.core.h>
#include <opencv2/opencv.hpp>
#include <gdstk/library.hpp>


using namespace Clipper2Lib;
using namespace gdstk;


namespace Warping
{

    void ApplyMatrix(std::vector<cv::Point2f>& src, cv::Mat& warp)
    {
        std::vector<cv::Point2f> output;
        cv::perspectiveTransform(src, src, warp);
    }


    std::vector<std::vector<cv::Point2f>> FindPolygonesInBox(const Library& lib, std::vector<cv::Point2f>& src_box)
    {
        std::vector<std::vector<cv::Point2f>> polys_in_box;

        for (size_t i = 0; i < lib.cell_array.count; i++)
            for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
            {
                std::vector<cv::Point2f> poly;
                bool pointInBox = false;

                // y a t'il au moins un point dans la box
                for (size_t m = 0; m < lib.cell_array[i]->polygon_array[k]->point_array.count; m++)
                {
                    Vec2 gdstk_point = lib.cell_array[i]->polygon_array[k]->point_array[m];
                    cv::Point2f point(gdstk_point.x, gdstk_point.y);

                    double inside = cv::pointPolygonTest(src_box, point, false);

                    if (inside > 0)
                    {
                        pointInBox = true;
                        break;
                    }
                }

                // inclure tout le polygone si un de ses points est dans la box
                if (pointInBox)
                {
                    for (size_t m = 0; m < lib.cell_array[i]->polygon_array[k]->point_array.count; m++)
                    {
                        Vec2 gdstk_point = lib.cell_array[i]->polygon_array[k]->point_array[m];
                        cv::Point2f point(gdstk_point.x, gdstk_point.y);
                        poly.push_back(point);
                    }
                }

                if (poly.size() > 0)
                    polys_in_box.push_back(poly);
            }

        return polys_in_box;
    }
}