#include "warning.h"
#include <vector>
#include <clipper2/clipper.core.h>
#include <opencv2/opencv.hpp>
#include <gdstk/library.hpp>
#include <vector_types.h>


using namespace Clipper2Lib;
using namespace gdstk;


namespace Warping
{
    void ThreadPolygonesInBox
    (
        int index_max,
        std::vector<std::vector<cv::Point2f>>& polys_in_box,
        Cell* cell, 
        std::vector<cv::Point2f>& src_box
    )
    {
        for (size_t k = 0; k < index_max; k++)
        {
            bool pointInBox = false;

            // y a t'il au moins un point dans la box
            for (size_t m = 0; m < cell->polygon_array[k]->point_array.count; m++)
            {
                Vec2 gdstk_point = cell->polygon_array[k]->point_array[m];
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
                std::vector<cv::Point2f> poly;
                poly.resize(cell->polygon_array[k]->point_array.count);

                for (size_t m = 0; m < cell->polygon_array[k]->point_array.count; m++)
                {
                    Vec2 gdstk_point = cell->polygon_array[k]->point_array[m];
                    cv::Point2f point(gdstk_point.x, gdstk_point.y);
                    poly[m] = std::move(point);
                }

                polys_in_box.push_back(poly);
            }
        }
    }


    std::vector<std::vector<cv::Point2f>> FindPolygonesInBox
    (
        const Library& lib, 
        std::vector<cv::Point2f>& src_box
    )
    {
        assert(lib.cell_array.count == 0);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::vector<std::vector<cv::Point2f>> polys_in_box;
        Cell* cell = lib.cell_array[0];

        ThreadPolygonesInBox
        (
            cell->polygon_array.count,
            std::ref(polys_in_box),
            std::ref(cell),
            std::ref(src_box)
        );

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "FindPolygonesInBox fait en : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
        return polys_in_box;
    }


    void TransformVerticesInBox
    (
        std::pair<std::vector<cv::Point2f>, std::vector<uint3>>& obj,
        std::vector<cv::Point2f>& src_box,
        cv::Mat& warp
    )
    {
        for (int i = 0; i < obj.first.size(); i++)
        {
            cv::Point2f point(obj.first[i].x, obj.first[i].y);

            if (cv::pointPolygonTest(src_box, point, false))
            {
                std::vector<cv::Point2f> temp = { point };
                cv::perspectiveTransform(temp, temp, warp);
                obj.first[i].x = temp[0].x;
                obj.first[i].y = temp[0].y;
            }
        }
    }
}