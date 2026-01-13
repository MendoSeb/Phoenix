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
        int index_min,
        int index_max,
        std::vector<std::vector<cv::Point2f>>& polys_in_box,
        Cell* cell, 
        std::vector<cv::Point2f>& src_box,
        std::mutex& mutex
    )
    {
        for (size_t k = index_min; k < index_max; k++)
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

                mutex.lock();
                polys_in_box.push_back(poly);
                mutex.unlock();
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

        const unsigned int NB_THREAD = 1;
        std::thread threads[NB_THREAD];
        std::mutex mutex;
        int nb_polys_per_thread = cell->polygon_array.count / NB_THREAD;

        for (int i = 0; i < NB_THREAD; i++)
        {
            int index_min = i * nb_polys_per_thread;
            int index_max = (i + 1) * nb_polys_per_thread;

            if (i == NB_THREAD - 1)
                index_max = cell->polygon_array.count;

            threads[i] = std::thread(
                ThreadPolygonesInBox,
                index_min,
                index_max,
                std::ref(polys_in_box),
                std::ref(cell),
                std::ref(src_box),
                std::ref(mutex));
        }

        for (int i = 0; i < NB_THREAD; i++)
            threads[i].join();

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