#include <opencv2/opencv.hpp>
#include <gdstk/library.hpp>


namespace Warping
{
    void ApplyMatrix(std::vector<cv::Point2f>& src, cv::Mat& warp);

    std::vector<std::vector<cv::Point2f>> FindPolygonesInBox(
        const Library& lib,
        std::vector<cv::Point2f>& box_pos);
}