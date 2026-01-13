#include <opencv2/opencv.hpp>
#include <gdstk/library.hpp>


namespace Warping
{
    /* Renvoie tous les polygones dont au moins un point est dans la "boite" */
    std::vector<std::vector<cv::Point2f>> FindPolygonesInBox(
        const Library& lib,
        std::vector<cv::Point2f>& box_pos
    );

    void TransformVerticesInBox(
        std::pair<std::vector<cv::Point2f>, std::vector<uint3>>& obj,
        std::vector<cv::Point2f>& src_box,
        cv::Mat& warp
    );

    /* Convertit les polygones avec points OpenCV en polygones de type gdstk */
    Library ConvertOpenCVPolygonesToGdstk(std::vector<std::vector<cv::Point2f>>& polys_in_boxs)
    {
        Library lib = {};
        lib.init("library", 1e-6, 1e-9);

        Cell* cell = new Cell();
        cell->name = copy_string("FIRST", NULL);
        lib.cell_array.append(cell);

        for (std::vector<cv::Point2f>& poly : polys_in_boxs)
        {
            Polygon* gdstk_poly = (Polygon*)allocate_clear(sizeof(Polygon));

            for (cv::Point2f& point : poly)
                gdstk_poly->point_array.append(Vec2{ point.x, point.y });

            cell->polygon_array.append(gdstk_poly);
        }

        return lib;
   }
}


static std::vector<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>> src_dst_boxes =
{
    // 1ere transformation
    {
        {cv::Point2f(-100000.0f, 50000.0f),
        cv::Point2f(-100000.0f, 0.0f),
        cv::Point2f(-50000.0f, 0.0f),
        cv::Point2f(-50000.0f, 50000.0f) },

        {cv::Point2f(-90000.0f, 90000.0f),
        cv::Point2f(-110000.0f, 50000.0f),
        cv::Point2f(-50000.0f, 0.0f),
        cv::Point2f(-10000.0f, 60000.0f)} 
    },

    // 2eme transformation
     {
        {cv::Point2f(-50000.0f, 50000.0f),
        cv::Point2f(-50000.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 50000.0f) },

        {cv::Point2f(10000.0f, 50000.0f),
        cv::Point2f(20000.0f, 0.0f),
        cv::Point2f(5000.0f, 0.0f),
        cv::Point2f(0.0f, 50000.0f) }
     },

     // 3eme transformation
     {
        {cv::Point2f(0.0f, 50000.0f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(50000.0f, 0.0f),
        cv::Point2f(50000.0f, 50000.0f) },

        {cv::Point2f(10000.0f, 0.0f),
        cv::Point2f(20000.0f, -60000.0f),
        cv::Point2f(5000.0f, -60000.0f),
        cv::Point2f(0.0f, -10000.0f) }
     },

     // 4eme transformation
     {
        {cv::Point2f(50000.0f, 50000.0f),
        cv::Point2f(50000.0f, 0.0f),
        cv::Point2f(100000.0f, 0.0f),
        cv::Point2f(100000.0f, 50000.0f) },

        {cv::Point2f(0.0f, -50000.0f),
        cv::Point2f(-10000.0f, -110000.0f),
        cv::Point2f(-25000.0f, -110000.0f),
        cv::Point2f(-40000.0f, -60000.0f) }
     },

    // 5eme transformation
    {
        {cv::Point2f(-100000.0f, 0.0f),
        cv::Point2f(-100000.0f, -50000.0f),
        cv::Point2f(-50000.0f, -50000.0f),
        cv::Point2f(-50000.0f, 0.0f) },

        {cv::Point2f(-20000.0f, 90000.0f),
        cv::Point2f(-40000.0f, 50000.0f),
        cv::Point2f(20000.0f, 0.0f),
        cv::Point2f(60000.0f, 60000.0f)}
    },

    // 6eme transformation
     {
        {cv::Point2f(-50000.0f, 0.0f),
        cv::Point2f(-50000.0f, -50000.0f),
        cv::Point2f(0.0f, -50000.0f),
        cv::Point2f(0.0f, 0.0f) },

        {cv::Point2f(80000.0f, 50000.0f),
        cv::Point2f(100000.0f, 0.0f),
        cv::Point2f(75000.0f, 0.0f),
        cv::Point2f(70000.0f, 50000.0f) }
     },

    // 7eme transformation
    {
       {cv::Point2f(0.0f, 0.0f),
       cv::Point2f(0.0f, -50000.0f),
       cv::Point2f(50000.0f, -50000.0f),
       cv::Point2f(50000.0f, 0.0f) },

       {cv::Point2f(80000.0f, 0.0f),
       cv::Point2f(100000.0f, -60000.0f),
       cv::Point2f(75000.0f, -60000.0f),
       cv::Point2f(70000.0f, -10000.0f) }
    },

    // 8eme transformation
    {
       {cv::Point2f(50000.0f, 0.0f),
       cv::Point2f(50000.0f, -50000.0f),
       cv::Point2f(100000.0f, -50000.0f),
       cv::Point2f(100000.0f, 0.0f) },

       {cv::Point2f(70000.0f, -50000.0f),
       cv::Point2f(60000.0f, -110000.0f),
       cv::Point2f(45000.0f, -110000.0f),
       cv::Point2f(30000.0f, -60000.0f) }
    }
};