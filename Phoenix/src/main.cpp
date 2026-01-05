#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include<string>  

#include "BoostOperators.h"
#include "Clipper2Operators.h"
#include "GdstkOperators.h"
#include "earcut.hpp"
#include "Optix.h"
#include "Utilities.cpp"

using namespace Utils;


void BoostGeometryDemo()
{
    BoostOperators bo;
    double degraissement = -0.005;
    std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/boost/";

    Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Solder PHC.gds", scale);
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/simple.gds", scale);

    repeatAndTranslate(lib, 1, 1, 12, 12);
    SaveToGdsii(lib, (root_path + "duplicated.gds").c_str());
    Library lib_ = LoadGDS((root_path + "duplicated.gds").c_str()); // pour garder la duplication

    multi_polygon_t polys = bo.ConvertGdstkToBoostPolygon(lib_);

    multi_polygon_t u_polys = bo.MakeUnion(polys);
    bo.ConvertBoostPolygonToGdstk(u_polys, (root_path + "union.gds").c_str());
}


void Clipper2Demo()
{
    Clipper2Operators co;
    double factor = 1.0; // pour passer d'un double à uint64_t
    double degraissement = -1;
    //std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/";
    std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/";

    // duplicate
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds", scale);
    Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Solder PHC.gds");
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Legend PHC.gds", scale);
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Percage PHC SUP 250.gds", scale);
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/simple.gds", scale);
    //Library lib = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/square.gds", scale);
    //Paths64 paths = RepeatAndTranslateClipper2(lib, 4, 3, 12, 12, factor); // pour primaire.gds
    Paths64 paths = RepeatAndTranslateClipper2(lib, 4, 3, 300000, 300000, factor); // pour solder.gds
    lib.free_all();

    // structures qui vont être utilisés en passage par référence
    PolyTree64 u, inverse, deg, diff;
    Library u_lib{}, inverse_lib{}, deg_lib{}, diff_lib{}, u_lib2{}, clipper2_lib{};

    // union
    co.MakeUnionPolyTree(paths, u);
    co.ConvertPolyTree64ToGdsiiPath(u, u_lib);
    SaveToGdsii(u_lib, (root_path + "union.gds").c_str());
    paths.clear();

    // inverse
    co.MakeInverse(u, inverse);
    co.ConvertPolyTree64ToGdsiiPath(inverse, inverse_lib);
    SaveToGdsii(inverse_lib, (root_path + "inverse.gds").c_str());
    inverse.Clear();

    co.MakeTriangulation(u, clipper2_lib);
    std::vector<Library> clipper2_layers = { clipper2_lib };
    WriteLibraryToObj(clipper2_layers, (root_path + "triangulation_full_clipper2.obj").c_str());
    clipper2_lib.free_all();

    // degraissement
    co.MakeDegraissement(u, degraissement, deg);
    co.ConvertPolyTree64ToGdsiiPath(deg, deg_lib);
    SaveToGdsii(deg_lib, (root_path + "degraissement.gds").c_str());

    // difference
    co.MakeDifference(u, deg, diff);
    co.ConvertPolyTree64ToGdsiiPath(diff, diff_lib);
    SaveToGdsii(diff_lib, (root_path + "difference.gds").c_str());
    deg.Clear();
    diff.Clear();

    // triangulation of union in one layer with earcutt
    std::vector<Library> union_layer = { u_lib };
    std::vector<earcutLayer> pair = Triangulation(union_layer);
    WriteLayersObj(pair, (root_path + "triangulation_full.obj").c_str());

    // triangulation of union in several layers with earcutt
    std::vector<Library> union_layers = co.ConvertPolyTree64ToGdsiiLayers(u);
    std::vector<earcutLayer> tris = Triangulation(union_layers);
    WriteLayersObj(tris, (root_path + "triangulation_layers.obj").c_str());

    u_lib.free_all();
}


void GdstkDemo()
{
    GdstkOperators go;
    double degraissement = -0.005;
    std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/gdstk/";

    Library lib_ = LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
    repeatAndTranslate(lib_, 4, 3, 12, 12);
    SaveToGdsii(lib_, (root_path + "duplicated_gdstk.gds").c_str());
    lib_.clear();

    // load duplicated circuit
    Library lib = LoadGDS((root_path + "duplicated_gdstk.gds").c_str());

    // scale pour la précision et union
    Library u_lib = go.MakeUnion(lib);
    SaveToGdsii(u_lib, (root_path + "union_gdstk.gds").c_str());
    lib.clear();

    // dégraissement
    Library deg = go.MakeDegraissement(u_lib, degraissement); // -1 pour test
    SaveToGdsii(deg, (root_path + "degraissement_gdstk.gds").c_str());
    deg.clear();

    // différence
    Library diff = go.MakeDifference(u_lib, deg);
    SaveToGdsii(diff, (root_path + "difference_gdstk.gds").c_str());
    diff.clear();

    // triangulation of union in one layer with earcutt
    std::vector<Library> union_layer = { u_lib };
    std::vector<earcutLayer> pair = Triangulation(union_layer);
    WriteLayersObj(pair, (root_path + "triangulation_full.obj").c_str());

    u_lib.clear();
}


void OptixDemo()
{
    Optix o(4096, 2176);
    o.init();
    o.loadShaders();

    CUdeviceptr d_list = o.initScene();
    o.initPipeline(d_list);

    o.render();
}


int main() 
{
    //BoostGeometryDemo();
    Clipper2Demo();
    //GdstkDemo();

    OptixDemo();

    return 0;
}