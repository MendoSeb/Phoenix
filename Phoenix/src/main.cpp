#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>  

#include "BoostUtils.h"
#include "Clipper2Utils.h"
#include "GdstkUtils.h"
#include "earcut.hpp"
#include "Utilities.h"
#include "Optix.h"


void BoostGeometryDemo()
{
    double degraissement = -1;
    std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/boost/";

    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Solder PHC.gds);
    Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/simple.gds");

    GdstkUtils::RepeatAndTranslateGdstk(lib, 1, 1, 12, 12);
    GdstkUtils::Normalize(lib);
    multi_polygon_t polys = BoostUtils::ConvertGdstkToBoostPolygon(lib);

    multi_polygon_t u_polys = BoostUtils::MakeUnion(polys);
    BoostUtils::ConvertBoostPolygonToGdstk(u_polys, (root_path + "union.gds").c_str());
}


void Clipper2Demo()
{
    std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/";
    //std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/";

    // duplicate
    Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Solder PHC.gds");
    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Legend PHC.gds",);
    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Percage PHC SUP 250.gds");
    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/simple.gds");
    //Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/square.gds");
    GdstkUtils::RepeatAndTranslateGdstk(lib, 4, 3, 12, 12); // pour solder.gds
    //RepeatAndTranslateGdstk(lib, 4, 3, 300000, 300000); // pour solder.gds
    GdstkUtils::Normalize(lib);

    Paths64 paths = Clipper2Utils::ConvertGdstkPolygonsToPaths64(lib);

    // structures qui vont être utilisés en passage par référence
    PolyTree64 u, inverse, deg, diff;
    Library u_lib{}, inverse_lib{}, deg_lib{}, diff_lib{}, u_lib2{}, clipper2_lib{}, clipper2_inverse_lib{};

    // union
    Clipper2Utils::MakeUnionPolyTree(paths, u);
    Clipper2Utils::ConvertPolyTree64ToGdsiiPath(u, u_lib, 0);
    GdstkUtils::SaveToGdsii(u_lib, (root_path + "union.gds").c_str());
    paths.clear();

    // inverse
    Clipper2Utils::MakeInverse(u, inverse);
    Clipper2Utils::ConvertPolyTree64ToGdsiiPath(inverse, inverse_lib, 0);
    GdstkUtils::SaveToGdsii(inverse_lib, (root_path + "inverse.gds").c_str());

    Clipper2Utils::MakeTriangulationPolyTree(u, clipper2_lib);
    std::vector<Library> clipper2_layers = { clipper2_lib };
    Utils::WriteLibraryToObj(clipper2_layers, (root_path + "triangulation_mono_couche_clipper2.obj").c_str());
    clipper2_lib.clear();

    Clipper2Utils::MakeTriangulationPolyTree(inverse, clipper2_inverse_lib);
    clipper2_layers = { clipper2_inverse_lib };
    Utils::WriteLibraryToObj(clipper2_layers, (root_path + "triangulation_mono_couche_inverse_clipper2.obj").c_str());
    clipper2_inverse_lib.clear();
    inverse.Clear();

    // degraissement
    Clipper2Utils::MakeDegraissement(u, -1, deg);
    Clipper2Utils::ConvertPolyTree64ToGdsiiPath(deg, deg_lib, 0);
    GdstkUtils::SaveToGdsii(deg_lib, (root_path + "degraissement.gds").c_str());

    // difference
    Clipper2Utils::MakeDifference(u, deg, diff);
    Clipper2Utils::ConvertPolyTree64ToGdsiiPath(diff, diff_lib, 0);
    GdstkUtils::SaveToGdsii(diff_lib, (root_path + "difference.gds").c_str());
    deg.Clear();
    diff.Clear();

    // triangulation of union in one layer with earcutt
    std::vector<Library> union_layer = { u_lib };
    std::vector<earcutLayer> pair = Utils::EarcutTriangulation(union_layer);
    Utils::WriteLayersObj(pair, (root_path + "triangulation_mono_couche_earcut.obj").c_str());

    // triangulation of union in several layers with earcutt
    std::vector<Library> union_layers = Clipper2Utils::ConvertPolyTree64ToGdsiiLayers(u);
    std::vector<earcutLayer> tris = Utils::EarcutTriangulation(union_layers);
    Utils::WriteLayersObj(tris, (root_path + "triangulation_multi_couches_earcut.obj").c_str());

    u_lib.free_all();
}


void GdstkDemo()
{
    double degraissement = -0.005;
    std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/gdstk/";

    Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
    GdstkUtils::RepeatAndTranslateGdstk(lib, 4, 3, 12, 12); // factor moins grand qu'avec clipper car pas de conversion en int64_t
    
    // scale pour la précision et union
    Library u_lib = GdstkUtils::MakeUnion(lib);
    GdstkUtils::SaveToGdsii(u_lib, (root_path + "union_gdstk.gds").c_str());
    lib.clear();

    // dégraissement
    Library deg = GdstkUtils::MakeDegraissement(u_lib, degraissement); // -1 pour test
    GdstkUtils::SaveToGdsii(deg, (root_path + "degraissement_gdstk.gds").c_str());

    // différence
    Library diff = GdstkUtils::MakeDifference(u_lib, deg);
    GdstkUtils::SaveToGdsii(diff, (root_path + "difference_gdstk.gds").c_str());
    diff.clear();
    deg.clear();

    // triangulation of union in one layer with earcutt
    std::vector<Library> union_layer = { u_lib };
    std::vector<earcutLayer> pair = Utils::EarcutTriangulation(union_layer);
    Utils::WriteLayersObj(pair, (root_path + "triangulation_full.obj").c_str());

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


void TriangulateWithoutUnion()
{
    Library lib1 = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
    GdstkUtils::RepeatAndTranslateGdstk(lib1, 4, 3, 12, 12);
    GdstkUtils::Normalize(lib1);
    Paths64 paths = Clipper2Utils::ConvertGdstkPolygonsToPaths64(lib1);

    Library lib = {};
    Clipper2Utils::MakeTriangulationPaths(paths, lib);

    std::vector<Library> layers = { lib };
    Utils::WriteLibraryToObj(layers, "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_clipper2_multi_couche_v2.obj");
}


int main()
{
    //BoostGeometryDemo();
    //Clipper2Demo();
    //GdstkDemo();

    //TriangulateWithoutUnion();

    //OptixDemo();

    return 0;
}
