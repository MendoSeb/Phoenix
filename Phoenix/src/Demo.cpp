#include "Demo.h"
#include <Clipper2Utils.h>
#include <Warping.h>
#include <Utilities.h>
#include "Optix.h"
#include <ODB++Parser.h>
#include "cudaCall.h"
#include <ImageToPolygons.h>


void Demo::clipper2Demo()
{
	std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/";

	// duplicate
	Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
	GdstkUtils::RepeatAndTranslateGdstk(lib, 4, 3, 12, 12); // pour solder.gds
	GdstkUtils::Normalize(lib, Vec2{ 120.0f, 90.0f });

	PathsD paths = Clipper2Utils::ConvertGdstkPolygonsToPathsD(lib);

	// structures qui vont être utilisés en passage par référence
	PolyTreeD u, inverse, deg, diff;
	Library u_lib{}, inverse_lib{}, deg_lib{}, diff_lib{}, u_lib2{}, clipper2_lib{}, clipper2_inverse_lib{};

	// union
	Clipper2Utils::MakeUnion(paths, u);
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(u, u_lib);
	GdstkUtils::SaveToGdsii(u_lib, (root_path + "union.gds").c_str(), false);
	paths.clear();

	// inverse
	/*Clipper2Utils::MakeInverse(u, inverse);
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(inverse, inverse_lib);
	GdstkUtils::SaveToGdsii(inverse_lib, (root_path + "inverse.gds").c_str(), false); */

	/* std::vector<Library> clipper2_layers = {clipper2_lib};
	Utils::WriteLibraryToObj(clipper2_layers, (root_path + "triangulation_mono_couche.obj").c_str());
	clipper2_lib.clear();

	Clipper2Utils::MakeTriangulationPolyTree(inverse, clipper2_inverse_lib);
	clipper2_layers = { clipper2_inverse_lib };
	Utils::WriteLibraryToObj(clipper2_layers, (root_path + "triangulation_mono_couche_inverse.obj").c_str());
	clipper2_inverse_lib.clear(); */
	inverse.Clear();

	/*
	// degraissement
	Clipper2Utils::MakeDegraissement(u, -1, deg);
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(deg, deg_lib);
	GdstkUtils::SaveToGdsii(deg_lib, (root_path + "degraissement.gds").c_str(), false);

	// difference
	Clipper2Utils::MakeDifference(u, deg, diff);
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(diff, diff_lib);
	GdstkUtils::SaveToGdsii(diff_lib, (root_path + "difference.gds").c_str(), false);
	deg.Clear();
	diff.Clear();*/

	// triangulation of union in one layer with earcut
	GdstkUtils::MakeFracture(u_lib, 8190);
	std::vector<Library> union_layer = { u_lib };
	std::vector<earcutLayer> tris = Utils::EarcutTriangulation(union_layer);
	Utils::WriteLayersObj(tris, (root_path + "triangulation_mono_couche_earcut.obj").c_str());
	tris.clear();

	// triangulation of union in several layers with earcut
	std::vector<Library> union_layers = Clipper2Utils::ConvertPolyTreeDToGdsiiLayers(u);
	tris = Utils::EarcutTriangulation(union_layers);
	Utils::WriteLayersObj(tris, (root_path + "triangulation_multi_couches_earcut.obj").c_str());

	u_lib.free_all();
}

void Demo::optixDemo()
{
	Optix o(10000, 10000); // 4096, 2176
	o.init();
	o.loadShaders();

	CUdeviceptr d_tris = o.initScene();
	o.initPipeline(d_tris);

	o.render();

	//o.DMDSimulation();
	//o.DMDSimulationV2();
	//o.maxRender();
}

/*
void Demo::threadWarpingDemo
(
	Library& lib,
	int index_box,
	std::vector<std::vector<cv::Point2f>>& polys_in_box,
	std::mutex& mutex
)
{
	std::vector<std::vector<cv::Point2f>> warped_polys = Warping::FindPolygonesInBox(lib, src_dst_boxes[index_box].first);

	// find transformation matrix from src_box to dst_box
	cv::Mat warp = cv::getPerspectiveTransform(src_dst_boxes[index_box].first, src_dst_boxes[index_box].second);

	// apply matrix to points in src_box
	std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();

	for (std::vector<cv::Point2f>& poly : warped_polys)
		cv::perspectiveTransform(poly, poly, warp);

	// ajouter les polygones transformés dans la liste globale
	mutex.lock();
	polys_in_box.insert(polys_in_box.end(), warped_polys.begin(), warped_polys.end());
	mutex.unlock();

	std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
	std::cout << "Thread:" << index_box << " ,Apply transformation fait en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << " ms" << std::endl;
}


void Demo::warpingDemo1()
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	int NB_ITERATION = 1;

	for (int i = 0; i < NB_ITERATION; i++)
	{
		const char* filename = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/union_mono_couche_triangulee.gds";
		Library lib = GdstkUtils::LoadGDS(filename);
		std::vector<std::vector<cv::Point2f>> polys_in_box;

		std::thread threads[8];
		std::mutex mutex;

		// déformer pour chaque paire de boite (src, dst)
		for (int i = 0; i < src_dst_boxes.size(); i++)
		{
			threads[i] = std::thread(
				threadWarpingDemo,
				std::ref(lib),
				i,
				std::ref(polys_in_box),
				std::ref(mutex)
			);
		}

		for (int i = 0; i < src_dst_boxes.size(); i++)
			threads[i].join();

		lib.clear();

		//GdstkUtils::SaveToGdsii(polys_in_box, "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/warp_test.gds");
		std::pair<float3*, uint3*> vertices_tris = Utils::GetVertexAndTriangles(polys_in_box);
		free(vertices_tris.first);
		free(vertices_tris.second);
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "WarpingDemo faite en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / NB_ITERATION << " ms" << std::endl;
}


void Demo::warpingDemo2()
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	int NB_ITERATION = 100;

	for (int i = 0; i < NB_ITERATION; i++)
	{
		const char* filename = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/union.gds";
		Library lib = GdstkUtils::LoadGDS(filename);
		std::vector<std::vector<cv::Point2f>> polys_in_box;

		std::thread threads[8];
		std::mutex mutex;

		// déformer pour chaque paire de boite (src, dst)
		for (int i = 0; i < src_dst_boxes.size(); i++)
		{
			threads[i] = std::thread(
				threadWarpingDemo,
				std::ref(lib),
				i,
				std::ref(polys_in_box),
				std::ref(mutex)
			);
		}

		for (int i = 0; i < src_dst_boxes.size(); i++)
			threads[i].join();

		lib.clear();

		// triangulation earcut après tranformation
		Library lib2 = Warping::ConvertOpenCVPolygonesToGdstk(polys_in_box);
		GdstkUtils::MakeFracture(lib2);

		std::vector<Library> union_layer = { lib2 };
		std::vector<earcutLayer> pair = Utils::EarcutTriangulation(union_layer);
		//Utils::WriteLayersObj(pair, "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/demo2.obj");
		//GdstkUtils::SaveToGdsii(lib2, "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/warp_test.gds", false);
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "WarpingDemo2 faite en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / NB_ITERATION << " ms" << std::endl;
}


void Demo::warpingDemo3()
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	int NB_ITERATION = 1;

	for (int i = 0; i < NB_ITERATION; i++)
	{
		const char* filename = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_mono_couche_earcut.obj";
		std::pair<std::vector<cv::Point2f>, std::vector<uint3>> obj = Utils::LoadObjVerticesTriangles(filename);

		for (int k = 0; k < 8; k++)
		{
			std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();

			cv::Mat warp = cv::getPerspectiveTransform(src_dst_boxes[k].first, src_dst_boxes[k].second);
			Warping::TransformVerticesInBox(obj, src_dst_boxes[k].second, warp);

			std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
			std::cout << "Transform: " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << " ms" << std::endl;
		}
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Demo 3 faite en : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / NB_ITERATION << " ms" << std::endl;
}
*/

void Demo::odbDemo()
{
	//Feature feature = ODB::readFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/assemt/features");
	//Feature feature = ODB::readFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/signal_8/features");
	//Feature feature = ODB::readFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/flex_6/features");
	//Feature feature = ODB::readFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/signal_7/features");

	std::string folder = "C:/Users/PC/Downloads/designodb_rigidflex/";
	std::map<std::string, std::vector<Polygon*>> symbols = ODB::readSymbols(folder);
	std::vector<Library> libs = ODB::readLayers(folder, symbols);

	/*Feature f = ODB::readFeatureFile(
		"C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/soldermask_top/features");

	Cell* c = ODB::convertODBToPolygons(f, symbols);

	Library lib = {};
	lib.init("library", 1e-6, 1e-9);
	lib.cell_array.append(c);
	GdstkUtils::Scale(lib, 1e5);

	PathsD paths = Clipper2Utils::ConvertGdstkPolygonsToPathsD(lib);

	PolyTreeD u;
	Library u_lib = {};
	Clipper2Utils::MakeUnion(paths, u);
	Clipper2Utils::ConvertPolyTreeDToGdsiiPath(u, u_lib);

	GdstkUtils::SaveToGdsii(u_lib, "C:/Users/PC/Desktop/poc/fichiers_gdsii/odb/test.gds", false);*/
}

void Demo::essaiClient()
{
	std::string file1 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/0 - Image Primaire PHC Mire Externe.gds";
	std::string file2 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/58a0_Solder CENTRE TROUS BOTTOM.gds";
	std::string file3 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/004672-D7720058a0.bot_CENTRE_Nettoyé.gds";

	Library i1_lib = GdstkUtils::LoadGDS(file1.c_str());
	Library i2_lib = GdstkUtils::LoadGDS(file2.c_str());
	Library i3_lib = GdstkUtils::LoadGDS(file3.c_str());

	//GdstkUtils::Normalize(i1_lib, Vec{});

	// pour pourvoir appliquer le sbons dégraissement
	using namespace Clipper2Utils;

	std::unique_ptr<PolyTreeD> i1 = MakeUnion(Clipper2Utils::ConvertGdstkPolygonsToPolyTreeD(i1_lib));
	std::unique_ptr<PolyTreeD> i2 = MakeUnion(Clipper2Utils::ConvertGdstkPolygonsToPolyTreeD(i2_lib));
	std::unique_ptr<PolyTreeD> i3 = MakeUnion(Clipper2Utils::ConvertGdstkPolygonsToPolyTreeD(i3_lib));

	// MirrorY(union(i1, Diff(Size(i2, -0.03), Size(i2, -0.07))))
	// MirrorY(union(i1,Size(diff(i2,interSize(i2,i3,-0.04)),-0.07)))
	// MirrorY(union(i1,Size(diff(i2,interSize(i2,i3,-0.06)),-0.03)))
	// MirrorY(union(i1,Size(i2,-0.03)))
	// MirrorY(union(i1,Size(inter(i2,interSize(i2,i3,0.06)),-0.03)))

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	std::vector<std::unique_ptr<PolyTreeD>> operations;
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDifference(MakeDegraissement(i2, -30), MakeDegraissement(i2, -70)))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(MakeDifference(i2, MakeDegraissement(MakeIntersection(i2, i3), -40)), -70))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(MakeDifference(i2, MakeDegraissement(MakeIntersection(i2, i3), -60)), -30))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(i2, -30))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(MakeIntersection(i2, MakeDegraissement(MakeIntersection(i2, i3), 60)), -30))));

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Opérations faites en: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;

	int i = 0;
	for (auto& op : operations)
	{
		Library lib = {};
		Clipper2Utils::ConvertPolyTreeDToGdsiiPath2(op, lib);
		GdstkUtils::SaveToGdsii(lib, ("C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test" + std::to_string(i) + ".gds").c_str(), false);
		i++;
	}

	end = std::chrono::steady_clock::now();
	std::cout << "total en: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
}

void Demo::essaiClientComparison()
{
	std::vector<std::string> artwork_files = {
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/marion/01_58a0_MIRES & CONTOUR SOLDER BOTTOM MY.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/marion/02_58a0_MIRES & COEUR REDUIT 70 BOTTOM MY.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/marion/03_58a0_MIRES & COEUR ETENDU 60 BOTTOM MY.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/marion/04_58a0_MIRES & SOLDER TROUS 30 BOTTOM MY.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/marion/05_58a0_MIRES & SOLDER TROUS SUR CUIVRE ETENDU 30 BOTTOM MY.gds"
	};

	std::vector<std::string> phoenix_files = {
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test0.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test1.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test2.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test3.gds",
		"C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test4.gds"
	};

	using namespace Clipper2Utils;
	int index = 0;

	for (int i = 0; i < 5; i++)
	{
		Library la = GdstkUtils::LoadGDS(artwork_files[i].c_str());
		Library lp = GdstkUtils::LoadGDS(phoenix_files[i].c_str());

		std::unique_ptr<PolyTreeD> pa = MakeUnion(Clipper2Utils::ConvertGdstkPolygonsToPolyTreeD(la));
		std::unique_ptr<PolyTreeD> pp = MakeUnion(Clipper2Utils::ConvertGdstkPolygonsToPolyTreeD(lp));

		std::unique_ptr<PolyTreeD> output = MakeDifference(pa, pp);

		Library lib = {};
		Clipper2Utils::ConvertPolyTreeDToGdsiiPath2(output, lib);
		GdstkUtils::SaveToGdsii(lib, ("C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/comparaison_artwork_phoenix/comparaison_" + std::to_string(index) + ".gds").c_str(), false);
		index++;
	}
}

void Demo::BMPToGDS()
{
	auto start = std::chrono::steady_clock::now();

	ImageToPolygons::ConvertBMPToPolygons("C:/Users/PC/Desktop/poc/mgi_logo.jpg");

	auto end = std::chrono::steady_clock::now();
	std::cout << "Image to .gds " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void Demo::MultiLayerRasterization(float2 circuit_inch_size, int dpi)
{	
	auto start3 = std::chrono::steady_clock::now();

	/// 1: triangulate every layers
	//const char* svg_filepath = "C:/Users/PC/Desktop/poc/test.svg";
	const char* svg_filepath = "C:/Users/PC/Desktop/poc/004672-647720058a0 (1).top_LAYERS.svg";
	std::vector<earcutPolys> polys_layers = Utils::ConvertSVGToEarcutLayers(svg_filepath);
	std::vector<earcutLayer> triangulation_layers;

	for (earcutPolys& polys : polys_layers)
	{
		earcutLayer tri_layer = Utils::earcutTriangulation(polys);
		triangulation_layers.push_back(tri_layer);
	}

	Utils::Triangulation t = Utils::convertEarcutLayersToPointer(triangulation_layers);

	uint2 img_dim = { (int)circuit_inch_size.x * dpi, (int)circuit_inch_size.y * dpi };
	printf("Image dimension: %i x %i\n", img_dim.x, img_dim.y);
	printf("Taille d'un pixel en microns: %0.1f\n", (25.4f / (float)dpi) * 1e3f);

	/// 2: scale triangles to simplify rasterization
	float scale = std::max(img_dim.x, img_dim.y);
	Utils::ScaleTriangulation(t, scale);

	/// 3: gpu memory allocation of vertices, triangles and the polarity of the triangles
	Utils::Triangulation dtriangulation;
	auto start = std::chrono::steady_clock::now();

	printf("Vertices: %0.2f Mo\n", (float)(t.nb_vertices * sizeof(float2)) / 1e6f);
	printf("Triangles: %0.2f Mo\n", (float)(t.nb_triangles * sizeof(uint3)) / 1e6f);
	printf("Polarity: %0.2f Mo\n", (float)(t.nb_triangles * sizeof(unsigned char)) / 1e6f);

	cudaMalloc((void**)&dtriangulation.v, t.nb_vertices * sizeof(float2));
	cudaMalloc((void**)&dtriangulation.t, t.nb_triangles * sizeof(uint3));
	cudaMalloc((void**)&dtriangulation.p, t.nb_triangles * sizeof(unsigned char));
	cudaMemcpy(dtriangulation.v, t.v, t.nb_vertices * sizeof(float2), cudaMemcpyHostToDevice);
	cudaMemcpy(dtriangulation.t, t.t, t.nb_triangles * sizeof(uint3), cudaMemcpyHostToDevice);
	cudaMemcpy(dtriangulation.p, t.p, t.nb_triangles * sizeof(unsigned char), cudaMemcpyHostToDevice);

	auto end = std::chrono::steady_clock::now();
	std::cout << "! creation contexte + allocation vram en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

	dtriangulation.nb_vertices = t.nb_vertices;
	dtriangulation.nb_triangles = t.nb_triangles;
	dtriangulation.layers_range = t.layers_range;

	/// 4: free host memory
	cudaFree(t.v);
	cudaFree(t.t);
	cudaFree(t.p);

	/// 5: warp and rasterize circuit
	//CudaCall::Warping(dtriangulation, src_dst_boxes);
	unsigned char* img = CudaCall::Rasterization(dtriangulation, scale, img_dim);
	CudaCall::saveToBmp(
		("C:/Users/PC/Desktop/poc/rasterization_" + std::to_string(dpi) + "dpi.bmp").c_str(), 
		img_dim.x, img_dim.y, 
		img);

	/// 6: free vram memory
	cudaFree(dtriangulation.v);
	cudaFree(dtriangulation.t);
	cudaFree(dtriangulation.p);
	delete[] img;

	auto end3 = std::chrono::steady_clock::now();
	std::cout << "! Total en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start3).count() << " ms" << std::endl;
}