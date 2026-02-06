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
#include "Warping.h"
#include "ODB++Parser.h"


void BoostGeometryDemo()
{
	double degraissement = -1;
	std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/boost/";

	//Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
	Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/0 - Image Solder PHC.gds");
	//Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/simple.gds");

	GdstkUtils::RepeatAndTranslateGdstk(lib, 1, 1, 12, 12);
	GdstkUtils::Normalize(lib, Vec2{10, 10});
	multi_polygon_t polys = BoostUtils::ConvertGdstkToBoostPolygon(lib);

	multi_polygon_t u_polys = BoostUtils::MakeUnion(polys);
	BoostUtils::ConvertBoostPolygonToGdstk(u_polys, (root_path + "union.gds").c_str());
}


void Clipper2Demo()
{
	std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/";

	// duplicate
	Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
	GdstkUtils::RepeatAndTranslateGdstk(lib, 4, 3, 12, 12); // pour solder.gds
	GdstkUtils::Normalize(lib, Vec2{120.0f, 90.0f});

	Paths64 paths = Clipper2Utils::ConvertGdstkPolygonsToPaths64(lib);

	// structures qui vont être utilisés en passage par référence
	PolyTree64 u, inverse, deg, diff;
	Library u_lib{}, inverse_lib{}, deg_lib{}, diff_lib{}, u_lib2{}, clipper2_lib{}, clipper2_inverse_lib{};

	// union
	Clipper2Utils::MakeUnionPolyTree(paths, u);
	Clipper2Utils::ConvertPolyTree64ToGdsiiPath(u, u_lib);
	GdstkUtils::SaveToGdsii(u_lib, (root_path + "union.gds").c_str(), false);
	paths.clear();

	// inverse
	/*Clipper2Utils::MakeInverse(u, inverse);
	Clipper2Utils::ConvertPolyTree64ToGdsiiPath(inverse, inverse_lib);
	GdstkUtils::SaveToGdsii(inverse_lib, (root_path + "inverse.gds").c_str(), false); */

	// triangulation avec clipper2
	Clipper2Utils::MakeTriangulationPolyTree(u, clipper2_lib);
	GdstkUtils::SaveToGdsii(clipper2_lib, (root_path + "union_mono_couche_triangulee.gds").c_str(), false);

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
	Clipper2Utils::ConvertPolyTree64ToGdsiiPath(deg, deg_lib);
	GdstkUtils::SaveToGdsii(deg_lib, (root_path + "degraissement.gds").c_str(), false);

	// difference
	Clipper2Utils::MakeDifference(u, deg, diff);
	Clipper2Utils::ConvertPolyTree64ToGdsiiPath(diff, diff_lib);
	GdstkUtils::SaveToGdsii(diff_lib, (root_path + "difference.gds").c_str(), false);
	deg.Clear();
	diff.Clear();*/

	// triangulation of union in one layer with earcut
	GdstkUtils::MakeFracture(u_lib);
	std::vector<Library> union_layer = { u_lib };
	std::vector<earcutLayer> tris = Utils::EarcutTriangulation(union_layer);
	Utils::WriteLayersObj(tris, (root_path + "triangulation_mono_couche_earcut.obj").c_str());
	tris.clear();

	// triangulation of union in several layers with earcut
	std::vector<Library> union_layers = Clipper2Utils::ConvertPolyTree64ToGdsiiLayers(u);
	tris = Utils::EarcutTriangulation(union_layers);
	Utils::WriteLayersObj(tris, (root_path + "triangulation_multi_couches_earcut.obj").c_str());

	u_lib.free_all();
}


void GdstkDemo()
{
	double degraissement = -0.005;
	std::string root_path = "C:/Users/PC/Desktop/poc/fichiers_gdsii/gdstk/";

	Library lib = GdstkUtils::LoadGDS("C:/Users/PC/Desktop/poc/fichiers_gdsii/Image Primaire V2.gds");
	GdstkUtils::RepeatAndTranslateGdstk(lib, 1, 1, 12, 12); // factor moins grand qu'avec clipper car pas de conversion en int64_t
	GdstkUtils::Normalize(lib, Vec2{10, 10});

	// scale pour la précision et union
	Library u_lib = GdstkUtils::MakeUnion(lib);
	GdstkUtils::SaveToGdsii(u_lib, (root_path + "union_gdstk.gds").c_str(), false);
	lib.clear();

	// dégraissement
	Library deg = GdstkUtils::MakeDegraissement(u_lib, degraissement); // -1 pour test
	GdstkUtils::SaveToGdsii(deg, (root_path + "degraissement_gdstk.gds").c_str(), false);

	// différence
	Library diff = GdstkUtils::MakeDifference(u_lib, deg);
	GdstkUtils::SaveToGdsii(diff, (root_path + "difference_gdstk.gds").c_str(), false);
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

	CUdeviceptr d_tris = o.initScene();
	o.initPipeline(d_tris);

	//o.render();

	//o.DMDSimulation();
	o.DMDSimulationV2();
}


void ThreadWarpingDemo
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


void WarpingDemo1()
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
				ThreadWarpingDemo,
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


void WarpingDemo2()
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
				ThreadWarpingDemo,
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


void WarpingDemo3()
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


void ODBDemo()
{
	//Feature feature = ODB::ReadFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/assemt/features");
	Feature feature = ODB::ReadFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/signal_8/features");
	//Feature feature = ODB::ReadFeatureFile("C:/Users/PC/Downloads/designodb_rigidflex/steps/cellular_flip-phone/layers/signal_7/features");

	Cell* cell = ODB::ConvertODBToPolygons(feature);
	Library lib = {};
	lib.init("library", 1e-6, 1e-9);
	lib.cell_array.append(cell);
	GdstkUtils::Normalize(lib, Vec2{10, 10});

	Paths64 paths = Clipper2Utils::ConvertGdstkPolygonsToPaths64(lib);

	PolyTree64 u;
	Library u_lib = {};
	Clipper2Utils::MakeUnionPolyTree(paths, u);
	Clipper2Utils::ConvertPolyTree64ToGdsiiPath(u, u_lib);

	GdstkUtils::SaveToGdsii(u_lib, "C:/Users/PC/Desktop/poc/fichiers_gdsii/odb/test.gds", false);
}


std::vector<double> FindMinMax(cv::Mat& frame, bool text_down)
{
	std::vector<double> limits = {1e10, -1e10, 0};

	if (text_down) 
		limits[2] = 1e10; 
	else 
		limits[2] = -1e10;

	for (size_t y = 0; y < frame.rows; y++)
	{
		for (size_t x = 0; x < frame.cols; x++)
		{
			cv::Vec3b pixel = frame.at<cv::Vec3b>(y, x);

			if (pixel[0] > 0)
			{
				if (x < limits[0])
					limits[0] = x;

				if (x > limits[1])
					limits[1] = x;

				if (text_down)
				{
					if (y < limits[2])
						limits[2] = y;
				}
				else
				{
					if (y > limits[2])
						limits[2] = y;
				}
			}
		}
	}

	return limits;
}


void Video()
{
	cv::VideoCapture cap("C:/Users/PC/Desktop/poc/153p_05_22.avi");

	if (!cap.isOpened()) 
		std::cout << "Error: Could not open video file." << std::endl;
	else 
		std::cout << "Video file opened successfully!" << std::endl;

	// Read the first frame to confirm reading
	const size_t FRAME_NUMBER = cap.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
	std::cout << FRAME_NUMBER << std::endl;

	int start_index = 32;
	size_t frame_gap = 36;
	size_t cam_side = 512;
	size_t nb_target_x = 17;
	size_t nb_target_y = 9;
	size_t nb_total_targets = 153;
	size_t x = 0;
	size_t y = 0;

	// 2560 x 832
	float img_width = std::ceil(2560.0f * (260.0f / 41.0f));
	float img_height = std::ceil(832.0f * (260.0f / 41.0f));

	float x_offset = (img_width - (17.0f * 512.0f)) / 17.0f;
	float y_offset = (img_height - (9.0f * 512.0f)) / 9.0f;

	cv::Mat res(
		img_height,
		img_width,
		CV_8UC3, 
		cv::Scalar(0, 0, 0)
	);

	for (int i = 0; i < start_index; i++)
		cap.grab();

	for (int i = 0; i < nb_total_targets; i++)
	{
		std::cout << "frame index: " << i << std::endl;

		cv::Mat frame;
		cap.read(frame);

		int offset = 110;
		cv::Mat cropped = frame(
			cv::Rect(offset, offset, 512 - 2 * offset, 512 - 2 * offset)
		);
		
		std::vector<double> limits = FindMinMax(cropped, i < 68);
		
		int x1 = limits[0];
		int y1 = limits[2];
		int w1 = limits[1] - limits[0];
		int h1 = w1;

		if (i >= 68)
			y1 = limits[2] - h1;

		cv::Mat cropped2 = cropped(cv::Rect(x1, y1, w1, h1));

		cv::imwrite(
			("C:/Users/PC/Desktop/poc/targets/" + std::to_string(i) + ".png").c_str(), 
			frame
		);

		//cv::Rect roi(x, y, frame.cols, frame.rows);
		//frame.copyTo(res(roi));

		int temp = frame_gap;

		if (i % (nb_target_x * 3) == 0 && i != 0)
			temp -= 30;

		for (int k = 0; k < temp; k++)
			cap.grab();

		x = (size_t)(x + cam_side + x_offset) % (size_t)(nb_target_x * (cam_side + x_offset));

		if (x == 0)
			y += cam_side + y_offset;
	}

	cv::imwrite("C:/Users/PC/Desktop/poc/res.png", res);
}


void Onehourdestroyer()
{
	// score final
	// opérations en 87 s
	// total (opérations + sauvegarde en .gds) en 140 s

	std::string file1 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/0 - Image Primaire PHC Mire Externe.gds";
	std::string file2 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/58a0_Solder CENTRE TROUS BOTTOM.gds";
	std::string file3 = "C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/004672-647720058a0.bot_CENTRE_Nettoyé.gds";

	Library i1_lib = GdstkUtils::LoadGDS(file1.c_str());
	Library i2_lib = GdstkUtils::LoadGDS(file2.c_str());
	Library i3_lib = GdstkUtils::LoadGDS(file3.c_str());

	// pour pourvoir appliquer le sbons dégraissement
	GdstkUtils::Scale(i1_lib, 100);
	GdstkUtils::Scale(i2_lib, 100);
	GdstkUtils::Scale(i3_lib, 100);

	std::unique_ptr<PolyTree64> i1 = Clipper2Utils::ConvertGdstkPolygonsToPolyTree64(i1_lib);
	std::unique_ptr<PolyTree64> i2 = Clipper2Utils::ConvertGdstkPolygonsToPolyTree64(i2_lib);
	std::unique_ptr<PolyTree64> i3 = Clipper2Utils::ConvertGdstkPolygonsToPolyTree64(i3_lib);
	
	// MirrorY(union(i1, Diff(Size(i2, -0.03), Size(i2, -0.07))))
	// MirrorY(union(i1,Size(diff(i2,interSize(i2,i3,-0.04)),-0.07)))
	// MirrorY(union(i1,Size(diff(i2,interSize(i2,i3,-0.06)),-0.03)))
	// MirrorY(union(i1,Size(i2,-0.03)))
	// MirrorY(union(i1,Size(inter(i2,interSize(i2,i3,0.06)),-0.03)))

	using namespace Clipper2Utils;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	std::vector<std::unique_ptr<PolyTree64>> operations;
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDifference(MakeDegraissement(i2, -3), MakeDegraissement(i3, -7)))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(MakeDifference(i2, MakeDegraissement(MakeIntersection(i2, i3), -4)), -7))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(MakeDifference(i2, MakeDegraissement(MakeIntersection(i2, i3), 6)), -3))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(i2, -3))));
	operations.push_back(MakeMirrorY(MakeUnion(i1, MakeDegraissement(MakeIntersection(i2, MakeDegraissement(MakeIntersection(i2, i3), 6)), -3))));

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Opérations faites en: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;

	int i = 0;
	for (auto& op : operations)
	{
		Library lib = {};
		Clipper2Utils::ConvertPolyTree64ToGdsiiPath2(op, lib);
		GdstkUtils::Scale(lib, 0.01);
		GdstkUtils::SaveToGdsii(lib, ("C:/Users/PC/Desktop/poc/fichiers_gdsii/essai_client/test" + std::to_string(i) + ".gds").c_str(), false);
		i++;
	}

	end = std::chrono::steady_clock::now();
	std::cout << "total en: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
}


int main()
{
	//BoostGeometryDemo();
	//Clipper2Demo();
	//GdstkDemo();

	//OptixDemo();

	//WarpingDemo1();
	//WarpingDemo2();
	//WarpingDemo3();

	//ODBDemo();

	//Video();

	Onehourdestroyer();

	return 0;
}
