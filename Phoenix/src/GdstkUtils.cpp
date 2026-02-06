#include "GdstkUtils.h"
#include <iostream>
#include <__msvc_chrono.hpp>
#include <opencv2/core/types.hpp>


namespace GdstkUtils
{
	Library LoadGDS(const char* fileName)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		Library lib = read_gds(fileName, 1e-6, 1e-9, nullptr, nullptr);
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		std::cout << "Unit: " << lib.unit << std::endl;
		std::cout << "Precision: " << lib.precision << std::endl;
		std::cout << "Nb cell: " << lib.cell_array.count << "\n\n";

		for (size_t i = 0; i < lib.cell_array.count; i++)
		{
			std::cout << "cell: " << i << std::endl;
			std::cout << "Polygones: " << lib.cell_array[i]->polygon_array.count << std::endl;
			std::cout << "Reference: " << lib.cell_array[i]->reference_array.count << std::endl;
			std::cout << "FlexPath: " << lib.cell_array[i]->flexpath_array.count << std::endl;
			std::cout << "Robustpath: " << lib.cell_array[i]->robustpath_array.count << std::endl;
			std::cout << std::endl;
		}

		ConvertFlexPathsToPolygon(lib);

		std::cout << "Fichier charge en polygones gdstk en " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
		return lib;
	}


	void SaveToGdsii(Library& lib, const char* fileName, bool make_fracture)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		int nb_points_max = INT32_MAX;

		if (make_fracture)
			nb_points_max = 8190;

		lib.write_gds(fileName, nb_points_max, NULL);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Sauvegarde vers " << fileName << " faite en: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
	}


	void SaveToGdsii(std::vector<std::vector<cv::Point2f>>& polys, const char* fileName)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		Library lib = {};
		lib.init("library", 1e-6, 1e-9);

		Cell cell = {};
		cell.name = copy_string("FIRST", NULL);
		lib.cell_array.append(&cell);

		for (auto& poly : polys)
		{
			Polygon* gdstk_poly = (Polygon*)allocate_clear(sizeof(Polygon));

			for (auto& point : poly)
				gdstk_poly->point_array.append(Vec2{ point.x, point.y });

			cell.polygon_array.append(gdstk_poly);
		}

		lib.write_gds(fileName, INT32_MAX, NULL);
		lib.clear();

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Sauvegarde gdsii OpenCV faite en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	}


	void ConvertFlexPathsToPolygon(Library& lib)
	{
		for (size_t i = 0; i < lib.cell_array.count; i++)
		{
			Cell* c = lib.cell_array[i];

			// flexpath
			for (size_t k = 0; k < c->flexpath_array.count; k++)
			{
				FlexPath* fp = c->flexpath_array[k];

				// pour les cercles
				if (fp->spine.point_array.count == 2
					&& (fp->spine.point_array[0].y == fp->spine.point_array[1].y)
					&& ((fp->spine.point_array[0].x - fp->spine.point_array[1].x) < 2e-5))
				{
					double max_radius = std::max(fp->elements->half_width_and_offset[0].x, fp->elements->half_width_and_offset[0].y);

					gdstk::Polygon* circle = new gdstk::Polygon(gdstk::ellipse(
						fp->spine.point_array[0],
						max_radius,
						max_radius,
						0.0, 0.0,
						0.0, 0.0,
						1e-4, // précision de la discrétisation du cercle
						0
					));

					c->polygon_array.append(circle);
					continue;
				}
				// non circle path
				else
					fp->to_polygons(false, 0, lib.cell_array[i]->polygon_array);
			}

			// destroy flexpath, they are now polygons
			lib.cell_array[i]->flexpath_array.clear();
		}
	}


	void RepeatAndTranslateGdstkNoTransformV2(Library& lib, int rep_x, int rep_y, double width, double height)
	{
		for (size_t i = 0; i < lib.cell_array.count; i++)
		{
			// polygones
			for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
			{
				Polygon* poly = lib.cell_array[i]->polygon_array[k];

				poly->repetition = Repetition{
					RepetitionType::Rectangular,
					(uint64_t)rep_x, (uint64_t)rep_y,
					(double)width, (double)height
				};
			}

			// flexpath
			for (size_t k = 0; k < lib.cell_array[i]->flexpath_array.count; k++)
			{
				FlexPath* flex_path = lib.cell_array[i]->flexpath_array[k];

				flex_path->repetition = Repetition{
					RepetitionType::Rectangular,
					(uint64_t)rep_x, (uint64_t)rep_y,
					(double)width, (double)height
				};
			}
		}
	}


	void RepeatAndTranslateGdstkNoTransformV1(Library& lib, int rep_x, int rep_y, double width, double height)
	{
		Cell* cell = (Cell*)allocate_clear(sizeof(Cell));
		cell->name = copy_string("FIRST", NULL);

		for (size_t i = 0; i < lib.cell_array.count; i++)
		{
			// polygones
			for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
			{
				Polygon* main_poly = lib.cell_array[i]->polygon_array[k];

				// duplicate
				for (size_t x = 0; x < rep_x; x++)
					for (size_t y = 0; y < rep_y; y++)
					{
						Polygon* new_poly = (Polygon*)allocate_clear(sizeof(Polygon));
						new_poly->point_array.copy_from(main_poly->point_array);
						new_poly->translate(Vec2{ x * width, y * height });
						cell->polygon_array.append(new_poly);
					}
			}

			// flexpath
			for (size_t k = 0; k < lib.cell_array[i]->flexpath_array.count; k++)
			{
				FlexPath* main_flex_path = lib.cell_array[i]->flexpath_array[k];

				// duplicate
				for (size_t x = 0; x < rep_x; x++)
					for (size_t y = 0; y < rep_y; y++)
					{
						FlexPath* new_flex_path = (FlexPath*)allocate_clear(sizeof(FlexPath));
						new_flex_path->copy_from(*main_flex_path);
						new_flex_path->translate(Vec2{ x * width, y * height });

						if (main_flex_path->spine.point_array.count == 2
							&& (main_flex_path->spine.point_array[0] == main_flex_path->spine.point_array[1]))
						{
							new_flex_path->spine.point_array[1] += Vec2{ 1e-5, 0 };
						}

						cell->flexpath_array.append(new_flex_path);
					}
			}
		}

		lib.cell_array.clear();
		lib.cell_array.append(cell);
		std::cout << "Polygones dupliques en polygones gdstk" << std::endl;
	}


	void RepeatAndTranslateGdstk(Library& lib, int rep_x, int rep_y, double width, double height)
	{
		Cell* cell = (Cell*)allocate_clear(sizeof(Cell));
		cell->name = copy_string("FIRST", NULL);

		for (size_t i = 0; i < lib.cell_array.count; i++)
			for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
			{
				Polygon* main_poly = lib.cell_array[i]->polygon_array[k];

				// duplicate
				for (size_t x = 0; x < rep_x; x++)
					for (size_t y = 0; y < rep_y; y++)
					{
						Polygon* new_poly = (Polygon*)allocate_clear(sizeof(Polygon));
						new_poly->point_array.copy_from(main_poly->point_array);
						new_poly->translate(Vec2{ x * width, y * height });
						cell->polygon_array.append(new_poly);
					}
			}

		lib.cell_array.clear();
		lib.cell_array.append(cell);
		std::cout << "Polygones dupliques en polygones gdstk" << std::endl;
	}


	void Normalize(Library& lib, Vec2&& real_dim_cm)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		assert(lib.cell_array.count == 1);
		double min = DBL_MAX;
		double max = DBL_MIN;
		double micron = 0.0001; // in cm

		// nombre de miroirs dmd qu'on peut caler dans le circuit dans sa plus grande dimension
		double scale = std::max(real_dim_cm.x / micron, real_dim_cm.y / micron) / 2.0;

		// find minimum and maximum coordinate
		for (size_t i = 0; i < lib.cell_array[0]->polygon_array.count; i++)
			for (size_t k = 0; k < lib.cell_array[0]->polygon_array[i]->point_array.count; k++)
			{
				Vec2& p = lib.cell_array[0]->polygon_array[i]->point_array[k];
				min = std::min(min, std::min(p.x, p.y));
				max = std::max(max, std::max(p.x, p.y));
			}

		// normalize
		for (size_t i = 0; i < lib.cell_array[0]->polygon_array.count; i++)
			for (size_t k = 0; k < lib.cell_array[0]->polygon_array[i]->point_array.count; k++)
			{
				Vec2& p = lib.cell_array[0]->polygon_array[i]->point_array[k];
				p.x = (((std::abs(min) + p.x) / (std::abs(min) + max)) * 2.0 - 1.0) * scale;
				p.y = (((std::abs(min) + p.y) / (std::abs(min) + max)) * 2.0 - 1.0) * scale;
			}

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Normalisation en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;
	}


	void MakeFracture(Library& lib)
	{
		Array<gdstk::Polygon*> fractured_polys = {};

		for (size_t i = 0; i < lib.cell_array.count; i++)
		{
			for (size_t k = 0; k < lib.cell_array[i]->polygon_array.count; k++)
			{
				Array<gdstk::Polygon*> fractured = {};

				gdstk::Polygon* poly = lib.cell_array[i]->polygon_array[k];
				poly->fracture(8190, 1e-3, fractured);

				for (size_t m = 0; m < fractured.count; m++)
					fractured_polys.append(fractured[m]);
			}
		}

		lib.cell_array.clear();
		Cell* cell = new Cell();
		cell->name = copy_string("FIRST", NULL);
		lib.cell_array.append(cell);

		cell->polygon_array = fractured_polys;
	}


	Library MakeUnion(Library& lib)
	{
		assert(lib.cell_array.count == 1);
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		gdstk::Library out_lib = {};
		out_lib.init("library", 1e-6, 1e-9);
		Cell* out_cell = (Cell*)allocate_clear(sizeof(Cell));
		out_cell->name = copy_string("cellule", NULL);

		boolean(
			lib.cell_array[0]->polygon_array,
			out_cell->polygon_array, Operation::Or,
			1, // scaling
			out_cell->polygon_array
		);

		out_lib.cell_array.append(out_cell);

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Union faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
		return out_lib;
	}


	Library MakeDegraissement(Library lib, double dist)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		gdstk::Library out_lib = {};
		out_lib.init("library", 1e-6, 1e-9);

		for (size_t i = 0; i < lib.cell_array.count; i++)
		{
			Cell* out_cell = (Cell*)allocate_clear(sizeof(Cell));
			out_cell->name = copy_string("cellule", NULL);

			// dégraissement
			gdstk::offset(
				lib.cell_array[i]->polygon_array,
				dist,
				OffsetJoin::Miter,
				2,
				1e3,
				false,
				out_cell->polygon_array
			);

			out_lib.cell_array.append(out_cell);
		}

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Degraissement fait en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
		return out_lib;
	}


	Library MakeDifference(Library& lib1, Library& lib2)
	{
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		gdstk::Library out_lib = {};
		out_lib.init("library", 1e-6, 1e-9);

		assert(lib1.cell_array.count == lib2.cell_array.count);

		for (size_t i = 0; i < lib1.cell_array.count; i++)
		{
			Cell* out_cell = (Cell*)allocate_clear(sizeof(Cell));
			out_cell->name = copy_string("cellule", NULL);

			boolean(
				lib1.cell_array[i]->polygon_array,
				lib2.cell_array[i]->polygon_array,
				Operation::Not,
				1e3,
				out_cell->polygon_array
			);

			out_lib.cell_array.append(out_cell);
		}

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		std::cout << "Difference faite en " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << std::endl;
		return out_lib;
	}

	void Scale(Library& lib, double scale)
	{
		for (size_t i = 0; i < lib.cell_array[0]->polygon_array.count; i++)
			for (size_t k = 0; k < lib.cell_array[0]->polygon_array[i]->point_array.count; k++)
				lib.cell_array[0]->polygon_array[i]->point_array[k] *= scale;
	}
}