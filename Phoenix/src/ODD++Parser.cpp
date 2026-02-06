#include "ODB++Parser.h"
#include <fstream>
#include <string>
#include <iostream>
#include <sstream> 


namespace ODB
{
	Feature readFeatureFile(const char* file_name)
	{
		std::ifstream file(file_name);

		if (!file)
		{
			std::cout << "erreur d'ouverture" << std::endl;
			assert(0);
		}

		std::string line;
		Feature feature;

		while (getline(file, line))
		{
			if (line[0] == '#')
				continue;

			else if (line.starts_with("ID"))
			{
				int res = sscanf_s(line.c_str(), "ID=%i", &feature.ID);
				assert(res > 0);
			}

			else if (line.compare(0, 5, "UNITS") == 0)
			{
				feature.UNITS = line.substr(6, line.size());
			}
			else if (line[0] == '$')
			{
				int id;

				if (line.find(" rect") != std::string::npos)
				{
					Rectangle* rect = new Rectangle();
					int res = sscanf_s(line.c_str(), "$%d rect%fx%fxr%f", &id, &rect->w, &rect->h, &rect->rad);
					rect->w /= UNIT_CONVERSION;
					rect->h /= UNIT_CONVERSION;
					rect->rad /= UNIT_CONVERSION;
					feature.feature_symbol_names.push_back(rect);
					assert(res > 0);
				}
				else if (sscanf_s(line.c_str(), "$%d r", &id))
				{
					Round* r = new Round();
					int res = sscanf_s(line.c_str(), "$%d r%f", &id, &r->r);
					r->r /= 2.0f * UNIT_CONVERSION;
					feature.feature_symbol_names.push_back(r);
					assert(res > 0);
				}
			}
			else if (line[0] == 'L')
			{
				Line* l = new Line();
				int res = sscanf_s(line.c_str(), "L %f %f %f %f %d %c",
					&l->xs, &l->ys, &l->xe, &l->ye, &l->sym_num, &l->polarity, 1);
				feature.layer_features.push_back(l);
				assert(res > 0);

				Round* r = dynamic_cast<Round*>(feature.feature_symbol_names[l->sym_num]);

				if (r)
				{
					l->is_poly_circle = true;
					l->circle_radius = r->r * 2.0f;
				}
			}

			else if (line[0] == 'A')
			{
				Arc* a = new Arc();
				int res = sscanf_s(line.c_str(), "A %f %f %f %f %f %f %i %c %i %c",
					&a->xs, &a->ys, &a->xe, &a->ye, &a->xc, &a->yc,
					&a->sym_num, &a->polarity, 1, &a->dcode, &a->cw, 1);

				Round* r = dynamic_cast<Round*>(feature.feature_symbol_names[a->sym_num]);

				// si c'est un cercle a tracer alors l'indiquer dans Arc
				if (r)
				{
					a->is_poly_circle = true;
					a->circle_radius = r->r * 2.0f;
				}

				feature.layer_features.push_back(a);
				assert(res > 0);
			}

			else if (line[0] == 'P')
			{
				Pad* p = new Pad();
				int res = sscanf_s(line.c_str(), "P %f %f %d %c %i %i",
					 &p->x, &p->y, &p->apt_def, &p->polarity, 1, &p->dcode, &p->orient_def);

				feature.layer_features.push_back(p);
				assert(res > 0);
			}

			else if (line[0] == 'S')
			{
				Surface* s = new Surface();
				int res = sscanf_s(line.c_str(), "S %c", &s->polarity, 1);
				assert(res > 0);

				while (getline(file, line))
				{
					if (line.starts_with("SE"))
						break;

					else if (line.starts_with("OB"))
					{
						OB* poly = new OB();
						sscanf_s(line.c_str(), "OB %f %f %c", 
							&poly->xbs, &poly->ybs, &poly->poly_type, 1);

						while (getline(file, line))
						{
							if (line.starts_with("OE"))
								break;

							if (line.starts_with("OS"))
							{
								OS* seg = new OS();
								int res = sscanf_s(line.c_str(), "OS %f %f %c",
									&seg->x, &seg->y, &seg->cw, 1);
								
								poly->arcs_segments.push_back(seg);
								assert(res > 0);
							}
							else if (line.starts_with("OC"))
							{
								OC* arc = new OC();
								int res = sscanf_s(line.c_str(), "OC %f %f %f %f %c",
									&arc->xe, &arc->ye, &arc->xc, &arc->yc, &arc->cw, 1);
								poly->arcs_segments.push_back(arc);
								assert(res > 0);
							}
						}

						s->polys.push_back(poly);
					}
				}
				feature.layer_features.push_back(s);
			}
			else if (line[0] == 'F')
			{
				int res = sscanf_s(line.c_str(), "F %i", &feature.F);
				assert(res > 0);
			}
			else if (line[0] != '&'
				&& line[0] != '@')
			{
				std::cout << "Cas non traite:\n" << line << std::endl;
			}
		}

		file.close();
		return feature;
	}


	Cell* convertODBToPolygons(Feature& feature)
	{
		Cell* symbols_polys = new Cell();
		Cell* feature_polys = new Cell();
		feature_polys->name = copy_string("FIRST", NULL);

		// convertir en polygones les symboles définis dans le fichier avant de les utiliser avec line, arc...
		for (Geometry* geo : feature.feature_symbol_names)
		{
			if (Round* r = dynamic_cast<Round*>(geo); r != nullptr)
				symbols_polys->polygon_array.append(roundToPolygon(r));

			if (Rectangle* rect = dynamic_cast<Rectangle*>(geo); rect != nullptr)
				symbols_polys->polygon_array.append(rectangleToPolygon(rect));
		}

		// convertit en polygone line, arc etc... à l'aide des polygones du dessus
		for (Geometry* geo : feature.layer_features)
		{
			/* if (Line* l = dynamic_cast<Line*>(geo); l != nullptr)
			{
				assert(l->sym_num != -1 && l->sym_num < symbols_polys->polygon_array.count);
				Polygon* poly = lineToPolygon(*l);

				if (poly)
					feature_polys->polygon_array.append(poly);
			}

			else if (Arc* a = dynamic_cast<Arc*>(geo); a != nullptr)
			{
				assert(a->sym_num != -1 && a->sym_num < symbols_polys->polygon_array.count);
				Polygon* poly = arcToPolygon(a);
				
				if (poly)
					feature_polys->polygon_array.append(poly);
			}

			else if (Pad* p = dynamic_cast<Pad*>(geo); p != nullptr)
			{
				assert(p->apt_def != -1 && p->apt_def < symbols_polys->polygon_array.count);
				Polygon* poly = padToPolygon(p, *symbols_polys->polygon_array[p->apt_def]);
				feature_polys->polygon_array.append(poly);
			} */

			if (Surface* s = dynamic_cast<Surface*>(geo); s != nullptr)
			{
				std::vector<Polygon*> polys = surfaceToPolygon(s);

				for (Polygon* poly : polys)
					feature_polys->polygon_array.append(poly);
			}
		}

		return feature_polys;
	}


	gdstk::Polygon* roundToPolygon(const Round* r)
	{
		size_t nb_points = 30;
		float step_angle = 2.0f * PI / nb_points;
		gdstk::Polygon* poly = new Polygon();

		for (size_t i = 0; i < nb_points; i++)
		{
			double x = std::cos(i * step_angle) * r->r;
			double y = std::sin(i * step_angle) * r->r;
			poly->point_array.append(Vec2{ x, y });
		}

		return poly;
	}


	gdstk::Polygon* rectangleToPolygon(const Rectangle* rect)
	{
		Polygon* poly = new Polygon();

		Vec2 p0{ 0.0f, 0.0f }; // Top Left
		Vec2 p1{ rect->w, 0.0f };
		Vec2 p2{ 0.0f, rect->h };
		Vec2 p3{ rect->w, rect->h };

		if (rect->rad == 0)
		{
			poly->point_array.append(p0);
			poly->point_array.append(p1);
			poly->point_array.append(p2);
			poly->point_array.append(p3);
		}
		else
		{
			size_t nb_points = 10;
			float step_angle = PI / (2.0 * nb_points);
			float start_angles[4] = { PI / 2.0, PI, 1.5 * PI, 0 };
			Vec2 corners[4] = { p0 + Vec2{rect->rad, -rect->rad},
								p1 + Vec2{rect->rad, rect->rad},
								p2 + Vec2{-rect->rad, rect->rad},
								p3 + Vec2{-rect->rad, -rect->rad} };

			for (size_t k = 0; k < 4; k++)
			{
				for (size_t i = 0; i < nb_points; i++)
				{
					Vec2 temp{ std::cos(start_angles[k] + i * step_angle) * rect->rad,
							   std::sin(start_angles[k] + i * step_angle) * rect->rad };

					poly->point_array.append(corners[k] + temp);
				}
			}
		}

		return poly;
	}


	gdstk::Polygon* arcToPolygon(const Arc* a)
	{
		if (a->is_poly_circle)
		{
			Vec2 start{ a->xs, a->ys };
			Vec2 end{ a->xe, a->ye };
			Vec2 center{ a->xc, a->yc };
			Vec2 sc = start - center;

			FlexPath* fp = new FlexPath();
			fp->init(start, 2, a->circle_radius, 0, 0.00000001, 0);
			fp->elements->end_type = gdstk::EndType::Round;

			float total_angle = std::acos(dotProduct(start - center, end - center));
			
			if (total_angle == 0)
				total_angle = PI * 2.0001;

			size_t nb_points = 200;
			float step_angle = total_angle / nb_points;
			float radius = norme(start - center);
			const double width = a->circle_radius;

			for (size_t i = 1; i <= nb_points; i++)
			{
				float current_angle = step_angle * i;

				if (a->cw == 'Y')
					current_angle *= -1;

				Vec2 point{
					center.x + std::cos(current_angle) * sc.x - std::sin(current_angle) * sc.y,
					center.y + std::sin(current_angle) * sc.x + std::cos(current_angle) * sc.y
				};

				fp->segment(point, &width, NULL, false);
			}

			Array<Polygon*> arc_poly = {};
			fp->to_polygons(false, 0, arc_poly);
			delete fp;

			if (arc_poly.count == 0)
				return nullptr;

			return arc_poly[0];
		}

		return nullptr;
	}


	std::vector<gdstk::Polygon*> surfaceToPolygon(const Surface* s)
	{
		std::vector<Polygon*> polys;

		for (const OB* ob : s->polys)
		{
			Polygon* poly = new Polygon();
			Vec2 previous_point{ ob->xbs, ob->ybs };
			poly->point_array.append( previous_point );

			// construire le polygone à partir des segments et des arcs
			for (const Geometry* geo : ob->arcs_segments)
			{
				if (const OS* l = dynamic_cast<const OS*>(geo); l != nullptr)
				{
					previous_point = Vec2{ l->x, l->y };
					poly->point_array.append( previous_point );
				}

				else if (const OC* a = dynamic_cast<const OC*>(geo); a != nullptr)
				{
					Vec2 arc_end{ a->xe, a->ye };
					Vec2 arc_center{ a->xc, a->yc };

					size_t nb_points = 10;
					Vec2 sc = previous_point - arc_center;
					float dist = norme(arc_end - arc_center);
					float step_angle = std::acos(
						dotProduct(arc_end - arc_center, previous_point - arc_center)) / nb_points;

					for (size_t i = 1; i <= nb_points; i++)
					{
						float angle = i * step_angle;
						if (a->cw == 'Y') angle *= -1;

						Vec2 temp = 
							arc_center 
							+ Vec2{ std::cos(angle) * sc.x - std::sin(angle) * sc.y,
									std::sin(angle)* sc.x + std::cos(angle) * sc.y };

						poly->point_array.append(temp);
					}

					previous_point = arc_end;
				}
			}

			// changer l'orientation selon "poly_type"
			if (ob->poly_type == 'I')
			{
				Array<Vec2> points = {};
				points.copy_from(poly->point_array);
				poly->point_array.clear();

				for (int i = points.count - 1; i >= 0; i--)
					poly->point_array.append(points[i]);
			}

			polys.push_back(poly);
		}

		return polys;
	}


	gdstk::Polygon* padToPolygon(const Pad* p, Polygon poly)
	{
		Polygon* t_poly = new Polygon();
		Vec2 pad_pos{ p->x, p->y };

		// polarité du pad
		if (p->polarity == 'N')
			for (int i = 0; i < poly.point_array.count; i++)
				t_poly->point_array.append(poly.point_array[i] + pad_pos);

		else
			for (int i = poly.point_array.count-1; i >= 0; i--)
				t_poly->point_array.append(poly.point_array[i] + pad_pos);


		// calcul du centre du polygone
		Vec2 center = { 0, 0 };

		for (size_t i = 0; i < t_poly->point_array.count; i++)
			center += t_poly->point_array[i];

		center /= t_poly->point_array.count;

		// rotation, miroir
		int rotations[4] = {0, 90, 180 ,270};
		t_poly->rotate(rotations[p->orient_def], center);

		return t_poly;
	}


gdstk::Polygon* lineToPolygon(const Line& l)
{
	if (l.is_poly_circle)
	{
		size_t nb_points = 200;
		Vec2 start{ l.xs, l.ys };
		Vec2 end{ l.xe, l.ye };
		const double width = l.circle_radius;

		FlexPath* fp = new FlexPath();
		fp->init(start, 2, l.circle_radius, 0, 0.00000001, 0);
		fp->segment(end, &width, NULL, false);
		fp->elements->end_type = gdstk::EndType::Round;

		Array<Polygon*> arc_poly = {};
		fp->to_polygons(false, 0, arc_poly);
		delete fp;

		if (arc_poly.count == 0)
			return nullptr;

		if (l.polarity == 'P')
		{
			Array<Vec2> points;
			points.copy_from(arc_poly[0]->point_array);
			arc_poly[0]->point_array.clear();

			for (int i = points.count - 1; i >= 0; i--)
				arc_poly[0]->point_array.append(points[i]);
		}

		return arc_poly[0];
	}

	return nullptr;
}


	float norme(const Vec2& v)
	{
		return std::sqrt(v.x * v.x + v.y * v.y);
	}


	float sinusOfVectors(const Vec2& v1, const Vec2& v2)
	{
		return (v1.x * v2.y - v1.y * v2.x) / (norme(v1) * norme(v2));
	}


	float dotProduct(const Vec2& v1, const Vec2& v2)
	{
		return (v1.x * v2.x + v1.y * v2.y) / (norme(v1) * norme(v2));
	}


	std::pair<size_t, size_t> findFarthestVertices(const Vec2& segment, const Polygon* poly)
	{
		// find the two farthest points of poly to the line's segment
		std::pair<size_t, size_t> farthest_points = { -1, -1 };
		float max_dists[2] = { 0, 0 };

		for (size_t i = 0; i < poly->point_array.count; i++)
		{
			float dist = sinusOfVectors(
				segment,
				Vec2{
					poly->point_array[i].x,
					poly->point_array[i].y
				}
			);

			// point le plus éloigné à gauche du segment
			if (dist >= max_dists[0])
			{
				max_dists[0] = dist;
				farthest_points.first = i;
			}

			// point le plus éloigné à droite du segment
			else if (dist <= max_dists[1])
			{
				max_dists[1] = dist;
				farthest_points.second = i;
			}
		}

		return farthest_points;
	}


	gdstk::Polygon* lineToPolygon(const Line& l, const gdstk::Polygon* poly)
	{
		Vec2 segment{ l.xe - l.xs, l.ye - l.ys };
		Vec2 line_points[2] = { {l.xs, l.ys}, {l.xe, l.ye} };

		// récupérer les indices des sommets les plus à gauche et à droite de la ligne
		std::pair<size_t, size_t> farthest_points = findFarthestVertices(segment, poly);

		// créer le polygone à partir des points max à gauche et à droite
		gdstk::Polygon* line_poly = new Polygon();
		int nb_vertices = poly->point_array.count;

		size_t index = farthest_points.first;
		bool run = true;

		while (run)
		{
			if (index == farthest_points.second)
				run = false;

			Vec2 vertex = poly->point_array[index] + line_points[0];
			line_poly->point_array.append(vertex);
			index = (index + 1) % nb_vertices;
		}

		index = farthest_points.second;
		run = true;

		while (run)
		{
			if (index == farthest_points.first)
				run = false;

			Vec2 vertex = poly->point_array[index] + line_points[1];
			line_poly->point_array.append(vertex);
			index = (index + 1) % nb_vertices;
		}

		// inverser le polygone selon la polarité
		Array<Vec2> points = line_poly->point_array;
		line_poly->point_array.clear();

		return line_poly;
	}
}