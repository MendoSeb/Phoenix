#pragma once
#include <gdstk/vec.hpp>
#include <vector>
#include <string>
#include <Clipper2Utils.h>


struct Geometry { virtual ~Geometry() {} };
struct Round : Geometry { size_t id = 0;  float r = 0; };
struct Square : Geometry { float s = 0; };

struct Rectangle : Geometry 
{ 
	float w = 0, h = 0;
	float rad = 0;
	int corners = 0;
};

struct Line : Geometry 
{
	float xs, ys;
	float xe, ye;
	int sym_num = -1; // symbole used with the line
	char polarity;
};

struct Arc : Geometry {
	float xs, ys;
	float xe, ye;
	float xc, yc;
	int sym_num = -1; // symbole used with the line
	char polarity;
	int dcode = -1;
	char cw;
	bool is_poly_circle = false;
	float circle_radius = 0.0f;
};

struct Pad : Geometry {
	float x, y;
	int apt_def = -1; // symbol used
	char polarity;
	int orient_def;
};

struct OS : Geometry{
	float x, y;
	char cw;
};

struct OC : Geometry {
	float xe, ye;
	float xc, yc;
	char cw;
};

struct OB : Geometry {
	char poly_type; // I for island, H for hole
	float xbs, ybs;
	std::vector<Geometry> arcs_segments;
};

struct Surface : Geometry {
	char polarity;
	std::vector<OB> polys;
};


struct Feature
{
	std::string UNITS;
	unsigned int ID;
	unsigned int F; // nombre de features
	std::vector<Geometry*> feature_symbol_names;
	std::vector<Geometry*> layer_features;

	~Feature()
	{
		for (Geometry* geo : feature_symbol_names)
			delete geo;

		for (Geometry* geo : layer_features)
			delete geo;
	}
};


struct OdbHierarchy
{
	std::string UNITS;
	std::vector<Feature> layers;
};


namespace ODB
{
	const float UNIT_CONVERSION = 1000.0;

	Feature ReadFeatureFile(const char* file_name);

	/// focntions pour convertir les objets en polygones
	Cell* ConvertODBToPolygons(Feature& feature);

	gdstk::Polygon* RoundToPolygon(const Round* r);
	gdstk::Polygon* RectangleToPolygon(const Rectangle* rect);
	gdstk::Polygon* ArcToPolygon(const Arc* a);
	gdstk::Polygon* SurfaceToPolygon(const Surface* s);
	gdstk::Polygon* PadToPolygon(const Pad* p, Polygon poly);

	/// fonction pour convertir en polygone un symbole tracé le long d'un segment ou d'un arc
	// calcule la norme d'un vecteur
	float Norme(const Vec2& v);
	float SinusOfVectors(const Vec2& v1, const Vec2& v2);
	float DotProduct(const Vec2& v1, const Vec2& v2);

	// trouve les indices des sommets les plus éloignés à gauche et à droite de segment
	std::pair<size_t, size_t> FindFarthestVertices(const Vec2& segment, const Polygon* poly);

	// trace un symbole le long d'une ligne
	gdstk::Polygon* LineToPolygon(const Line& l, const gdstk::Polygon* poly);
}