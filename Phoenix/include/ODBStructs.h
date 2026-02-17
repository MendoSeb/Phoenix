#pragma once


struct Geometry { virtual ~Geometry() {} };
struct Round : Geometry { size_t id = 0;  float r = 0; };
struct Square : Geometry { float s = 0; };

struct Rectangle : Geometry
{
	float w = 0;
	float h = 0;
	float rad = 0;
	int corners = -1;
};

struct Donut : Geometry
{
	int id = -1;
	float outerd = 0;
	float innerd = 0;
};

struct SymbolString : Geometry
{
	int id = -1;
	char s[100] = "";
};

struct Line : Geometry
{
	float xs = 0, ys = 0;
	float xe = 0, ye = 0;
	int sym_num = -1; // symbole used with the line
	char polarity;
	bool is_poly_circle = false;
	float circle_radius;
};

struct Arc : Geometry {
	float xs = 0, ys = 0;
	float xe = 0, ye = 0;
	float xc = 0, yc = 0;
	int sym_num = -1; // symbole used with the line
	char polarity = '_';
	int dcode = -1;
	char cw = '_';
	bool is_poly_circle = false;
	float circle_radius = 0.0f;
};

struct Pad : Geometry {
	float x = 0;
	float y = 0;
	int apt_def = -1; // symbol used
	char polarity = '_';
	unsigned int dcode = 0;
	unsigned int orient_def = 0;
};

struct OS : Geometry {
	float x = 0;
	float y = 0;
	char cw = '_';
};

struct OC : Geometry {
	float xe = 0, ye = 0;
	float xc = 0, yc = 0;
	char cw = '_';
};

struct OB : Geometry {
	char poly_type = '_'; // I for island, H for hole
	float xbs = 0, ybs = 0;
	std::vector<Geometry*> arcs_segments;
};

struct Surface : Geometry {
	char polarity = '_';
	std::vector<OB*> polys;
};

struct Feature
{
	std::string UNITS;
	int ID = -1;
	int F = -1; // nombre de features
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