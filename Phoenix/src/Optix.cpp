#include "Optix.h"
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>
#include <cuda_runtime.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <sstream>
#include <gdstk/vec.hpp>
#include <bitset>
#include <GdstkUtils.h>


Optix::Optix(int width, int height) : width(width), height(height)
{
	gas_handle = 0;
	d_gas_output_buffer = 0;

	x_limit.first = FLT_MAX;
	x_limit.second = FLT_MIN;

	y_limit.first = FLT_MAX;
	y_limit.second = FLT_MIN;
}


Optix::~Optix()
{
	OPTIX_CHECK(optixPipelineDestroy(pipeline));
	OPTIX_CHECK(optixProgramGroupDestroy(raygen_program_group));
	OPTIX_CHECK(optixProgramGroupDestroy(hit_program_group));
	OPTIX_CHECK(optixProgramGroupDestroy(miss_program_group));
	OPTIX_CHECK(optixModuleDestroy(module));
	OPTIX_CHECK(optixDeviceContextDestroy(context));
}


void Optix::loadObj
(
	const std::string& filename,
	float3*& vertices,
	uint3*& triangles,
	int& nb_v,
	int& nb_f
)
{
	std::ifstream file(filename);
	int layer_type = 0;
	std::vector<unsigned char> triangles_type_vector;

	if (!file.is_open()) {
		std::cerr << "Error: Could not open file " << filename << std::endl;
		return;
	}

	std::vector<optix_struct::Vertex> temp_vertices;
	std::vector<Triangle> temp_triangles;

	std::string line;
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::string line_type;
		ss >> line_type;

		if (line_type == "v") {
			// Found a vertex
			optix_struct::Vertex v;
			ss >> v.x >> v.y >> v.z;
			temp_vertices.push_back(v);
			x_limit.first = min(x_limit.first, v.x);
			x_limit.second = max(x_limit.second, v.x);

			y_limit.first = min(y_limit.first, v.y);
			y_limit.second = max(y_limit.second, v.y);
		}

		else if (line_type == "c")
		{
			std::string sub_string;
			ss >> sub_string;
			bool pair = std::stoi(sub_string) % 2 == 0;
			layer_type = pair ? 0 : 255;
		}
		else if (line_type == "f")
		{
			Triangle t;

			std::string sub_string;
			for (int i = 0; i < 3; ++i) {
				ss >> sub_string;
				std::stringstream sub_ss(sub_string);
				std::string v_str;
				std::getline(sub_ss, v_str, '/'); // Lire la partie du sommet (v)
				int v_index = std::stoi(v_str);

				// Soustraire 1 car les indices OBJ commencent à 1
				if (i == 0) t.v1 = v_index - 1;
				else if (i == 1) t.v2 = v_index - 1;
				else if (i == 2) t.v3 = v_index - 1;
			}

			temp_triangles.push_back(t);
			triangles_type_vector.push_back(layer_type);
		}
	}
	file.close();

	nb_v = temp_vertices.size();
	nb_f = temp_triangles.size();

	// conversion en int* avec adresses cuda
	vertices = new float3[temp_vertices.size()];
	triangles = new uint3[temp_triangles.size()];

	for (size_t i = 0; i < temp_vertices.size(); i++)
		vertices[i] = make_float3(temp_vertices[i].x, temp_vertices[i].y, temp_vertices[i].z);

	for (size_t i = 0; i < temp_triangles.size(); i++)
		triangles[i] = make_uint3(temp_triangles[i].v1, temp_triangles[i].v2, temp_triangles[i].v3);

	std::cout << "Loaded " << temp_vertices.size() << " vertices and " << temp_triangles.size() << " triangles." << std::endl;
}


int Optix::init()
{
	CUcontext cudaContext = 0; // Utilise le contexte CUDA par défaut
	cudaFree(0);

	// Initialise les pointeurs de fonctions OptiX
	OPTIX_CHECK(optixInit());

	// Crée le contexte OptiX
	OptixDeviceContextOptions options = {};
	options.logCallbackFunction = [](unsigned int level, const char* tag, const char* message, void* /*cbdata*/) {
		std::cerr << "[OptiX log] (" << level << ") [" << tag << "]: " << message << std::endl;
		};
	options.logCallbackLevel = 0; // Log tous les messages d'erreur et d'avertissement

	OPTIX_CHECK(optixDeviceContextCreate(cudaContext, &options, &context));

	// 2. Création du module et de la chaîne de pipeline
	// Définition du programme de lancer de rayons (raygen)
	OptixProgramGroup raygen_program_group = nullptr;
	OptixProgramGroup miss_program_group = nullptr;
	OptixProgramGroup hit_program_group = nullptr;

	// Configure les options de la pipeline
	pipeline_options.usesMotionBlur = false;
	pipeline_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
	pipeline_options.numPayloadValues = 1;
	pipeline_options.numAttributeValues = 1;
	pipeline_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	pipeline_options.pipelineLaunchParamsVariableName = "params";
	pipeline_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;


	// Charge le code depuis le fichier OPTIX
	std::ifstream ptx_file("Phoenix.optixir", std::ios::binary | std::ios::ate);
	if (!ptx_file.good()) {
		std::cerr << "Erreur : Impossible de trouver le fichier OPTIX." << std::endl;
		return -1;
	}

	size_t file_size = ptx_file.tellg();
	ptx_file.seekg(0);

	std::string ptx_source(file_size, '\0');
	ptx_file.read(&ptx_source[0], file_size);

	// Niveau d'optimisation maximum (Lvl 3)
	module_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_3;

	// Facultatif : Désactiver les symboles de debug pour gagner encore en perf
	module_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;

	// Crée un module 
	OPTIX_CHECK_LOG(optixModuleCreate(
		context,
		&module_options,
		&pipeline_options,
		ptx_source.c_str(),
		ptx_source.size(),
		LOG, &LOG_SIZE,
		&module
	));

	printf("OPTIX: init fait\n");
}

void Optix::loadShaders()
{
	// Crée les groupes de programmes
	// Programme Raygen
	OptixProgramGroupOptions program_group_options = {}; // Initialize to zeros

	OptixProgramGroupDesc raygen_desc = {};
	raygen_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
	raygen_desc.raygen.module = module;
	raygen_desc.raygen.entryFunctionName = "__raygen__rg";
	OPTIX_CHECK_LOG(optixProgramGroupCreate(context, &raygen_desc, 1, &program_group_options, LOG, &LOG_SIZE, &raygen_program_group));

	// Missgroup (programme de non intersection)
	OptixProgramGroupDesc missgroup_desc = {};
	missgroup_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
	missgroup_desc.hitgroup.moduleCH = module;
	missgroup_desc.hitgroup.entryFunctionNameCH = "__miss__ms";
	missgroup_desc.hitgroup.moduleIS = nullptr; // Pas de programme d'intersection personnalisé
	OPTIX_CHECK_LOG(optixProgramGroupCreate(context, &missgroup_desc, 1, &program_group_options, LOG, &LOG_SIZE, &miss_program_group));

	// Hitgroup (programme d'intersection)
	OptixProgramGroupDesc hitgroup_desc = {};
	hitgroup_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
	hitgroup_desc.hitgroup.moduleCH = module;
	hitgroup_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
	hitgroup_desc.hitgroup.moduleIS = nullptr; // Pas de programme d'intersection personnalisé
	OPTIX_CHECK_LOG(optixProgramGroupCreate(context, &hitgroup_desc, 1, &program_group_options, LOG, &LOG_SIZE, &hit_program_group));

	printf("OPTIX: chargement shader fait\n");
}

void Optix::initPipeline(CUdeviceptr d_tris)
{
	// --- 1. Création de la Pipeline ---
	OptixProgramGroup program_groups[] = { raygen_program_group, miss_program_group, hit_program_group };
	OptixPipelineLinkOptions pipeline_link_options = { 1 }; // maxTraceDepth = 1

	OPTIX_CHECK_LOG(optixPipelineCreate(context, &pipeline_options, &pipeline_link_options,
		program_groups, 3, LOG, &LOG_SIZE, &pipeline));

	// --- 2. Configuration simplifiée de la Stack (pour maxTraceDepth = 1) ---
	// Au lieu de calculs complexes, on définit des tailles minimales suffisantes pour un rendu simple
	OPTIX_CHECK(optixPipelineSetStackSize(pipeline, 512, 512, 512, 1));

	// --- 3. Construction de la SBT (Shader Binding Table) ---
	// Utilitaire pour allouer et copier un record SBT en une fois
	auto create_sbt_record = [&](OptixProgramGroup pg, auto& record, size_t size) {
		CUdeviceptr d_ptr;
		OPTIX_CHECK(optixSbtRecordPackHeader(pg, &record));
		cudaMalloc(reinterpret_cast<void**>(&d_ptr), size);
		cudaMemcpy(reinterpret_cast<void*>(d_ptr), &record, size, cudaMemcpyHostToDevice);
		return d_ptr;
		};

	RayGenSbtRecord rg_sbt;
	sbt.raygenRecord = create_sbt_record(raygen_program_group, rg_sbt, sizeof(rg_sbt));

	MissSbtRecord ms_sbt;
	sbt.missRecordBase = create_sbt_record(miss_program_group, ms_sbt, sizeof(ms_sbt));
	sbt.missRecordStrideInBytes = sizeof(MissSbtRecord);
	sbt.missRecordCount = 1;

	HitGroupSbtRecord hg_sbt;
	hg_sbt.data.triangles_type = reinterpret_cast<int*>(d_tris); // pour donner un booléen à chaque triangle
	sbt.hitgroupRecordBase = create_sbt_record(hit_program_group, hg_sbt, sizeof(hg_sbt));
	sbt.hitgroupRecordStrideInBytes = sizeof(HitGroupSbtRecord);
	sbt.hitgroupRecordCount = 1;

	cudaFree(reinterpret_cast<void*>(d_tris));

	printf("OPTIX: init pipeline fait\n");
}

CUdeviceptr Optix::initScene(TrisUtils::Triangulation& t)
{
	// Use default options for simplicity.  In a real use case we would want to
	// enable compaction, etc
	OptixAccelBuildOptions accel_options = {};
	accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
	accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

	float3* d_vertices = nullptr;
	CUdeviceptr d_tris;

	//loadObj("C:/Users/PC/Desktop/poc/test3.obj", vertices, triangles, nb_v, nb_f);
	float3* vertices = new float3[t.nb_vertices];
	float nb_layer = 0;

	for (auto& layer_range : t.layers_range)
	{
		for (size_t i = layer_range.first; i < layer_range.second; i++)
		{
			vertices[t.t[i].x].x = t.v[t.t[i].x].x;
			vertices[t.t[i].x].y = t.v[t.t[i].x].y;
			vertices[t.t[i].x].z = nb_layer;

			vertices[t.t[i].y].x = t.v[t.t[i].y].x;
			vertices[t.t[i].y].y = t.v[t.t[i].y].y;
			vertices[t.t[i].y].z = nb_layer;

			vertices[t.t[i].z].x = t.v[t.t[i].z].x;
			vertices[t.t[i].z].y = t.v[t.t[i].z].y;
			vertices[t.t[i].z].z = nb_layer;
		}

		nb_layer++;
	}

	// allouer de la mémoire cuda
	cudaMalloc((void**)&d_vertices, t.nb_vertices * sizeof(float3));
	cudaMalloc((void**)&d_tris, t.nb_triangles * sizeof(uint3));

	// remplir cette mémoire cuda
	cudaMemcpy(d_vertices, vertices, t.nb_vertices * sizeof(float3), cudaMemcpyHostToDevice);
	cudaMemcpy(reinterpret_cast<void*>(d_tris), t.t, t.nb_triangles * sizeof(uint3), cudaMemcpyHostToDevice);

	OptixBuildInput triangle_input = {};
	triangle_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

	// Liaison des sommets
	triangle_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
	triangle_input.triangleArray.numVertices = static_cast<uint32_t>(t.nb_vertices);
	triangle_input.triangleArray.vertexBuffers = (CUdeviceptr*)&d_vertices; // Adresse du pointeur
	triangle_input.triangleArray.vertexStrideInBytes = sizeof(float3);

	// Liaison des indices
	triangle_input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
	triangle_input.triangleArray.numIndexTriplets = static_cast<uint32_t>(t.nb_triangles);
	triangle_input.triangleArray.indexBuffer = (CUdeviceptr)d_tris;
	triangle_input.triangleArray.indexStrideInBytes = sizeof(uint3);

	// Flags et Matériaux
	const uint32_t triangle_input_flags[1] = { OPTIX_GEOMETRY_FLAG_NONE };
	triangle_input.triangleArray.flags = triangle_input_flags;
	triangle_input.triangleArray.numSbtRecords = 1; // Obligatoire si 1 seul matériau

	OptixAccelBufferSizes gas_buffer_sizes;
	OPTIX_CHECK(optixAccelComputeMemoryUsage(
		context,
		&accel_options,
		&triangle_input,
		1, // Number of build inputs
		&gas_buffer_sizes
	));
	CUdeviceptr d_temp_buffer_gas;
	cudaMalloc(
		reinterpret_cast<void**>(&d_temp_buffer_gas),
		gas_buffer_sizes.tempSizeInBytes
	);
	cudaMalloc(reinterpret_cast<void**>(&d_gas_output_buffer), gas_buffer_sizes.outputSizeInBytes);

	OPTIX_CHECK(optixAccelBuild(
		context,
		0,                  // CUDA stream
		&accel_options,
		&triangle_input,
		1,                  // num build inputs
		d_temp_buffer_gas,
		gas_buffer_sizes.tempSizeInBytes,
		d_gas_output_buffer,
		gas_buffer_sizes.outputSizeInBytes,
		&gas_handle,
		nullptr,            // emitted property list
		0                   // num emitted properties
	));

	CUDA_CHECK(cudaDeviceSynchronize());

	cudaFree(reinterpret_cast<void*>(d_temp_buffer_gas));
	cudaFree(reinterpret_cast<void*>(d_vertices));
	delete[] vertices;

	printf("OPTIX: init scene fait\n");

	return d_tris;
}

void Optix::render(TrisUtils::Triangulation& tris)
{
	CUstream stream;
	cudaStreamCreate(&stream);

	Params params;

	float* output_buffer_d;
	CUDA_CHECK(cudaMalloc((void**)(&output_buffer_d), width * height * sizeof(float)));
	cudaMemset((void**)&output_buffer_d, 100, width * height);

	params.image = output_buffer_d;
	params.dmd_width = width;
	params.dmd_height = height;

	// triangles polarity
	cudaMalloc((void**)&params.polarity, tris.nb_triangles * sizeof(unsigned char));
	cudaMemcpy(params.polarity, tris.p, tris.nb_triangles * sizeof(unsigned char), cudaMemcpyHostToDevice);

	// anti distorsion array
	float2* distorsion = CreateDistorsionArray();

	cudaMalloc((void**)&params.distorsion, width * height * sizeof(float2));
	cudaMemcpy(params.distorsion, distorsion, width * height * sizeof(float2), cudaMemcpyHostToDevice);

	params.handle = gas_handle;

	//creation d'un rendu
	CUdeviceptr d_param;
	CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_param), sizeof(Params)));
	CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_param), &params, sizeof(params), cudaMemcpyHostToDevice));

	int nbIter = 1;
	auto start = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < nbIter; i++)
		optixLaunch(pipeline, stream, d_param, sizeof(Params), &sbt, width, height, /*depth=*/1);

	cudaDeviceSynchronize();

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> duration_ms = (end - start);
	std::cout << 1000.0f / (duration_ms.count() / nbIter) << " fps" << std::endl;

	unsigned char* output_buffer_h = new unsigned char[width * height];
	CUDA_CHECK(cudaMemcpy(output_buffer_h, output_buffer_d, width * height * sizeof(unsigned char), cudaMemcpyDeviceToHost));

	saveToBmp("C:/Users/PC/Desktop/poc/ray_casting.bmp", width, height, output_buffer_h);

	cudaFree(reinterpret_cast<void*>(d_param));
	cudaFree(output_buffer_d);
	cudaFree(params.polarity);
	cudaFree(params.distorsion);
	delete[] output_buffer_h;
	delete[] distorsion;
}

template <size_t nb_samples_x, size_t nb_samples_y>
float2** Optix::CreateDistorsionSamples()
{
	float sample_size_w = (float)width / (nb_samples_x - 1);
	float sample_size_h = (float)height / (nb_samples_y - 1);
	float norme = sqrt(pow(width / 2.0f, 2) + pow(height / 2.0f, 2));

	float2** distorsion = new float2 * [nb_samples_y];

	for (int y = 0; y < nb_samples_y; y++) {

		distorsion[y] = new float2[nb_samples_x];

		for (int x = 0; x < nb_samples_x; x++)
		{
			float tempx = x;
			float tempy = y;

			// pour dupliquer les valeurs sur les bords de l'image (pas de marqueur ici potentiellement)
			/*if (x == 0) tempx = 1;
			if (x == nb_samples_x - 1) tempx = nb_samples_x - 2;

			if (y == 0) tempy = 1;
			if (y == nb_samples_y - 1) tempy = nb_samples_y - 2;*/

			distorsion[y][x] = {
				(tempx * sample_size_w) - (width / 2.0f),
				(tempy * sample_size_h) - (height / 2.0f)
			};

			float l = sqrt(pow(distorsion[y][x].x, 2) + pow(distorsion[y][x].y, 2)) / norme + 1.0f;
			distorsion[y][x].x = (distorsion[y][x].x / norme) * pow(l, 8);
			distorsion[y][x].y = (distorsion[y][x].y / norme) * pow(l, 8);

			printf("%.1f %.1f, ", distorsion[y][x].x, distorsion[y][x].y);
		}
		printf("\n");
	}

	return distorsion;
}

float2* Optix::CreateDistorsionArray()
{
	const int nb_samples_x = 21; // 2 vrais, 2 dupliqués en réalité
	const int nb_samples_y = 11;

	float sample_size_w = (float)width / (nb_samples_x - 1);
	float sample_size_h = (float)height / (nb_samples_y - 1);

	float2** distorsion = CreateDistorsionSamples<nb_samples_x, nb_samples_y>();
	float2* distorsion_i = new float2[width * height]; // for every pixel, interpolated

	// interpolate distorsion for every pixel
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++)
		{
			float2 pixel{ x, y };
			int pixel_index_i = y * width + x;

			// trouver les 4 marqueurs autour du pixel
			uint2 sample_1_index = { std::floor(x / sample_size_w), std::floor(y / sample_size_h) };
			uint2 sample_2_index = { std::floor(x / sample_size_w), std::floor(y / sample_size_h) + 1 };
			uint2 sample_3_index = { std::floor(x / sample_size_w) + 1, std::floor(y / sample_size_h) };
			uint2 sample_4_index = { std::floor(x / sample_size_w) + 1, std::floor(y / sample_size_h) + 1 };

			// interpoler les valeurs en x et y de leur distorsion pour ce pixel
			float t_u = (pixel.x - sample_1_index.x * sample_size_w) / sample_size_w;
			float t_v = (pixel.y - sample_1_index.y * sample_size_h) / sample_size_h;

			distorsion_i[pixel_index_i].x =
				(1.0f - t_v) *
				((1.0f - t_u) * distorsion[sample_1_index.y][sample_1_index.x].x
					+ t_u * distorsion[sample_3_index.y][sample_3_index.x].x)
				+ t_v *
				((1.0f - t_u) * distorsion[sample_2_index.y][sample_2_index.x].x
					+ t_u * distorsion[sample_4_index.y][sample_4_index.x].x);

			distorsion_i[pixel_index_i].y =
				(1.0f - t_v) *
				((1.0f - t_u) * distorsion[sample_1_index.y][sample_1_index.x].y
					+ t_u * distorsion[sample_3_index.y][sample_3_index.x].y)
				+ t_v *
				((1.0f - t_u) * distorsion[sample_2_index.y][sample_2_index.x].y
					+ t_u * distorsion[sample_4_index.y][sample_4_index.x].y);
		}
	}

	delete[] distorsion;
	return distorsion_i;
}

OptixTraversableHandle Optix::GetGasHandle()
{
	return gas_handle;
}

void Optix::saveToBmp(const std::string& filename, int width, int height,
	unsigned char* hostData)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// 2. Prepare BMP Headers and Palette
	BitmapFileHeader fileHeader;
	BitmapInfoHeader infoHeader;

	infoHeader.width = width;
	infoHeader.height = height;
	infoHeader.bit_count = 8;
	infoHeader.size = sizeof(BitmapInfoHeader);

	uint32_t palette_size = 256 * 4; // 256 grayscale entries, 4 bytes each
	fileHeader.offset_data = sizeof(BitmapFileHeader)
		+ sizeof(BitmapInfoHeader) + palette_size;

	fileHeader.file_size = fileHeader.offset_data + (width * height);

	std::vector<char> palette(palette_size);
	for (int i = 0; i < 256; ++i) {
		palette[i * 4 + 0] = i; // Blue
		palette[i * 4 + 1] = i; // Green
		palette[i * 4 + 2] = i; // Red
		palette[i * 4 + 3] = 0;  // Reserved
	}

	// 3. Write data to the file
	std::ofstream file(filename, std::ios::binary);
	if (!file.is_open()) {
		std::cerr << "Error: Could not open file for writing." << std::endl;
		delete[] hostData;
		return;
	}

	file.write(reinterpret_cast<const char*>(&fileHeader), sizeof(fileHeader));
	file.write(reinterpret_cast<const char*>(&infoHeader), sizeof(infoHeader));
	file.write(palette.data(), palette.size());
	file.write(reinterpret_cast<const char*>(hostData), width * height);

	file.close();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Sauvegarde en .bmp en: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
}

void Optix::RayCasting(
	int dmd_width,
	int dmd_height,
	CUstream& stream,
	CUdeviceptr& dparam
)
{
	optixLaunch(pipeline, stream, dparam, sizeof(Params), &sbt, dmd_width, dmd_height, 1);
	cudaDeviceSynchronize();
}