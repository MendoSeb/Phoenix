#include "Optix.h"
#include <optix.h>
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
    std::vector<int> triangles_type_vector;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    std::vector<Vertex> temp_vertices;
    std::vector<Triangle> temp_triangles;

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string line_type;
        ss >> line_type;

        if (line_type == "v") {
            // Found a vertex
            Vertex v;
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
}


CUdeviceptr Optix::initScene()
{
    // Use default options for simplicity.  In a real use case we would want to
    // enable compaction, etc
    OptixAccelBuildOptions accel_options = {};
    accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
    accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

    float3* vertices = nullptr;
    uint3* triangles = nullptr;
    int nb_v = 0;
    int nb_f = 0;

    void* d_vertices = nullptr;
    CUdeviceptr d_tris;

    loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_mono_couche_earcut.obj", vertices, triangles, nb_v, nb_f);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/triangulation_mono_couche_earcut.obj", vertices, triangles, nb_v, nb_f);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_multi_couches_earcut.obj", objVertices, objTriangles);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_clipper2_multi_couche_v2.obj", objVertices, objTriangles);

    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/triangulation_full_clipper2.obj", objVertices, objTriangles);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/solder/triangulation_full_clipper2_inverse.obj", vertices, triangles, nb_v, nb_f);

    int v_size = nb_v * sizeof(float3);
    int f_size = nb_f * sizeof(uint3);

    // allouer de la mémoire cuda
    cudaMalloc(&d_vertices, v_size);
    cudaMalloc(reinterpret_cast<void**>(&d_tris), f_size);

    // remplir cette mémoire cuda
    cudaMemcpy(d_vertices, vertices, v_size, cudaMemcpyHostToDevice);
    cudaMemcpy(reinterpret_cast<void*>(d_tris), triangles, f_size, cudaMemcpyHostToDevice);

    free(vertices);
    free(triangles);
    
    OptixBuildInput triangle_input = {};
    triangle_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

    // Liaison des sommets
    triangle_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    triangle_input.triangleArray.numVertices = static_cast<uint32_t>(nb_v);
    triangle_input.triangleArray.vertexBuffers = (CUdeviceptr*)&d_vertices; // Adresse du pointeur
    triangle_input.triangleArray.vertexStrideInBytes = sizeof(float3);

    // Liaison des indices
    triangle_input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    triangle_input.triangleArray.numIndexTriplets = static_cast<uint32_t>(nb_f);
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

    cudaFree(reinterpret_cast<void*>(d_temp_buffer_gas));
    cudaFree(reinterpret_cast<void*>(d_vertices));

    return d_tris;
}


void Optix::render()
{
    CUstream stream, streamcpy;
    cudaStreamCreate(&stream);
    cudaStreamCreate(&streamcpy);

    Params params;

    unsigned char* output_buffer_d;
    cudaMalloc((void**)(&output_buffer_d), width * height);

    params.image = output_buffer_d;
    params.image_width = width;
    params.image_height = height;
    params.handle = gas_handle;
    params.cam_u = make_float3(0.828427136f, 0.0f, 0.0f);
    params.cam_v = make_float3(0.0f, 0.828427136, 0.0f);
    params.cam_w = make_float3(0.0f, 0.0f, 1.0f);
    params.cam_eye = make_float3(0.0f, 0.0f, 1.0f); // reculer assez car en multi couche c'est épais
    //params.cam_eye = make_float3(0.3f, 0.24f, 5.0f);

    //creation d'un rendu
    CUdeviceptr d_param;
    cudaMalloc(reinterpret_cast<void**>(&d_param), sizeof(Params));
    cudaMemcpy(reinterpret_cast<void*>(d_param), &params, sizeof(params), cudaMemcpyHostToDevice);

    int nbIter = 1;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < nbIter; i++)
        optixLaunch(pipeline, stream, d_param, sizeof(Params), &sbt, width, height, /*depth=*/1);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration_ms = (end - start);
    std::cout << 1000.0f / (duration_ms.count() / nbIter) << " fps" << std::endl;

    cudaDeviceSynchronize();
    cudaFree(reinterpret_cast<void*>(d_param));

    saveGrayscaleBitmapCuda("output.bmp", width, height, output_buffer_d);
}


void Optix::DMDSimulation()
{
    CUstream stream, streamcpy;
    cudaStreamCreate(&stream);
    cudaStreamCreate(&streamcpy);

    // real sizes in cm
    // taille réelle de la partie miroir du dmd
    float dmd_real_width = 2.21f;
    float dmd_real_height = 1.74f;

    // taille réelle du circuit à imprimé
    float circuit_real_width = 2.0f;
    float circuit_real_height = 2.0f;

    // nombre de pixels dans le dmd
    int dmd_pixel_width = 4096;
    int dmd_pixel_height = 2176;

    int sp = 8; // sous pixélisation

    // image size
    int img_width = std::ceil(circuit_real_width / dmd_real_width) * dmd_pixel_width * sp;
    int img_height = std::ceil(circuit_real_height / dmd_real_height) * dmd_pixel_height * sp;

    std::cout << img_width << " x " << img_height << std::endl;
    std::cout << "Image size: " << ((float)img_width * (float)img_height) / 1000000000.0f << " Go" << std::endl;

    unsigned char* output_buffer_d;
    cudaMalloc((void**)(&output_buffer_d), img_width * img_height);

    Params params;
    params.image = output_buffer_d;
    params.image_width = img_width;
    params.image_height = img_height;
    params.total_pixels = img_width * img_height;
    params.sp = sp;
    params.min_x = abs(x_limit.first);
    params.min_y = abs(y_limit.first);
	params.handle = gas_handle;
	params.cam_u = make_float3(0.828427136f, 0.0f, 0.0f);
	params.cam_v = make_float3(0.0f, 0.828427136, 0.0f);
	params.cam_w = make_float3(0.0f, 0.0f, 1.0f);

	// creation d'un rendu
	CUdeviceptr d_param;
    cudaMalloc(reinterpret_cast<void**>(&d_param), sizeof(Params));
    cudaMemcpy(reinterpret_cast<void*>(d_param), &params, sizeof(params), cudaMemcpyHostToDevice);

    float init_pos_x = x_limit.first;
    float init_pos_y = y_limit.first;

    int nb_step_x = std::ceil(circuit_real_width / dmd_real_width);
    int nb_step_y = std::ceil(circuit_real_height / dmd_real_height);

    int nb_images_to_calculate = nb_step_x * nb_step_y * sp^2;
    std::cout << "Nb images to calculate: " << nb_images_to_calculate << std::endl;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    gdstk::Vec2 dmd_direction{1.0f / (sp^2), 1.0f / sp + (dmd_pixel_height / (sp^2))};

    for (int k = 0; k < nb_step_x; k++) // nombre de dmd en x
    {
        params.cam_eye = make_float3(init_pos_x + k * dmd_pixel_width, init_pos_y, 1.0f);

        for (float i = 0; i < nb_step_y * (sp ^ 2); i++) // nb_step_y (nb dmd) * sp^2 déplacements pour faire de la sous pixélisation sp X sp
        {
            if (i > 0)
            {
                params.cam_eye.x += dmd_direction.x;
                params.cam_eye.y += dmd_direction.y;
            }

            cudaMemcpyAsync(reinterpret_cast<void*>(d_param), &params, sizeof(params), cudaMemcpyHostToDevice);
            optixLaunch(pipeline, stream, d_param, sizeof(Params), &sbt, width, height, 1);
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    float time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    std::cout << "Simulation DMD en " << time_elapsed << " ms" << std::endl;
    std::cout << "Fps: " << 1000.0f / (time_elapsed / nb_images_to_calculate) << std::endl;

    saveGrayscaleBitmapCuda("output.bmp", img_width, img_height, output_buffer_d);
}


void Optix::saveGrayscaleBitmapCuda(const std::string& filename, int width, int height, unsigned char* img) {
    
    // 1. Copy GPU data to a CPU buffer (Host)
    unsigned char* hostData = new unsigned char[width * height];
    cudaMemcpyAsync(hostData, img, width * height, cudaMemcpyDeviceToHost);

    for (int i = 0; i < width * height; i++)
    {
        if (hostData[i] > 0)
            hostData[i] = 80 + rand() % 176;
    }

    // Check for CUDA errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error: " << cudaGetErrorString(err) << std::endl;
        delete[] hostData;
        return;
    }

    // 2. Prepare BMP Headers and Palette
    BitmapFileHeader fileHeader;
    BitmapInfoHeader infoHeader;

    infoHeader.width = width;
    infoHeader.height = height;
    infoHeader.bit_count = 8;
    infoHeader.size = sizeof(BitmapInfoHeader);

    uint32_t palette_size = 256 * 4; // 256 grayscale entries, 4 bytes each
    fileHeader.offset_data = sizeof(BitmapFileHeader) + sizeof(BitmapInfoHeader) + palette_size;
    fileHeader.file_size = fileHeader.offset_data + (width * height);

    std::vector<char> palette(palette_size);
    for (int i = 0; i < 256; ++i) {
        palette[i * 4 + 0] = 255 - i; // Blue
        palette[i * 4 + 1] = 255 - i; // Green
        palette[i * 4 + 2] = 255 - i; // Red
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
    delete[] hostData;
}