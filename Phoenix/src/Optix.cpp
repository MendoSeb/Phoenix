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


Optix::Optix(int width, int height) : width(width), height(height) {}


Optix::~Optix()
{
    OPTIX_CHECK(optixPipelineDestroy(pipeline));
    OPTIX_CHECK(optixProgramGroupDestroy(raygen_program_group));
    OPTIX_CHECK(optixProgramGroupDestroy(hit_program_group));
    OPTIX_CHECK(optixProgramGroupDestroy(miss_program_group));
    OPTIX_CHECK(optixModuleDestroy(module));
    OPTIX_CHECK(optixDeviceContextDestroy(context));
}


void Optix::loadObj(
    const std::string& filename,
    std::vector<Vertex>& out_vertices,
    std::vector<Triangle>& out_triangles,
    CUdeviceptr& d_list,
    float& min, 
    float& max) 
{
    std::ifstream file(filename);
    int layer_type = 0;
    std::vector<int> triangles_type_vector;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string line_type;
        ss >> line_type;

        if (line_type == "v") {
            // Found a vertex
            Vertex v;
            ss >> v.x >> v.y >> v.z;
            out_vertices.push_back(v);
            float min_ = min(v.x, v.y);
            float max_ = max(v.x, v.y);

            if (min_ < min) min = min_;
            if (max_ > max) max = max_;
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

            out_triangles.push_back(t);
            triangles_type_vector.push_back(layer_type);
        }
    }
    file.close();

    // conversion en int* avec adress cuda
    int* triangles_types = new int[triangles_type_vector.size()];

    for (int i = 0; i < triangles_type_vector.size(); i++)
        triangles_types[i] = triangles_type_vector[i];

    cudaMalloc(reinterpret_cast<void**>(&d_list), triangles_type_vector.size() * sizeof(int));
    cudaMemcpy(reinterpret_cast<void*>(d_list), triangles_types, triangles_type_vector.size() * sizeof(int), cudaMemcpyHostToDevice);

    std::cout << "Loaded " << out_vertices.size() << " vertices and " << out_triangles.size() << " triangles." << std::endl;
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
    pipeline_options.numPayloadValues = 3;
    pipeline_options.numAttributeValues = 3;
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



void Optix::initPipeline(CUdeviceptr d_list)
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
    hg_sbt.data.triangles_type = reinterpret_cast<int*>(d_list); // pour donner un booléen à chaque triangle
    sbt.hitgroupRecordBase = create_sbt_record(hit_program_group, hg_sbt, sizeof(hg_sbt));
    sbt.hitgroupRecordStrideInBytes = sizeof(HitGroupSbtRecord);
    sbt.hitgroupRecordCount = 1;
}


CUdeviceptr Optix::initScene()
{
    // Use default options for simplicity.  In a real use case we would want to
    // enable compaction, etc
    OptixAccelBuildOptions accel_options = {};
    accel_options.buildFlags = OPTIX_BUILD_FLAG_NONE;
    accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

    std::vector<Vertex> objVertices;
    std::vector<Triangle> objTriangles;
    CUdeviceptr d_list;
    float min = INT_MAX;
    float max = INT_MIN;

    loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_mono_couche_clipper2.obj", objVertices, objTriangles, d_list, min, max);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_mono_couche_earcut.obj", objVertices, objTriangles, d_list, min, max);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_multi_couches_earcut.obj", objVertices, objTriangles, d_list, min, max);
    //loadObj("C:/Users/PC/Desktop/poc/fichiers_gdsii/clipper2/primaire/triangulation_clipper2_multi_couche_v2.obj", objVertices, objTriangles, d_list, min, max);

    int numTriangle = objTriangles.size();
    int totalNumTriangle = numTriangle;
    float3* vertices;
    vertices = (float3*)malloc(totalNumTriangle * 3 * sizeof(float3));

    for (int i = 0; i < numTriangle; i++)
    {
        vertices[i * 3] = make_float3(objVertices[objTriangles[i].v1].x, objVertices[objTriangles[i].v1].y, objVertices[objTriangles[i].v1].z);
        vertices[i * 3 + 1] = make_float3(objVertices[objTriangles[i].v2].x, objVertices[objTriangles[i].v2].y, objVertices[objTriangles[i].v2].z);
        vertices[i * 3 + 2] = make_float3(objVertices[objTriangles[i].v3].x, objVertices[objTriangles[i].v3].y, objVertices[objTriangles[i].v3].z);
    }

    const size_t vertices_size = sizeof(float3) * totalNumTriangle * 3;
    std::cout << "MemSize = " << (vertices_size / 1024 / 1024) << " Mo" << std::endl;
    CUdeviceptr d_vertices = 0;
    cudaMalloc(reinterpret_cast<void**>(&d_vertices), vertices_size);
    cudaMemcpy(
        reinterpret_cast<void*>(d_vertices),
        vertices,
        vertices_size,
        cudaMemcpyHostToDevice
    );

    free(vertices);

    // Our build input is a simple list of non-indexed triangle vertices
    const uint32_t triangle_input_flags[1] = { OPTIX_GEOMETRY_FLAG_NONE };
    OptixBuildInput triangle_input = {};
    triangle_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    triangle_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    triangle_input.triangleArray.numVertices = static_cast<uint32_t>(totalNumTriangle * 3);
    triangle_input.triangleArray.vertexBuffers = &d_vertices;
    triangle_input.triangleArray.flags = triangle_input_flags;
    triangle_input.triangleArray.numSbtRecords = 1;

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
    return d_list;
}


void Optix::render()
{
    CUstream stream, streamcpy;
    cudaStreamCreate(&stream);
    cudaStreamCreate(&streamcpy);

    Params params;

    unsigned char* output_buffer;
    output_buffer = (unsigned char*)malloc(width * height);
    unsigned char* output_buffer_d;
    cudaMalloc((void**)(&output_buffer_d), width * height);
    params.image = output_buffer_d;
    params.image_width = width;
    params.image_height = height;
    params.handle = gas_handle;
    params.cam_u = make_float3(0.828427136f, 0.0f, 0.0f);
    params.cam_v = make_float3(0.0f, 0.828427136, 0.0f);
    params.cam_w = make_float3(0.0f, 0.0f, 1.0f);
    params.cam_eye = make_float3(0.0f, 0.0f, 5.0f); // reculer assez car en multi couche c'est épais
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

    saveDeviceArrayToFile(output_buffer_d, width, height, "output.txt");
    saveGrayscaleBitmapCuda("output.bmp", width, height, output_buffer_d);
}


template<typename T>
void Optix::saveDeviceArrayToFile(const T* device_array, size_t width, size_t height, const std::string& filename) {
    // 1. Allouer de la mémoire sur le CPU pour la copie des données
    size_t array_size_bytes = width * height * sizeof(T);
    T* host_array = new T[width * height];
    if (host_array == nullptr) {
        std::cerr << "Erreur: échec de l'allocation de la mémoire hôte." << std::endl;
        return;
    }

    // 2. Copier les données de la mémoire du GPU vers le CPU
    cudaMemcpy(host_array, device_array, array_size_bytes, cudaMemcpyDeviceToHost);

    // 3. Ouvrir un fichier pour l'écriture
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Erreur: impossible d'ouvrir le fichier " << filename << std::endl;
        delete[] host_array;
        return;
    }

    // 4. Écrire chaque élément dans le fichier
    for (size_t j = 0; j < height; ++j) {
        for (size_t i = 0; i < width; ++i) {
            if (host_array[i + j * width] == 0)
                outfile << "0";
            else
                outfile << "X";
        }
        outfile << "\n";
    }

    //std::cout << "Données enregistrées dans " << filename << std::endl;

    // 5. Fermer le fichier et libérer la mémoire de l'hôte
    outfile.close();
    delete[] host_array;
}


void Optix::saveGrayscaleBitmapCuda(const std::string& filename, int width, int height, unsigned char* dev_data) {
    // 1. Copy GPU data to a CPU buffer (Host)
    unsigned char* hostData = new unsigned char[width * height];
    cudaMemcpy(hostData, dev_data, width * height, cudaMemcpyDeviceToHost);

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