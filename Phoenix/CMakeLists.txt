cmake_minimum_required(VERSION 3.18)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(PhoenixApp LANGUAGES CXX CUDA)

find_package(Clipper2 REQUIRED)
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Qhull REQUIRED)
find_package(CUDAToolkit 12.4 REQUIRED)
find_package(OpenCV REQUIRED)

# --- Configuration des chemins OptiX ---
set(OPTIX_INSTALL_DIR "C:/ProgramData/NVIDIA Corporation/OptiX SDK 9.1.0")
set(OPTIX_INCLUDE_DIR "${OPTIX_INSTALL_DIR}/include")
set(OPTIX_SAMPLES_SDK_DIR "${OPTIX_INSTALL_DIR}/SDK") # Pour vec_math.h

file(GLOB_RECURSE MY_SOURCES "src/*.cpp")
file(GLOB_RECURSE GDSTK_SOURCES "gdstk/src/*.cpp")
file(GLOB_RECURSE CLIPPER1_SOURCES "clipper1/cpp/*.cpp")

# Fichier source CUDA/OptiX
set(OPTIX_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Phoenix.cu")
set(OPTIX_OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/Phoenix.optixir")

# --- Compilation du fichier .cu en .optixir avec NVCC ---
add_custom_command(
    OUTPUT "${OPTIX_OUTPUT_FILE}"
    COMMAND ${CMAKE_CUDA_COMPILER}
    -optix-ir
    -I"${CMAKE_CURRENT_SOURCE_DIR}/include"
    -I"${OPTIX_INCLUDE_DIR}"
    -I"${OPTIX_SAMPLES_SDK_DIR}/cuda"
    -I"${CMAKE_CURRENT_SOURCE_DIR}/gdstk/include"
    --use_fast_math
    -o "${OPTIX_OUTPUT_FILE}"
    "${OPTIX_SOURCE_FILE}"
    MAIN_DEPENDENCY "${OPTIX_SOURCE_FILE}"
    COMMENT "Compiling OptiX IR: ${OPTIX_SOURCE_FILE}"
)

# Cible pour forcer la génération
add_custom_target(GenerateOptiXIR ALL DEPENDS "${OPTIX_OUTPUT_FILE}")

# --- Création de l'exécutable ---

# IMPORTANT : On ajoute Phoenix.cu ici pour que Visual Studio active l'IntelliSense
# On le marque HEADER_FILE_ONLY pour qu'il ne soit pas compilé avec le reste du C++
set_source_files_properties("${OPTIX_SOURCE_FILE}" PROPERTIES HEADER_FILE_ONLY ON)

add_executable(executable 
    ${MY_SOURCES} 
    ${GDSTK_SOURCES} 
    ${CLIPPER1_SOURCES}
    "${OPTIX_SOURCE_FILE}"
)

add_dependencies(executable GenerateOptiXIR)

# Inclusion des dossiers pour l'exécutable (et donc pour l'IntelliSense de Phoenix.cu)
target_include_directories(executable PRIVATE 
    "include"
    "gdstk/include"
    "clipper1/cpp"
    ${CUDAToolkit_INCLUDE_DIRS}
    "${OPTIX_INCLUDE_DIR}"
    "${OPTIX_SAMPLES_SDK_DIR}/cuda"
)

target_link_libraries(executable PRIVATE 
    Clipper2::Clipper2
    Boost::boost
    ZLIB::ZLIB
    Qhull::qhull_r
    CUDA::cudart
    CUDA::cuda_driver
    ${OpenCV_LIBS}
)

# Copie du fichier .optixir à côté de l'exécutable final
add_custom_command(TARGET executable POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${OPTIX_OUTPUT_FILE}"
    "$<TARGET_FILE_DIR:executable>/Phoenix.optixir"
)