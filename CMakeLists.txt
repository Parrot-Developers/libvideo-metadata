cmake_minimum_required(VERSION 3.15)
project(libvideo-metadata VERSION 1.0)

add_subdirectory(proto)

set(LIB_SOURCES
        src/vmeta_session.c
        src/vmeta_frame.c
        src/vmeta_json.c
        src/vmeta_json_proto.c
        src/vmeta_csv.c
        src/vmeta_frame_proto.c
        src/vmeta_frame_v1.c
        src/vmeta_frame_v2.c
        src/vmeta_frame_v3.c
        src/vmeta_utils.c
)

add_library(video-metadata ${LIB_SOURCES})
add_library(video-metadata_compiler_flags INTERFACE)
target_compile_features(video-metadata_compiler_flags INTERFACE cxx_std_11)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(video-metadata PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(video-metadata PRIVATE "VMETA_API_EXPORTS")
target_compile_options(video-metadata PRIVATE "-fvisibility=hidden")

find_package(json-c CONFIG REQUIRED)
target_link_libraries(video-metadata json-c::json-c json-c::json-c-static)
target_link_libraries(video-metadata video-metadata-proto ulog futils video-metadata_compiler_flags)