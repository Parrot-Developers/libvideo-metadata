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

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(video-metadata PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(video-metadata PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(video-metadata PRIVATE "VMETA_API_EXPORTS")
target_compile_options(video-metadata PRIVATE "-fvisibility=hidden")
target_compile_options(video-metadata PRIVATE "-std=gnu99")

find_package(json-c CONFIG REQUIRED)
target_link_libraries(video-metadata PUBLIC json-c::json-c)
target_link_libraries(video-metadata PUBLIC 
                        video-metadata-proto 
                        video-metadata-proto-generated 
                        ulog 
                        futils
)