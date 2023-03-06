cmake_minimum_required(VERSION 3.15)
project(video-metadata VERSION 1.0)


add_subdirectory(proto) # video-metadata-proto

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

add_library(${PROJECT_NAME} ${LIB_SOURCES})

# checks if set up rpath exists for install
if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(${PROJECT_NAME} PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "VMETA_API_EXPORTS")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")

find_package(json-c CONFIG REQUIRED)

target_link_libraries(${PROJECT_NAME} 
                      PUBLIC
                      video-metadata-proto 
                      PRIVATE
                      json-c::json-c 
                      ulog 
                      futils 
                      mp4
                      protobuf-c
                     )

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    PUBLIC_HEADER DESTINATION include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
