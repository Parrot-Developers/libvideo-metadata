cmake_minimum_required(VERSION 3.15)
project(libvideo-metadata-proto VERSION 1.0)



add_library(video-metadata-proto-generated INTERFACE)
find_package(Protobuf CONFIG REQUIRED)
list(APPEND INCLUDE_PATH ${CMAKE_BINARY_DIR} ${Protobuf_INCLUDE_DIRS})

target_compile_definitions(video-metadata-proto-generated INTERFACE "PROTOBUF_C_USE_SHARED_LIB")

protobuf_generate(
                    TARGET video-metadata-proto-generated
                    PROTOS vmeta.proto
                    LANGUAGE c
                    GENERATE_EXTENSIONS .pb-c.c .pb-c.h
                    IMPORT_DIRS ${PROTO_IMPORT_DIRS}
                    PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}
)

add_library(video-metadata-proto ${CMAKE_BINARY_DIR}/vmeta.pb-c.c)
target_include_directories(video-metadata-proto PUBLIC 
                                "$<BUILD_INTERFACE:${INCLUDE_PATH}>"
                                $<INSTALL_INTERFACE:include>)
#target_link_directories(video-metadata-proto PUBLIC ${Protobuf_INCLUDE_DIRS}/../debug/lib)
target_link_libraries(video-metadata-proto PRIVATE video-metadata-proto-generated protobuf-c)
target_link_libraries(video-metadata-proto PRIVATE protobuf::libprotoc protobuf::libprotobuf protobuf::libprotobuf-lite)


# # Proto files
# set(vmeta_protos
#     vmeta.proto
# )

# # Generated sources
# set(vmeta_protos_srcs
#     ${CMAKE_BINARY_DIR}/proto/test.pb.cc
# )
# set(vmeta_protos_hdrs
#     ${CMAKE_BINARY_DIR}/proto/test.pb.h
# )

# add_custom_command(
#     OUTPUT ${my_protos_hdrs}
#     COMMAND ${_PROTOBUF_PROTOC}
#     ARGS --cpp_out ${CMAKE_BINARY_DIR}/proto
#         -I ${CMAKE_SOURCE_DIR}/proto
#         ${my_protos} 
#     DEPENDS ${my_protos}
# )

# add_library(protos
#     ${my_protos_srcs}
#     ${my_protos_hdrs}
# )
# target_link_libraries(protos
#     ${_PROTOBUF_LIBPROTOBUF}
#     ${_GRPC_GRPCPP}
# )