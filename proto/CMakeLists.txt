cmake_minimum_required(VERSION 3.20)
project(video-metadata-proto VERSION 1.0)



add_library(video-metadata-proto-generated INTERFACE)
find_package(Protobuf REQUIRED)

list(APPEND INCLUDE_PATH ${CMAKE_BINARY_DIR} ${Protobufc_INCLUDE_DIR})





set(_PROTOBUF_PROTOC "/se/work/oscsj/sw/protobuf-c/bin/protoc-c")


# Proto files
set(vmeta_protos
    ${CMAKE_CURRENT_SOURCE_DIR}/vmeta.proto
)

# Generated sources
set(vmeta_protos_srcs
    ${CMAKE_BINARY_DIR}/vmeta.pb-c.c
)
set(vmeta_protos_hdrs
    ${CMAKE_BINARY_DIR}/vmeta.pb-c.h
)


add_custom_command(
    OUTPUT ${vmeta_protos_srcs}
    COMMAND ${_PROTOBUF_PROTOC}
    ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} 
    ${vmeta_protos}
)


add_custom_command(
    OUTPUT ${vmeta_protos_hdrs}
    COMMAND ${_PROTOBUF_PROTOC}
    ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} 
    ${vmeta_protos}
)



add_library(${PROJECT_NAME} ${vmeta_protos_srcs})
target_include_directories(${PROJECT_NAME} PUBLIC 
                           "$<BUILD_INTERFACE:${INCLUDE_PATH}>"
                        )

target_link_directories(${PROJECT_NAME} PUBLIC ${Protobufc_INCLUDE_DIR}/../lib)
target_link_libraries(${PROJECT_NAME} PUBLIC video-metadata-proto-generated protobuf-c)


install(TARGETS ${PROJECT_NAME}
        # PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)



# add_library(protos
#     ${my_protos_srcs}
#     ${my_protos_hdrs}
# )
