cmake_minimum_required(VERSION 3.20)
project(video-metadata-proto VERSION 1.0)

add_library(video-metadata-proto-generated INTERFACE)
find_package(Protobuf REQUIRED)

list(APPEND INCLUDE_PATH ${CMAKE_BINARY_DIR} ${Protobufc_INCLUDE_DIR})


if(NOT DEFINED PROTOBUF_PROTOCC)
    message(FATAL_ERROR "Unable to locate protoc-c executable please set with -DPROTOBUF_PROTOCC!")
else()

    message(STATUS 
        "Using the following binary \"${PROTOBUF_PROTOCC}\" to create vmeta"                          )

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
        COMMAND ${PROTOBUF_PROTOCC}
        ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} 
        ${vmeta_protos}
    )


    add_custom_command(
        OUTPUT ${vmeta_protos_hdrs}
        COMMAND ${PROTOBUF_PROTOCC}
        ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} 
        ${vmeta_protos}
    )



    add_library(${PROJECT_NAME} ${vmeta_protos_srcs})
    target_include_directories(${PROJECT_NAME} PUBLIC 
                               "$<BUILD_INTERFACE:${INCLUDE_PATH}>"
                            )

    target_link_directories(${PROJECT_NAME} PRIVATE 
                             ${Protobufc_INCLUDE_DIR}/../lib)

    target_link_libraries(${PROJECT_NAME} PRIVATE 
                            video-metadata-proto-generated 
                            protobuf-c)


    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

endif()
