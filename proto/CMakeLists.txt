cmake_minimum_required(VERSION 3.20)
project(video-metadata-proto VERSION 1.0)

add_library(${PROJECT_NAME}-generated INTERFACE)
find_package(Protobuf REQUIRED)

list(APPEND INCLUDE_PATH ${CMAKE_BINARY_DIR} ${Protobuf_INCLUDE_DIR})


if(NOT DEFINED Protobuf_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "Unable to locate protoc-c executable please set with -DPROTOBUF_PROTOCC!")
else()
    get_filename_component(Protobuf_PROTOC_EXECUTABLE_PATH ${Protobuf_INCLUDE_DIR} DIRECTORY)
    set(Protobuf_PROTOC_GEN_C_EXECUTABLE_PATH ${Protobuf_PROTOC_EXECUTABLE_PATH}/tools/protobuf-c)
    set(Protobuf_PROTOC_GEN_C_EXECUTABLE ${Protobuf_PROTOC_GEN_C_EXECUTABLE_PATH}/protoc-gen-c${CMAKE_EXECUTABLE_SUFFIX})
    if(CMAKE_BUILD_TYPE MATCHES "Debug")
        list(FIND Protobuf_LIBRARY debug lib_index)
    else()
        list(FIND Protobuf_LIBRARY optimized lib_index)
    endif()
    math(EXPR lib_index "${lib_index} + 1")
    list(GET Protobuf_LIBRARY ${lib_index} Protobuf_LIBRARY_USE)
    get_filename_component(Protobuf_LIBRARY_DIRECTORY ${Protobuf_LIBRARY_USE} DIRECTORY)   

    message(STATUS 
        "Using the following binary \"${Protobuf_PROTOC_EXECUTABLE}\" to create vmeta")
    message(STATUS 
        "Using the following protobuf-c plugin \"${Protobuf_PROTOC_GEN_C_EXECUTABLE}\"")
    message(STATUS 
        "Using the following protobuf-c library \"${Protobuf_LIBRARY_DIRECTORY}\"")

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
        OUTPUT ${vmeta_protos_srcs} ${vmeta_protos_hdrs}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} --plugin "protoc-gen-c=${Protobuf_PROTOC_GEN_C_EXECUTABLE}" 
        ${vmeta_protos}
    )

    add_library(${PROJECT_NAME} SHARED ${vmeta_protos_srcs})
    set_target_properties(${PROJECT_NAME} PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
    )
    target_link_directories(${PROJECT_NAME} PUBLIC ${Protobuf_LIBRARY_DIRECTORY})
    target_include_directories(${PROJECT_NAME} PUBLIC 
                               "$<BUILD_INTERFACE:${INCLUDE_PATH}>"
                            )

    target_link_libraries(${PROJECT_NAME} PRIVATE 
                            ${PROJECT_NAME}-generated 
                            protobuf-c)

    install(FILES ${vmeta_protos_hdrs} DESTINATION include)
    file(GLOB protobuf_c_hdrs "${Protobuf_INCLUDE_DIR}/protobuf-c/*.h")
    install(FILES ${protobuf_c_hdrs} DESTINATION include/protobuf-c)

    if(WIN32)
            install(CODE "file(GLOB dll_files \"${CMAKE_CURRENT_BINARY_DIR}/*.dll\")")
            install(CODE "file(INSTALL DESTINATION \"\$\{CMAKE_INSTALL_PREFIX\}/bin\" TYPE SHARED_LIBRARY FILES \$\{dll_files\})")
    else()
        install(CODE [[
            file(GET_RUNTIME_DEPENDENCIES
                LIBRARIES $<TARGET_FILE:video-metadata>
                RESOLVED_DEPENDENCIES_VAR _r_deps
                UNRESOLVED_DEPENDENCIES_VAR _u_deps
                POST_EXCLUDE_REGEXES "^/lib" "^/usr/lib"
            )
            foreach(_file ${_r_deps})
                file(INSTALL
                DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
                TYPE SHARED_LIBRARY
                FOLLOW_SYMLINK_CHAIN
                FILES "${_file}"
                )
            endforeach()
            list(LENGTH _u_deps _u_length)
        ]])
    endif()

    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

endif()
