cmake_minimum_required(VERSION 3.20)
project(video-metadata-proto VERSION 1.0)

add_library(video-metadata-proto-generated INTERFACE)
find_package(Protobuf REQUIRED)

list(APPEND INCLUDE_PATH ${CMAKE_BINARY_DIR} ${Protobuf_INCLUDE_DIR})


if(NOT DEFINED Protobuf_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "Unable to locate protoc-c executable please set with -DPROTOBUF_PROTOCC!")
else()

    message(STATUS 
        "Using the following binary \"${Protobuf_PROTOC_EXECUTABLE}\" to create vmeta"                          )

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
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} 
        ${vmeta_protos}
    )


    add_custom_command(
        OUTPUT ${vmeta_protos_hdrs}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --c_out ${CMAKE_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} 
        ${vmeta_protos}
    )



    add_library(${PROJECT_NAME} ${vmeta_protos_srcs})
    target_link_directories(${PROJECT_NAME} PUBLIC ${Protobuf_INCLUDE_DIRS}/../debug/lib)
    target_include_directories(${PROJECT_NAME} PUBLIC 
                               "$<BUILD_INTERFACE:${INCLUDE_PATH}>"
                            )
    target_include_directories(${PROJECT_NAME} PUBLIC 
                             ${Protobufc_INCLUDE_DIR}
			     )
    target_link_directories(${PROJECT_NAME} PUBLIC 
                             ${Protobufc_INCLUDE_DIR}/../lib)

    target_link_libraries(${PROJECT_NAME} PRIVATE 
                            video-metadata-proto-generated 
                            protobuf-c)


if(WIN32)
	if(CMAKE_BUILD_TYPE STREQUAL "Debug")

		message("PROTOBUF INCLUDE DIR ${PROTOBUF_INCLUDE_DIR}")
		file(GLOB protobuf_dll "${PROTOBUF_INCLUDE_DIR}/../debug/bin/*.dll")
		file(GLOB protobufc_dll "${Protobufc_INCLUDE_DIR}/../debug/bin/*.dll")
		message(${protobuf_dll})
		message(${protobufc_dll})
		# message(${protobuf_pdb})
        	install(FILES ${protobuf_dll} DESTINATION bin)
		install(FILES ${protobufc_dll} DESTINATION bin)
	else()
		file(GLOB protobuf_dll "${PROTOBUF_INCLUDE_DIR}/../bin/*.dll")
		file(GLOB protobufc_dll "${Protobufc_INCLUDE_DIR}/../bin/*.dll")
		message(${protobuf_dll})
        	install(FILES ${protobuf_dll} DESTINATION bin)
		install(FILES ${protobufc_dll} DESTINATION bin)
	endif()
endif()

    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

endif()
