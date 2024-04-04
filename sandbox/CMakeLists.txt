option(WITH_SANDBOX "Build the sandbox" OFF)

if (${WITH_SANDBOX})
    #
    # PARAMETERS
    #
    set(project acquire-driver-egrabber) # CMAKE_PROJECT_NAME gets overridden if this is a subtree of another project

    ### Playground
    set(tgt ${project}-playground)
    add_executable(${tgt} playground.cpp)
    set_target_properties(${tgt} PROPERTIES
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    target_include_directories(${tgt} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../")
    target_link_libraries(${tgt} PRIVATE
            egrabber
            acquire-core-logger
            acquire-core-platform)

    ### Delivered height vs. height
    set(tgt ${project}-delivered-height-vs-height)

    set(NO_TEST ${NOTEST})
    set(NOTEST "FALSE")
    add_subdirectory(../acquire-common/acquire-driver-common ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-common)
    add_subdirectory(../acquire-common/acquire-video-runtime ${CMAKE_CURRENT_BINARY_DIR}/acquire-video-runtime)
    set(NOTEST ${NO_TEST})
    add_subdirectory(../tests/acquire-driver-zarr ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-zarr)

    add_executable(${tgt} delivered-height-vs-height.cpp)
    set_target_properties(${tgt} PROPERTIES
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    target_include_directories(${tgt} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../")
    target_link_libraries(${tgt} PRIVATE
            egrabber
            acquire-core-logger
            acquire-core-platform
            acquire-video-runtime)

    foreach(driver
            acquire-driver-egrabber
            acquire-driver-zarr)
        add_custom_target(${project}-copy-${driver}-for-delivered-height-vs-height
                COMMAND ${CMAKE_COMMAND} -E copy
                $<TARGET_FILE:${driver}>
                $<TARGET_FILE_DIR:${tgt}>
                DEPENDS ${driver}
                COMMENT "Copying ${driver} to $<TARGET_FILE_DIR:${tgt}>")

        foreach(name ${tests})
            add_dependencies(${tgt} ${project}-copy-${driver}-for-delivered-height-vs-height)
        endforeach()
    endforeach()
endif ()
