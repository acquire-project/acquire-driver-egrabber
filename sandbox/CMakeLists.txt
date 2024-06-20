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
endif ()
