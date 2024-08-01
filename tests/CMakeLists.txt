if(${NOTEST})
        message(STATUS "Skipping test targets")
else()
        set(NOTEST "TRUE")
        add_subdirectory(../acquire-common/acquire-driver-common ${CMAKE_CURRENT_BINARY_DIR}/acquire-driver-common)
        add_subdirectory(../acquire-common/acquire-video-runtime ${CMAKE_CURRENT_BINARY_DIR}/acquire-video-runtime)
        #        aq_require(acquire-driver-zarr)
        set(NOTEST "FALSE")

        #
        # PARAMETERS
        #
        set(project acquire-driver-egrabber) # CMAKE_PROJECT_NAME gets overridden if this is a subtree of another project

        #
        # Tests
        #
        set(tests
                list-devices
                abort-while-waiting-for-trigger
                configure-triggering
                one-video-stream
                repeat-start
                repeat-start-no-stop
        )

        foreach(name ${tests})
                set(tgt "${project}-${name}")
                add_executable(${tgt} ${name}.cpp)
                target_compile_definitions(${tgt} PUBLIC "TEST=\"${tgt}\"")
                set_target_properties(${tgt} PROPERTIES
                        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
                )
                target_include_directories(${tgt} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../")
                target_link_libraries(${tgt}
                        acquire-core-logger
                        acquire-core-platform
                        acquire-video-runtime
                )

                add_test(NAME test-${tgt} COMMAND ${tgt})
                set_tests_properties(test-${tgt} PROPERTIES LABELS acquire-driver-egrabber)
        endforeach()

        #
        # Copy driver to tests
        #
        list(POP_FRONT tests onename)

        foreach(driver
                acquire-driver-common
                acquire-driver-egrabber
        )
                add_custom_target(${project}-copy-${driver}-for-tests
                        COMMAND ${CMAKE_COMMAND} -E copy
                        $<TARGET_FILE:${driver}>
                        $<TARGET_FILE_DIR:${project}-${onename}>
                        DEPENDS ${driver}
                        COMMENT "Copying ${driver} to $<TARGET_FILE_DIR:${project}-${onename}>"
                )

                foreach(name ${tests})
                        add_dependencies(${tgt} ${project}-copy-${driver}-for-tests)
                endforeach()
        endforeach()

        # download the acquire-driver-zarr shared library to the tests directory
        include(FetchContent)

        set(ZARR_RELEASE_URL "https://github.com/acquire-project/acquire-driver-zarr/releases/download")
        set(ZARR_VERSION "0.1.12")

        if (WIN32)
                set(ZARR_DRIVER_SUFFIX "win64")
        elseif (APPLE)
                set(ZARR_DRIVER_SUFFIX "Darwin")
        elseif (UNIX)
                set(ZARR_DRIVER_SUFFIX "Linux")
        endif ()

        FetchContent_Declare(
                acquire_driver_zarr
                URL "${ZARR_RELEASE_URL}/v${ZARR_VERSION}/acquire-driver-zarr-v${ZARR_VERSION}-${ZARR_DRIVER_SUFFIX}.zip"
                DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )

        FetchContent_MakeAvailable(acquire_driver_zarr)
        set(ZARR_DRIVER_PATH ${acquire_driver_zarr_SOURCE_DIR}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}acquire-driver-zarr${CMAKE_SHARED_LIBRARY_SUFFIX})

        add_custom_target(copy_zarr_driver
                COMMAND ${CMAKE_COMMAND} -E copy
                ${ZARR_DRIVER_PATH}
                $<TARGET_FILE_DIR:${project}-${onename}>
                DEPENDS ${ZARR_DRIVER_PATH}
                COMMENT "Copying acquire-driver-zarr to $<TARGET_FILE_DIR:${project}-${onename}>"
        )

        add_dependencies(${tgt} copy_zarr_driver)
endif()
