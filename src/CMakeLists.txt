aq_require(acquire-core-logger)
aq_require(acquire-core-platform)
aq_require(acquire-device-properties)
aq_require(acquire-device-kit)

set(tgt acquire-driver-egrabber)
add_library(${tgt} MODULE euresys.egrabber.cpp)
target_enable_simd(${tgt})
set_target_properties(${tgt} PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
)
target_link_libraries(${tgt} PRIVATE
        acquire-core-logger
        acquire-core-platform
        acquire-device-kit
        egrabber
)

install(TARGETS ${tgt} LIBRARY DESTINATION lib)
