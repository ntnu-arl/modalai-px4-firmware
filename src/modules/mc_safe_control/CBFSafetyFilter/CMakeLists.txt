px4_add_library(CBFSafetyFilter
    CBFSafetyFilter.cpp
	CBFSafetyFilter.hpp
)
target_compile_options(CBFSafetyFilter PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(CBFSafetyFilter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}  ${CMAKE_SOURCE_DIR}/src/lib/qpoases/qpOASES/include)