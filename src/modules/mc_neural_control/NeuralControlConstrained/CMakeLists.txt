add_compile_options(-Wno-error=float-equal)
find_package(Eigen3 REQUIRED)
#find_package(Eigen3 PATHS "/usr/local/workspace/px4-firmware/src/lib/eigen/build" NO_DEFAULT_PATH REQUIRED)

px4_add_library(NeuralControlConstrained
	NeuralControlConstrained.cpp
	NeuralControlConstrained.hpp
)

include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(NeuralControlConstrained PRIVATE Eigen3::Eigen)

#target_compile_options(NeuralControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(NeuralControlConstrained PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

