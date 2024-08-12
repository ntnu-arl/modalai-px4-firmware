px4_add_library(PDAttitudeControl
	PDAttitudeControl.cpp
	PDAttitudeControl.hpp
)
target_compile_options(PDAttitudeControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(PDAttitudeControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# px4_add_unit_gtest(SRC AttitudeControlTest.cpp LINKLIBS AttitudeControl)
# px4_add_unit_gtest(SRC AttitudeControlMathTest.cpp LINKLIBS AttitudeControl)
