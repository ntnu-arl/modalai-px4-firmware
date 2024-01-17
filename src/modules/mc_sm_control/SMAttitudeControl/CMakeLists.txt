px4_add_library(SMAttitudeControl
	SMAttitudeControl.cpp
	SMAttitudeControl.hpp
)
target_compile_options(SMAttitudeControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(SMAttitudeControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# px4_add_unit_gtest(SRC AttitudeControlTest.cpp LINKLIBS AttitudeControl)
# px4_add_unit_gtest(SRC AttitudeControlMathTest.cpp LINKLIBS AttitudeControl)
