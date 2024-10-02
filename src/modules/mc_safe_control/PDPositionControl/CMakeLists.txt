px4_add_library(PDPositionControl
	PDPositionControl.cpp
	PDPositionControl.hpp
)
target_compile_options(PDPositionControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(PDPositionControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# px4_add_unit_gtest(SRC PositionControlTest.cpp LINKLIBS PositionControl)
# px4_add_unit_gtest(SRC PositionControlMathTest.cpp LINKLIBS PositionControl)
