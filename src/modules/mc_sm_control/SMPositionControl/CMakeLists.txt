px4_add_library(SMPositionControl
	SMPositionControl.cpp
	SMPositionControl.hpp
)
target_compile_options(SMPositionControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(SMPositionControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# px4_add_unit_gtest(SRC PositionControlTest.cpp LINKLIBS PositionControl)
# px4_add_unit_gtest(SRC PositionControlMathTest.cpp LINKLIBS PositionControl)
