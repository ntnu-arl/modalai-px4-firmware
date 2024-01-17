add_subdirectory(SMPositionControl)
add_subdirectory(SMAttitudeControl)

px4_add_module(
	MODULE modules__mc_sm_control
	MAIN mc_sm_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		mc_sm_control.cpp
		mc_sm_control.hpp
	DEPENDS
		SMPositionControl
		SMAttitudeControl
		mathlib
		px4_work_queue
	)
