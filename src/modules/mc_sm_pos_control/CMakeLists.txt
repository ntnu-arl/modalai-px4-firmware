add_subdirectory(SMPositionControl)

px4_add_module(
	MODULE modules__mc_sm_pos_control
	MAIN mc_sm_pos_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		mc_sm_pos_control.cpp
		mc_sm_pos_control.hpp
	DEPENDS
		SMPositionControl
		mathlib
		px4_work_queue
	)
