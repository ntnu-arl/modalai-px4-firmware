/**
 * @file mc_nmpc_control_params.c
 * Parameters for NMPC multicopter controller.
 */

/**
 * Vehicle mass [kg]
 *
 * @min 0.01
 * @max 50.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_MASS, 0.317f);

/**
 * Inertia Ixx
 *
 * @min 0.0
 * @max 1.0
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_IXX, 0.0004933f);

/**
 * Inertia Ixy
 *
 * @min -1.0
 * @max 1.0
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_IXY, 0.0f);

/**
 * Inertia Ixz
 *
 * @min -1.0
 * @max 1.0
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_IXZ, 0.0f);

/**
 * Inertia Iyy
 *
 * @min 0.0
 * @max 1.0
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_IYY, 0.0005977f);

/**
 * Inertia Iyz
 *
 * @min -1.0
 * @max 1.0
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_IYZ, 0.0f);

/**
 * Inertia Izz
 *
 * @min 0.0
 * @max 1.0
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_IZZ, 0.0008339f);

/**
 * Thrust coefficient motor 1
 *
 * @min 0.000001
 * @max 0.001
 * @decimal 9
 * @increment 0.00000001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_KF1, 0.000014247f);

/**
 * Thrust coefficient motor 2
 *
 * @min 0.000001
 * @max 0.001
 * @decimal 9
 * @increment 0.00000001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_KF2, 0.000014247f);

/**
 * Thrust coefficient motor 3
 *
 * @min 0.000001
 * @max 0.001
 * @decimal 9
 * @increment 0.00000001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_KF3, 0.000014247f);

/**
 * Thrust coefficient motor 4
 *
 * @min 0.000001
 * @max 0.001
 * @decimal 9
 * @increment 0.00000001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_KF4, 0.000014247f);

/**
 * Motor time constant 1
 *
 * @min 0.001
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_TC1, 0.05f);

/**
 * Motor time constant 2
 *
 * @min 0.001
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_TC2, 0.05f);

/**
 * Motor time constant 3
 *
 * @min 0.001
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_TC3, 0.05f);

/**
 * Motor time constant 4
 *
 * @min 0.001
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_TC4, 0.05f);

/**
 * COM offset X
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_COMX, 0.0f);

/**
 * COM offset Y
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_COMY, 0.0f);

/**
 * COM offset Z
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_FLOAT(MC_NMPC_COMZ, 0.0f);

/**
 * Min motor RPM
 *
 * @min 0
 * @max 10000
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_INT32(MC_NMPC_MINRPM, 1000);

/**
 * Max motor RPM
 *
 * @min 10000
 * @max 50000
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_INT32(MC_NMPC_MAXRPM, 22000);

/**
 * Verbose logging
 *
 * @boolean
 * @group Multicopter NMPC Control
 */
PARAM_DEFINE_INT32(MC_NMPC_VERBOSE, 0);
