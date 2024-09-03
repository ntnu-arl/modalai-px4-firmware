/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_params.c
 * Parameters for multicopter controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Toggle for manual control
 *
 * @boolean
 * @group Multicopter SM Control
 */
PARAM_DEFINE_INT32(SM_MANUAL_CTRL, 0);

/**
 * mass [kg]
 *
 * ...
 *
 * @min 0.0
 * @max 5000
 * @decimal 2
 * @increment 0.001
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_MASS, 0.317f);

/**
 * Thrust max
 *
 * ...
 *
 * @min 0.0
 * @max 50000
 * @decimal 2
 * @increment 0.01
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_T_MAX, 32.0f);

/**
 * Moment max roll/pitch
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.001
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_M_RP_MAX, 0.26f);

/**
 * Moment max yaw
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 3
 * @increment 0.001
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_M_Y_MAX, 0.20f); // 1.0f

/**
 * Hover percentage
 *
 * ...
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_HOVER, 0.5f);

/**
 * inertia xx
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_I_XX, 0.0004933f);

/**
 * inertia yy
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_I_YY, 0.0005977f);

/**
 * inertia zz
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 9
 * @increment 0.00001
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_I_ZZ, 0.0008339f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter SM Control
 */
PARAM_DEFINE_INT32(SM_BAT_SCALE_EN, 0);

/**
 * XYZ Kp
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 3
 * @increment 0.001
 * @group Multicopter PD Position Control
 */

PARAM_DEFINE_FLOAT(SM_PD_KP_XYZ, 1.6f);

/**
 * XYZ Kd
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 3
 * @increment 0.001
 * @group Multicopter PD Position Control
 */

PARAM_DEFINE_FLOAT(SM_PD_KD_XYZ, 2.26f);

/**
 * RP Kp
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 3
 * @increment 0.001
 * @group Multicopter PD Position Control
 */

PARAM_DEFINE_FLOAT(SM_PD_KP_RP, 2.0f);

/**
 * Y Kp
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 3
 * @increment 0.001
 * @group Multicopter PD Position Control
 */
PARAM_DEFINE_FLOAT(SM_PD_KP_Y, 0.2f);

/**
 * RP Kd
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 3
 * @increment 0.001
 * @group Multicopter PD Position Control
 */
PARAM_DEFINE_FLOAT(SM_PD_KD_RP, 0.28f);

/**
 * Y Kd
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 3
 * @increment 0.001
 * @group Multicopter PD Position Control
 */
PARAM_DEFINE_FLOAT(SM_PD_KD_Y, 0.13f);

/**
 * Control Selector (0: NL-PD, 1: SMC, etc)
 *
 * ...
 *
 * @min 0
 * @max 1
 * @group Multicopter PD Position Control
 */
PARAM_DEFINE_INT32(SM_CONTROLLER, 0);

/**
 * Verbosity (printing commanded torque and thrust)
 *
 * @boolean
 * @group Multicopter SM Control
 */
PARAM_DEFINE_INT32(SM_VERBOSE, 1);

/**
 * Minimal RPM for the motors
 *
 * ...
 *
 * @min 0
 * @max 5000
 * @group Multicopter Neural Control
 */

PARAM_DEFINE_INT32(SM_MIN_RPM, 1000);

/**
 * Maximal RPM for the motors
 *
 * ...
 *
 * @min 15000
 * @max 30000
 * @group Multicopter Neural Control
 */

PARAM_DEFINE_INT32(SM_MAX_RPM, 22000);


/**
 * Thrust Coefficient
 *
 * ...
 *
 * @min 0.000004
 * @max 0.0001
 * @decimal 9
 * @increment 0.00000001
 * @group Multicopter Neural Control
 */
PARAM_DEFINE_FLOAT(SM_CT, 0.000006514f);


/**
 * range in which the error is allowed to be (clamped when exceeded)
 * 
 * ...
 *
 * @min -10.0
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Neural Control
 */
PARAM_DEFINE_FLOAT(SM_MAX_ERR, 0.5f);
