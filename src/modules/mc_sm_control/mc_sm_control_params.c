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
 * X sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_LAM_X, 0.2f);

/**
 * Y sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_LAM_Y, 0.2f);

/**
 * Z sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_LAM_Z, 0.2f);

/**
 * X switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_GAIN_X, 2.0f);

/**
 * Y switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_GAIN_Y, 2.0f);

/**
 * Z switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_GAIN_Z, 2.0f);

/**
 * tanh factor
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Control
 */
PARAM_DEFINE_FLOAT(SM_POS_TANH, 10.0f);

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
PARAM_DEFINE_FLOAT(SM_POS_MASS, 0.25f);

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
PARAM_DEFINE_FLOAT(SM_POS_T_MAX, 14.0f);

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
PARAM_DEFINE_FLOAT(SM_M_Y_MAX, 1.0f);

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
 * Roll sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_LAM_X, 10.0f);

/**
 * Pitch sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_LAM_Y, 10.0f);

/**
 * Yaw sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_LAM_Z, 5.0f);

/**
 * Roll switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_GAIN_X, 25.0f);

/**
 * Pitch switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_GAIN_Y, 25.0f);

/**
 * Yaw switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_GAIN_Z, 5.0f);

/**
 * tanh factor
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_TANH, 1.0f);

/**
 * inertia xx
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 5
 * @increment 0.00001
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_I_XX, 0.00085f);

/**
 * inertia yy
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 5
 * @increment 0.00001
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_I_YY, 0.00087f);

/**
 * inertia zz
 *
 * ...
 *
 * @min 0.0
 * @max 1
 * @decimal 5
 * @increment 0.00001
 * @group Multicopter SM Attitude Control
 */
PARAM_DEFINE_FLOAT(SM_ATT_I_ZZ, 0.00162f);

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
PARAM_DEFINE_INT32(SM_VERBOSE, 0);
