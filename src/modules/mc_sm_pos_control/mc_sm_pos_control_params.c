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
 * Parameters for multicopter position controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * X sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(LAM_X, 2.0f);

/**
 * Y sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(LAM_Y, 2.0f);

/**
 * Z sliding surface dynamics
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(LAM_Z, 2.0f);

/**
 * X switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(GAIN_X, 2.0f);

/**
 * Y switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(GAIN_Y, 2.0f);

/**
 * Z switching gain
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(GAIN_Z, 2.0f);

/**
 * tanh factor
 *
 * ...
 *
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(TANH_FACTOR, 10.0f);

/**
 * mass [g]
 *
 * ...
 *
 * @min 0.0
 * @max 5000
 * @decimal 2
 * @increment 0.001
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(MASS, 1000.0f);

/**
 * Thrust max
 *
 * ...
 *
 * @min 0.0
 * @max 50000
 * @decimal 2
 * @increment 0.01
 * @group Multicopter SM Position Control
 */
PARAM_DEFINE_FLOAT(THRUST_MAX, 100.0f);
