/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file SCS.h
 * Suicide control system, this control system is used when the switchblade drone attacks.
 *
 * This file is derived from PX4 codebase with additional developments by YuXiuChen.
 *
 * @author YuXiuChen <yuxiuchen407@gmail.com>
 */

#ifndef SCS_H
#define SCS_H

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include "matrix/Matrix.hpp"
#include "matrix/Vector2.hpp"

using matrix::Vector2f;

class PID
{
public:
    void init(float kp, float ki, float kd, float ff);
    float pid_calculate(float sp, float val, float dt);
    void reset_integral() { _integral = 0.f; }

private:
    float _kp{0.f};
    float _ki{0.f};
    float _kd{0.f};
    float _ff{0.f};

    float _integral{0.f};
    float _prev_error{0.f};
    bool _is_first_calculate{true};
};

class __EXPORT SCS
{
public:
    SCS() = default;
    ~SCS() = default;

    void init(float kp_pitch,
              float ki_pitch,
              float kd_pitch,
              float ff_pitch,
              float kp_heading,
              float ki_heading,
              float kd_heading,
              float kp_roll,
              float ki_roll,
              float kd_roll,
              float ff_roll);

    void update(float pitch_setpoint, float pitch_actual,
                float heading_setpoint, float heading_actual, float roll_actual, float airspeed,
                float dt);

    inline float get_pitch_output() { return _pitch_output; }
    inline float get_roll_output() { return _roll_output; }

private:
    PID pitch_pid;
    PID heading_pid;
    PID roll_pid;

    float _pitch_output{0.f};
    float _heading_rate{0.f};
    float _roll_output{0.f};
};

#endif /* SCS_H */
