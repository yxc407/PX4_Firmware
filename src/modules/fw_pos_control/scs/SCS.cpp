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
 * @file SCS.cpp
 * Suicide control system, this control system is used when the switchblade drone attacks.
 *
 * This file is derived from PX4 codebase with additional developments by YuXiuChen.
 *
 * @author YuXiuChen <yuxiuchen407@gmail.com>
 */

#include "SCS.h"

#include <lib/geo/geo.h>

using math::constrain;
using matrix::Vector2f;
using matrix::wrap_pi;
using namespace time_literals;

void PID::init(float kp, float ki, float kd, float ff) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _ff = ff;
    _integral = 0.f;
    _prev_error = 0.f;
    _is_first_calculate = true;
}

float PID::pid_calculate(float setpoint, float actual, float dt)
{
    float error = setpoint - actual;

    _integral += error * dt;

    float derivative = (error - _prev_error) / dt;
    _prev_error = error;
    if (_is_first_calculate) {
        derivative = 0.f;
        _is_first_calculate = false;
    }

    float output = _ff * setpoint + _kp * error + _ki * _integral + _kd * derivative;

    return output;
}

void SCS::init(float kp_pitch,
               float ki_pitch,
               float kd_pitch,
               float ff_pitch,
               float kp_heading,
               float ki_heading,
               float kd_heading,
               float kp_roll,
               float ki_roll,
               float kd_roll,
               float ff_roll)
{
    pitch_pid.init(kp_pitch,
                   ki_pitch,
                   kd_pitch,
                   ff_pitch);
    

    heading_pid.init(kp_heading,
                     ki_heading,
                     kd_heading,
                     0.f);

    roll_pid.init(kp_roll,
                  ki_roll,
                  kd_roll,
                  ff_roll);
}

void SCS::update(float pitch_setpoint, float pitch_actual, float heading_setpoint, float heading_actual, float roll_actual, float airspeed, float dt)
{
    _pitch_output = pitch_pid.pid_calculate(pitch_setpoint, pitch_actual, dt);

    _heading_rate = heading_pid.pid_calculate(heading_setpoint, heading_actual, dt);
    // PX4_INFO("yaw curr: %f, yaw sp: %f, yaw err: %f, yaw op: %f",
    //         (double)heading_actual,
    //         (double)heading_setpoint,
    //         (double)(heading_setpoint - heading_actual),
    //         (double)_heading_rate);
    // ideal situation: roll(rad) = arctan(yaw_rate(rad / s) * airspeed(m / s) / G(m / s^2))
    float roll_setpoint = atanf(_heading_rate * airspeed / CONSTANTS_ONE_G);
    _roll_output = roll_pid.pid_calculate(roll_setpoint, roll_actual, dt);
    PX4_INFO("roll curr: %f, roll sp: %f,roll err: %f,roll op: %f",
            (double)roll_actual * 180.0 / M_PI,
            (double)roll_setpoint * 180.0 / M_PI,
            (double)(roll_setpoint - roll_actual) * 180.0 / M_PI,
            (double)_roll_output * 180.0 / M_PI);
}
