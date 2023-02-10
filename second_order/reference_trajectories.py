#    LMPC_walking is a python software implementation of one of the algorithms
#    refered in this paper https://hal.inria.fr/inria-00391408v2/document
#    Copyright (C) 2019 @ahmad gazar

#    LMPC_walking is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    LMPC_walking is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

import numpy as np

# Description:
# -----------
# this function implements a desired zig-zag fixed foot step plan located
# in the middle of the robot's foot starting with the right foot

# Parameters:
# ----------
#  foot_step_0 : initial foot step location
#            [foot_step_x0, foot_step_y0].T (2x1 numpy.array)
#  no_steps    : number of desired walking foot steps  (scalar)

# Returns:
# -------
#  Foot_steps = [Foot_steps_x, Foot_steps_y].T   foot steps locations
#                                                (no_steps x 2 numpy.array)

def manual_foot_placement(foot_step_0, fixed_step_x, no_steps):
    Foot_steps   = np.zeros((no_steps, 2))
    for i in range(Foot_steps.shape[0]):
        if i == 0:
            Foot_steps[i,:] = foot_step_0
        else:
            Foot_steps[i,0] = Foot_steps[i-1,0] + fixed_step_x
            Foot_steps[i,1] = -Foot_steps[i-1,1]
    return Foot_steps

# Description:
# -----------
# this function computes a CoP reference trajectory based on a desired
# fixed foot step plan, a desired foot step duration and a sampling time

# Parameters:
# ----------
#  no_steps      : number of desired walking foot steps  (scalar)
#  Foot_steps    := [Foot_steps_x, Foot_steps_y]   foot steps locations
#                                                  (no_steps x 2 numpy.array)
# walking_time   : desired walking time duration   (scalar)
# no_steps_per_T : step_duration/T  (scalar)

# Returns:
# -------
# Z_ref  := [z_ref_x_k             , z_ref_y_k              ]   CoP reference trajectory
#                   .              ,    .                       (walking_timex2 numpy.array)
#                   .              ,    .
#           [z_ref_x_k+walking_time, z_ref_y_k+walking_time]

def create_CoP_trajectory(no_steps, Foot_steps, walking_time, no_steps_per_T):
    Z_ref  = np.zeros((walking_time,2))
    j = 0
    for i in range (Foot_steps.shape[0]):
        Z_ref[j:j+no_steps_per_T, :] = Foot_steps[i,:]
        j = j + no_steps_per_T
    return Z_ref

# Description:
# -----------
# this function computes a CoP reference trajectory based on a desired fixed foot step plan
# on the horizon, with a variable duration for the first foot step and a sampling time

# Parameters:
# ----------
#  foot_steps       : foot steps locations
#  horizon          : MPC horizon
#  nb_steps_per_T   : time steps during one foot step of the robot
#  nb_steps_HS      : time steps in the first foot step,
#                     considering the current MPC time step

# Returns:
# -------
# Z_ref             : CoP reference trajectory for the current horizon
# foot_steps        : updated foot steps locations

def varying_CoP_trajectory(foot_steps, horizon, nb_steps_per_T, nb_steps_HS):
    Z_ref = np.zeros((horizon, 2))
    if nb_steps_HS + nb_steps_per_T < horizon and nb_steps_HS > 1:
        Z_ref[:nb_steps_HS,:] = foot_steps[0,:]
        Z_ref[nb_steps_HS:nb_steps_HS + nb_steps_per_T,:] = foot_steps[1,:]
        Z_ref[nb_steps_HS + nb_steps_per_T:horizon,:] = foot_steps[2,:]
    else:
        if nb_steps_HS < 2:
            foot_steps = foot_steps[1:,:]
            Z_ref[:nb_steps_per_T,:] = foot_steps[0,:]
            Z_ref[nb_steps_per_T:,:] = foot_steps[1,:]
        else:
            Z_ref[:nb_steps_HS,:] = foot_steps[0,:]
            Z_ref[nb_steps_HS:,:] = foot_steps[1,:]
    return Z_ref, foot_steps
