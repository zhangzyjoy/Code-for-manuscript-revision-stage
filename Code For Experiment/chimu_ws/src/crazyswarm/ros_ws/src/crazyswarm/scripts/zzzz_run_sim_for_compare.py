#!/usr/bin/env python3

#### Script for software-in-loop simulation with the comparative method
#### Save workspace file <chimu_ws> in main directory "/home/<username>/"
#### please refer to <https://crazyswarm.readthedocs.io/en/latest/installation.html> to install the requirements

#### Enter "~/chimu_ws/src/crazyswarm/ros_ws"
#### Execute <catkin_make> in "~/chimu_ws/src/crazyswarm/ros_ws" directory and compile

#### Open another terminal and enter "~/chimu_ws/src/crazyswarm/ros_ws/src/crazyswarm/scripts/"
#### Run "python3 chooser.py" to choose UAV number used in the experiment
#### from "/launch/allCrazyflies.yaml" into "/launch/crazyflies.yaml"

#### Use "Ctrl+C" to close chooser control center
#### Run "python3 zzzz_run_sim_for_compare.py --sim" to run the software-in-loop simulation

import numpy as np
from pycrazyswarm import Crazyswarm
from fractions import Fraction

import random
random.seed(2)

# Parameter setting for takeoff and landing control
takeoff_height = np.array([0.7, 0.7, 0.7, 0.7, 0.8])
land_height = 0.02

takeoff_duration = 2.0
land_duration = 2.0

Kpz = 0.5

# Interaction topology
num_follower = 4

wil_mat = np.array([1, 0, 0, 0])
wij_mat = np.array([[0, 0, 0, 0], \
                    [1, 0, 0, 0], \
                    [0, 1, 0, 0], \
                    [0, 1, 0, 0]])

# Parameter setting for leader trajectory tracking control
Kp_track = 0.5 * np.eye(3)
Ki_track = 0.05 * np.eye(3)
Kd_track = 0.05 * np.eye(3)

# Parameter setting for leader desired trajectory
time_seq_1 = 20.0
time_seq_1_2 = 20.0
time_seq_2 = 40.0
total_time = time_seq_1 + time_seq_2
radius_0 = 0.4
radius_1 = 0.6
pos_z_0 = takeoff_height[num_follower]
pos_z_1 = 1.0
radius_x_2 = 0.5
radius_y_2 = 0.4
radius_z_2 = 0.05

# Parameter setting for desired formation configuration
fl_x_1 = 0.8
fl_x_2 = 0.3
fl_x_3 = 0.3
fl_y_1 = 0.6
fl_y_2 = 0.2
fl_y_3 = 0.1
fl_z_1 = 0.05
fl_z_2 = 0.1
fl_z_3 = 0.05
fl_theta = 45.0 * np.pi / 180.0
kai_3_t = 0.5
desire_fl_config = np.zeros((num_follower, 3))

# Parameter setting for distributed observer
gamma_pos_1 = 2.0
gamma_pos_2 = 0.5
gamma_vel_1 = 2.0
gamma_vel_2 = 0.5

mu_obs_pos = 100.0

l_pos_1 = 1.5 * np.eye(3)
l_pos_2 = 2.0 * np.eye(3)
l_vel_1 = 1.5 * np.eye(3)
l_vel_2 = 2.0 * np.eye(3)

pos_des_hat_fl = np.zeros((num_follower, 3))
d_pos_des_hat_fl = np.zeros((num_follower, 3))

vel_des_hat_fl = np.zeros((num_follower, 3))
d_vel_des_hat_fl = np.zeros((num_follower, 3))

err_vel_obs_nodes = np.zeros((num_follower, 3))
err_pos_obs_nodes = np.zeros((num_follower, 3))

err_pos_obs_tilt = np.zeros((num_follower, 3))
err_vel_obs_tilt = np.zeros((num_follower, 3))
sig_err_pos_obs_1 = np.zeros((num_follower, 3))
sig_err_pos_obs_2 = np.zeros((num_follower, 3))
sig_err_vel_obs_1 = np.zeros((num_follower, 3))
sig_err_vel_obs_2 = np.zeros((num_follower, 3))

total_time_wait_vel_obs = 5.0
total_time_wait_pos_obs = 5.0

# Parameter setting for high-order sliding mode differentiator
zeta_diff_sigma_bar = 0.08
lambda_diff_sigma_bar = 0.2
c1_diff_sigma_bar = np.diag([2.0, 2.0, 2.0])
c2_diff_sigma_bar = np.diag([5.0, 5.0, 5.0])
c3_diff_sigma_bar = np.diag([8.0, 8.0, 8.0])

zeta_1 = 1 + zeta_diff_sigma_bar
zeta_2 = 1 + ( 2 / 3 ) * zeta_diff_sigma_bar
zeta_3 = 1 + ( 1 / 3 ) * zeta_diff_sigma_bar

sigma_bar_hat_0 = np.zeros((num_follower, 3))
sigma_bar_hat_1 = np.zeros((num_follower, 3))
sigma_bar_hat_2 = np.zeros((num_follower, 3))
sigma_bar_tilt_0 = np.zeros((num_follower, 3))
d_sigma_bar_hat_0 = np.zeros((num_follower, 3))
d_sigma_bar_hat_1 = np.zeros((num_follower, 3))
d_sigma_bar_hat_2 = np.zeros((num_follower, 3))

# Parameter setting for disturbance observer
mu_d_vel = 100.0
c_vel_1 = np.diag([2.0, 2.0, 2.0])
c_vel_2 = np.diag([2.0, 2.0, 2.0])
c_vel_3 = np.diag([3.0, 3.0, 3.0])
alfa_vel_1 = 2.0
alfa_vel_2 = 0.5

disturb_pos = np.zeros((num_follower, 3))
disturb_pos_hat = np.zeros((num_follower, 3))
vel_dist_obs = np.zeros((num_follower, 3))
acc_dist_obs = np.zeros((num_follower, 3))
sigma_vel = np.zeros((num_follower, 3))
sigma_vel_bar = np.zeros((num_follower, 3))
sigma_vel_bar_hat = np.zeros((num_follower, 3))
sigma_vel_bar_tilt = np.zeros((num_follower, 3))
d_sigma_vel = np.zeros((num_follower, 3))
d_sigma_vel_bar = np.zeros((num_follower, 3))
d_sigma_vel_bar_hat = np.zeros((num_follower, 3))

# Parameter setting for formation controller
kai_chi_1 = 0.2
kai_chi_2 = 0.2
kai_u_1 = 0.2
kai_u_2 = 0.2
beta_1 = 2.0
beta_2 = 0.5

mu_ctrl_pos = 100.0

chi_fl_nodes = np.zeros((num_follower, 3))
d_chi_fl_nodes = np.zeros((num_follower, 3))
phi_fl_nodes = np.zeros((num_follower, 3))
pos_fl_nodes = np.zeros((num_follower, 3))
vel_fl_nodes = np.zeros((num_follower, 3))
err_vel_nodes = np.zeros((num_follower, 3))
err_pos_nodes = np.zeros((num_follower, 3))
d_err_pos_nodes = np.zeros((num_follower, 3))
sig_err_pos_beta_1 = np.zeros((num_follower, 3))
sig_err_pos_beta_2 = np.zeros((num_follower, 3))
sig_phi_beta_1 = np.zeros((num_follower, 3))
sig_phi_beta_2 = np.zeros((num_follower, 3))
vel_cmd_fl_nodes = np.zeros((num_follower, 3))
acc_cmd_fl_nodes = np.zeros((num_follower, 3))

# f(x) = sig(x)^a = (|x|^a)*sign(x)
def cal_fracional_item(x, a):
    x_abs = abs(x)
    a_frac = Fraction(a)
    a_num = a_frac.limit_denominator().numerator
    a_den = a_frac.limit_denominator().denominator
    x_a = np.power(x_abs, a_num)
    sgn_x = np.sign(np.power(x, a_num))
    squ_x_a = np.power(x_a, (1 / a_den))
    y = squ_x_a * sgn_x
    return y

# f(x) = a*(|x|^(a-1))
def cal_abs_squ(x, a):
    x_abs = abs(x)
    a_frac = Fraction(a - 1)
    a_num = a_frac.limit_denominator().numerator
    a_den = a_frac.limit_denominator().denominator
    x_a = np.power(x_abs, a_num)
    y = a * np.power(x_a, (1 / a_den))
    return y

# f(x) = |x|^a
def cal_abs_x_a(x, a):
    x_abs = abs(x)
    a_frac = Fraction(a)
    a_num = a_frac.limit_denominator().numerator
    a_den = a_frac.limit_denominator().denominator
    x_a = np.power(x_abs, a_num)
    y = np.power(x_a, (1 / a_den))
    
    return y


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    # Initialize position of leader UAV
    leader_cf = allcfs.crazyflies[num_follower]
    leader_cf.init_uav_pos = leader_cf.position()
    leader_cf.current_uav_pos = leader_cf.position()
    
    # Takeoff
    flag_all_uav_takeoff = False
    for cf in allcfs.crazyflies:
        cf.flag_takeoff = False
    while flag_all_uav_takeoff == False:
        timeHelper.sleepForRate(10)
        for cf in allcfs.crazyflies:
            cf.current_uav_pos = cf.position()
            if cf.current_uav_pos[2] > takeoff_height[cf.id - 1] - 0.1:
                cf.flag_takeoff = True
                cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, 0.0]), yawRate = 0)
            else:
                cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, Kpz * (takeoff_height[cf.id - 1] - cf.current_uav_pos[2])]), yawRate = 0)
        cnt_uav_takeoff = 0
        for cf in allcfs.crazyflies:
            if cf.flag_takeoff == True:
                cnt_uav_takeoff = cnt_uav_takeoff + 1
        if cnt_uav_takeoff == len(allcfs.crazyflies):
            flag_all_uav_takeoff = True

    # Hovering and waiting for convergence of 
    # follower desire velocity distributed observer
    time_stamp_begin = timeHelper.time()
    time_stamp_last = timeHelper.time()
    time_stamp_now = timeHelper.time()

    while time_stamp_now - time_stamp_begin <= total_time_wait_vel_obs:
        timeHelper.sleepForRate(10)
        leader_cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, 0.0]), yawRate = 0)

        time_stamp_now = timeHelper.time()
        leader_cf.current_uav_vel = leader_cf.velocity()
        delta_time = time_stamp_now - time_stamp_last
        
        for i in range(num_follower):
            cf = allcfs.crazyflies[i]
            cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, 0.0]), yawRate = 0)
            
            err_vel_obs_tilt[i] = np.zeros((3,))
            if wil_mat[i] != 0:
                err_vel_obs_tilt[i] += ( wil_mat[i] ) * ( vel_des_hat_fl[i] - leader_cf.current_uav_vel )
            for j in range(num_follower):
                if wij_mat[i][j] != 0:
                    err_vel_obs_tilt[i] += ( wij_mat[i][j] ) * ( vel_des_hat_fl[i] - vel_des_hat_fl[j] )
            
            sig_err_vel_obs_1[i] = cal_fracional_item(err_vel_obs_tilt[i], gamma_vel_1)
            sig_err_vel_obs_2[i] = cal_fracional_item(err_vel_obs_tilt[i], gamma_vel_2)

            d_vel_des_hat_fl[i] = -np.matmul(l_vel_1, sig_err_vel_obs_1[i]) \
                                    - np.matmul(l_vel_2, sig_err_vel_obs_2[i])
            vel_des_hat_fl[i] += delta_time * d_vel_des_hat_fl[i]
        
        time_stamp_last = time_stamp_now

    # Initialize the formation configuration
    delta_fl_nodes = np.zeros( ( num_follower, 3 ) )
    delta_fl_nodes[0] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2, fl_y_1 * np.sin( fl_theta ) - fl_y_2, fl_z_1 - fl_z_2 ] )
    delta_fl_nodes[1] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2, fl_y_1 * np.sin( fl_theta ) - fl_y_2, fl_z_1 + fl_z_2 ] )
    delta_fl_nodes[2] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2, -fl_y_1 * np.sin( fl_theta ) + fl_y_2, fl_z_1 + fl_z_2 ] )
    delta_fl_nodes[3] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2, -fl_y_1 * np.sin( fl_theta ) + fl_y_2, fl_z_1 - fl_z_2 ] )
    
    # Hovering and waiting for convergence of 
    # follower desire position distributed observer
    time_stamp_begin = timeHelper.time()
    time_stamp_last = timeHelper.time()
    time_stamp_now = timeHelper.time()

    while time_stamp_now - time_stamp_begin <= total_time_wait_pos_obs:
        timeHelper.sleepForRate(10)
        leader_cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, 0.0]), yawRate = 0)

        time_stamp_now = timeHelper.time()
        delta_time = time_stamp_now - time_stamp_last
        leader_cf.current_uav_vel = leader_cf.velocity()
        leader_cf.current_uav_pos = leader_cf.position()

        for i in range(num_follower):
            cf = allcfs.crazyflies[i]
            cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, 0.0]), yawRate = 0)
            
            err_vel_obs_tilt[i] = np.zeros((3,))
            err_pos_obs_tilt[i] = np.zeros((3,))
            if wil_mat[i] != 0:
                err_vel_obs_tilt[i] += ( wil_mat[i] ) * ( vel_des_hat_fl[i] - leader_cf.current_uav_vel )
                err_pos_obs_tilt[i] += ( wil_mat[i] ) * ( pos_des_hat_fl[i] - leader_cf.current_uav_pos - delta_fl_nodes[i] )
            for j in range(num_follower):
                if wij_mat[i][j] != 0:
                    err_vel_obs_tilt[i] += ( wij_mat[i][j] ) * ( vel_des_hat_fl[i] - vel_des_hat_fl[j] )
                    err_pos_obs_tilt[i] += ( wij_mat[i][j] ) * ( ( pos_des_hat_fl[i] - delta_fl_nodes[i] ) \
                                                                - ( pos_des_hat_fl[j] - delta_fl_nodes[j] ) )
            
            sig_err_vel_obs_1[i] = cal_fracional_item(err_vel_obs_tilt[i], gamma_vel_1)
            sig_err_vel_obs_2[i] = cal_fracional_item(err_vel_obs_tilt[i], gamma_vel_2)
            sig_err_pos_obs_1[i] = cal_fracional_item(err_pos_obs_tilt[i], gamma_vel_1)
            sig_err_pos_obs_2[i] = cal_fracional_item(err_pos_obs_tilt[i], gamma_vel_2)
            
            d_vel_des_hat_fl[i] = -np.matmul(l_vel_1, sig_err_vel_obs_1[i]) \
                                    - np.matmul(l_vel_2, sig_err_vel_obs_2[i])
            d_pos_des_hat_fl[i] = vel_des_hat_fl[i] - np.matmul(l_pos_1, sig_err_pos_obs_1[i]) \
                                                    - np.matmul(l_pos_2, sig_err_pos_obs_2[i])
            vel_des_hat_fl[i] += delta_time * d_vel_des_hat_fl[i]
            pos_des_hat_fl[i] += delta_time * d_pos_des_hat_fl[i]
        
        time_stamp_last = time_stamp_now
    
    # begin formation control
    leader_cf.current_uav_pos = leader_cf.position()
    leader_cf.err_pos_now = np.array([leader_cf.init_uav_pos[0], leader_cf.init_uav_pos[1], takeoff_height[num_follower]]) - leader_cf.current_uav_pos
    leader_cf.err_pos_last = leader_cf.err_pos_now
    leader_cf.err_pos_integ = 0.0
    leader_cf.err_pos_deriv = 0.0
    leader_cf.desire_pos_tmp = np.zeros((3,))
    leader_cf.desire_vel_tmp = np.zeros((3,))
    
    time_stamp_begin = timeHelper.time()
    time_stamp_last = timeHelper.time()
    time_stamp_now = timeHelper.time()
    
    while time_stamp_now - time_stamp_begin <= ( total_time + 1E-05 ):
        timeHelper.sleepForRate(10)
        
        # leader UAV current state
        time_stamp_now = timeHelper.time()
        t_k = time_stamp_now - time_stamp_begin
        delta_time = time_stamp_now - time_stamp_last
        leader_cf.current_uav_vel = leader_cf.velocity()
        leader_cf.current_uav_pos = leader_cf.position()
        
        # leader desired trajectory and velocity
        if t_k <= ( time_seq_1 + 1E-05 ):
            leader_cf.desire_pos_tmp[0] = ( leader_cf.init_uav_pos[0] - radius_0 ) \
                                            + ( radius_0 + ( ( radius_1 - radius_0 ) / time_seq_1 ) * t_k ) \
                                            * np.cos( ( 2.0 * np.pi / time_seq_1 ) * t_k )
            leader_cf.desire_pos_tmp[1] = leader_cf.init_uav_pos[1] - ( radius_0 + ( ( radius_1 - radius_0 ) / time_seq_1 ) * t_k ) \
                                            * np.sin( ( 2.0 * np.pi / time_seq_1 ) * t_k )
            leader_cf.desire_pos_tmp[2] = takeoff_height[num_follower] + ( ( pos_z_1 - pos_z_0 ) / time_seq_1 ) * t_k
            leader_cf.desire_vel_tmp[0] = ( ( radius_1 - radius_0 ) / time_seq_1 ) * np.cos( ( 2.0 * np.pi / time_seq_1 ) * t_k ) \
                                            - ( 2.0 * np.pi / time_seq_1 ) * ( radius_0 + ( ( radius_1 - radius_0 ) / time_seq_1 ) * t_k ) \
                                            * np.sin( ( 2.0 * np.pi / time_seq_1 ) * t_k )
            leader_cf.desire_vel_tmp[1] = -( ( radius_1 - radius_0 ) / time_seq_1 ) * np.sin( ( 2.0 * np.pi / time_seq_1 ) * t_k ) \
                                            - ( 2.0 * np.pi / time_seq_1 ) * ( radius_0 + ( ( radius_1 - radius_0 ) / time_seq_1 ) * t_k ) \
                                            * np.cos( ( 2.0 * np.pi / time_seq_1 ) * t_k )
            leader_cf.desire_vel_tmp[2] = ( pos_z_1 - pos_z_0 ) / time_seq_1
        else:
            leader_cf.desire_pos_tmp[0] = leader_cf.init_uav_pos[0] - radius_0 + radius_1 + radius_x_2 \
                                            - radius_x_2 * np.cos( ( 2.0 * np.pi / time_seq_2 ) * ( t_k - time_seq_1 ) )
            leader_cf.desire_pos_tmp[1] = leader_cf.init_uav_pos[1] \
                                            - radius_y_2 * np.sin( ( 4.0 * np.pi / time_seq_2 ) * ( t_k - time_seq_1 ) )
            leader_cf.desire_pos_tmp[2] = pos_z_1 \
                                            + radius_z_2 * np.sin( ( 4.0 * np.pi / time_seq_2 ) * ( t_k - time_seq_1 ) )
            leader_cf.desire_vel_tmp[0] = ( 2.0 * np.pi / time_seq_2 ) * radius_x_2 * np.sin( ( 2.0 * np.pi / time_seq_2 ) * ( t_k - time_seq_1 ) )
            leader_cf.desire_vel_tmp[1] = -( 4.0 * np.pi / time_seq_2 ) * radius_y_2 * np.cos( ( 4.0 * np.pi / time_seq_2 ) * ( t_k - time_seq_1 ) )
            leader_cf.desire_vel_tmp[2] = ( 4.0 * np.pi / time_seq_2 ) * radius_z_2 * np.cos( ( 4.0 * np.pi / time_seq_2 ) * ( t_k - time_seq_1 ) )

        # leader UAV track desired trajectory
        leader_cf.err_pos_now = leader_cf.desire_pos_tmp - leader_cf.current_uav_pos
        leader_cf.err_pos_integ = leader_cf.err_pos_integ + leader_cf.err_pos_now * delta_time
        leader_cf.err_pos_deriv = ( leader_cf.err_pos_now - leader_cf.err_pos_last ) / delta_time
        leader_cf.cmd_vel = leader_cf.desire_vel_tmp + \
                            np.matmul(Kp_track, leader_cf.err_pos_now) + \
                            np.matmul(Ki_track, leader_cf.err_pos_integ) + \
                            np.matmul(Kd_track, leader_cf.err_pos_deriv)
        leader_cf.cmdVelocityWorld(vel = leader_cf.cmd_vel, yawRate = 0)
        
        if np.linalg.norm( leader_cf.current_uav_pos - leader_cf.desire_pos_tmp ) < 0.1:
            leader_cf.err_pos_integ = 0.0
        
        leader_cf.err_pos_last = leader_cf.err_pos_now
        
        # formation configuration
        t_12 = time_seq_1 + time_seq_1_2
        if time_stamp_now - time_stamp_begin <= ( time_seq_1 + 1E-05 ):
            delta_fl_nodes[0] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2 * np.exp( -0.02 * t_k ), fl_y_1 * np.sin( fl_theta ) - fl_y_2 * np.exp( -0.02 * t_k ), fl_z_1 - fl_z_2 * np.exp( -0.02 * t_k ) ] )
            delta_fl_nodes[1] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2 * np.exp( -0.02 * t_k ), fl_y_1 * np.sin( fl_theta ) - fl_y_2 * np.exp( -0.02 * t_k ), fl_z_1 + fl_z_2 * np.exp( -0.02 * t_k ) ] )
            delta_fl_nodes[2] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2 * np.exp( -0.02 * t_k ), -fl_y_1 * np.sin( fl_theta ) + fl_y_2 * np.exp( -0.02 * t_k ), fl_z_1 + fl_z_2 * np.exp( -0.02 * t_k ) ] )
            delta_fl_nodes[3] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2 * np.exp( -0.02 * t_k ), -fl_y_1 * np.sin( fl_theta ) + fl_y_2 * np.exp( -0.02 * t_k ), fl_z_1 - fl_z_2 * np.exp( -0.02 * t_k ) ] )
        elif time_stamp_now - time_stamp_begin <= ( t_12 + 1E-05 ):
            delta_fl_nodes[0] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2, fl_y_1 * np.sin( fl_theta ) - fl_y_2, fl_z_1 - fl_z_2 ] )
            delta_fl_nodes[1] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2, fl_y_1 * np.sin( fl_theta ) - fl_y_2, fl_z_1 + fl_z_2 ] )
            delta_fl_nodes[2] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2, -fl_y_1 * np.sin( fl_theta ) + fl_y_2, fl_z_1 + fl_z_2 ] )
            delta_fl_nodes[3] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2, -fl_y_1 * np.sin( fl_theta ) + fl_y_2, fl_z_1 - fl_z_2 ] )
        else:
            delta_fl_nodes[0] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2 + fl_x_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            fl_y_1 * np.sin( fl_theta ) - fl_y_2 + fl_y_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            fl_z_1 - fl_z_2 + fl_z_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ) ] )
            delta_fl_nodes[1] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2 - fl_x_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            fl_y_1 * np.sin( fl_theta ) - fl_y_2 + fl_y_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            fl_z_1 + fl_z_2 - fl_z_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ) ] )
            delta_fl_nodes[2] = np.array( [ -fl_x_1 * np.cos( fl_theta ) + fl_x_2 - fl_x_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            -fl_y_1 * np.sin( fl_theta ) + fl_y_2 - fl_y_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            fl_z_1 + fl_z_2 - fl_z_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ) ] )
            delta_fl_nodes[3] = np.array( [ fl_x_1 * np.cos( fl_theta ) - fl_x_2 + fl_x_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            -fl_y_1 * np.sin( fl_theta ) + fl_y_2 - fl_y_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ), \
                                            fl_z_1 - fl_z_2 + fl_z_3 / ( 1 + np.exp( -kai_3_t * ( t_k - t_12 - 5 ) ) ) ] )
        
        for i in range(num_follower):
            desire_fl_config[i] = leader_cf.current_uav_pos + delta_fl_nodes[i]

        # distributed observer and distributed controller
        for i in range(num_follower):
            cf = allcfs.crazyflies[i]
            vel_fl_nodes[i] = cf.velocity()
            pos_fl_nodes[i] = cf.position()
        for i in range(num_follower):
            # distributed follower desired state observer
            err_vel_obs_tilt[i] = np.zeros((3,))
            err_pos_obs_tilt[i] = np.zeros((3,))
            if wil_mat[i] != 0:
                err_vel_obs_tilt[i] += ( wil_mat[i] ) * ( vel_des_hat_fl[i] - leader_cf.current_uav_vel )
                err_pos_obs_tilt[i] += ( wil_mat[i] ) * ( pos_des_hat_fl[i] - leader_cf.current_uav_pos - delta_fl_nodes[i] )
            for j in range(num_follower):
                if wij_mat[i][j] != 0:
                    err_vel_obs_tilt[i] += ( wij_mat[i][j] ) * ( vel_des_hat_fl[i] - vel_des_hat_fl[j] )
                    err_pos_obs_tilt[i] += ( wij_mat[i][j] ) * ( ( pos_des_hat_fl[i] - delta_fl_nodes[i] ) \
                                                                - ( pos_des_hat_fl[j] - delta_fl_nodes[j] ) )
            
            sig_err_vel_obs_1[i] = cal_fracional_item(err_vel_obs_tilt[i], gamma_vel_1)
            sig_err_vel_obs_2[i] = cal_fracional_item(err_vel_obs_tilt[i], gamma_vel_2)
            sig_err_pos_obs_1[i] = cal_fracional_item(err_pos_obs_tilt[i], gamma_vel_1)
            sig_err_pos_obs_2[i] = cal_fracional_item(err_pos_obs_tilt[i], gamma_vel_2)
            
            d_vel_des_hat_fl[i] = -np.matmul(l_vel_1, sig_err_vel_obs_1[i]) \
                                    - np.matmul(l_vel_2, sig_err_vel_obs_2[i])
            d_pos_des_hat_fl[i] = vel_des_hat_fl[i] - np.matmul(l_pos_1, sig_err_pos_obs_1[i]) \
                                                    - np.matmul(l_pos_2, sig_err_pos_obs_2[i])
            vel_des_hat_fl[i] += delta_time * d_vel_des_hat_fl[i]
            pos_des_hat_fl[i] += delta_time * d_pos_des_hat_fl[i]

            # distributed controller
            err_vel_nodes[i] = vel_fl_nodes[i] - vel_des_hat_fl[i]
            err_pos_nodes[i] = pos_fl_nodes[i] - pos_des_hat_fl[i]
            sig_err_pos_beta_1[i] = cal_fracional_item(err_pos_nodes[i], beta_1)
            sig_err_pos_beta_2[i] = cal_fracional_item(err_pos_nodes[i], beta_2)
            chi_fl_nodes[i] = -kai_chi_1 * sig_err_pos_beta_1[i] \
                                - kai_chi_2 * sig_err_pos_beta_2[i]
            phi_fl_nodes[i] = err_vel_nodes[i] - chi_fl_nodes[i]
            d_err_pos_nodes[i] = vel_fl_nodes[i] - d_pos_des_hat_fl[i]
            d_chi_fl_nodes[i] = -kai_chi_1 * cal_abs_squ(err_pos_nodes[i], beta_1) * d_err_pos_nodes[i] \
                                -kai_chi_2 * cal_abs_squ(err_pos_nodes[i], beta_2) * d_err_pos_nodes[i]
            sig_phi_beta_1[i] = cal_fracional_item(phi_fl_nodes[i], beta_1)
            sig_phi_beta_2[i] = cal_fracional_item(phi_fl_nodes[i], beta_2)
            acc_cmd_fl_nodes[i] = d_chi_fl_nodes[i] - kai_u_1 * sig_phi_beta_1[i] - kai_u_2 * sig_phi_beta_2[i] - disturb_pos_hat[i]
            
            # begin disturbance observation
            sigma_vel_bar[i] = vel_fl_nodes[i] - sigma_vel[i]
            sigma_vel_bar_tilt[i] = sigma_vel_bar[i] - sigma_vel_bar_hat[i]
            
            # high-order nonlinear differentiator
            sigma_bar_tilt_0[i] = sigma_bar_hat_0[i] - sigma_vel_bar[i]
            sig_sigma_0 = np.sign( sigma_bar_tilt_0[i] )
            sig_sigma_1 = cal_abs_x_a(sigma_bar_tilt_0[i], zeta_1) * sig_sigma_0
            sig_sigma_2 = cal_abs_x_a(sigma_bar_tilt_0[i], 1 / 3) * sig_sigma_0
            sig_sigma_3 = cal_abs_x_a(sigma_bar_tilt_0[i], zeta_2) * sig_sigma_0
            sig_sigma_4 = cal_abs_x_a(sigma_bar_tilt_0[i], 2 / 3) * sig_sigma_0
            sig_sigma_5 = cal_abs_x_a(sigma_bar_tilt_0[i], zeta_3) * sig_sigma_0
            d_sigma_bar_hat_2[i] = -lambda_diff_sigma_bar * np.matmul( c1_diff_sigma_bar, sig_sigma_0 ) \
                                    - ( 1 - lambda_diff_sigma_bar ) * np.matmul( c1_diff_sigma_bar, sig_sigma_1 )
            d_sigma_bar_hat_1[i] = sigma_bar_hat_2[i] - lambda_diff_sigma_bar * np.matmul( c2_diff_sigma_bar, sig_sigma_2 ) \
                                    - ( 1 - lambda_diff_sigma_bar ) * np.matmul( c2_diff_sigma_bar, sig_sigma_3 )
            d_sigma_bar_hat_0[i] = sigma_bar_hat_1[i] - lambda_diff_sigma_bar * np.matmul( c3_diff_sigma_bar, sig_sigma_4 ) \
                                    - ( 1 - lambda_diff_sigma_bar ) * np.matmul( c3_diff_sigma_bar, sig_sigma_5 )
            sigma_bar_hat_2[i] += delta_time * d_sigma_bar_hat_2[i]
            sigma_bar_hat_1[i] += delta_time * d_sigma_bar_hat_1[i]
            sigma_bar_hat_0[i] += delta_time * d_sigma_bar_hat_0[i]
            
            # disturbance observer
            d_sigma_vel_bar[i] = sigma_bar_hat_1[i]
            sig_sigma_hat_1 = cal_fracional_item(sigma_vel_bar_tilt[i], alfa_vel_1)
            sig_sigma_hat_2 = cal_fracional_item(sigma_vel_bar_tilt[i], alfa_vel_2)
            d_sigma_vel_bar_hat[i] = d_sigma_vel_bar[i] + np.matmul( c_vel_1, sig_sigma_hat_1 ) \
                                                        + np.matmul( c_vel_2, sig_sigma_hat_2 )
            sigma_vel_bar_hat[i] += delta_time * d_sigma_vel_bar_hat[i]
            disturb_pos_hat[i] = d_sigma_vel_bar[i] + np.matmul( c_vel_3, sigma_vel_bar_hat[i] )
            d_sigma_vel[i] = acc_cmd_fl_nodes[i] + np.matmul( c_vel_3, sigma_vel_bar[i] )
            sigma_vel[i] += delta_time * d_sigma_vel[i]
            
            # disturbance setting
            disturb_pos[i][0] = 0.16 * np.sin( 0.4 * t_k ) + 0.2 * np.cos( 0.5 * t_k ) + 0.14 * np.sin( 0.7 * t_k ) \
                                + 0.4 * np.exp( -0.1 * t_k ) * np.sin( 0.8 * t_k ) \
                                + 0.3 / ( 1 + np.exp( -0.2 * ( t_k - 30.0 ) ) )
            disturb_pos[i][1] = 0.12 * np.sin( 0.2 * t_k ) + 0.3 * np.cos( 0.4 * t_k ) + 0.08 * np.sin( 0.6 * t_k ) \
                                + 0.2 * np.exp( -0.15 * t_k ) * np.sin( 1.2 * t_k ) \
                                + 0.3 / ( 1 + np.exp( -0.3 * ( t_k - 50.0 ) ) )
            disturb_pos[i][2] = 0.18 * np.sin( 0.2 * t_k ) + 0.16 * np.sin( 0.3 * t_k ) + 0.16 * np.cos( 0.4 * t_k ) \
                                + 0.4 * np.exp( -0.05 * t_k ) * np.sin( 0.5 * t_k + ( np.pi / 4 ) ) \
                                + 0.3 / ( 1 + np.exp( -0.5 * ( t_k - 40.0 ) ) )
            dp_aa_t = 0.02 + ( 0.1 - 0.02 ) * np.random.rand(3, 5)
            dp_ff_t = np.zeros((3, 5))
            dp_ff_t[0] = 0.04 + ( 0.1 - 0.04 ) * np.random.rand(1, 5)
            dp_ff_t[1] = 0.04 + ( 0.1 - 0.04 ) * np.random.rand(1, 5)
            dp_ff_t[2] = 0.02 + ( 0.06 - 0.02 ) * np.random.rand(1, 5)
            dp_phi_t = - ( np.pi / 2 ) + ( np.pi / 2 - ( - ( np.pi / 2 ) ) ) * np.random.rand(3, 5)
            for k in range(5):
                disturb_pos[i][0] += dp_aa_t[0][k] * np.sin( 2 * np.pi * dp_ff_t[0][k] * t_k + dp_phi_t[0][k] )
                disturb_pos[i][1] += dp_aa_t[1][k] * np.sin( 2 * np.pi * dp_ff_t[1][k] * t_k + dp_phi_t[1][k] )
                disturb_pos[i][2] += dp_aa_t[2][k] * np.sin( 2 * np.pi * dp_ff_t[2][k] * t_k + dp_phi_t[2][k] )
            
            # control input
            vel_cmd_fl_nodes[i] += delta_time * ( acc_cmd_fl_nodes[i] + disturb_pos[i] )
            cf = allcfs.crazyflies[i]
            cf.cmdVelocityWorld(vel = vel_cmd_fl_nodes[i], yawRate = 0)
        
        time_stamp_last = time_stamp_now

    # Landing
    flag_all_uav_land = False
    for cf in allcfs.crazyflies:
        cf.flag_uav_land = False
    while flag_all_uav_land == False:
        timeHelper.sleepForRate(10)
        for cf in allcfs.crazyflies:
            cf.current_uav_pos = cf.position()
            if cf.current_uav_pos[2] < 0.1:
                cf.flag_uav_land = True
                cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, 0.0]), yawRate = 0)
            else:
                cf.cmdVelocityWorld(vel = np.array([0.0, 0.0, Kpz * (land_height - cf.current_uav_pos[2])]), yawRate = 0)
        cnt_uav_land = 0
        for cf in allcfs.crazyflies:
            if cf.flag_uav_land == True:
                cnt_uav_land = cnt_uav_land + 1
        if cnt_uav_land == len(allcfs.crazyflies):
            flag_all_uav_land = True
    
    # Motor emergency stop
    for cf in allcfs.crazyflies:
        cf.cmdStop()
