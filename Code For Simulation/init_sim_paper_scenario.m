%% For leader-follower formation control in the resubmitted manuscript entitled
%% <Decentralized Formation Control for Unmanned Aerial Vehicles 
%% via Nonsingular Lie Algebra Sliding Mode and Practical Fixed-Time Observation>
%% SImulation Environment : Matlab R2020a / SImulink R2020a Ver1.1591
%% Including : 
%% 1) Practical fixed-time distributed observer for followers' desired positions/velocities
%% 2) fixed-time disturbance observer
%% 3) nonsingular lie algebra sliding mode attitude controller
%% 4) practical fixed-time distributed position controller
%% Initialization script <init_sim_paper_scenario.m>

clc;
clear all;
close all;

rng(12);

%% Directed interaction topology for <1 leader + 5 followers>

NF = 5;

Wij = [0,0,1,0,0; ...
            0,0,1,1,0; ...
            1,0,0,0,1; ...
            0,1,1,0,0; ...
            0,0,1,0,0];
Mii = diag([1;0;0;0;1]);

Lij = diag(sum(Wij,2));
Lij = Lij - Wij;
Lbarij = Lij + Mii;

%% Parameter Setting for generating Leader reference trajectory
%% Phase t = 0s - 10s : ascending while circling
%% Phase t = 10s - 20s: traversing the first half of a "8"-sharp trajectory
%% Phase t = 20s - 30s: traversing the second half of a "8"-sharp trajectory
%% 

P0 = [4; -6; 0];
R0 = 4;
R1 = 8;
PZ0 = P0(3);
PZ1 = 10;

Rx2 = 10;
Ry2 = 6;
Rz2 = 4;


tseq1 = 10;
tseq12 = 10;
tseq2 = 10;

T0 = 0;
T1 = tseq1;
T12 = tseq1 + tseq12;
T2 = tseq1 + tseq12 + tseq2;
dt = 0.01;

acc0 = zeros(3, 1);
vel0 = zeros(3, 1);
pos0 = zeros(3, 1);

acc0(1, 1) = - ( 4 * pi * sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( ( T0 - T1 ) ^ 2 ) ...
                        - ( 4 * ( pi ^ 2 ) * cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( ( T0 - T1 ) ^ 2 );
acc0(2, 1) = ( 4 * pi * cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( ( T0 - T1 ) ^ 2 ) ...
                        - ( 4 * ( pi ^ 2 ) * sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( ( T0 - T1 ) ^ 2 );
acc0(3, 1) = 0;

vel0(1, 1) = ( cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( T0 - T1 ) ...
                        - ( 2 * pi * sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( T0 - T1 );
vel0(2, 1) = ( sin( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 - R1 ) ) / ( T0 - T1 ) ...
                        + ( 2 * pi * cos( ( 2 * pi * ( T0 - T0 ) ) / ( T0 - T1 ) ) * ( R0 + ( ( R0 - R1 ) * ( T0 - T0 ) ) / ( T0 - T1 ) ) ) / ( T0 - T1 );
vel0(3, 1) = ( P0(3) - PZ1 ) / ( T0 - T1 );

pos0(1, 1) = ( P0(1) - R0 ) + ( R0 + ( ( R1 - R0 ) / ( T1 - T0 ) ) * ( T0 - T0 ) ) * cos( ( 2 * pi / ( T1 - T0 ) ) * ( T0 - T0 ) );
pos0(2, 1) = P0(2) - ( R0 + ( ( R1 - R0 ) / ( T1 - T0 ) ) * ( T0 - T0 ) )  * sin( ( 2 * pi / ( T1 - T0 ) ) * ( T0 - T0 ) );
pos0(3, 1) = P0(3) + ( ( ( PZ1 - P0(3) ) / ( T1 - T0 ) ) * ( T0 - T0 ) );


%% Parameter setting for followers' ideal formation configuration

X1 = 8;
X2 = 2;
X3 = 3;
Y1 = 10;
Y2 = 2;
Y3 = 4;
Z1 = 4;
Z2 = -6;
Z3 = 2;
theta0 = ( 2 / 5 ) * pi;
kai_1_t = 0.02;
kai_3_t = 0.1;

delta_v_i_0 = zeros(3, NF);
delta_v_i_0(:,1) = kai_1_t * [ X2; Y2; Z2 ];
delta_v_i_0(:,2) = kai_1_t * [ -X2; Y2; -Z2 ];
delta_v_i_0(:,3) = kai_1_t * [ X2; -Y2; -Z2 ];
delta_v_i_0(:,4) = kai_1_t * [ -X2; -Y2; -Z2 ];
delta_v_i_0(:,5) = kai_1_t * [ X2; -Y2; Z2 ];

delta_p_i_0 = zeros(3, NF);
delta_p_i_0(:,1) = [ X1 * cos( theta0 ) - X2; Y1 * sin( theta0 ) - Y2; Z1 - Z2 ];
delta_p_i_0(:,2) = [ -X1 * cos( theta0 ) + X2; Y1 * sin( theta0 ) - Y2; Z1 + Z2 ];
delta_p_i_0(:,3) = [ X1 - X2; Y2; Z1 * ( 3 / 2 ) - Z2 ];
delta_p_i_0(:,4) = [ -X1 * cos( theta0 ) + X2; -Y1 * sin( theta0 ) + Y2; Z1 + Z2 ];
delta_p_i_0(:,5) = [ X1 * cos( theta0 ) - X2; -Y1 * sin( theta0 ) + Y2; Z1 - Z2 ];


%% Initial position setting for followers
ksii_0 = zeros(3,5);
etai_0 = zeros(3,5);

ksii_0(:,1) = [-15; -15; -1];
ksii_0(:,2) = [-10; -5; 1];
ksii_0(:,3) = [-5; -5; -1];
ksii_0(:,4) = [-15; -5; 1];
ksii_0(:,5) = [-10; -15; -1];

etai_0(:,1) = [2; 2; 1];
etai_0(:,2) = [2; 2; 1];
etai_0(:,3) = [2; 2; 1];
etai_0(:,4) = [2; 2; 1];
etai_0(:,5) = [2; 2; 1];

%% Parameter setting for fixed-time distributed observer

gamma1_pos = 1.2;
gamma2_pos = 0.8;
gamma1_vel = 1.2;
gamma2_vel = 0.8;

[p_gamma1_pos, q_gamma1_pos] = rat(gamma1_pos);
[p_gamma2_pos, q_gamma2_pos] = rat(gamma2_pos);
[p_gamma1_vel, q_gamma1_vel] = rat(gamma1_vel);
[p_gamma2_vel, q_gamma2_vel] = rat(gamma2_vel);


alfa_theta_obs = 100;
l1p = diag([20; 20; 20]);
l2p = diag([50; 50; 50]);
l1v = diag([20; 20; 20]);
l2v = diag([30; 30; 30]);

eta_est_err_init = zeros(3, NF) - repmat(vel0(:), 1, NF) - delta_v_i_0;
kei_est_err_init = zeros(3, NF) - repmat(pos0(:), 1, NF) - delta_p_i_0;

eta_est_err_tilt_init = zeros(3, NF);
ksi_est_err_tilt_init = zeros(3, NF);

for iiuav = 1:5
    eta_est_err_tilt_init(:,iiuav) = Mii(iiuav, iiuav) * (zeros(3,1) - vel0(:) - delta_v_i_0(:,iiuav));
    ksi_est_err_tilt_init(:,iiuav) = Mii(iiuav, iiuav) * (zeros(3,1) - pos0(:) - delta_p_i_0(:,iiuav));
    for jjuav = 1:5
        eta_est_err_tilt_init(:,iiuav) = eta_est_err_tilt_init(:,iiuav) + ...
                                                            Wij(iiuav, jjuav) * ( (zeros(3,1) - delta_v_i_0(:,iiuav)) - (zeros(3,1) - delta_v_i_0(:,jjuav)) );
        ksi_est_err_tilt_init(:,iiuav) = ksi_est_err_tilt_init(:,iiuav) + ...
                                                            Wij(iiuav, jjuav) * ( (zeros(3,1) - delta_p_i_0(:,iiuav)) - (zeros(3,1) - delta_p_i_0(:,jjuav)) );
    end
end

%% Parameter setting for attitude controller
cS0 = diag( [4.5; 4.5; 4.5] );

pp0 = 8 / 11;
gamma0 = 20;
eps_s_bar = 0.03 * pi;
eps_s_bar_div_pi = eps_s_bar / pi;
k_J_0 = diag( [1; 1; 1] );

c_w_1 = diag( [20; 15; 15] );
c_w_2 = diag( [20; 15; 15] );
c_w_3 = diag( [2; 2; 2] );

beta_w_1 = 1.2;
beta_w_2 = 0.2;
mu_c_theta = 100;


[p_p0_0, q_p0_0] = rat( pp0 );
[p_2_p0, q_2_p0] = rat( 2 * pp0 );
[p_p0_plus_1, q_p0_plus_1] = rat( pp0 + 1 );
[p_p0_min_1, q_p0_min_1] = rat( pp0 - 1 );
[p_p0_min_2, q_p0_min_2] = rat( pp0 - 2 );
[p_1_min_p0, q_1_min_p0] = rat( 1 - pp0 );

[p_beta_w_1, q_beta_w_1] = rat( beta_w_1 );
[p_beta_w_2, q_beta_w_2] = rat( beta_w_2 );

thr_p_plus_1 = cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 );

beta_Phi_1 = pp0 * cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) ...
                        * cal_frac_squ( eps_s_bar / pi, p_p0_min_2, q_p0_min_2 ) ...
                        / ( 2 * ( 2 * pp0 - 1 ) * tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ...
                        - ( cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) * gamma0 / ( 4 * ( 2 * pp0 -1 ) ) ) ...
                        * cal_frac_squ( eps_s_bar / pi, p_p0_min_1, q_p0_min_1 ) ...
                        * ( ( sech( gamma0 * ( eps_s_bar / pi ) / 2 ) / tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ^ 2 );

beta_Phi_2 = ( 1 - pp0 ) * cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) ...
                        / ( 2 * ( 1 - 2 * pp0 ) * cal_frac_squ( eps_s_bar / pi, p_p0_0, q_p0_0 ) * tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ...
                        - ( cal_frac_squ( 3, p_p0_plus_1, q_p0_plus_1 ) * gamma0 / ( 4 * ( 1 - 2 * pp0 ) ) ) ...
                        * cal_frac_squ( eps_s_bar / pi, p_1_min_p0, q_1_min_p0 ) ...
                        * ( ( sech( gamma0 * ( eps_s_bar / pi ) / 2 ) / tanh( gamma0 * ( eps_s_bar / pi ) / 2 ) ) ^ 2 );

%% Parameter setting for higher-order sliding mode differentiator used in 
%% solving <auxiliary variables in translational disturbance observation>

zeta_diff_sigma_bar_v = 0.08;
Lamd_diff_sigma_bar_v = 0.5;
c1_diff_sigma_bar_v = diag([25; 30; 20]);
c2_diff_sigma_bar_v = diag([50; 50; 10]);
c3_diff_sigma_bar_v = diag([15; 15; 10]);

%% Parameter setting for higher-order sliding mode differentiator used in 
%% solving <auxiliary variables in rotational disturbance observation>

zeta_diff_sigma_bar_w = 0.3;
Lamd_diff_sigma_bar_w = 0.8;
c1_diff_sigma_bar_w = diag([12; 12; 12]);
c2_diff_sigma_bar_w = diag([12; 12; 12]);
c3_diff_sigma_bar_w = diag([15; 15; 15]);

%% Parameter setting for disturbance in rotational subsystem

dw_aa_t = 0.2 * ones(3, 5) + ( 1.0 - 0.2 ) * rand(3, 5);
dw_ff_t = [ 0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.2 * ones(1, 5) + ( 0.6 - 0.2 ) * rand(1, 5) ];
dw_phi_t = ( - pi / 2 ) * ones(3, 5) + ( pi / 2 - ( - pi / 2 ) ) * rand(3, 5);

%% Parameter setting for rotational disturbance observer

h1w = diag([20.0; 20.0; 20.0]);
h2w = diag([30.0; 35.0; 35.0]);
h3w = diag([30.0; 25.0; 30.0]);

mu_d_w = 100;
alfa_w_1 = 1.2;
alfa_w_2 = 0.4;

[p_alfaw1, q_alfaw1] = rat(alfa_w_1);
[p_alfaw2, q_alfaw2] = rat(alfa_w_2);

%% Parameter setting for translational distributed controller

alfa_theta = 20;

beta1 = 1.2;
beta2 = 0.8;
[p_beta1, q_beta1] = rat(beta1);
[p_beta2, q_beta2] = rat(beta2);

dbeta_1 = beta1 - 1;
dbeta_2 = beta2 - 1;
[p_dbeta1, q_dbeta1] = rat(dbeta_1);
[p_dbeta2, q_dbeta2] = rat(dbeta_2);

kai_chi_1 = diag([6; 6; 6]);
kai_chi_2 = diag([6; 6; 6]);
kai_u_1 = diag([8; 8; 8]);
kai_u_2 = diag([8; 8; 8]);

%% Parameter setting for disturbance in translational subsystem

dp_aa_t = 0.2 * ones(3, 5) + ( 1.0 - 0.2 ) * rand(3, 5);
dp_ff_t = [ 0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.4 * ones(1, 5) + ( 1.0 - 0.4 ) * rand(1, 5); ...
                    0.2 * ones(1, 5) + ( 0.6 - 0.2 ) * rand(1, 5) ];
dp_phi_t = ( - pi / 2 ) * ones(3, 5) + ( pi / 2 - ( - pi / 2 ) ) * rand(3, 5);


%% Parameter setting for translational disturbance observer

c3v = diag([4.8; 6.4; 6.0]);
c1v = diag([5.8; 6.5; 6.5]);
c2v = diag([5.8; 6.5; 6.5]);

alfa_v_1 = 1.1;
alfa_v_2 = 0.2;
[p_alfav1, q_alfav1] = rat(alfa_v_1);
[p_alfav2, q_alfav2] = rat(alfa_v_2);

%% Parameters setting for higher-order sliding mode differentiator used to 
%% obtain the first/second derivatives of control input from the control input

zeta_diff_ui = 0.05;
Lamd_diff_ui = 0.8;
c1_diff_ui = diag([25; 30; 20]);
c2_diff_ui = diag([50; 50; 10]);
c3_diff_ui = diag([15; 15; 10]);

%% Initialize system states

%%%% Mass of UAV
mass_i = zeros(1,NF);
mass_i(1,1) = 0.35;
mass_i(1,2) = 0.35;
mass_i(1,3) = 0.35;
mass_i(1,4) = 0.35;
mass_i(1,5) = 0.35;

%%%% Inertial Matrix
Ji = zeros(3,3,NF);

Ji(:,:,1) = [20, 2, 0.9; 2, 17, 0.5; 0.9, 0.5, 15] * 1e-00;
Ji(:,:,2) = [22, 1, 0.9; 1, 19, 0.5; 0.9, 0.5, 15] * 1e-00;
Ji(:,:,3) = [18, 1, 1.5; 1, 15, 0.5; 1.5, 0.5, 17] * 1e-00;
Ji(:,:,4) = [18, 1,    1; 1, 20, 0.5;    1, 0.5, 15] * 1e-00;
Ji(:,:,5) = [18, 1,    1; 1, 20, 0.5;    1, 0.5, 15] * 1e-00;


%%%% Initialize quaternion and angular velocity in rotational subsystem
Qi_init(:,1) = [0.9110; 0.3; -0.2; 0.2];
Qi_init(:,2) = [0.9274; -0.1; 0.2; 0.3];
Qi_init(:,3) = [0.8185; 0.1; -0.4; 0.4];
Qi_init(:,4) = [0.8185; -0.4; -0.1; 0.4];
Qi_init(:,5) = [0.9274; 0.2; -0.1; 0.3];
wi_init(:,1) = [-0.5;0.5;-0.45];
wi_init(:,2) = [0.5;-0.3;0.1];
wi_init(:,3) = [0.1;0.6;-0.1];
wi_init(:,4) = [0.4;0.4;-0.5];
wi_init(:,5) = [0.4;-0.4;0.5];


%%%% Initial translational control input
g0 = 9.80663;
e3 = [0;0;1];
dv0_init = [acc0(1); acc0(2); acc0(3)];
ui_init = zeros(3,NF);
for iiNF = 1:NF
    ui_init(:,iiNF) = -g0 * e3 + dv0_init(:);
end

%%%% Initialize quaternion command
Qci_init = repmat([1;0;0;0],1,NF);

%%%% Initialize rotational matrix command
Ri_init = zeros(3,3,NF);
Rci_init = zeros(3,3,NF);

for iuav = 1:NF
    Ri_init(:, :, iuav) = cal_R_with_Q(Qi_init(:,iuav));
    Rci_init(:, :, iuav) = cal_R_with_Q(Qci_init(:,iuav));
end

%%%% Initialize angular velocity command
wci_init = zeros(3,5);
dwci_init = zeros(3,5);
for iuav = 1:NF
    [wci_init(:,iuav), dwci_init(:,iuav)] = cal_angrate_wc_wcdot(Rci_init(:,:,iuav), ui_init(:,iuav), zeros(3,1), zeros(3,1));
end

%%%% Initialize rotational error ( quaternion / rotational matrix / angular velocity / rotational error in exponential coordinate )
for iuav = 1:NF
    Qei_init(:,iuav) = cal_Q_mul_Q(cal_inv_Q(Qci_init(:,iuav)), Qi_init(:,iuav));
    Rei_init(:,:,iuav) = cal_R_with_Q(Qei_init(:,iuav));
    wei_init(:,iuav) = wi_init(:,iuav) - (Rei_init(:,:,iuav)') * wci_init(:,iuav);
    V_Phi_Rei_init(:,iuav) = cal_LOG_coord(Rei_init(:,:,iuav));
end

%% 
