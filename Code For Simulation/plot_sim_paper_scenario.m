%% run <init_sim_paper_scenario.m> first
%% then run <run_sim_paper_scenario.slx> second
%% then run <plot_sim_paper_scenario.m> last

clc;
close all;

linestyle_vec = {'--', '-.', ':'};

color_vec = [252, 170, 103; ...
                        189, 30, 30; ...
                        124, 187, 0; ...
                        54, 195, 201; ...
                        0, 70, 222; ...
                        0, 0, 0] ./ 255;

%% Extract simulation data after <run_sim_paper_scenario.slx> already running

NF = 5;
tLen = length(tout);

%%%% translational formation control error

%%%% follower desired state observation error
ksi_obs_err_x = zeros(NF,tLen);
ksi_obs_err_y = zeros(NF,tLen);
ksi_obs_err_z = zeros(NF,tLen);
eta_obs_err_x = zeros(NF,tLen);
eta_obs_err_y = zeros(NF,tLen);
eta_obs_err_z = zeros(NF,tLen);

%%%% follower position
ksi_fl_x = zeros(NF,tLen);
ksi_fl_y = zeros(NF,tLen);
ksi_fl_z = zeros(NF,tLen);

%%%% follower velocity
eta_fl_x = zeros(NF,tLen);
eta_fl_y = zeros(NF,tLen);
eta_fl_z = zeros(NF,tLen);

%%%% follower position tracking error
ksi_err_fl_x = zeros(NF,tLen);
ksi_err_fl_y = zeros(NF,tLen);
ksi_err_fl_z = zeros(NF,tLen);

%%%% follower velocity tracking error
eta_err_fl_x = zeros(NF,tLen);
eta_err_fl_y = zeros(NF,tLen);
eta_err_fl_z = zeros(NF,tLen);

%%%% virtual velocity control error
phi_fl_x = zeros(NF,tLen);
phi_fl_y = zeros(NF,tLen);
phi_fl_z = zeros(NF,tLen);

%%%% virtual velocity tracking variable
sigma_v_x = zeros(NF,tLen);
sigma_v_y = zeros(NF,tLen);
sigma_v_z = zeros(NF,tLen);

%%%% auxiliary virtual velocity tracking variable
sigma_bar_v_x = zeros(NF,tLen);
sigma_bar_v_y = zeros(NF,tLen);
sigma_bar_v_z = zeros(NF,tLen);

%%%% auxiliary virtual velocity observation
sigma_bar_v_hat_x = zeros(NF,tLen);
sigma_bar_v_hat_y = zeros(NF,tLen);
sigma_bar_v_hat_z = zeros(NF,tLen);

%%%% auxiliary virtual velocity observation error
sigma_bar_v_tilt_x = zeros(NF,tLen);
sigma_bar_v_tilt_y = zeros(NF,tLen);
sigma_bar_v_tilt_z = zeros(NF,tLen);

%%%% translational disturbance
dp_x = zeros(NF,tLen);
dp_y = zeros(NF,tLen);
dp_z = zeros(NF,tLen);

%%%% translational disturbance observation
dp_hat_x = zeros(NF,tLen);
dp_hat_y = zeros(NF,tLen);
dp_hat_z = zeros(NF,tLen);

%%%% translational disturbance observation error
dp_tilt_x = zeros(NF,tLen);
dp_tilt_y = zeros(NF,tLen);
dp_tilt_z = zeros(NF,tLen);


%%%% rotational control error

%%%% angular velocity error
wei_x = zeros(NF,tLen);
wei_y = zeros(NF,tLen);
wei_z = zeros(NF,tLen);

%%%% angular velocity
wi_x = zeros(NF,tLen);
wi_y = zeros(NF,tLen);
wi_z = zeros(NF,tLen);

%%%% rotational error in Lie Algebra
V_Phi_Rei_x = zeros(NF,tLen);
V_Phi_Rei_y = zeros(NF,tLen);
V_Phi_Rei_z = zeros(NF,tLen);

%%%% intermediate auxiliary rotational error
Phi_psi_e_x = zeros(NF,tLen);
Phi_psi_e_y = zeros(NF,tLen);
Phi_psi_e_z = zeros(NF,tLen);

%%%% derivative of intermediate auxiliary rotational error
Phi_bar_psi_e_x = zeros(NF,tLen);
Phi_bar_psi_e_y = zeros(NF,tLen);
Phi_bar_psi_e_z = zeros(NF,tLen);

%%%% auxiliary sliding mode surface
S_bar_x = zeros(NF,tLen);
S_bar_y = zeros(NF,tLen);
S_bar_z = zeros(NF,tLen);

%%%% sliding mode surface
Si_x = zeros(NF,tLen);
Si_y = zeros(NF,tLen);
Si_z = zeros(NF,tLen);

%%%% virtual angular velocity control variable
sigma_w_x = zeros(NF,tLen);
sigma_w_y = zeros(NF,tLen);
sigma_w_z = zeros(NF,tLen);

%%%% auxiliary virtual angular velocity control variable
sigma_bar_w_x = zeros(NF,tLen);
sigma_bar_w_y = zeros(NF,tLen);
sigma_bar_w_z = zeros(NF,tLen);

%%%% auxiliary virtual angular velocity control observation
sigma_bar_w_hat_x = zeros(NF,tLen);
sigma_bar_w_hat_y = zeros(NF,tLen);
sigma_bar_w_hat_z = zeros(NF,tLen);

%%%% rotational disturbance
dwi_x = zeros(NF,tLen);
dwi_y = zeros(NF,tLen);
dwi_z = zeros(NF,tLen);

%%%% rotational disturbance observation
dwi_hat_x = zeros(NF,tLen);
dwi_hat_y = zeros(NF,tLen);
dwi_hat_z = zeros(NF,tLen);

%%%% rotational disturbance observation error
dwi_tilt_x = zeros(NF,tLen);
dwi_tilt_y = zeros(NF,tLen);
dwi_tilt_z = zeros(NF,tLen);

if size(tout,2) == 1
    tout = tout';
    u0 = u0';
    eta0 = eta0';
    ksi0 = ksi0';
end

for tt = 1:tLen
    for iiFF = 1:NF
        %%%% translational control result
        ksi_obs_err_x(iiFF,tt) = ksi_est_err(1,iiFF,tt);
        ksi_obs_err_y(iiFF,tt) = ksi_est_err(2,iiFF,tt);
        ksi_obs_err_z(iiFF,tt) = ksi_est_err(3,iiFF,tt);
        eta_obs_err_x(iiFF,tt) = eta_est_err(1,iiFF,tt);
        eta_obs_err_y(iiFF,tt) = eta_est_err(2,iiFF,tt);
        eta_obs_err_z(iiFF,tt) = eta_est_err(3,iiFF,tt);
        ksi_fl_x(iiFF,tt) = ksi_i_vec(1,iiFF,tt);
        ksi_fl_y(iiFF,tt) = ksi_i_vec(2,iiFF,tt);
        ksi_fl_z(iiFF,tt) = ksi_i_vec(3,iiFF,tt);
        eta_fl_x(iiFF,tt) = eta_i_vec(1,iiFF,tt);
        eta_fl_y(iiFF,tt) = eta_i_vec(2,iiFF,tt);
        eta_fl_z(iiFF,tt) = eta_i_vec(3,iiFF,tt);
        ksi_err_fl_x(iiFF,tt) = ksi_track_err_vec(1,iiFF,tt);
        ksi_err_fl_y(iiFF,tt) = ksi_track_err_vec(2,iiFF,tt);
        ksi_err_fl_z(iiFF,tt) = ksi_track_err_vec(3,iiFF,tt);
        eta_err_fl_x(iiFF,tt) = eta_track_err_vec(1,iiFF,tt);
        eta_err_fl_y(iiFF,tt) = eta_track_err_vec(2,iiFF,tt);
        eta_err_fl_z(iiFF,tt) = eta_track_err_vec(3,iiFF,tt);
        phi_fl_x(iiFF,tt) = phi_vec(1,iiFF,tt);
        phi_fl_y(iiFF,tt) = phi_vec(2,iiFF,tt);
        phi_fl_z(iiFF,tt) = phi_vec(3,iiFF,tt);
        sigma_v_x(iiFF,tt) = sigma_v_vec(1,iiFF,tt);
        sigma_v_y(iiFF,tt) = sigma_v_vec(2,iiFF,tt);
        sigma_v_z(iiFF,tt) = sigma_v_vec(3,iiFF,tt);
        sigma_bar_v_x(iiFF,tt) = sigma_bar_v_vec(1,iiFF,tt);
        sigma_bar_v_y(iiFF,tt) = sigma_bar_v_vec(2,iiFF,tt);
        sigma_bar_v_z(iiFF,tt) = sigma_bar_v_vec(3,iiFF,tt);
        sigma_bar_v_hat_x(iiFF,tt) = sigma_bar_v_hat_vec(1,iiFF,tt);
        sigma_bar_v_hat_y(iiFF,tt) = sigma_bar_v_hat_vec(2,iiFF,tt);
        sigma_bar_v_hat_z(iiFF,tt) = sigma_bar_v_hat_vec(3,iiFF,tt);
        sigma_bar_v_tilt_x(iiFF,tt) = sigma_bar_v_tilt_vec(1,iiFF,tt);
        sigma_bar_v_tilt_y(iiFF,tt) = sigma_bar_v_tilt_vec(2,iiFF,tt);
        sigma_bar_v_tilt_z(iiFF,tt) = sigma_bar_v_tilt_vec(3,iiFF,tt);
        dp_x(iiFF,tt) = dp_vec(1,iiFF,tt);
        dp_y(iiFF,tt) = dp_vec(2,iiFF,tt);
        dp_z(iiFF,tt) = dp_vec(3,iiFF,tt);
        dp_hat_x(iiFF,tt) = dp_hat_vec(1,iiFF,tt);
        dp_hat_y(iiFF,tt) = dp_hat_vec(2,iiFF,tt);
        dp_hat_z(iiFF,tt) = dp_hat_vec(3,iiFF,tt);
        dp_tilt_x(iiFF,tt) = dp_tilt_vec(1,iiFF,tt);
        dp_tilt_y(iiFF,tt) = dp_tilt_vec(2,iiFF,tt);
        dp_tilt_z(iiFF,tt) = dp_tilt_vec(3,iiFF,tt);
        
        %%%% rotational control result
        wei_x(iiFF,tt) = wei(1,iiFF,tt);
        wei_y(iiFF,tt) = wei(2,iiFF,tt);
        wei_z(iiFF,tt) = wei(3,iiFF,tt);
        wi_x(iiFF,tt) = wi(1,iiFF,tt);
        wi_y(iiFF,tt) = wi(2,iiFF,tt);
        wi_z(iiFF,tt) = wi(3,iiFF,tt);
        V_Phi_Rei_x(iiFF,tt) = V_Phi_Rei(1,iiFF,tt);
        V_Phi_Rei_y(iiFF,tt) = V_Phi_Rei(2,iiFF,tt);
        V_Phi_Rei_z(iiFF,tt) = V_Phi_Rei(3,iiFF,tt);
        Phi_psi_e_x(iiFF,tt) = Phi_psi_e(1,iiFF,tt);
        Phi_psi_e_y(iiFF,tt) = Phi_psi_e(2,iiFF,tt);
        Phi_psi_e_z(iiFF,tt) = Phi_psi_e(3,iiFF,tt);
        Phi_bar_psi_e_x(iiFF,tt) = Phi_bar_psi_e(1,iiFF,tt);
        Phi_bar_psi_e_y(iiFF,tt) = Phi_bar_psi_e(2,iiFF,tt);
        Phi_bar_psi_e_z(iiFF,tt) = Phi_bar_psi_e(3,iiFF,tt);
        S_bar_x(iiFF,tt) = S_bar(1,iiFF,tt);
        S_bar_y(iiFF,tt) = S_bar(2,iiFF,tt);
        S_bar_z(iiFF,tt) = S_bar(3,iiFF,tt);
        Si_x(iiFF,tt) = Si(1,iiFF,tt);
        Si_y(iiFF,tt) = Si(2,iiFF,tt);
        Si_z(iiFF,tt) = Si(3,iiFF,tt);
        sigma_w_x(iiFF,tt) = sigma_w(1,iiFF,tt);
        sigma_w_y(iiFF,tt) = sigma_w(2,iiFF,tt);
        sigma_w_z(iiFF,tt) = sigma_w(3,iiFF,tt);
        sigma_bar_w_x(iiFF,tt) = sigma_bar_w(1,iiFF,tt);
        sigma_bar_w_y(iiFF,tt) = sigma_bar_w(2,iiFF,tt);
        sigma_bar_w_z(iiFF,tt) = sigma_bar_w(3,iiFF,tt);
        sigma_bar_w_hat_x(iiFF,tt) = sigma_bar_w_hat(1,iiFF,tt);
        sigma_bar_w_hat_y(iiFF,tt) = sigma_bar_w_hat(2,iiFF,tt);
        sigma_bar_w_hat_z(iiFF,tt) = sigma_bar_w_hat(3,iiFF,tt);
        dwi_x(iiFF,tt) = dwi(1,iiFF,tt);
        dwi_y(iiFF,tt) = dwi(2,iiFF,tt);
        dwi_z(iiFF,tt) = dwi(3,iiFF,tt);
        dwi_hat_x(iiFF,tt) = dwi_hat(1,iiFF,tt);
        dwi_hat_y(iiFF,tt) = dwi_hat(2,iiFF,tt);
        dwi_hat_z(iiFF,tt) = dwi_hat(3,iiFF,tt);
        dwi_tilt_x(iiFF,tt) = dwi_tilt(1,iiFF,tt);
        dwi_tilt_y(iiFF,tt) = dwi_tilt(2,iiFF,tt);
        dwi_tilt_z(iiFF,tt) = dwi_tilt(3,iiFF,tt);
    end
end

t0 = 0;
t1 = 10;
t12 = 20;
t2 = 30;
del_t = 0.1;

%% follower desired position distributed observation error
figure(1)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_obs_err_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,p}^{d,x} \rm (m)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');


subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_obs_err_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,p}^{d,y} \rm (m)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_obs_err_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
set(gca, 'ylim', [-0.2, 0.2]);
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,p}^{d,z} \rm (m)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

ax1_1 = axes('Position', [0.2, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ksi_obs_err_x(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.45, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.2));
[~, id_end] = min(abs(tout - 20.8));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%%%% 子图3/4 -- 位置Y估计误差局部放大图
ax2_1 = axes('Position', [0.2, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ksi_obs_err_y(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.45, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.5, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.2));
[~, id_end] = min(abs(tout - 20.8));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.2, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.45, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.2));
[~, id_end] = min(abs(tout - 20.8));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% follower desired velocity distributed observation error
figure(2)
subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_obs_err_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,v}^{d,x} \rm (m/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_obs_err_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,v}^{d,y} \rm (m/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_obs_err_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,v}^{d,z} \rm (m/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

ax1_1 = axes('Position', [0.2, 0.87, 0.15, 0.05]);
[~, id_start] = min(abs(tout - 0.05));
[~, id_end] = min(abs(tout - 0.2));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), eta_obs_err_x(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.45, 0.87, 0.15, 0.05]);
[~, id_start] = min(abs(tout - 10.1));
[~, id_end] = min(abs(tout - 10.4));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.87, 0.15, 0.05]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.2));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.2, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), eta_obs_err_y(iiFF,id_start:id_end), 'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.45, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.1));
[~, id_end] = min(abs(tout - 10.4));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.6));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.2, 0.18, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.1));
[~, id_end] = min(abs(tout - 0.3));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.45, 0.18, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.1));
[~, id_end] = min(abs(tout - 10.4));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 20.6));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_obs_err_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% three-dimensional trajectories

k_0_1 = 1:fix( length( T0:dt:T1 ) / 5 ):length( T0:dt:T1 );
k_1_12 = length( T0:dt:(T1+dt) ) + ( 0:fix( length( T1:dt:T12 ) / 5 ):length( T1:dt:T12 ) );
k_12_2 = length( T0:dt:(T12+dt) ) + ( 0:fix( length( T12:dt:T2 ) / 5 ):length( T12:dt:T2 ) );

k_arr = [ k_0_1( 2:2:( length( k_0_1 ) - 1 ) ), fix( k_0_1(end) * ( 4/5 ) ), ...
                    k_1_12( 2:2:( length( k_1_12 ) - 1 ) ), k_12_2( 2:2:( length( k_12_2 ) - 1 ) ) ];

color_arr = { 'r', 'g', 'b', 'c', 'y' };

figure(3)
pp0 = plot3(ksi0(1,:), ksi0(2,:), ksi0(3,:), 'color', 'k', ...
                        'linestyle', '-', 'linewidth', 1.0);
hold on;
plot3(ksi0(1,1), ksi0(2,1), ksi0(3,1), 'color', 'k', 'marker', 'p', ...
            'markeredgecolor', 'k', 'markerfacecolor', 'k', 'linewidth', 1.0);
hold on;
plot3(ksi0(1,end), ksi0(2,end), ksi0(3,end), 'color', 'k', 'marker', 'h', ...
            'markeredgecolor', 'k', 'markerfacecolor', 'k', 'linewidth', 1.0);
hold on;

Nl = size(ksi0,2);
indices = round(linspace(2, Nl-1, 10));

for i = 1:length(indices)
    idl = indices(i);
    if idl > 1 && idl < Nl
        dx = ksi0(1, idl+1) - ksi0(1, idl-1);
        dy = ksi0(2, idl+1) - ksi0(2, idl-1);
        dz = ksi0(3, idl+1) - ksi0(3, idl-1);
    elseif idx == 1
        dx = ksi0(1, 2) - ksi0(1, 1);
        dy = ksi0(2, 2) - ksi0(2, 1);
        dz = ksi0(3, 2) - ksi0(3, 1);
    else % idx == N
        dx = ksi0(1, Nl) - ksi0(1, Nl-1);
        dy = ksi0(2, Nl) - ksi0(2, Nl-1);
        dz = ksi0(3, Nl) - ksi0(3, Nl-1);
    end
    vec = [dx; dy; dz];
    vec_len = norm(vec);
    if vec_len > 0
        vec = vec / vec_len;
        data_range = max(ksi0(1,:))-min(ksi0(1,:)) + max(ksi0(2,:))-min(ksi0(2,:)) + max(ksi0(3,:))-min(ksi0(3,:));
        arrow_len = data_range * 0.08;
        vec = vec * arrow_len;
        quiver3(ksi0(1,idl), ksi0(2,idl), ksi0(3,idl), vec(1), vec(2), vec(3), ...
                        'color', 'k', 'linewidth', 1.5, 'MaxHeadSize', 2.0);
    end
end
hold on;

for iiFF = 1:5
    plot3( ksi_fl_x(iiFF, :), ksi_fl_y(iiFF, :), ksi_fl_z(iiFF, :), ...
                'color', color_arr{iiFF}, 'linestyle', '-', 'linewidth', 1.0 );
    hold on;
    plot3( ksi_fl_x(iiFF, 1), ksi_fl_y(iiFF, 1), ksi_fl_z(iiFF, 1), 'color', color_arr{iiFF}, ...
                'marker', 'p', 'linewidth', 1.0, 'markeredgecolor', color_arr{iiFF}, 'markerfacecolor', color_arr{iiFF} );
    hold on;
    plot3( ksi_fl_x(iiFF, end), ksi_fl_y(iiFF, end), ksi_fl_z(iiFF, end), 'color', color_arr{iiFF}, ...
                'marker', 'h', 'linewidth', 1.0, 'markeredgecolor', color_arr{iiFF}, 'markerfacecolor', color_arr{iiFF} );
    hold on;
end

for kkt = k_arr
    pp0 = plot3( ksi0(1, kkt), ksi0(2, kkt), ksi0(3, kkt), ...
                            'color', 'k', 'marker', 'o', 'markersize', 5, ...
                            'markerfacecolor', 'k', 'markeredgecolor', 'k' );
    for iiFF = 1:5
        ppFk(iiFF) = plot3( ksi_fl_x(iiFF, kkt), ksi_fl_y(iiFF, kkt), ksi_fl_z(iiFF, kkt), ...
                                            'color', color_arr{iiFF}, 'marker', 'o', 'markersize', 5, ...
                                            'markerfacecolor', color_arr{iiFF}, 'markeredgecolor', color_arr{iiFF} );
        hold on;
        plot3( [ ksi0(1, kkt), ksi_fl_x(iiFF, kkt) ], ...
                    [ ksi0(2, kkt), ksi_fl_y(iiFF, kkt) ], ...
                    [ ksi0(3, kkt), ksi_fl_z(iiFF, kkt) ], ...
                    'color', 'm', 'linestyle', '--', 'linewidth', 0.8 );
        hold on;
    end
    for iiFF = 1:4
        plot3( [ ksi_fl_x(iiFF, kkt), ksi_fl_x(iiFF + 1, kkt) ], ...
                    [ ksi_fl_y(iiFF, kkt), ksi_fl_y(iiFF + 1, kkt) ], ...
                    [ ksi_fl_z(iiFF, kkt), ksi_fl_z(iiFF + 1, kkt) ], ...
                    'color', 'm', 'linestyle', '--', 'linewidth', 0.8 );
        hold on;
    end
    plot3( [ ksi_fl_x(NF, kkt), ksi_fl_x(1, kkt) ], ...
                [ ksi_fl_y(NF, kkt), ksi_fl_y(1, kkt) ], ...
                [ ksi_fl_z(NF, kkt), ksi_fl_z(1, kkt) ], ...
                'color', 'm', 'linestyle', '--', 'linewidth', 0.8 );
    hold on;
end

lgd = legend([pp0,ppFk(1),ppFk(2),ppFk(3),ppFk(4),ppFk(5)], ...
                        {'$$i=0$$', '$$i=1$$', '$$i=2$$', ...
                         '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','color','none', ...
                         'box','off','NumColumns',2);
box on;

ax_pos = get(gca, 'Position');
lgd_width = 0.2;
lgd_height = 0.08;
margin_right = -0.17;
margin_bottom = 0.13;
lgd_left = ax_pos(1) + ax_pos(3) - lgd_width - margin_right;
lgd_bottom = ax_pos(2) + margin_bottom;
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_height]);

axis equal
xlabel('$$\it {p}_{i,x} \rm (m)$$','interpreter','latex');
ylabel('$$\it {p}_{i,y} \rm (m)$$','interpreter','latex');
zlabel('$$\it {p}_{i,z} \rm (m)$$','interpreter','latex');

view(-10, 55);

%% position tracking error
linestyle_vec = {'--', '-.', ':'};

figure(4)
subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_err_fl_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,x}^{p} \rm (m)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_err_fl_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,y}^{p} \rm (m)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, ksi_err_fl_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('time (s)');
ylabel('$$\it {e}_{i,z}^{p} \rm (m)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

ax1_1 = axes('Position', [0.2, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 0.7));
[~, id_end] = min(abs(tout - 1.0));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.45, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 11.4));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.78, 0.15, 0.1]);
[~, id_start] = min(abs(tout - 20.4));
[~, id_end] = min(abs(tout - 21.4));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.2, 0.48, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.5));
[~, id_end] = min(abs(tout - 0.8));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.45, 0.48, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.48, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.4));
[~, id_end] = min(abs(tout - 21.4));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.2, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.5));
[~, id_end] = min(abs(tout - 2.0));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.45, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.4));
[~, id_end] = min(abs(tout - 21.4));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            ksi_err_fl_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% velocity tracking error
figure(5)
subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_err_fl_x(iiFF,:), 'color', color_vec(id_color,:), ...
                            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','location','best','NumColumns',5);
box on;
set(gca, 'ylim', [-10, 10]);
xlabel('time (s)');
ylabel('$$\it {e}_{i,x}^{v} \rm (m/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_err_fl_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
set(gca, 'ylim', [-10, 10]);
xlabel('time (s)');
ylabel('$$\it {e}_{i,y}^{v} \rm (m/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, eta_err_fl_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
set(gca, 'ylim', [-5, 5]);
xlabel('time (s)');
ylabel('$$\it {e}_{i,z}^{v} \rm (m/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

ax1_1 = axes('Position', [0.2, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.85));
[~, id_end] = min(abs(tout - 1.0));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.45, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.8));
[~, id_end] = min(abs(tout - 11.6));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.5));
[~, id_end] = min(abs(tout - 21.5));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.2, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.6));
[~, id_end] = min(abs(tout - 1.0));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.45, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.8));
[~, id_end] = min(abs(tout - 11.6));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.4));
[~, id_end] = min(abs(tout - 21.2));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.2, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 3.0));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.45, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.8));
[~, id_end] = min(abs(tout - 12.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.4));
[~, id_end] = min(abs(tout - 21.2));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            eta_err_fl_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% virtual velocity tracking in translational disturbance observation
kk_plot_disturb = 1;
kk_gain = 10;

figure(6)
subplot(311)
ppk(1) = plot(tout(1,:), eta_fl_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), sigma_v_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it {\sigma}_{i,x}^{v} \rm (m/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
                {'$$\it {v}_{i}$$', '$$\it {\sigma}_{i}^{v}$$'}, ...
                 'interpreter','latex','box','off','color','none', ...
                 'location','northeast','NumColumns',2);
set(gca, 'ylim', [-1, 1] .* kk_gain);

subplot(312)
ppk(1) = plot(tout(1,:), eta_fl_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), sigma_v_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it {\sigma}_{i,y}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-1, 1] .* kk_gain);

subplot(313)
ppk(1) = plot(tout(1,:), eta_fl_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), sigma_v_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it {\sigma}_{i,z}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.5, 1] .* kk_gain);


%% auxiliary virtual velocity tracking observation in translational disturbance observation

figure(7)
subplot(311)
ppk(1) = plot(tout(1,:), sigma_bar_v_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), sigma_bar_v_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {\overline {\sigma}}_{i,x}^{v} \rm (m/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it \overline {\sigma}_{i}^{v}$$', '$$\it \hat {\overline {\sigma}}_{i}^{v}$$'}, ...
             'interpreter','latex','location','northeast','color','none','box','off','NumColumns',2);
set(gca, 'ylim', [-0.3, 0.3] .* kk_gain);

subplot(312)
ppk(1) = plot(tout(1,:), sigma_bar_v_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), sigma_bar_v_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {\overline {\sigma}}_{i,y}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.1, 0.3] .* kk_gain);

subplot(313)
ppk(1) = plot(tout(1,:), sigma_bar_v_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), sigma_bar_v_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {\overline {\sigma}}_{i,z}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.3, 0.3] .* kk_gain);

%% translational disturbance observation
figure(8)
subplot(311)
ppk(1) = plot(tout(1,:), dp_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), dp_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,x}^{v} \rm (m/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it {d}_{i}^{v}$$', '$$\it \hat {d}_{i}^{v}$$'}, ...
             'interpreter','latex','location','southeast', ...
             'color','none','box','off','NumColumns', 2);
set(gca, 'ylim', [-1.0, 1.0] .* kk_gain);
 
subplot(312)
ppk(1) = plot(tout(1,:), dp_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), dp_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,y}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-1.0, 1.0] .* kk_gain);

subplot(313)
ppk(1) = plot(tout(1,:), dp_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout(1,:), dp_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('time (s)');
ylabel('$$\it \hat {d}_{i,z}^{v} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8, 1.0] .* kk_gain);

%% angular velocity error
figure(9)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, wei_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off', 'color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\varpi}_{i,x}^{e} \rm \ (rad/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, wei_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\varpi}_{i,y}^{e} \rm \ (rad/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, wei_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\varpi}_{i,z}^{e} \rm \ (rad/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.2));
[~, id_end] = min(abs(tout - 2.7));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.46, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.4));
[~, id_end] = min(abs(tout - 12.0));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.23, 0.47, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.4));
[~, id_end] = min(abs(tout - 2.2));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.46, 0.47, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 11.2));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.47, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.23, 0.23, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 4.4));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.46, 0.23, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.23, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 22.0));
[~, id_end] = min(abs(tout - 23.0));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            wei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% rotational error in Lie Algebra
figure(10)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, V_Phi_Rei_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\psi}_{i,x}^{e} \rm \ (rad/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, V_Phi_Rei_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\psi}_{i,y}^{e} \rm \ (rad/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, V_Phi_Rei_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\psi}_{i,z}^{e} \rm \ (rad/s)$$','interpreter','latex');

y_lim = ylim;
rect1 = rectangle('Position', [0, y_lim(1), t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [0.9, 0.95, 1], 'EdgeColor', 'none'); % 浅蓝色
rect2 = rectangle('Position', [t1, y_lim(1), t2 - t1, y_lim(2)-y_lim(1)], ...
                                    'FaceColor', [1, 1, 0.9], 'EdgeColor', 'none'); % 浅黄色
hold on;
uistack(rect1, 'bottom');
uistack(rect2, 'bottom');
hold on;
set(gca, 'layer', 'top');

ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.5));
[~, id_end] = min(abs(tout - 2.5));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.46, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.5));
[~, id_end] = min(abs(tout - 14.0));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.23, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.4));
[~, id_end] = min(abs(tout - 3.0));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.46, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 11.2));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.5));
[~, id_end] = min(abs(tout - 22.0));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.23, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 5.0));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_2 = axes('Position', [0.46, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.5));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            V_Phi_Rei_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

%% intermediate auxiliary rotational error
figure(11)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Phi_psi_e_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','color','none','box','off','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Phi}_{x}({\psi}_{i,x}^{e}) \rm \ (rad/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Phi_psi_e_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Phi}_{y}({\psi}_{i,y}^{e}) \rm \ (rad/s)$$','interpreter','latex');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Phi_psi_e_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\Phi}_{z}({\psi}_{i,z}^{e}) \rm \ (rad/s)$$','interpreter','latex');

ax1_1 = axes('Position', [0.23, 0.77, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.46, 0.77, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.4));
[~, id_end] = min(abs(tout - 12.4));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.77, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.23, 0.53, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.46, 0.53, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.2));
[~, id_end] = min(abs(tout - 11.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.53, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.23, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 4.4));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.46, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.5));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end
set(gca, 'ylim', [-0.02, 0.02]);

ax3_3 = axes('Position', [0.7, 0.18, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% derivative of intermediate auxiliary rotational error
figure(12)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Phi_bar_psi_e_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','color','none','box','off','NumColumns',5);
box on;
set(gca, 'ylim', [-1.0, 1.0]);
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{\Phi}_{x}({\psi}_{i,x}^{e}) \rm \ (rad/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);


subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Phi_bar_psi_e_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
set(gca, 'ylim', [-1.0, 1.0]);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{\Phi}_{y}({\psi}_{i,y}^{e}) \rm \ (rad/s)$$','interpreter','latex');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Phi_bar_psi_e_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
set(gca, 'ylim', [-1.0, 1.0]);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{\Phi}_{z}({\psi}_{i,z}^{e}) \rm \ (rad/s)$$','interpreter','latex');


ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.48, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.4));
[~, id_end] = min(abs(tout - 11.8));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.23, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.0));
[~, id_end] = min(abs(tout - 2.5));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end
set(gca,'ylim',[-1.0, 2.0]);

ax2_2 = axes('Position', [0.48, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end
set(gca,'ylim',[-0.4, 0.4]);

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.23, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 2.0));
[~, id_end] = min(abs(tout - 3.4));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.48, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.4));
[~, id_end] = min(abs(tout - 12.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end
set(gca, 'ylim', [-0.02, 0.02]);

ax3_3 = axes('Position', [0.7, 0.24, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Phi_bar_psi_e_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% auxiliary sliding mode surface
figure(13)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, S_bar_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{S}_{i,x} \rm \ (rad/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);

subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, S_bar_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{S}_{i,y} \rm \ (rad/s)$$','interpreter','latex');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, S_bar_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \overline{S}_{i,z} \rm \ (rad/s)$$','interpreter','latex');


ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.5));
[~, id_end] = min(abs(tout - 3.0));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.48, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.0));
[~, id_end] = min(abs(tout - 12.5));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.23, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.6));
[~, id_end] = min(abs(tout - 2.6));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.48, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.54, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.23, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 5.0));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.48, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 12.0));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            S_bar_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% sliding mode surface
figure(14)

subplot(311)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Si_x(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
lgd = legend([ppk(1),ppk(2),ppk(3),ppk(4),ppk(5)], ...
                        {'$$i=1$$', '$$i=2$$', '$$i=3$$', '$$i=4$$', '$$i=5$$'}, ...
                         'interpreter','latex','box','off','color','none','NumColumns',5);
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it S_{i}(x) \rm \ (rad/s)$$','interpreter','latex');

ax_pos = get(gca, 'Position');
lgd_hgt = 0.05;
vertical_gap = 0.01;
lgd_left   = ax_pos(1);
lgd_bottom = ax_pos(2) + ax_pos(4) + vertical_gap;
lgd_width  = ax_pos(3);
set(lgd, 'Units', 'normalized', 'Position', [lgd_left, lgd_bottom, lgd_width, lgd_hgt]);


subplot(312)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Si_y(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it S_{i}(y) \rm \ (rad/s)$$','interpreter','latex');

subplot(313)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    ppk(iiFF) = plot(tout, Si_z(iiFF,:), 'color', color_vec(id_color,:), ...
                                'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.2);
    hold on;
end
box on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it S_{i}(z) \rm \ (rad/s)$$','interpreter','latex');


ax1_1 = axes('Position', [0.23, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.5));
[~, id_end] = min(abs(tout - 3.0));
axes(ax1_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_2 = axes('Position', [0.48, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 11.0));
[~, id_end] = min(abs(tout - 12.5));
axes(ax1_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax1_3 = axes('Position', [0.7, 0.84, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 21.0));
axes(ax1_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_x(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_1 = axes('Position', [0.23, 0.515, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 0.6));
[~, id_end] = min(abs(tout - 2.6));
axes(ax2_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax2_2 = axes('Position', [0.48, 0.515, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 12.0));
axes(ax2_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax2_3 = axes('Position', [0.7, 0.515, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
axes(ax2_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_y(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_1 = axes('Position', [0.23, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 1.6));
[~, id_end] = min(abs(tout - 3.0));
axes(ax3_1)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 1.0);
    hold on;
end

ax3_2 = axes('Position', [0.48, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 10.0));
[~, id_end] = min(abs(tout - 11.5));
axes(ax3_2)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end

ax3_3 = axes('Position', [0.7, 0.17, 0.15, 0.08]);
[~, id_start] = min(abs(tout - 20.0));
[~, id_end] = min(abs(tout - 22.0));
axes(ax3_3)
for iiFF = 1:NF
    id_color = mod(iiFF,size(color_vec,1));
    if ~id_color
        id_color = size(color_vec,1);
    end
    id_linestyle = mod(iiFF,size(linestyle_vec,2));
    if ~id_linestyle
        id_linestyle = size(linestyle_vec,2);
    end
    plot(tout(1,id_start:id_end), ...
            Si_z(iiFF,id_start:id_end), ...
            'color', color_vec(id_color,:), ...
            'linestyle', linestyle_vec{id_linestyle}, 'linewidth', 0.8);
    hold on;
end


%% virtual angular velocity tracking in rotational disturbance observation
kk_plot_disturb = 1;
kdgain = 10;

figure(15)

subplot(311)
ppk(1) = plot(tout, wi_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,x}^{\varpi}  \rm (rad/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it {\varpi}_{i} $$', '$$\it {\sigma}_{i}^{\varpi}$$'}, ...
             'interpreter','latex','location','northeast','color','none','box','off','NumColumns',2);

subplot(312)
ppk(1) = plot(tout, wi_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,y}^{\varpi}  \rm (rad/s)$$','interpreter','latex');

subplot(313)
ppk(1) = plot(tout, wi_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_w_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it {\sigma}_{i,z}^{\varpi}  \rm (rad/s)$$','interpreter','latex');

%% auxiliary virtual velocity control observation in rotational disturbance observation

figure(16)

subplot(311)
ppk(1) = plot(tout, sigma_bar_w_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_bar_w_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat { \overline {\sigma}}_{i,x}^{\varpi} \rm (rad/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it \overline {\sigma}_{i}^{\varpi}$$', '$$\it \hat { \overline {\sigma}}_{i}^{\varpi}$$'}, ...
             'interpreter','latex','location','southeast','NumColumns',2,'color','none','box','off');
set(gca, 'ylim', [-0.6, 0.3]);

subplot(312)
ppk(1) = plot(tout, sigma_bar_w_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_bar_w_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat { \overline {\sigma}}_{i,y}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.4, 0.6]);

subplot(313)
ppk(1) = plot(tout, sigma_bar_w_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, sigma_bar_w_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat { \overline {\sigma}}_{i,z}^{\varpi} \rm (m/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.5, 0.3]);

%% rotational disturbance observation

figure(17)

subplot(311)
ppk(1) = plot(tout, dwi_x(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_x(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,x}^{\varpi} \rm (rad/s)$$','interpreter','latex');
legend([ppk(1),ppk(2)], ...
            {'$$\it {d}_{i}^{\varpi}$$', '$$\it \hat {d}_{i}^{\varpi}$$'}, ...
             'interpreter','latex','location','southeast','NumColumns',2,'color','none','box','off');
set(gca, 'ylim', [-1.0 * kdgain, 1.0 * kdgain]);

subplot(312)
ppk(1) = plot(tout, dwi_y(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_y(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,y}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8 * kdgain, 0.8 * kdgain]);

subplot(313)
ppk(1) = plot(tout, dwi_z(kk_plot_disturb,:), ...
                        'color', 'r', 'linestyle', '-', 'linewidth', 1.2);
hold on;
ppk(2) = plot(tout, dwi_hat_z(kk_plot_disturb,:), ...
                        'color', 'b', 'linestyle', '--', 'linewidth', 1.2);
hold on;
xlabel('$$\it t \ (s) \rm$$','interpreter','latex');
ylabel('$$\it \hat {d}_{i,z}^{\varpi} \rm (rad/s)$$','interpreter','latex');
set(gca, 'ylim', [-0.8 * kdgain, 1.0 * kdgain]);


%% 
