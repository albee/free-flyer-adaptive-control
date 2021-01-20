%{
 Two-link free-flying manipulator adaptive and non-adaptive control.

 Keenan Albee and Alex Cabrales, Jan-2021
%}

addpath('./SPART');
addpath('./linearizedFunctions/');
addpath('./plot');
addpath(genpath('./codegen'));
clc; close all; clearvars;

fontsize = 25;

%% Setup
%--- Parameters ---%
COMPUTE_ANALYTICAL = false;  % loads analytical adaptive control instead of computing it
tf = 20.0; % specified in create_ref_traj()
CONTROLLER = 'adaptive'; %  ['adaptive', 'feedback_lin', 'pd']
REF_TRAJ = 'VeryRich'; %  ['AFF', 'Step2D', 'Rich', 'Sinusoid', 'VeryRich']
SPACE = '3D'; % ['3D'] only 3D currently supported!
real_system_urdf = './SPART/urdf/astrobee_planar_2_link_grapple.urdf';  % grappled a point mass
model_system_urdf = './SPART/urdf/astrobee_planar_2_link.urdf';  % no knowledge of grappled point mass

%--- Set up simulated real system ---%
filename = real_system_urdf;
[robot] = urdf_to_spart_model(filename);
truemass = 8;
robot.links(end).mass = truemass;
ff = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics

%--- Set up model system ---%
filename = model_system_urdf;
[robot] = urdf_to_spart_model(filename);
robot.links(end).mass = 0;
ff_model = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics

num_joints = ff.robot.n_q;
n = 2*(6 + num_joints);
m = 3 + num_joints;

% ---- Create Trajectory ----- %
ref_traj = create_ref_traj(REF_TRAJ);  % this actually creates the reference trajectory. See create_ref_traj.m.
t_setpoints = ref_traj(:, 1);
x_des_setpoints = ref_traj(:, 2:n+1);
xdd_des_setpoints = ref_traj(:, n+2:end);

x_des_hist = [];  % actual setpoints used by propagator
control_hist = [];  % actual control issued by controller

if COMPUTE_ANALYTICAL
    [Y_lin, Y_aff] = analytical_dynamics(filename, SPACE);
end

%% Controller selection
if strcmp(CONTROLLER, 'adaptive')
    % Adaptive control
    %Initial conditions
    % state : [r1 r2 q0 q1 r1d r2d q0d q1d, a_hatd]' INERTIAL
    a_hat_0 = 0.0;
    x_0 = zeros(n+1,1);
    x_0(end) = a_hat_0;  % initial guess for a_hat
    tspan = [0, tf]; % seconds
    
    dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, 0, 0, CONTROLLER, SPACE);
    odeOptions = odeset('RelTol',1e-4,'AbsTol',1e-4);
elseif strcmp(CONTROLLER, 'feedback_lin')
    % Feedback linearization
    %Initial conditions
    % state : [r1 r2 q0 q1 r1d r2d q0d q1d]' INERTIAL
    %         [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]' INERTIAL
    x_0 = zeros(n,1);
    tspan = [0, tf]; % seconds

    dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, 0, 0, CONTROLLER, SPACE);
    odeOptions = odeset('RelTol',1e-4,'AbsTol',1e-4);
elseif strcmp(CONTROLLER, 'pd')
    % PD control
    %Initial conditions
    % state : [r1 r2 q0 q1 r1d r2d q0d q1d]' INERTIAL
    %         [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]' INERTIAL
    x_0 = zeros(n,1);
    tspan = [0, tf]; % seconds

    dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, 0, 0, CONTROLLER, SPACE);
    odeOptions = odeset('RelTol',1e-4,'AbsTol',1e-4);
end

%% Dynamics
disp("Running dynamics simulation...")
[tvec, state_hist] = ode23(dynamics, tspan, x_0, odeOptions);  % produces [t, n] of state history
disp("Simulation complete! x_des_hist contains reference trajectory. See 'Plotting, animation, analysis' for more outputs.")

%% Plotting, animation, analysis
% Recreate control and setpoint history
for i = 1:1:size(tvec,1)
    [x_des, xdd_des] = lookup_ref_traj(tvec(i), ff, ref_traj);
    x_des_hist(i, :) = x_des';
    if strcmp(CONTROLLER, 'adaptive')
        ff_model.a_hat = state_hist(i,end);
        control_hist(i, :) = freeflyer_adaptive_control(x_des, state_hist(i,:)', xdd_des, ff, ff_model, 0, 0)';
    elseif strcmp(CONTROLLER, 'feedback_lin')
        control_hist(i, :) = freeflyer_feedback_lin_control(x_des, state_hist(i,:)', xdd_des, ff, ff_model)';
    elseif strcmp(CONTROLLER, 'pd')
        control_hist(i, :) = freeflyer_pd_control(x_des, state_hist(i,:)', xdd_des, ff, ff_model)';
    end
end

r1_hist = state_hist(:,1);  % deg
r2_hist = state_hist(:,2);  % deg
r3_hist = state_hist(:,3);  % deg
q0x_hist = state_hist(:,4);  % deg
q0y_hist = state_hist(:,5);  % deg
q0_hist = state_hist(:,6);  % deg
q1_hist = state_hist(:,7);  % deg
q2_hist = state_hist(:,8);  % deg/s
r1d_hist = state_hist(:,9);  % deg/s
r2d_hist = state_hist(:,10);  % deg/s
q0xd_hist = state_hist(:,11);  % deg/s
q0yd_hist = state_hist(:,12);  % deg/s
q0d_hist = state_hist(:,13);  % deg/s
q1d_hist = state_hist(:,15);
q2d_hist = state_hist(:,16);

r1_des_hist = x_des_setpoints(:,1);
r2_des_hist = x_des_setpoints(:,2);
q0_des_hist = x_des_setpoints(:,6);
q1_des_hist = x_des_setpoints(:,7);
q2_des_hist = x_des_setpoints(:,8);

F1_hist = control_hist(:,1);
F2_hist = control_hist(:,2);
tau0_hist = control_hist(:,6);
tau1_hist = control_hist(:,7);
tau2_hist = control_hist(:,8);

%--- Metrics ---%
[RMS, vec_RMS] = calc_RMS_error(x_des_hist, state_hist(:,1:16), false);  % no parameters

%--- Set up animation ---%
% Choose an animation rate
% fig = figure('units','normalized','outerposition',[0 0 1 1])
% view(45, 20);
% hold on;
% grid on;
% xyz = [0, 0, 0;  % x y z
%        1, 0, 0;
%        1, 0, 1;
%        1, 1, 1]';
% plot3(xyz(1,:),xyz(2,:),xyz(3,:),'ro','LineWidth',2);
%        hold on;
% pp = cscvn(xyz);
% fnplt(cscvn(xyz(:,[1:end 1])),'r',2)
% 
% axis equal;
% % axis([-5, 1, -3, 3, -2, 2]*1.0)
% axis([-2, 2, -2, 2, -2, 2]*1.0)
% 
% plot_dt = 0.5;  % period for plot frames, s
% t = 0:plot_dt:tf;
% cntr = 1;
% frames = [];  % vector of indices to save
% for i = 1:1:size(tvec,1)
%     if tvec(i) > t(cntr)
%         cntr = cntr+1;
%         frames = [frames; i];
%     end
% end
% fprintf('Plotting with timestep of %4.2f', plot_dt)
% ff.plot_from_history_FK3(fig, state_hist(frames,:), control_hist(frames,:));

%%
%--- Default plotting ---%
% Translational Position History
figure
hold on
plot(tvec, r1_hist, '-r', 'linewidth', 2)
plot(tvec, r2_hist,'-b', 'linewidth', 2)
plot(t_setpoints, r1_des_hist, '--r', 'linewidth', 2)
plot(t_setpoints, r2_des_hist, '--b', 'linewidth', 2)
legend('$r_1$', '$r_2$', '$r_{1,des}$', '$r_{2,des}$','Interpreter', 'latex', 'FontSize', fontsize);
grid on;
set(gca, 'FontSize', fontsize-5)
xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Position [$m$]', 'Interpreter', 'latex', 'FontSize', fontsize)
title('2-Link Manipulator Position Performance','Interpreter', 'latex', 'FontSize', fontsize)

% Angular History
figure
hold on
plot(tvec, q0_hist, '-r', 'linewidth', 2)
plot(tvec, q1_hist, '-black', 'linewidth', 2)
plot(tvec, q2_hist, '-b', 'linewidth', 2)
plot(t_setpoints, q0_des_hist, '--r', 'linewidth', 2)
plot(t_setpoints, q1_des_hist, '--black', 'linewidth', 2)
plot(t_setpoints, q2_des_hist, '--b', 'linewidth', 2)
legend('$q_0$', '$q_1$', '$q_2$', '$q_{0,des}$', '$q_{1,des}$', '$q_{2,des}$', 'Interpreter', 'latex', 'FontSize', fontsize);
grid on;
set(gca, 'FontSize', fontsize-5)
xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Angle [$rad$]', 'Interpreter', 'latex', 'FontSize', fontsize)
title('2-Link Manipulator Angular Performance','Interpreter', 'latex', 'FontSize', fontsize)

% Input History
figure
hold on
plot(tvec(1:end-20), F1_hist(1:end-20), '-r', 'linewidth', 2)
plot(tvec(1:end-20), F2_hist(1:end-20), '-b', 'linewidth', 2)
plot(tvec(1:end-20), tau0_hist(1:end-20), '-black', 'linewidth', 2)
plot(tvec(1:end-20), tau1_hist(1:end-20), '-g', 'linewidth', 2)
plot(tvec(1:end-20), tau2_hist(1:end-20), '-c', 'linewidth', 2)
legend('$F_1$', '$F_2$', '$\tau_0$', '$\tau_{1}$', '$\tau_{2}$', 'Interpreter', 'latex', 'FontSize', fontsize);
grid on;
set(gca, 'FontSize', fontsize-5)
xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Input [$N$, $N-m$]', 'Interpreter', 'latex', 'FontSize', fontsize)
title('2-Link Manipulator Input History','Interpreter', 'latex', 'FontSize', fontsize)

if strcmp(CONTROLLER, 'adaptive')
    % Adaptation History
    truemass = 8.0;  % [kg]
    figure  
    a1_hist = state_hist(:,end);     
    plot(tvec(1:end-10), truemass*ones(size(tvec,1)-10),'--black', 'linewidth', 2)
    hold on
    plot(tvec(1:end-10), a1_hist(1:end-10),'-black', 'linewidth', 2)
    plot(tvec(1:end-10), a1_hist(1:end-10),'LineWidth',2)
    
%     ylim([-5,10]); 
%     legend('$m_3$', '$\hat{m}_3$', 'Interpreter', 'latex', 'FontSize', fontsize);
    xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
    ylabel('mass [$kg$]', 'Interpreter', 'latex', 'FontSize', fontsize)
%     title('Estimated End Effector Mass','Interpreter', 'latex', 'FontSize', fontsize)
end
% end