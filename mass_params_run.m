% Mass_parameters_Run



addpath('./SPART');
addpath('./plot');
clc; %close all;
clearvars;
fontsize = 25;

COMPUTE_ANALYTICAL = true;  % loads analytical instead of computing it
tf = 2.0; % specified in create_ref_traj()
CONTROLLER = 'feedback_lin'; % ['adaptive', 'feedback_lin', 'pd']
REF_TRAJ = 'Shelf'; % ['Circular, 'Step', 'Sinusoid', 'Shelf']
SPACE = '3D'; % ['2D', '3D']
real_system_urdf = './SPART/urdf/astrobee_planar_2_link_grapple.urdf';  % grappled a point mass
model_system_urdf = './SPART/urdf/astrobee_planar_2_link.urdf';


%--- Set up simulated real system ---%
filename = real_system_urdf;
[robot] = urdf_to_spart_model(filename);
ff = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics
    
%--- Set up model system ---%
filename = model_system_urdf;
[robot] = urdf_to_spart_model(filename);
robot.link(3).mass = 0;
ff_model = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics

num_joints = ff.robot.n_q;
n = 2*(6 + num_joints);
m = 3 + num_joints;

% ---- Create Trajectory ----- %
ref_traj = create_ref_traj(REF_TRAJ);
t_setpoints = ref_traj(:, 1);
x_des_setpoints = ref_traj(:, 2:n+1);
xdd_des_setpoints = ref_traj(:, n+2:end);

x_des_hist = [];  % actual setpoints used by propagator
control_hist = [];  % actual control issued by controller

% --- Mass vector

mass_vec = linspace(0, 20, 11);

results.controller = CONTROLLER;
results.traj = REF_TRAJ;


odeOptions = odeset('RelTol',1e-4,'AbsTol',1e-4);
for massid = 1:length(mass_vec)
    fprintf('Current Run %d\n', massid)
    
    filename = real_system_urdf;
    [robot] = urdf_to_spart_model(filename);
    robot.links(end).mass = mass_vec(massid);
    ff = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics
    
    
    if strcmp(CONTROLLER, 'adaptive')
        %% Adaptive control
        %Initial conditions
        % state : [r1 r2 q0 q1 r1d r2d q0d q1d, a_hatd]' INERTIAL
        a_hat_0 = 0.0;
        x_0 = zeros(n+1,1);
        x_0(end) = a_hat_0;  % initial guess for a_hat
        tspan = [0, tf]; % seconds
        
        dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, 0, 0, CONTROLLER, SPACE);
        
    elseif strcmp(CONTROLLER, 'feedback_lin')
        %% Nominal system control
        %Initial conditions
        % state : [r1 r2 q0 q1 r1d r2d q0d q1d]' INERTIAL
        %         [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]' INERTIAL
        x_0 = zeros(n,1);
        tspan = [0, tf]; % seconds
        
        dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, 0, 0, CONTROLLER, SPACE);
    elseif strcmp(CONTROLLER, 'pd')
        %% Nominal system control
        %Initial conditions
        % state : [r1 r2 q0 q1 r1d r2d q0d q1d]' INERTIAL
        %         [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]' INERTIAL
        x_0 = zeros(n,1);
        tspan = [0, tf]; % seconds
        
        dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, 0, 0, CONTROLLER, SPACE);
    end
    
    % Dynamics
    [tvec, state_hist] = ode23(dynamics, tspan, x_0, odeOptions);  % produces [t, n] of state history
    
    x_des_hist = [];
    
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
    [RMS, vec_RMS] = calc_RMS_error(x_des_hist, state_hist(:,1:16), false);  % no parameters
    RMS
    
    results.run{massid}.tvec = tvec;
    results.run{massid}.state_hist = state_hist;
    results.run{massid}.x_des_hist = x_des_hist;
    results.run{massid}.control_hist = control_hist;
    results.run{massid}.RMS = RMS;
    results.run{massid}.vec_RMS = vec_RMS;
    results.run{massid}.mass = mass_vec(massid);
    
    
    
    
end

save('resultsFLSHELF2.mat', 'results')

%% figure

load('resultsPDSHELF2.mat')
rmspd = [];
for ii = 1:length(results.run)
    rmspd(ii) = results.run{ii}.RMS
end


figure;
plot(mass_vec, rmspd, '--^k', 'linewidth',1.5)
xlabel('Mass [kg]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('RMS', 'Interpreter', 'latex', 'FontSize', fontsize)
    
hold on
if 1
load('resultsAdaptiveSHELF2.mat')
rmspd = [];
for ii = 1:length(results.run)
    rmspd(ii) = results.run{ii}.RMS
end


plot(mass_vec, rmspd, '--*b', 'linewidth',1.5)


leg = legend({'PD', 'Adaptive'}, 'Interpreter', 'latex');
leg.FontSize = fontsize;
leg.Location = 'Best'

end

%% Plots 

steprms_pd = [];


steprms_pd_Single = []
load('resultsPDStep2.mat')

for ii = 1:length(results.run)
   steprms_pd(ii,:) = results.run{ii}.vec_RMS;
   steprms_pd_Single = results.run{ii}.RMS;
end

steprms_adaptive = [];

load('resultsAdaptiveStep2.mat')

for ii = 1:length(results.run)
   steprms_adaptive(ii,:) = results.run{ii}.vec_RMS;
end


load('resultsFLStep2.mat')

for ii = 1:length(results.run)
   steprms_FL(ii,:) = results.run{ii}.vec_RMS;
end


shelfrms_pd = [];

load('resultsPDSHELF2.mat')

for ii = 1:length(results.run)
   shelfrms_pd(ii,:) = results.run{ii}.vec_RMS;
end

shelfrms_adaptive = [];

load('resultsAdaptiveSHELF2.mat')

for ii = 1:length(results.run)
   shelfrms_adaptive(ii,:) = results.run{ii}.vec_RMS;
end
mass_vec = linspace(0, 20, 11);


load('resultsFLSHELF2.mat')

for ii = 1:length(results.run)
   shelfrms_FL(ii,:) = results.run{ii}.vec_RMS;
end
mass_vec = linspace(0, 20, 11);



totalmass = robot.base_link.mass + robot.links(1).mass + + robot.links(2).mass;

frac_vec = mass_vec./(mass_vec + totalmass);

figure

blue =      [ 0    0.4470    0.7410];
red = [ 0.8500    0.3250    0.0980];

plot(frac_vec, mean(steprms_pd(:,1:3),2), '-k^', 'linewidth', 1.5)
hold on
plot(frac_vec, mean(steprms_adaptive(:,1:3),2), '-*','Color', blue ,'linewidth', 1.5)

plot(frac_vec, mean(steprms_FL(:,1:3),2), '-o','Color', red ,'linewidth', 1.5)


plot(frac_vec, mean(shelfrms_pd(:,1:3),2), '-.k^', 'linewidth', 1.5)
hold on
plot(frac_vec, mean(shelfrms_adaptive(:,1:3),2), '-.*','Color', blue, 'linewidth', 1.5)

plot(frac_vec(1:5), mean(shelfrms_FL(1:5,1:3),2), '-.o','Color', red , 'linewidth', 1.5)


leg = legend({'Step, PD', 'Step, Adaptive','Step, FL', 'AFF, PD','AFF, Adaptive','AFF, FL'}, 'Interpreter', 'latex');
leg.FontSize = fontsize;
leg.Location = 'best';
xlabel('Mass Fraction, $\frac{m_3}{M+ m_3}$', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Position RMS [m]', 'Interpreter', 'latex', 'FontSize', fontsize)
  
% angles 
figure 

plot(frac_vec, mean(steprms_pd(:,4:8),2), '-k^', 'linewidth', 1.5)
hold on
plot(frac_vec, mean(steprms_adaptive(:,4:8),2), '-k*','Color', blue, 'linewidth', 1.5)

plot(frac_vec, mean(steprms_FL(:,4:8),2), '-o','Color', red , 'linewidth', 1.5)


plot(frac_vec, mean(shelfrms_pd(:,4:8),2), '-.k^', 'linewidth', 1.5)
hold on
plot(frac_vec, mean(shelfrms_adaptive(:,4:8),2), '-.*','Color', blue, 'linewidth', 1.5)

plot(frac_vec(1:5), mean(shelfrms_FL(1:5,4:8),2), '-.o','Color', red , 'linewidth', 1.5)

leg = legend({'Step, PD', 'Step, Adaptive', 'Step, FL', 'AFF, PD','AFF, Adaptive', 'AFF, FL'}, 'Interpreter', 'latex');
leg.FontSize = fontsize;
leg.Location = 'best';
xlabel('Mass Fraction, $\frac{m_3}{M+ m_3}$', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Angle RMS [$^\circ$]', 'Interpreter', 'latex', 'FontSize', fontsize)
  
