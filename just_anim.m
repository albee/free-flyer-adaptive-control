% --- Set up animation ---%
% Choose an animation rate

addpath('./paper-plot-data');
addpath('./plot');
addpath('./SPART');

% load('resultsAdaptiveSHELF2.mat');
load('resultsAdaptiveStep_paper.mat');


real_system_urdf = './SPART/urdf/astrobee_planar_2_link_grapple.urdf';  % grappled a point mass

%--- Set up simulated real system ---%
filename = real_system_urdf;
[robot] = urdf_to_spart_model(filename);
truemass = 0;
robot.links(end).mass = truemass;
ff = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics

tf = 19.0;

state_hist = results.run{11}.state_hist;
x_des_hist = results.run{11}.x_des_hist;
control_hist = results.run{11}.control_hist;
tvec = results.run{11}.tvec;

fig = figure('units','normalized','outerposition',[0 0 1 1]);
view(45, 20);
hold on;
grid on;
% xyz = [0, 0, 0;  % x y z
%        1, 0, 0;
%        1, 0, 1;
%        1, 1, 1]';
% plot3(xyz(1,:),xyz(2,:),xyz(3,:),'ro','LineWidth',2);
%        hold on;
% pp = cscvn(xyz);
% fnplt(cscvn(xyz(:,[1:end 1])),'r',2)

axis equal;
% axis([-5, 1, -3, 3, -2, 2]*1.0)
axis([-2, 2, -2, 2, -2, 2]*1.0)

plot_dt = 0.5;  % period for plot frames, s
t = 0:plot_dt:tf;
cntr = 1;
frames = [];  % vector of indices to save
for i = 1:1:size(tvec,1)
    if tvec(i) > t(cntr)
        cntr = cntr+1;
        frames = [frames; i];
    end
end
fprintf('Plotting with timestep of %4.2f', plot_dt)
ff.plot_from_history_FK3(fig, state_hist(frames,:), control_hist(frames,:));