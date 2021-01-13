%{
Create a reference trajectory for n-link manipulator to follow

Output:
ref_traj : [t_des_hist, x_des_hist xdd_des_hist]

Note: x_des_hist contains the state (positions and velocities).
%}
function ref_traj = create_ref_traj(varargin)
% First input is the type of trajectory ['Circular', 'Step']
% [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]'
% x = [r1 r2 r3, q0x q0y q0z, q1,q2, rd1 rd2 rd3, q0d1 q0d2 q0d3, q1d, q2d]
num_setpoints = 1001;
tf = 20;  % final time for parameterized trajectory
dt = tf/num_setpoints;
t_des_hist = linspace(0,tf,num_setpoints)';

if nargin == 0
    REFTYPE = 'Shelf'; % ['Circular', 'Step']
elseif nargin == 1
    REFTYPE = varargin{1};
end

switch REFTYPE
    
    case 'Shelf'
        %% Shelf
        % [r1 r2 r3, q0x q0y q0z, q1,q2, rd1 rd2 rd3, q0d1 q0d2 q0d3, q1d, q2d]
        % [x                             xd
        xyz = [0, 0, 0;  % x y z
            1, 0, 0;
            1, 0, 1;
            1, 1, 1]';
        %        plot3(xyz(1,:),xyz(2,:),xyz(3,:),'ro','LineWidth',2);
        %                hold on;
        pp = cscvn(xyz);
        sample_pts = linspace(0, 3, num_setpoints);
        r = ppval(pp, sample_pts)';
        %        scatter3(r(:,1), r(:,2), r(:,3));
        
        q0x = linspace(0,pi/3,num_setpoints)';  % roll
        q0y = linspace(0,-pi/3,num_setpoints)';  % pitch
        q0z = linspace(0,pi/6,num_setpoints)';  % yaw
        
        q1 = linspace(0,pi/4,num_setpoints)';  % q1
        q2 = linspace(0,pi/4,num_setpoints)';  % q2
        
        x_des_hist = [r, q0x, q0y, q0z, q1, q2];
        xd_des_hist = [diff(x_des_hist); zeros(1,8)]/dt;
        x_des_hist = [x_des_hist, xd_des_hist];
        
        xdd_des_hist = [diff(xd_des_hist); zeros(1,8)]/dt;  % accelerations
        
    case 'Circular'
        %% Circular
        x = pi*[0:.5:2];
        y = [0  0.0  -1.0  -2.0  -1.0  0  0;
            1  0      1   0    -1.0  0  1];
        
        x = pi*[0:.5:2];
        y = [0  0.5 ];
        pp = spline(x,y);
        r = ppval(pp, linspace(0,2*pi,num_setpoints))';
        q0 = linspace(0,2*pi,num_setpoints)';
        
        samples1 = linspace(0,2*pi*3, num_setpoints);
        samples2 = linspace(0,2*pi*5, num_setpoints);
        q1 = (exp(-samples1./(2*pi*3).*.00001).*sin(samples1))';
        q2 = (exp(-samples2./(2*pi*5).*.00001).*sin(samples2))';
        
        x_des_hist = [r, q0, q1, q2];
        xd_des_hist = [diff(x_des_hist); zeros(1,5)]/dt;
        x_des_hist = [x_des_hist, xd_des_hist];
        
        xdd_des_hist = [diff(xd_des_hist); zeros(1,5)]/dt;  % accelerations
        
        samples = ones(length(t_des_hist), length(amps)) .* amps;
        samples(t_des_hist <= timestep,:) = samples(t_des_hist <= timestep,:)*0;
        
%         r = samples(:,1:2); 
%         q0 = samples(:,3); q1 = samples(:,4); q2 = samples(:,5);
        
%         x_des_hist = [r, q0, q1, q2];
        x_des_hist = samples; 
        xd_des_hist = zeros(size(x_des_hist));
        
        % Obtain the full state
        x_des_hist = [x_des_hist, xd_des_hist];
        xdd_des_hist = zeros(size(xd_des_hist)); 
        
   case 'Step2D'
        %% Step Response
        %     x_des_hist =  zeros(num_setpoints, 10);
        %     x_des_hist(:,1:5) = repmat([0, 0, 0, 1.5, -1.5], num_setpoints, 1);
        amps = [.5,.5, .1, deg2rad([1, 2, 1]), deg2rad([6, 7])]; % [x, y, q0, q1, q2];
        timestep = 1/20*tf; % sec
        
        ramptime = 5;
        rates = amps/ramptime; % 3 seconds
        
        samples = zeros(length(t_des_hist), length(amps)) .* amps;
        samples(t_des_hist >= timestep & t_des_hist<= ramptime + timestep,:) = rates.*(t_des_hist(t_des_hist >= timestep & t_des_hist<= ramptime+ timestep)- timestep);
        samples(t_des_hist >= timestep + ramptime, :) = amps.*ones(size(t_des_hist(t_des_hist >= timestep + ramptime)));
        
        
        %         r = samples(:,1:2);
        %         q0 = samples(:,3); q1 = samples(:,4); q2 = samples(:,5);
        
        %         x_des_hist = [r, q0, q1, q2];
        x_des_hist = samples;
        xd_des_hist = zeros(size(x_des_hist));
        
        xd_des_hist = zeros(length(t_des_hist), length(amps)) .* amps;
        xd_des_hist(t_des_hist >= timestep & t_des_hist<= ramptime + timestep,:) = rates .* ones(size(t_des_hist(t_des_hist >= timestep & t_des_hist<= ramptime + timestep)));
        
        % Obtain the full state
        x_des_hist = [x_des_hist, xd_des_hist];
        xdd_des_hist = zeros(size(xd_des_hist));
        
    case 'Rich'
        %% Rich response
        amp_vec = 0.2*[2, 1,0.5, deg2rad(2), deg2rad(3), deg2rad(5), deg2rad(12), deg2rad(7)];
        f_vec = [2, 3, 1, 0.03, 0.05 0.1, 1.3, 4];
        
        gen = [1/sqrt(2)]; % To add to the richness
        
        x_des_hist = [amp_vec(1)*cos(f_vec(1)*t_des_hist), ...  % x
            amp_vec(2)*cos(f_vec(2)*t_des_hist), ...            % y
            amp_vec(3)*cos(f_vec(3)*t_des_hist), ...            % z
            amp_vec(4)*cos(f_vec(4)*t_des_hist),...             % q01
            amp_vec(5)*cos(f_vec(5)*t_des_hist),...             % q02
            amp_vec(6)*cos(f_vec(6)*t_des_hist),...             % q03
            amp_vec(7)*cos(f_vec(7)*t_des_hist),...             % q03
            amp_vec(8)*cos(f_vec(8)*t_des_hist)].*sin(gen(1)*t_des_hist);
        
        xd_des_hist = -[amp_vec(1)*f_vec(1)*sin(f_vec(1)*t_des_hist), ...
            amp_vec(2)*f_vec(2)*sin(f_vec(2)*t_des_hist), ...
            amp_vec(3)*f_vec(3)*sin(f_vec(3)*t_des_hist), ...
            amp_vec(4)*f_vec(4)*sin(f_vec(4)*t_des_hist),...
            amp_vec(5)*f_vec(5)*sin(f_vec(5)*t_des_hist), ...
            amp_vec(6)*f_vec(6)*sin(f_vec(6)*t_des_hist), ...
            amp_vec(7)*f_vec(7)*sin(f_vec(7)*t_des_hist),...
            amp_vec(8)*f_vec(8)*sin(f_vec(8)*t_des_hist)].*sin(gen(1)*t_des_hist) + ...
            [amp_vec(1)*cos(f_vec(1)*t_des_hist), ...
            amp_vec(2)*cos(f_vec(2)*t_des_hist), ...
            amp_vec(3)*cos(f_vec(3)*t_des_hist), ...
            amp_vec(4)*cos(f_vec(4)*t_des_hist),...
            amp_vec(5)*cos(f_vec(5)*t_des_hist), ...
            amp_vec(6)*cos(f_vec(6)*t_des_hist), ...
            amp_vec(7)*cos(f_vec(7)*t_des_hist), ...
            amp_vec(8)*cos(f_vec(8)*t_des_hist)].*gen(1).*cos(gen(1)*t_des_hist);
        
        xdd_des_hist = -[amp_vec(1)*f_vec(1)^2*cos(f_vec(1)*t_des_hist), ...
            amp_vec(2)*f_vec(2)*cos(f_vec(2)^2*t_des_hist), ...
            amp_vec(3)*f_vec(3)*cos(f_vec(3)^2*t_des_hist), ...
            amp_vec(4)*f_vec(4)*cos(f_vec(4)^2*t_des_hist),...
            amp_vec(5)*f_vec(5)*cos(f_vec(5)^2*t_des_hist), ...
            amp_vec(6)*f_vec(6)*cos(f_vec(6)^2*t_des_hist), ...
            amp_vec(7)*f_vec(7)*cos(f_vec(7)^2*t_des_hist),...
            amp_vec(8)*f_vec(8)*cos(f_vec(8)^2*t_des_hist)].*sin(gen(1)*t_des_hist) -2* ...
            [amp_vec(1)*f_vec(1)*sin(f_vec(1)*t_des_hist), ...
            amp_vec(2)*f_vec(2)*sin(f_vec(2)*t_des_hist), ...
            amp_vec(3)*f_vec(3)*sin(f_vec(3)*t_des_hist), ...
            amp_vec(4)*f_vec(4)*sin(f_vec(4)*t_des_hist),...
            amp_vec(5)*f_vec(5)*sin(f_vec(5)*t_des_hist), ...
            amp_vec(6)*f_vec(6)*sin(f_vec(6)*t_des_hist), ...
            amp_vec(7)*f_vec(7)*sin(f_vec(7)*t_des_hist),...
            amp_vec(8)*f_vec(8)*sin(f_vec(8)*t_des_hist)].*gen(1).*cos(gen(1)*t_des_hist) - ...
            [amp_vec(1)*cos(f_vec(1)*t_des_hist), ...
            amp_vec(2)*cos(f_vec(2)*t_des_hist), ...
            amp_vec(3)*cos(f_vec(3)*t_des_hist), ...
            amp_vec(4)*cos(f_vec(4)*t_des_hist),...
            amp_vec(5)*cos(f_vec(5)*t_des_hist), ...
            amp_vec(6)*cos(f_vec(6)*t_des_hist), ...
            amp_vec(7)*cos(f_vec(7)*t_des_hist),...
            amp_vec(8)*cos(f_vec(8)*t_des_hist)].*gen(1)^2.*sin(gen(1)*t_des_hist);
        
        
        x_des_hist = [x_des_hist, xd_des_hist];
        
        %         xdd_des_hist = [diff(xd_des_hist); zeros(1,5)]/dt;  % accelerations
        
    case 'Sinusoid'
        amp_vec = 0.1*[2, 1, 1, deg2rad(1), deg2rad(2), deg2rad(5), deg2rad(12), deg2rad(7)];
        f_vec = [.2, .3, .4, .01, .05, 0.1, 1.3, 4];
        x_des_hist = [amp_vec(1)*sin(f_vec(1)*t_des_hist), ...
            amp_vec(2)*sin(f_vec(2)*t_des_hist), ...
            amp_vec(3)*sin(f_vec(3)*t_des_hist), ...
            amp_vec(4)*sin(f_vec(4)*t_des_hist),...
            amp_vec(5)*sin(f_vec(5)*t_des_hist), ...
            amp_vec(6)*sin(f_vec(6)*t_des_hist), ...
            amp_vec(7)*sin(f_vec(7)*t_des_hist),...
            amp_vec(8)*sin(f_vec(8)*t_des_hist)];
        
        xd_des_hist = [amp_vec(1)*f_vec(1)*cos(f_vec(1)*t_des_hist), ...
            amp_vec(2)*f_vec(2)*cos(f_vec(2)*t_des_hist), ...
            amp_vec(3)*f_vec(3)*cos(f_vec(3)*t_des_hist), ...
            amp_vec(4)*f_vec(4)*cos(f_vec(4)*t_des_hist),...
            amp_vec(5)*f_vec(5)*cos(f_vec(5)*t_des_hist), ...
            amp_vec(6)*f_vec(6)*cos(f_vec(6)*t_des_hist), ...
            amp_vec(7)*f_vec(7)*cos(f_vec(7)*t_des_hist),...
            amp_vec(8)*f_vec(8)*cos(f_vec(8)*t_des_hist)];
        
        xdd_des_hist = -[amp_vec(1)*f_vec(1)^2*sin(f_vec(1)*t_des_hist), ...
            amp_vec(2)*f_vec(2)^2*sin(f_vec(2)*t_des_hist), ...
            amp_vec(3)*f_vec(3)^2*sin(f_vec(3)*t_des_hist), ...
            amp_vec(4)*f_vec(4)^2*sin(f_vec(4)*t_des_hist),...
            amp_vec(5)*f_vec(5)^2*sin(f_vec(5)*t_des_hist), ...
            amp_vec(6)*f_vec(6)^2*sin(f_vec(6)*t_des_hist), ...
            amp_vec(7)*f_vec(7)^2*sin(f_vec(7)*t_des_hist),...
            amp_vec(8)*f_vec(8)^2*sin(f_vec(8)*t_des_hist)];
        
        x_des_hist = [x_des_hist, xd_des_hist];
        
    case 'veryRich'
        %% Rich, very rich
        %     x_des_hist =  zeros(num_setpoints, 10);
        %     x_des_hist(:,1:5) = repmat([0, 0, 0, 1.5, -1.5], num_setpoints, 1);
        % [r1 r2 r3, q0x q0y q0z, q1,q2, rd1 rd2 rd3, q0d1 q0d2 q0d3, q1d, q2d]
        %         x_des_hist = [r, q0, q1, q2]; 
        
        % Max amplitudes
        amp = 1*[1,1, .5, deg2rad([.5, .2, .3]), deg2rad([.6, .7])];
        % Define initial values 
        xd0 = zeros(1,8);
        x0 = zeros(1,8);
        xdd_des_hist = amp.*(round(rand(length(t_des_hist), 8))-0.5); 
        xd_des_hist = cumtrapz(t_des_hist, xdd_des_hist); 
        xd_des_hist = xd_des_hist + xd0;
        r =  cumtrapz(t_des_hist, xd_des_hist(:,1:3));
        q = eul_from_omegahist([1, 0, 0, 0], xd_des_hist(:,4:6), t_des_hist);
        qb = cumtrapz(t_des_hist, xd_des_hist(:,7:8));
        x_des_hist = [r, q, qb];
        
%         x_des_hist = cumtrapz(t_des_hist, xd_des_hist); 
        x_des_hist = x_des_hist + x0;
        
        % Obtain the full state
        x_des_hist = [x_des_hist, xd_des_hist];
%         xdd_des_hist = zeros(size(xd_des_hist));
        
end
ref_traj = [t_des_hist, x_des_hist xdd_des_hist];
end


%% Helping Functions
function [eul] = eul_from_omegahist(q0, wBvec, t)
%[qvec] = q_from_omegahist(q0, omega, t)
%Generates the time history of the quaternion vector
%   Quaternion is a Hamiltonian quaternion that describes the motion from
%   Body to Inertial. Scalar is first. Angular velocity vector is described
%   in the body frame axes.
%
% Inputs
% - q0      [4x1] Initial quaternion from body to inertial. Scalar First
% - wvec    [nx3] Angular velocity vector history expressed in body-fixed
%           axis
% - t       [nx1] Time history 

qvec = nan(size(wBvec,1),4);

qvec(1, :)  = q0(:)';

eul = nan(size(wBvec,1),3);


eul0 = quat2eul(q0(:)', 'XYZ');
eul(1, :)  = eul0;

% 
% qvec2 = nan(size(wBvec,1),4);
% 
% qvec2(1, :)  = q0(:)';

for ii = 2:size(wBvec,1)
    % Using Exponential Matrix
    q_minus = qvec(ii-1,:)'; % [4x1] previous quaternion
    
    wB_minus = wBvec(ii, :)';
    
    S_w = [0 -wB_minus(1) -wB_minus(2) -wB_minus(3);
    wB_minus(1) 0 wB_minus(3) -wB_minus(2);
    wB_minus(2) -wB_minus(3) 0 wB_minus(1);
    wB_minus(3) wB_minus(2) -wB_minus(1) 0];

    
    delta_t = t(ii) - t(ii-1);
    expquat = expm(S_w*delta_t);
    
    q_plus = expquat*q_minus;
    
    qvec(ii, :) = q_plus(:)';    
    eul(ii,:) = quat2eul(q_plus(:)', 'XYZ');

end


end
