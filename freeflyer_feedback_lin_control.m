% Compute adaptive control for the robot
% Configured for 3D!
function u = freeflyer_feedback_lin_control(x_des, x, xdd_des, ff, ff_model)
    x = x(1:16);  % shave off parameter
    x_err = x - x_des;
   
    if ff.robot.n_q == 1
        u = -0.1*[eye(4) eye(4)]*x_err;
    elseif ff.robot.n_q == 2        
        %% Trajectory controller (feedback linearization)
        Kp = 1*diag([3, 3, 3, 1, 1, 1, 1.5, 1.5]);
        Kp = 3*eye(8);
         Kd = 3*eye(8);
        Kp = 10*diag([3, 3, 3, 0.1, 0.1, 0.1, .15, .15]);
        Kd = 5.0*eye(8);
         
%          Kp = diag([3, 3, 3, .1, 0.1, 0.1, .15, .15]);
%         Kp = 1*eye(8);
%          Kd = diag([3, 3, 3, .1, 0.1, 0.1, .15, .15]);
        
        % Use system model (best estimate of params)
        C = ff_model.calc_CIM();
        H = ff_model.calc_GIM();
        
        % Convert to Keenan convention: make 2D, rearrange state order
        C = [C(4,:);   % rd1
             C(5,:);   % rd2
             C(6,:);   % rd3
             C(1,:);   % w_x
             C(2,:);   % w_y
             C(3,:);   % w_z
             C(7,:);   % qd1
             C(8,:)];  % qd2
        C = [C(:,4), C(:,5), C(:,6), C(:,1), C(:,2), C(:,3), C(:,7), C(:,8)];  % [F1 F2 tau0 tau1 tau2] 
        
        H = [H(4,:);   % rd1
             H(5,:);   % rd2
             H(6,:);   % rd3
             H(1,:);   % w_x
             H(2,:);   % w_y
             H(3,:);   % w_z
             H(7,:);   % qd1
             H(8,:)];  % qd2
        H = [H(:,4), H(:,5), H(:,6), H(:,1), H(:,2), H(:,3), H(:,7), H(:,8)];  % [F1 F2 tau0 tau1 tau2] 
        
        v = x(1:8);
%         v = x(9:16);
        
        u = C*v + H*(-[Kp, Kd]*x_err + xdd_des);        
    elseif ff.robot.n_q == 3
        u = -0.005*[eye(6) eye(6)]*x_err; 
end