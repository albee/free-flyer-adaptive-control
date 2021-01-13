% Compute adaptive control for the robot
% Configured for 3D!
function u = freeflyer_pd_control(x_des, x, xdd_des, ff, ff_model)
    x = x(1:16);  % shave off parameter
    x_err = x - x_des;
   
    if ff.robot.n_q == 1
        u = -0.1*[eye(4) eye(4)]*x_err;
    elseif ff.robot.n_q == 2
        %% PD position controller
        Kp = 10*diag([3, 3, 3, 0.1, 0.1, 0.1, 1.5, 1.5]);
        Kd = 5.0*eye(8);
               
        u = -[Kp, Kd]*x_err;  % u = [F1 F2 tau0 tau1 tau2] INERTIAL
        % u = [F1, F2, F3, tau0x, tau0y, tau0z, tau1, tau2] INERTIAL
    elseif ff.robot.n_q == 3
        u = -0.005*[eye(6) eye(6)]*x_err; 
end