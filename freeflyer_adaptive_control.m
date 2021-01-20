% Compute adaptive control for the robot
% Configured for 3D!
function u = freeflyer_adaptive_control(x_des, x, xdd_des, ff, ff_model, Y_lin, Y_aff)
    x = x(1:16);  % shave off parameter
    x_err = x - x_des;
   
    if ff.robot.n_q == 1
        u = -0.1*[eye(4) eye(4)]*x_err;
    elseif ff.robot.n_q == 2        
        %% Adaptive controller (single unknown, m3)
        lambda = 1.0;
        gamma = 1000.0;
        Kds = 5.0*eye(8);
        
        xd_r = x_des(9:end) - 2*lambda*x_err(1:8);
        xdd_r = xdd_des - 2*lambda*x_err(9:end);
        s = x_err(9:end) + 2*lambda*x_err(1:8);
        
%         Y_lin_t = Y_lin(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), xd_r(1), xd_r(2), xd_r(3), xd_r(4), xd_r(5), xdd_r(1), xdd_r(2), xdd_r(3), xdd_r(4), xdd_r(5));
%         Y_lin_t = double(Y_lin_t);
        
        Y_lin_t = Y_linF(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10), ...
            x(11),x(12),x(13),x(14),x(15),x(16),xd_r(1),xd_r(2),xd_r(3),xd_r(4),...
            xd_r(5),xd_r(6),xd_r(7),xd_r(8),xdd_r(1),xdd_r(2),xdd_r(3),...
            xdd_r(4),xdd_r(5),xdd_r(6),xdd_r(7),xdd_r(8));
        Y_lin_t = double(Y_lin_t);
        
%         Y_aff_t = Y_aff(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), xd_r(1), xd_r(2), xd_r(3), xd_r(4), xd_r(5), xdd_r(1), xdd_r(2), xdd_r(3), xdd_r(4), xdd_r(5));
%         Y_aff_t = double(Y_aff_t);
%         
        Y_aff_t = Y_affF(x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10), ...
            x(11),x(12),x(13),x(14),x(15),x(16),xd_r(1),xd_r(2),xd_r(3),xd_r(4),...
            xd_r(5),xd_r(6),xd_r(7),xd_r(8),xdd_r(1),xdd_r(2),xdd_r(3),...
            xdd_r(4),xdd_r(5),xdd_r(6),xdd_r(7),xdd_r(8));
        Y_aff_t = double(Y_aff_t);
        
        u = Y_lin_t*ff_model.a_hat + Y_aff_t - Kds*s;
        a_hatd = -gamma*Y_lin_t'*s; % adaptation law
        ff_model.a_hatd = a_hatd;
    elseif ff.robot.n_q == 3
        u = -0.005*[eye(6) eye(6)]*x_err; 
end