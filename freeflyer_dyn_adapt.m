function dxdt = freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, Y_lin, Y_aff, CONTROLLER, SPACE)
    % 1-, 2-, or 3-link free-flying manipulator dynamics (no gravity)
    %
    % Inputs:
    % t         (1x1) time vector
    % x         (17x1) state-param vector [r1 r2 r3 q0x q0y q0z q1 q2 r1d r2d q0yd q0xd q0zd q1d q2d a_hat]'
    %                              [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d a_hat]'
    % ff        free-flyer, real
    % ff_model  free-flyer, internal model
    % ref_traj  reference trajectory
    % Y         nonlinear parameterized vector
    % SPACE     ['2D', '3D'] interchangeability
    %
    % Output:
    % dxdt      (17x1) derivative of state vector [r1d r2d q0d q1d r1dd r2dd q0dd q1dd]'
    %                                      [r1d r2d q0d q1d q2d r1dd r2dd q0dd q1dd q2dd a_hatd]'
    
%     fprintf('Sim time is : %4.2f\n', t)
    
    % Unpack state vec 
    if ff.robot.n_q == 1
        r1 = x(1);
        r2 = x(2);
        q0 = x(3);
        q1 = x(4);
        r1d = x(5);
        r2d = x(6);
        q0d = x(7);
        q1d = x(8);
                
        % Set ff params
        ff.r0 = [r1; r2; 0];
        ff.R0 = eul2rotm([q0, 0, 0]);
        ff.qm = [q1];
        ff.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff.um = [q1d];
        
        ff_model.r0 = [r1; r2; 0];
        ff_model.R0 = eul2rotm([q0, 0, 0]);
        ff_model.qm = [q1; q2];
        ff_model.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff_model.um = [q1d; q2d];
        
    elseif ff.robot.n_q == 2 %% COMPLETED 
        % x = [r1 r2 r3, q0x q0y q0z, q1,q2, rd1 rd2 rd3, q0d1 q0d2 q0d3, q1d, q2d]
        r = x(1:3); r = r(:); % Ensure vector form
        q0 = x(4:6); q0 = q0(:); 
        q1 = x(7); q2 = x(8);
        rd = x(9:11); rd = rd(:);
        q0d = x(12:14); q0d = q0d(:);
        q1d = x(15); q2d = x(16); 
        
        if strcmp(CONTROLLER, 'adaptive')
            ff_model.a_hat = x(17); 
        end
        
        % Set ff params
        switch SPACE
            case '2D'
                ff.r0 = [r(1:2); 0];
                ff.R0 = eul2rotm([q0(3), 0, 0]);
                ff.qm = [q1; q2];
                ff.u0 = [0; 0; q0d(3); rd(1); rd(2); 0];
                ff.um = [q1d; q2d];
            case '3D'     
                ff.r0 = r;
%                 ff.R0 = eul2rotm(q0(end:-1:1)'); % NOTE eul2rot convention is ZYX
                ff.R0 = eul2rotm(q0', 'XYZ');
                ff.qm = [q1; q2];
                ff.u0 = [q0d; rd];
                ff.um = [q1d; q2d];
                
        end
        
        ff_model.r0 = ff.r0;
        ff_model.R0 = ff.R0;
        ff_model.qm = ff.qm;
        ff_model.u0 = ff.u0;
        ff_model.um = ff.um;
        
    elseif ff.robot.n_q == 3
        r1 = x(1);
        r2 = x(2);
        q0 = x(3);
        q1 = x(4);
        q2 = x(5);
        q3 = x(6);
        r1d = x(7);
        r2d = x(8);
        q0d = x(9);
        q1d = x(10);
        q2d = x(11);
        q3d = x(12);
        
        % Set ff params
        ff.r0 = [r1; r2; 0];
        ff.R0 = eul2rotm([q0, 0, 0]);
        ff.qm = [q1; q2; q3];
        ff.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff.um = [q1d; q2d; q3d];
        
        ff_model.r0 = [r1; r2; 0];
        ff_model.R0 = eul2rotm([q0, 0, 0]);
        ff_model.qm = [q1; q2; q3];
        ff_model.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff_model.um = [q1d; q2d; q3d];
    end

    %% Desired traj.
    [x_des, xdd_des] = lookup_ref_traj(t, ff, ref_traj);
    
    switch SPACE
        case '2D'
            idx = [3, 4, 5];
            x_des([idx, idx+8]) =zeros(length([idx, idx+8]), 1);
            xdd_des(idx) = zeros(length(idx), 1);
        case '3D'
    end

    %% Control
    if strcmp(CONTROLLER, 'adaptive')
        control = freeflyer_adaptive_control(x_des, x, xdd_des, ff, ff_model, Y_lin, Y_aff);  % u = [F1 F2 tau0 tau1 tau2] INERTIAL
        a_hatd = ff_model.a_hatd;
    elseif strcmp(CONTROLLER, 'feedback_lin')
        control = freeflyer_feedback_lin_control(x_des, x, xdd_des, ff, ff_model);  % u = [F1 F2 tau0 tau1 tau2] INERTIAL
    elseif strcmp(CONTROLLER, 'pd')
        control = freeflyer_pd_control(x_des, x, xdd_des, ff, ff_model);  % u = [F1 F2 tau0 tau1 tau2] INERTIAL
    end
    
    %% Dynamics, using the SPART interface for forward dynamics
    switch SPACE
        
        case '2D'
            % Set wrenches (F/T in INERTIAL frame)
            ff.wF0 = [0; 0; control(6); control(1); control(2); 0];  % torques and forces to base, in INERTIAL frame
            ff.wFm = zeros(6,ff.robot.n_links_joints);               % torques to links, in INERTIAL frame (includes base again)
            % Set generalized forces (F/T in LINK frames)
            ff.tauq0 = zeros(6,1);                                   % torques and forces to base, in LINK frames
            ff.tauqm = [control(7:end)];                             % torques to links, in LINK frames
        case '3D'
            % Set wrenches (F/T in INERTIAL frame)
            ff.wF0 = [control(4); control(5); control(6); control(1); control(2); control(3)];  % torques and forces to base, in INERTIAL frame
            ff.wFm = zeros(6,ff.robot.n_links_joints);               % torques to links, in INERTIAL frame (includes base again)
            % Set generalized forces (F/T in LINK frames)
            ff.tauq0 = zeros(6,1);                                   % torques and forces to base, in LINK frames
            ff.tauqm = [control(7:end)];                             % torques to links, in LINK frames
            
    end
    % Calculate FD on (simulated) real system
    ff.calc_fd();
    
    % Grab accelerations
    if ff.robot.n_q == 1
        r1dd = ff.u0dot_FD(4);
        r2dd = ff.u0dot_FD(5);
        q0dd = ff.u0dot_FD(3);
        q1dd = ff.umdot_FD(1);
        
        % Output derivative of state vector
        dxdt = [r1d; r2d; q0d; q1d; r1dd; r2dd; q0dd; q1dd];
    elseif ff.robot.n_q == 2
        q0dd = ff.u0dot_FD(1:3);
        rdd  = ff.u0dot_FD(4:6);
        q1dd = ff.umdot_FD(1);
        q2dd = ff.umdot_FD(2);
        % Output derivative of state vector
        if strcmp(CONTROLLER, 'adaptive')
%             dxdt = [r1d; r2d; q0d; q1d; q2d; r1dd; r2dd; q0dd; q1dd; q2dd; a_hatd];
            dxdt = [rd; q0d; q1d; q2d; rdd; q0dd; q1dd; q2dd; a_hatd];
        elseif strcmp(CONTROLLER, 'feedback_lin') || strcmp(CONTROLLER, 'pd')
%             dxdt = [r1d; r2d; q0d; q1d; q2d; r1dd; r2dd; q0dd; q1dd; q2dd];
            dxdt = [rd; q0d; q1d; q2d; rdd; q0dd; q1dd; q2dd];
        end
        
    elseif ff.robot.n_q == 3
        r1dd = ff.u0dot_FD(4);
        r2dd = ff.u0dot_FD(5);
        q0dd = ff.u0dot_FD(3);
        q1dd = ff.umdot_FD(1);
        q2dd = ff.umdot_FD(2);
        q3dd = ff.umdot_FD(3);
        % Output derivative of state vector
        dxdt = [r1d; r2d; q0d; q1d; q2d; q3d; r1dd; r2dd; q0dd; q1dd; q2dd; q3dd];
    end
end

