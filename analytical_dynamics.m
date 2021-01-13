function [Y_lin, Y_aff] = analytical_dynamics(filename, space)
    % filename = string with filename of model
    % space = '2D' or '3D' (whether we want planar or 6DOF)

    fprintf('Converting robot model . . . \n')
    [robot] = urdf_to_spart_model(filename);
    ff = spart_free_flyer_dynamics(robot);

    m3 = sym('m3',[1,1],'real');
    ff.robot.links(end).mass = m3;  % unknown mass at last link


    r = sym('r', [3,1], 'real'); % [x; y; z]
    q0=sym('q0', [3,1], 'real'); % [q01x, q02y, q03z]' (attitude angles, RPY body-fixed)

    %Base-link position
    q1=sym('q1',[1],'real');
    q2=sym('q2',[1],'real');

    % general velocities
    rd = sym('rd', [3,1], 'real');
    w0=sym('w0', [3,1], 'real');
    q1d=sym('q1d',[1],'real');
    q2d=sym('q2d',[1],'real');


    % reference trajectory
    rd_r=sym('rd_r', [3,1], 'real');
    w0_r=sym('w0_r', [3,1], 'real');
    q1d_r=sym('q1d_r',[1],'real');
    q2d_r=sym('q2d_r',[1],'real');


    rdd=sym('rdd', [3,1], 'real');
    w0d=sym('w0d', [3,1], 'real');
    q1dd=sym('q1dd',[1],'real');
    q2dd=sym('q2dd',[1],'real');

    switch space
        case '2D'
            r = subs(r, r(3), 0);
            rd = subs(rd, rd(3), 0);
            rd_r = subs(rd_r, rd_r(3), 0);
            rdd = subs(rdd, rdd(3),0);
            q0 = subs(q0, [q0(1), q0(2)], [0, 0]);
            w0 = subs(w0, [w0(1), w0(2)], [0, 0]);
            w0_r = subs(w0_r, [w0_r(1), w0_r(2)], [0, 0]);
            w0d = subs(w0d, [w0d(1), w0d(2)], [0, 0]);

            xd = [r(1:2); w0(end); q1d; q2d];
            xd_r = [rd_r(1:2); w0_r(end); q1d_r; q2d_r];
            xdd_r = [rdd(1:2); w0d(end); q1dd; q2dd];

            idx = [4 5 3 7 8];
        case '3D'
            xd = [r; w0; q1d; q2d];
            xd_r = [rd_r; w0_r; q1d_r; q2d_r];
            xdd_r = [rdd; w0d; q1dd; q2dd];

            idx = [4 5 6 1 2 3 7 8];
    end


    ff.r0 = r;
%     ff.R0 = Angles321_DCM(q0')';
    ff.R0 = Angles123_DCM(q0')';
    ff.qm=[q1; q2];

    ff.calc_fk();
    ff.calc_vel_kinematics();

    ff.u0=[w0', rd']';       % base linear and angular velocities
    ff.um=[q1d; q2d]; % joint velocities

    ff.calc_twist();

    H = ff.calc_GIM;
    C = ff.calc_CIM;

    % This will be most challenging thing to do ------
    % excelent

    % Keenan dynamics is of the form [rd, w, qd1,qd2]
    C = C(idx, idx);
%     size(C)
    H = H(idx, idx);
%     size(H)
    Y = collect(H*xdd_r + C*xd_r, m3); % should be needed
%     Y = H*xdd_r + C*xd_r;

    Y_lin = (Y - subs(Y, m3, 0))/m3;  % linear portion
    Y_aff = subs(Y, m3, 0);  % affine portion
    
    xvec = [r', q0', q1, q2, rd', w0', q1d, q2d, rd_r', w0_r', q1d_r, q2d_r, rdd', w0d', q1dd, q2dd];
    % xvec
    % [ r1, r2, r3, q01, q02, q03, q11, q21, rd1, rd2, rd3, w01, w02, w03, q1d1, q2d1, rd_r1, rd_r2, rd_r3, w0_r1, w0_r2, w0_r3, q1d_r1, q2d_r1, rdd1, rdd2, rdd3, w0d1, w0d2, w0d3, q1dd1, q2dd1]

    % fix for 2D case
    xvec = xvec(xvec ~= 0);

    fprintf('Creating sym functions . . . (1/2)\n')
%     Y_lin = simplify(Y_lin);
%     Y_aff = simplify(Y_aff);

    Y_lin = symfun(Y_lin, xvec);
    Y_aff = symfun(Y_aff, xvec);

    fprintf('Writing MATLAB functions . . . (1/2)\n')
    Y_lin = matlabFunction(Y_lin);
    save('Y_linsimp3.mat', 'Y_lin')
    fprintf('Writing Matlab functions . . . (2/2)\n')
    Y_aff = matlabFunction(Y_aff);
    save('Y_affsimp3.mat', 'Y_aff')

%     Y(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
%     Y(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1)
end
