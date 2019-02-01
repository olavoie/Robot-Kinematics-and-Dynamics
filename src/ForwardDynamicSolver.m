function acceleration = ForwardDynamicSolver(joint, jointVelocity, tau)
    % Robot Frame
    robotData = RobotParam;
    
    % Mass Center
    Pc(:, 1) = [ -3.94e-06;  0.001804699;  -0.0748562];
    Pc(:, 2) = [2.2339e-05;   0.07515658;  -0.0301928];
    Pc(:, 3) = [-2.234e-05;  0.009582299;  -0.0966774];
    Pc(:, 4) = [-3.269e-05;   0.09546485;  -0.0287837];
    Pc(:, 5) = [6.4423e-05;  -0.00987523;  -0.0607337];
    Pc(:, 6) = [5.6819e-05;   0.04279498; -0.00982194];
    Pc(:, 7) = [         0;       -2e-05;    -0.01702];

    % Mass
    m = [1.290800 1.344200 1.344200 1.094520 0.590200 0.590200 0.239480];

    % Inertia
    I(:, :, 1) = [101, 0, 0; 0, 101, 0; 0, 0, 101];
    I(:, :, 2) = [102, 0, 0; 0, 102, 0; 0, 0, 102];
    I(:, :, 3) = [103, 0, 0; 0, 103, 0; 0, 0, 103];
    I(:, :, 4) = [104, 0, 0; 0, 104, 0; 0, 0, 104];
    I(:, :, 5) = [105, 0, 0; 0, 105, 0; 0, 0, 105];
    I(:, :, 6) = [106, 0, 0; 0, 106, 0; 0, 0, 106];
    I(:, :, 7) = [107, 0, 0; 0, 107, 0; 0, 0, 107];

    R = zeros(3, 3, robotData.mActuatorCount);
    P = zeros(3, robotData.mActuatorCount);

    % Robot referential
    for n = 1:robotData.mActuatorCount
        T = robotData.mJointReferentiel(:, :, n) * rotm2tform(rotz(joint(n) + robotData.mJointOffset(n)));
        R(:, :, n) = tform2rotm(T);
        P(:, n)    = tform2trvec(T);
    end
    
    R(:, :, robotData.mActuatorCount + 1) = [1, 0, 0; 0, 1, 0; 0, 1, 0];
    P(:, robotData.mActuatorCount + 1)    = [0; 0; 0];

    C = InverseDynamicSolver(R, P, Pc, I, m, jointVelocity, [0; 0; 0; 0; 0; 0; 0; 0], robotData.mActuatorCount)';

    for n = 1:robotData.mActuatorCount
        a = [0; 0; 0; 0; 0; 0; 0];
        a(n) = 1;
        M(:, n) = InverseDynamicSolver(R, P, Pc, I, m, jointVelocity, a, robotData.mActuatorCount)' - C;
    end 
    acceleration = inv(M) * (tau - C);
end