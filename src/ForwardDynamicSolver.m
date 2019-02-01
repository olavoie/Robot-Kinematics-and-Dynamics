function acceleration = ForwardDynamicSolver(joint, jointVelocity, tau)
    % Robot Frame
    robotData = RobotParam;
    
    % Mass Center
    Pc(:, 1) = [0; 0; 0];
    Pc(:, 2) = [0; 0; 0];
    Pc(:, 3) = [0; 0; 0];
    Pc(:, 4) = [0; 0; 0];
    Pc(:, 5) = [0; 0; 0];
    Pc(:, 6) = [0; 0; 0];
    Pc(:, 7) = [0; 0; 0];

    % Mass
    m = [0 0 0 0 0 0 0];

    % Inertia
    I(:, :, 1) = [0, 0, 0; 0, 0, 0; 0, 0, 0];
    I(:, :, 2) = [0, 0, 0; 0, 0, 0; 0, 0, 0];
    I(:, :, 3) = [0, 0, 0; 0, 0, 0; 0, 0, 0];
    I(:, :, 4) = [0, 0, 0; 0, 0, 0; 0, 0, 0];
    I(:, :, 5) = [0, 0, 0; 0, 0, 0; 0, 0, 0];
    I(:, :, 6) = [0, 0, 0; 0, 0, 0; 0, 0, 0];
    I(:, :, 7) = [0, 0, 0; 0, 0, 0; 0, 0, 0];

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
