function tau = InverseDynamicSolver(R, P, Pc, I, m, jointVelocity, jointAcceleration, actuatorCount)
    vv(:, 1) = [0; 0; 9.81];
    w(:, 1)  = [0; 0;    0];
    ww(:, 1) = [0; 0; jointAcceleration(1)];

    for n = 1:(actuatorCount - 1)
       w(:, n + 1)   = R(:, :, n + 1)' * w(:, n) + [0; 0; jointVelocity(n + 1)];
       ww(:, n + 1)  = R(:, :, n + 1)' * ww(:, n) + cross(R(:, :, n + 1)' * w(:, n), [0; 0; jointVelocity(n + 1)]) + [0; 0; jointAcceleration(n + 1)];
       vv(:, n + 1)  = R(:, :, n + 1)' * (cross(ww(:, n), P(:, n + 1)) + cross(w(:, n), cross(w(:, n), P(:, n + 1))) + vv(:, n));
       vvc(:, n + 1) = (cross(ww(:, n + 1), Pc(:, n + 1)) + cross(w(:, n + 1), cross(w(:, n + 1), Pc(:, n + 1))) + vv(:, n + 1));
       F(:, n + 1)   = m(n) * vvc(:, n + 1);
       N(:, n + 1)   = I(:, :, n + 1) * ww(:, n + 1) + cross(w(:, n + 1), I(:, :, n + 1) * w(:, n + 1));
    end

    f(:, actuatorCount + 1)  = [0; 0; 0];
    nn(:, actuatorCount + 1) = [0; 0; 0];
    
    for n = actuatorCount:-1:1
        f(:, n)  = R(:, :, n + 1) * f(:, n + 1) + F(:, n);
        nn(:, n) = N(:, n) + R(:, :, n + 1) * nn(:, n + 1) + cross(Pc(:, n),F(:, n)) + cross(P(:, n + 1), R(:, :, n + 1) * f(:, n + 1));
        tau(n)   = nn(:, n)' * [0; 0; 1];
    end

end