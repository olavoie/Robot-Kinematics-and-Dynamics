function [position, velocity, acceleration] = Trajectory(startPosition, endPosition, time)
    timeF  = time;
    S      = [startPosition; 0; 0; endPosition; 0; 0];
    M      = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 2 0 0 0; 1 timeF timeF^2 timeF^3 timeF^4 timeF^5; 0 1 2*timeF 3*timeF^2 4*timeF^3 5*timeF^4; 0 0 2 (3*2)*timeF 4*3*timeF^2 5*4*timeF^3];

    a = M^-1 * S;

    i = 1;
    for t = 0:0.001:timeF
        position(i)     = a(1) + a(2) * t + a(3) * t^2 + a(4) * t^3 + a(5) * t^4 + a(6) * t^5;
        velocity(i)     = a(2) + 2 * a(3) * t + 3 * a(4) * t^2 + 4 * a(5) * t^3 + 5 * a(6) * t^4;
        acceleration(i) = 2 * a(3) + (3 * 2) * a(4) * t + (4 * 3) * a(5) * t^2 + (4 * 5) * a(6) * t^3;
        i = i + 1;
    end
end