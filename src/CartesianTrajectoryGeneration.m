[positionX, velocityX, accelerationX] = Trajectory(0.5643, 0.7, 1);
[positionY, velocityY, accelerationY] = Trajectory(0.2250, 0.0, 1);
[positionZ, velocityZ, accelerationZ] = Trajectory(0.3080, 0.0, 1);
         
inverseKinematic = InverseKinematic;
jointAngleStart = [175.0; 145.0; 178.0; 90.0; 180.0; 215.0; 260.0];
jointAngle      = jointAngleStart;

for i = 1:length(positionX)
    endPose = [[-0.1135   -0.0221   -0.9933    positionX(i)];
               [ 0.9746    0.1921   -0.1156    positionY(i)];
               [ 0.1933   -0.9811   -0.0003    positionZ(i)];
               [      0         0         0          1.0000]];
    jointAngle = inverseKinematic.ComputeInverseKinematic(jointAngle, endPose);
end

for i = 1:7
    [position(i, :), velocity(i, :), acceleration(i, :)] = Trajectory(jointAngleStart(i), jointAngle(i), 1);
end
