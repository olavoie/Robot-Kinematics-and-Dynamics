classdef InverseKinematic
    properties
        mMaxIteration     = 200;
        mEpsilon          = 0.0001;
        mForwardKinematic = ForwardKinematic;
    end
    methods
        function joint = ComputeInverseKinematic(obj, actualJoint, desiredIncrementPose)
            i = 0;
            joint = actualJoint;
            forwardKinematic = obj.mForwardKinematic.updateJointFrame(joint);
            forwardKinematic.mLastFrame;
            minNorm = norm(obj.ComputeCartesianIncrementVectorFromPose(forwardKinematic.mLastFrame, desiredIncrementPose));
            while (minNorm > obj.mEpsilon && i < obj.mMaxIteration)
                joint = joint + pinv(forwardKinematic.mJacobian) * obj.ComputeCartesianIncrementVectorFromPose(forwardKinematic.mLastFrame, desiredIncrementPose);
                forwardKinematic = obj.mForwardKinematic.updateJointFrame(joint);
                minNorm = norm(obj.ComputeCartesianIncrementVectorFromPose(forwardKinematic.mLastFrame, desiredIncrementPose));
                i = i + 1;
            end
        end
        
        function cartesianIncrementVector = ComputeCartesianIncrementVectorFromPose(obj, pose1, pose2)
            rot1 = tform2rotm(pose1);
            rot2 = tform2rotm(pose2);
            v1   = tform2trvec(pose1);
            v2   = tform2trvec(pose2);
            
            omegaX = rot2(3,1) * rot1(2,1) + rot2(3,2) * rot1(2,2) + rot2(3,3) * rot1(2,3);
            omegaY = rot2(1,1) * rot1(3,1) + rot2(1,2) * rot1(3,2) + rot2(1,3) * rot1(3,3);
            omegaZ = rot2(2,1) * rot1(1,1) + rot2(2,2) * rot1(1,2) + rot2(2,3) * rot1(1,3);
            
            cartesianIncrementVector = [v2 - v1, omegaX, omegaY, omegaZ]';
        end
    end
end