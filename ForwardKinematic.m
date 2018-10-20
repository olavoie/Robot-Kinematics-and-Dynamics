classdef ForwardKinematic
    properties
        mRobotParam = RobotParam 
    end
    methods
        function T07 = updateJointFrame(obj, jointAngles)
            obj.updateJoint(jointAngles, 1)
            T07 = obj.updateJoint(jointAngles, 1) * obj.updateJoint(jointAngles, 2) * obj.updateJoint(jointAngles, 3) * obj.updateJoint(jointAngles, 4) * obj.updateJoint(jointAngles, 5) * obj.updateJoint(jointAngles, 6) * obj.updateJoint(jointAngles, 7);
        end
        function T = updateJoint(obj, angle, joint)
            T = obj.mRobotParam.mJointReferentiel(:, :, joint) * rotm2tform(rotz(angle(joint) + obj.mRobotParam.mJointOffset(joint)));
        end
    end
end