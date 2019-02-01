classdef RobotParam
    properties
        mActuatorCount = 7;
        mJointOffset = [180, 180, 180, 180, 180, 180, 0.0]; 
        
        mT01 = RobotParam.HomogeniousMatrix(0, 0, 0 , 0, 0, 0);
        mT12 = RobotParam.HomogeniousMatrix(0, 0,  0, 0, 0, 0);
        mT23 = RobotParam.HomogeniousMatrix(0, 0, 0 , 0, 0, 0);
        mT34 = RobotParam.HomogeniousMatrix(0, 0, 0 , 0, 0, 0);
        mT45 = RobotParam.HomogeniousMatrix(0, 0, 0 , 0, 0, 0);
        mT56 = RobotParam.HomogeniousMatrix(0, 0, 0 , 0, 0, 0);
        mT67 = RobotParam.HomogeniousMatrix(0, 0, 0 , 0, 0, 0);
        
        mJointReferentiel = zeros(4,4,7);
    end
    methods
        function obj = RobotParam
            obj.mJointReferentiel(:,:,1) = obj.mT01;
            obj.mJointReferentiel(:,:,2) = obj.mT12;
            obj.mJointReferentiel(:,:,3) = obj.mT23;
            obj.mJointReferentiel(:,:,4) = obj.mT34;
            obj.mJointReferentiel(:,:,5) = obj.mT45;
            obj.mJointReferentiel(:,:,6) = obj.mT56;
            obj.mJointReferentiel(:,:,7) = obj.mT67;
        end
    end
    
    methods(Static)
        function T = HomogeniousMatrix(r, p, y1, x, y, z)
            eul = [y1 p r];
            T = rotm2tform(eul2rotm(eul)) + [0 0 0 x; 0 0 0 y; 0 0 0 z; 0 0 0 0];   
        end
    end
end
