classdef RobotParam
    properties
        mActuatorCount = 7;
        mJointOffset = [180, 180, 180, 180, 180, 180, 0.0]; 
        
        mT01 = RobotParam.HomogeniousMatrix(pi, 0, 0 , 0, 0, 0.09275);
        mT12 = RobotParam.HomogeniousMatrix(-pi/2, 0,  0, 0, -0.04, -0.12875);
        mT23 = RobotParam.HomogeniousMatrix(pi/2, 0, 0 , 0, 0.1725, -0.04);
        mT34 = RobotParam.HomogeniousMatrix(-pi/2, 0, 0 , 0, -0.04, -0.1725);
        mT45 = RobotParam.HomogeniousMatrix(pi/2, 0, 0 , 0, 0.24125, -0.04);
        mT56 = RobotParam.HomogeniousMatrix(-pi/2, 0, 0 , 0, 0, -0.10375);
        mT67 = RobotParam.HomogeniousMatrix(pi/2, 0, 0 , 0, 0.10375, 0);
        
        mJointReferentiel;
    end
    methods
        function obj = RobotParam
            obj.mJointReferentiel        = obj.mT01;
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
