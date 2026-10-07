classdef CacheChassisStateSpy < handle
    properties
        cornerDisplacement = struct('FL',0,'FR',0,'RL',0,'RR',0)
        cornerVelocity = struct('FL',0,'FR',0,'RL',0,'RR',0)
        frontRollAngle = 0
        rearRollAngle = 0
        rollAngle = 0
        frontGeometricLateralLoadTransfer = 0
        rearGeometricLateralLoadTransfer = 0
        additionalLongitudinalLoadTransfer = 0
        frontAdditionalLateralLoadTransfer = 0
        rearAdditionalLateralLoadTransfer = 0
    end
end
