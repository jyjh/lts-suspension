classdef DynamicChassisStateSpy < dynamicprops
    properties
        cornerDisplacement = struct('FL',0,'FR',0,'RL',0,'RR',0)
        cornerVelocity = struct('FL',0,'FR',0,'RL',0,'RR',0)
        rollAngle = 0
    end
end
