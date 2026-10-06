classdef PrescribedChassis < handle
    properties
        state = struct('cornerDisplacement', struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0), ...
            'cornerVelocity', struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0))
    end
    methods
        function computeCornerKinematics(~)
        end
    end
end
