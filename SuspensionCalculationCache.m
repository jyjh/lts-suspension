classdef SuspensionCalculationCache < handle
    % Value-model copies may share this cache: every entry is keyed by all
    % inputs and no entry retains a component or its changing physics state.
    properties
        boundInputs = []
        boundStep = NaN
        chassisClass = ''
        hasChassisState = false
        stateClass = ''
        stateCapabilities = struct()
    end
end
