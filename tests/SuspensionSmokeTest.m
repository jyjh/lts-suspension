function tests = SuspensionSmokeTest
tests = functiontests(localfunctions);
end

function vm = vehicleFixture()
% Plain struct with exactly the geometry fields the constructors read —
% proves the package works without the main repository's classes.
vm = struct( ...
    'totalMass', 264, ...
    'wheelbase', 1.558, ...
    'trackWidth', 1.21, ...
    'cgHeight', 0.30, ...
    'staticFrontWeight', 0.5038, ...
    'g', 9.80665);
end

function mgr = managerFixture()
mgr = lts.components.Suspension.SuspensionManager(vehicleFixture(), ...
    43780, 3500, 4000, ...      % front spring, compression, rebound
    39400, 3200, 3600, ...      % rear spring, compression, rebound
    1.0, ...                    % motion ratio
    0.03, 350000, ...           % bump-stop length, rate
    190000, 9.3);               % tire spring rate, unsprung mass
end

function testConstructorBuildsFourCorners(testCase)
mgr = managerFixture();
corners = {mgr.frontLeft, mgr.frontRight, mgr.rearLeft, mgr.rearRight};
names = {'FL', 'FR', 'RL', 'RR'};
for i = 1:4
    verifyTrue(testCase, ~isempty(corners{i}), ...
        sprintf('corner %s missing', names{i}));
    verifyTrue(testCase, isfinite(corners{i}.springRate), ...
        sprintf('corner %s springRate not finite', names{i}));
    verifyGreaterThan(testCase, corners{i}.springRate, 0);
end
end

function testAxleRollStiffnessIsPositiveAndFinite(testCase)
% getAxleRollStiffness is the capability the chassis probes for; it must
% stay available and physical.
[KwF, KwR] = managerFixture().getAxleRollStiffness();
verifyTrue(testCase, isfinite(KwF) && isfinite(KwR));
verifyGreaterThan(testCase, KwF, 0);
verifyGreaterThan(testCase, KwR, 0);
end

function testGeometryLoadsFromVehicleStruct(testCase)
geom = lts.components.Suspension.SuspensionGeometry(vehicleFixture());
verifyEqual(testCase, geom.wheelbase, 1.558);
verifyEqual(testCase, geom.trackWidth, 1.21);
verifyEqual(testCase, geom.staticFrontWeight, 0.5038);
end
