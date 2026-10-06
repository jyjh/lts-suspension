function tests = AntiRollBarPhysicsTest
tests = functiontests(localfunctions);
end

function [s, chassis] = fixture(rate)
vm = struct('totalMass', 256, 'wheelbase', 1.558, 'trackWidth', 1.21, ...
    'cgHeight', 0.30, 'staticFrontWeight', 0.5, 'g', 9.80665);
s = lts.components.Suspension.SuspensionManager(vm, ...
    45000, 3000, 3000, 42000, 2800, 2800, 1, 0.025, 200000, 200000, 9, ...
    [], rate, rate);
s.warmup(vm.totalMass, 0.001);
chassis = PrescribedChassis();
end

function testCoupledWarmStartMatchesAnalyticalEquilibrium(testCase)
for rate = [0, 24000, 1e6]
    [s, c] = fixture(rate);
    z = 0.002;
    c.state.cornerDisplacement.FL = -z;
    c.state.cornerDisplacement.FR = z;
    loads = s.initializeCornerLoadsFromChassis(c, 0);
    k = s.frontRight.springRate;
    kt = s.frontRight.tireSpringRate;
    expected = kt * (k + 2 * rate) / (kt + k + 2 * rate) * z;
    verifyEqual(testCase, loads.FR - s.frontRight.state.staticLoad, expected, 'AbsTol', 1e-7);
    f = s.getAntiRollBarForces();
    verifyEqual(testCase, s.frontRight.state.antiRollBarForce, f.FR, 'AbsTol', 1e-7);
    verifyEqual(testCase, s.frontLeft.state.suspensionForce, loads.FL, 'AbsTol', 1e-7);
    verifyEqual(testCase, loads.FL + loads.FR, 2 * s.frontLeft.state.staticLoad, 'AbsTol', 1e-7);
end
end

function testWarmStartIncludesBumpStopAndUnequalWheelRates(testCase)
[s, c] = fixture(1e5);
s.frontLeft.springRate = 30000;
s.frontRight.springRate = 60000;
c.state.cornerDisplacement.FL = 0.035;
c.state.cornerDisplacement.FR = 0.055;
s.initializeCornerLoadsFromChassis(c, 0);
verifyGreaterThan(testCase, s.frontRight.state.damperPosition, s.frontRight.bumpStopLength);
f = s.getAntiRollBarForces();
verifyEqual(testCase, s.frontRight.state.antiRollBarForce, f.FR, 'AbsTol', 1e-7);
for unit = {s.frontLeft, s.frontRight}
    st = unit{1}.state;
    verifyEqual(testCase, st.suspensionForce, st.tireNormalForce, 'AbsTol', 1e-7);
end
end

function testPureHeaveDoesNotTwistBar(testCase)
[s, c] = fixture(1e6);
c.state.cornerDisplacement.FL = 0.01;
c.state.cornerDisplacement.FR = 0.01;
s.initializeCornerLoadsFromChassis(c, 0);
verifyEqual(testCase, s.frontLeft.state.antiRollBarForce, 0, 'AbsTol', 1e-8);
verifyEqual(testCase, s.frontRight.state.antiRollBarForce, 0, 'AbsTol', 1e-8);
end

function testWarmStartHandlesAnUnloadedInsideTire(testCase)
[s, c] = fixture(1e6);
c.state.cornerDisplacement.FL = -.012;
c.state.cornerDisplacement.FR = .012;
loads = s.initializeCornerLoadsFromChassis(c, 0);
verifyEqual(testCase, loads.FL, 0, 'AbsTol', 1e-8);
verifyGreaterThan(testCase, loads.FR, 0);
for unit = {s.frontLeft, s.frontRight}
    st = unit{1}.state;
    verifyEqual(testCase, st.suspensionForce, st.tireNormalForce, 'AbsTol', 1e-7);
end
force = s.getAntiRollBarForces();
verifyEqual(testCase, s.frontRight.state.antiRollBarForce, force.FR, 'AbsTol', 1e-7);
end

function testSharedSubstepsMatchResolvedRamp(testCase)
[coarse, c] = fixture(24000);
[fine, f] = fixture(24000);
c.state.cornerDisplacement.FL = -0.004;
c.state.cornerDisplacement.FR = 0.004;
c.state.cornerVelocity.FL = -0.4;
c.state.cornerVelocity.FR = 0.4;
coarse.computeCornerLoadsFromChassis(c, 0, 0.01);
for idx = 1:10
    f.state.cornerDisplacement.FL = -0.004 * idx / 10;
    f.state.cornerDisplacement.FR = 0.004 * idx / 10;
    f.state.cornerVelocity.FL = -0.4 * idx / 10;
    f.state.cornerVelocity.FR = 0.4 * idx / 10;
    fine.computeCornerLoadsFromChassis(f, 0, 0.001);
end
verifyEqual(testCase, coarse.frontRight.state.unsprungPosition, ...
    fine.frontRight.state.unsprungPosition, 'AbsTol', 1e-12);
forces = coarse.getAntiRollBarForces();
verifyEqual(testCase, coarse.frontRight.state.antiRollBarForce, forces.FR, 'AbsTol', 1e-10);
end

function testStiffBarResolvesAntiphaseWheelHop(testCase)
[s, c] = fixture(2e6);
verifyLessThan(testCase, s.getMaxIntegrationStep(), 0.001);
for unit = {s.frontLeft, s.frontRight}
    unit{1}.state.unsprungPosition = 1e-4;
end
s.frontLeft.maxIntegrationStep = 1e-5;
s.frontRight.maxIntegrationStep = 1e-5;
s.frontLeft.state.unsprungPosition = -1e-4;
s.computeCornerLoadsFromChassis(c, 0, 0);
omega2 = (45000 + 200000 + 4e6) / 9;
reference = expm([0, 1; -omega2, -3000/9] * 0.01) * [1e-4; 0];
s.computeCornerLoadsFromChassis(c, 0, 0.01);
verifyEqual(testCase, s.frontRight.state.unsprungPosition, reference(1), 'AbsTol', 2e-7);
verifyEqual(testCase, s.frontRight.state.unsprungVelocity, reference(2), 'AbsTol', 2e-4);
end

function testConstructorRejectsZeroArmAndAllowsAmplifyingRatio(testCase)
verifyError(testCase, @() lts.components.Suspension.AntiRollBar(1800, .95, 0, true), ...
    'lts_suspension_AntiRollBar:InvalidParameter');
bar = lts.components.Suspension.AntiRollBar(100, 2, .2, true);
verifyEqual(testCase, bar.getWheelRateStiffness(), 10000, 'RelTol', 1e-12);
bar = lts.components.Suspension.AntiRollBar(0, 1, 0, false);
verifyEqual(testCase, bar.getWheelRateStiffness(), 0);
end
