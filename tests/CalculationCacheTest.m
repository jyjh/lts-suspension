function tests = CalculationCacheTest
tests = functiontests(localfunctions);
end

function s = fixture()
vm = struct('totalMass',256,'wheelbase',1.558,'trackWidth',1.21, ...
    'cgHeight',.3,'staticFrontWeight',.5,'g',9.80665);
s = lts.components.Suspension.SuspensionManager(vm, ...
    45000,3000,3000,42000,2800,2800,1,.025,200000,200000,9,[],2e6,2e6);
s.warmup(vm.totalMass,.001);
end

function testBoundInvalidatesForEveryPhysicalInput(testCase)
s = fixture();
verifyEqual(testCase,s.getMaxIntegrationStep(),s.getMaxIntegrationStep());
% Isolate one corner as the controlling mode, then change each independent
% stiffness/mass input. A stale bound can silently skip necessary substeps.
s.frontRight.unsprungMass = 100;
s.rearLeft.unsprungMass = 100;
s.rearRight.unsprungMass = 100;
base = s.getMaxIntegrationStep();
for field = {'springRate','tireSpringRate','bumpStopRate'}
    unit = s.frontLeft;
    old = unit.(field{1});
    unit.(field{1}) = old * 2;
    copy = s;
    copy.frontLeft = unit;
    verifyLessThan(testCase,copy.getMaxIntegrationStep(),base);
    verifyEqual(testCase,s.getMaxIntegrationStep(),base);
end
copy = s; copy.frontAntiRollBarWheelRate = 4e6;
verifyLessThan(testCase,copy.getMaxIntegrationStep(),base);
copy = s; copy.frontLeft.unsprungMass = 4.5;
verifyLessThan(testCase,copy.getMaxIntegrationStep(),base);
copy = s; copy.frontLeft.maxIntegrationStep = base / 2;
verifyEqual(testCase,copy.getMaxIntegrationStep(),base / 2);
s.frontLeft.state.motionRatioEffective = 2;
verifyLessThan(testCase,s.getMaxIntegrationStep(),base);
end

function testCapabilitiesKeepLiveValuesAndReplacementState(testCase)
s = fixture(); c = PrescribedChassis(); c.state = CacheChassisStateSpy();
s.chassis = c;
for transfer = [10,30]
    c.state.frontAdditionalLateralLoadTransfer = transfer;
    loads = s.computeCornerLoadsFromChassis(c,0,0);
    verifyEqual(testCase,loads.FR - loads.FL,2*transfer,'AbsTol',1e-10);
end
c.state = CacheChassisStateSpy();
c.state.frontAdditionalLateralLoadTransfer = 40;
loads = s.computeCornerLoadsFromChassis(c,0,0);
verifyEqual(testCase,loads.FR-loads.FL,80,'AbsTol',1e-10);
c.state = struct('cornerDisplacement',struct('FL',0,'FR',0,'RL',0,'RR',0), ...
    'cornerVelocity',struct('FL',0,'FR',0,'RL',0,'RR',0));
loads = s.computeCornerLoadsFromChassis(c,0,0);
verifyEqual(testCase,loads.FR-loads.FL,0,'AbsTol',1e-10);
end

function testDynamicPropertiesRemainLive(testCase)
s = fixture(); c = PrescribedChassis(); c.state = DynamicChassisStateSpy();
s.chassis = c;
s.computeCornerLoadsFromChassis(c,0,0);
p = addprop(c.state,'frontAdditionalLateralLoadTransfer');
c.state.frontAdditionalLateralLoadTransfer = 25;
loads = s.computeCornerLoadsFromChassis(c,0,0);
verifyEqual(testCase,loads.FR-loads.FL,50,'AbsTol',1e-10);
delete(p);
loads = s.computeCornerLoadsFromChassis(c,0,0);
verifyEqual(testCase,loads.FR-loads.FL,0,'AbsTol',1e-10);
end

function testTransientCacheReloadsWithValueModel(testCase)
s = fixture(); s.getMaxIntegrationStep();
file = [tempname,'.mat']; cleanup = onCleanup(@() delete(file)); %#ok<NASGU>
save(file,'s'); saved = load(file,'s');
verifyEqual(testCase,saved.s.getMaxIntegrationStep(),s.getMaxIntegrationStep());
saved.s.frontAntiRollBarWheelRate = 4e6;
verifyLessThan(testCase,saved.s.getMaxIntegrationStep(),s.getMaxIntegrationStep());
end
