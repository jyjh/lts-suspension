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
s = fixture(); c = PrescribedChassis(); c.state = CacheChassisStateSpy();
s.chassis = c; c.state.frontAdditionalLateralLoadTransfer = 10;
s.computeCornerLoadsFromChassis(c,0,0);
file = [tempname,'.mat']; cleanup = onCleanup(@() delete(file)); %#ok<NASGU>
save(file,'s'); saved = load(file,'s');
saved.s.chassis.state.frontAdditionalLateralLoadTransfer = 35;
loads = saved.s.computeCornerLoadsFromChassis(saved.s.chassis,0,0);
verifyEqual(testCase,loads.FR-loads.FL,70,'AbsTol',1e-10);
verifyEqual(testCase,c.state.frontAdditionalLateralLoadTransfer,10);
end
