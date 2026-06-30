function tests = test_allocate
%TEST_ALLOCATE Unit tests for src/allocate.m
% Run with: results = runtests('tests/test_allocate.m')
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = currentProject().RootFolder;
    addpath(fullfile(root, 'src'));
    run(fullfile(root, 'data', 'vehicle_params.m'));
    testCase.TestData.P = P;
end

function testZeroDemandGivesEqualTorques(testCase)
    P = testCase.TestData.P;
    [T_FL,T_FR,T_RL,T_RR,Mz_real,~] = allocate(0, 0, P);
    verifyEqual(testCase, [T_FL T_FR T_RL T_RR], [0 0 0 0]);
    verifyEqual(testCase, Mz_real, 0, 'AbsTol', 1e-9);
end

function testPositiveMzShiftsTorqueRight(testCase)
    P = testCase.TestData.P;
    [T_FL,T_FR,T_RL,T_RR,~,~] = allocate(100, 0, P);
    % ISO convention: +Mz -> more torque on right wheels (allocate.m)
    verifyGreaterThan(testCase, T_FR, T_FL);
    verifyGreaterThan(testCase, T_RR, T_RL);
end

function testSaturatesAtPeakTorque(testCase)
    P = testCase.TestData.P;
    [T_FL,T_FR,T_RL,T_RR,~,~] = allocate(1e6, 0, P);   % absurdly large demand
    verifyLessThanOrEqual(testCase, max([T_FL T_FR T_RL T_RR]), P.Tmot_pk + 1e-9);
end

function testRegenDisabledClampsAtZero(testCase)
    P = testCase.TestData.P;
    P.regen_on = false;
    [T_FL,~,~,~,~,~] = allocate(1e6, 0, P);   % inner wheel would want negative torque
    verifyGreaterThanOrEqual(testCase, T_FL, 0);
end
