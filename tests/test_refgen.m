function tests = test_refgen
%TEST_REFGEN Unit tests for src/refgen.m
% Run with: results = runtests('tests/test_refgen.m')
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = currentProject().RootFolder;
    addpath(fullfile(root, 'src'));
    run(fullfile(root, 'data', 'vehicle_params.m'));
    run(fullfile(root, 'data', 'controller_params.m'));
    testCase.TestData.P = P;
end

function testZeroSteerGivesZeroRef(testCase)
    P = testCase.TestData.P;
    r_ref = refgen(0, 10, P);
    verifyEqual(testCase, r_ref, 0, 'AbsTol', 1e-9);
end

function testMatchesHandCalcInLinearRegion(testCase)
    P = testCase.TestData.P;
    delta = 0.05; Vx = 10;
    expected = Vx*delta / (P.L + P.Kus_ref*Vx^2);
    r_ref = refgen(delta, Vx, P);
    verifyEqual(testCase, r_ref, expected, 'RelTol', 1e-9);
end

function testSaturatesAtFrictionLimit(testCase)
    P = testCase.TestData.P;
    % Large steering angle at low speed should hit the mu*g/Vx ceiling
    Vx = 3;
    r_ref = refgen(1.0, Vx, P);   % ~57 deg, unrealistic on purpose
    r_max = P.mu*9.81/Vx;
    verifyLessThanOrEqual(testCase, abs(r_ref), r_max + 1e-9);
end

function testLowSpeedFade(testCase)
    P = testCase.TestData.P;
    % Below P.Vx_min, reference should be attenuated vs. the un-faded value
    Vx = P.Vx_min/2;
    r_ref = refgen(0.05, Vx, P);
    r_lin = Vx*0.05 / (P.L + P.Kus_ref*Vx^2);
    verifyLessThan(testCase, abs(r_ref), abs(r_lin) + 1e-9);
end
