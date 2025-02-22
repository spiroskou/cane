classdef testFEMComputationalFluidDynamicsAnalysis < matlab.unittest.TestCase
%% Licensing
%
% License:         BSD License
%                  cane Multiphysics default license: cane/license.txt
%
% Main authors:    Andreas Apostolatos
%
%% Class definition
%
% Test suites for the nonlinear steady-state and transient stabilized
% finite element analysis for the 2D and 3D Navier-Stokes equations.
%
%% Method definitions
methods (Test)
    testFEM4TransientTaylorGreenVortices2D(testCase)
    testFEM4NavierStokesSteadyState2D(testCase)
    testFEM4NavierStokesSteadyStateFlowAroundCylinder2D(testCase)
    testFEM4NavierStokesTransientALE2D(testCase)
    testFEM4NavierStokesTransientBossak3D(testCase)
end

end
