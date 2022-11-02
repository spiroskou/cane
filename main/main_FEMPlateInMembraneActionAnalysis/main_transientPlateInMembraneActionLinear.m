%% Licensing
%
% License:         BSD License
%                  cane Multiphysics default license: cane/license.txt
%
% Main authors:    Andreas Apostolatos
%
%% Script documentation
%
% Task : Geometrically nonlinear transient plane stress analysis
%
% Date : 19.02.2014
%
%% Preamble

% Clear memory
clear;

% Clear the command window
clc;

% Close all generated windows
close all;

%% Includes

% Add general math functions
addpath('../../generalMath/');

% Add all functions related to parsing
addpath('../../parsers/');

% Add all functions related to the low order basis functions
addpath('../../basisFunctions/');

% Equation system solvers
addpath('../../equationSystemSolvers/');

% Equation system solvers
addpath('../../efficientComputation/');

% Include functions related to transient analysis
addpath('../../transientAnalysis/');

% Add all functions related to plate in membrane action analysis
addpath('../../FEMPlateInMembraneActionAnalysis/solvers/',...
        '../../FEMPlateInMembraneActionAnalysis/solutionMatricesAndVectors/',...
        '../../FEMPlateInMembraneActionAnalysis/loads/',...
        '../../FEMPlateInMembraneActionAnalysis/graphics/',...
        '../../FEMPlateInMembraneActionAnalysis/output/',...
        '../../FEMPlateInMembraneActionAnalysis/initialConditions/',...
        '../../FEMPlateInMembraneActionAnalysis/postprocessing/', ...
        '../../FEMPlateInMembraneActionAnalysis/transient/');

%% Parse data from GiD input file

% Define the path to the case
pathToCase = '../../inputGiD/FEMPlateInMembraneActionAnalysis/';

% caseName = 'curvedPlateTipShearPlaneStressTransient';
% caseName = 'cantileverBeamPlaneStressTransientNLinear';
caseName = 'PlateWithMultipleHolesInVibrationTransientAnalysisTriangles';
% caseName = 'turek_csd';

% Parse the data from the GiD input file
[strMsh, homDBC, inhomDBC, valuesInhomDBC, propNBC, propAnalysis, ...
    parameters, propNLinearAnalysis, propStrDynamics, propGaussInt] = ...
    parse_StructuralModelFromGid(pathToCase, caseName, 'outputEnabled');
% propNLinearAnalysis.eps = 1e-3;
propNLinearAnalysis.method = 'UNDEFINED';
% propStrDynamics.method = 'BOSSAK';

% Define traction vector
propNBC.tractionLoadVct = [0; -1e1; 0];

%% UI

% On the computation of the body forces
computeBodyForces = @computeConstantVerticalStructureBodyForceVct;

% Equation system solver
solve_LinearSystem = @solve_LinearSystemMatlabBackslashSolver;

% Function handle to the computation of the initial conditions
computeInitCnds = @computeInitCndsFEMPlateInMembraneAction;

% Define the amplitude of the externally applied load and time duration
propNBC.tractionVector = [-1e-1; 0; 0];
propNBC.endTime = 1;

% Assign the function handles for the computation of the stiffness matrix 
% and choose solver for the finite element system based on whether the 
% analysis is linear or nonlinear
isLinear = true;
if ~isempty(propNLinearAnalysis)
    if ~ischar(propNLinearAnalysis)
        if isfield(propNLinearAnalysis, 'method')
            if strcmp(propNLinearAnalysis.method, 'NEWTON_RAPHSON')
                isLinear = false;
            end
        else
            error('Structure propNLinearAnalysis should define member variable method')
        end
    end
end
if isLinear
%     computeProblemMatricesSteadyState = @computeStiffMtxAndLoadVctFEMPlateInMembraneActionMixed;
    computeProblemMatricesSteadyState = @computeStiffMtxAndLoadVctFEMPlateInMembraneActionCST;
    solve_FEMSystem = @solve_FEMLinearSystem;
else
%     computeProblemMatricesSteadyState = @computeTangentStiffMtxResVctFEMPlateInMembraneAction;
    computeProblemMatricesSteadyState = @computeTangentStiffMtxResVctFEMPlateInMembraneActionCST;
    solve_FEMSystem = @solve_FEMNLinearSystem;
end

% On the writing the output function
propVTK.isOutput = false;
propVTK.writeOutputToFile = @writeOutputFEMPlateInMembraneActionToVTK;
propVTK.VTKResultFile = 'undefined';

% On transient inhomogeneous Dirichlet boundary conditions
updateInhomDOFs = 'undefined';
propIDBC = [];

% Not a unit test case
isUnitTest = false;

% Choose the matric computation corresponding to the chosen time
% integration scheme
if strcmp(propStrDynamics.method,'EXPLICIT_EULER')
    propStrDynamics.computeProblemMtrcsTransient = ...
        @computeProblemMtrcsExplicitEuler;
    propStrDynamics.computeUpdatedVct = ...
        @computeBETITransientUpdatedVctAccelerationField;
elseif strcmp(propStrDynamics.method,'BOSSAK')
    propStrDynamics.computeProblemMtrcsTransient = ...
        @computeProblemMtrcsBossak;
    propStrDynamics.computeUpdatedVct = ...
        @computeBossakTransientUpdatedVctAccelerationField;
else
    error('Invalid time integration method selected in propStrDynamics.method as %s',propStrDynamics.method);
end

% Choose time integration scheme parameters
if strcmp(propStrDynamics.method,'BOSSAK')
    propStrDynamics.alphaB = -.1; % -.1
    propStrDynamics.betaB = .5; % .5
    propStrDynamics.gammaB = .6; % .6
end

% Assign two significant eigenfrequencies
freqI = 4.656892249680108;
freqJ = 8.953736617642974;

% Assign the corresponding logarithmic decrements
zetaI = .1;
zetaJ = .1;

% Compute the corresponding circular frequencies
omegaI = 1e2*199.77; % 2*pi*freqI;
omegaJ = 1e2*300; % 2*pi*freqJ;

% Compute the Rayleigh damping parameters
detM = omegaJ/omegaI - omegaI/omegaJ;
coef = 2/detM;
alphaR = coef*(omegaJ*zetaI - omegaI*zetaJ);
betaR = coef*(-1/omegaJ*zetaI + 1/omegaI*zetaJ);

% Rayleigh damping
propStrDynamics.damping.method = 'rayleigh';
propStrDynamics.damping.computeDampMtx = @computeDampMtxRayleighFEM;
propStrDynamics.damping.alpha = alphaR;
propStrDynamics.damping.beta = betaR;

% Initialize graphics index
graph.index = 1;

%% Visualization of the configuration
% graph.index = plot_referenceConfigurationFEMPlateInMembraneAction(strMsh,analysis,F,homDBC,graph,'outputEnabled');

%% Solve the plate in membrane action problem
[dHat, minElSize] = solve_FEMPlateInMembraneActionTransient ...
    (propAnalysis, strMsh, homDBC, inhomDBC, valuesInhomDBC, ...
    updateInhomDOFs, propNBC, @computeLoadVctFEMPlateInMembraneActionPointLoad, ...
    parameters, computeBodyForces, computeInitCnds, ...
    computeProblemMatricesSteadyState, propNLinearAnalysis, propIDBC, ...
    propStrDynamics, solve_LinearSystem, solve_FEMSystem, propGaussInt, ...
    propVTK, caseName, 'outputEnabled');

%% Postprocessing
% graph.visualization.geometry = 'reference_and_current';
% resultant = 'stress';
% component = 'y';
% graph.index = plot_currentConfigurationAndResultants(strMsh,homDBC,dHat,parameters,analysis,resultant,component,graph);
id = ismember(strMsh.nodes(:,2:4), [5 1 0], 'rows');
pos = find(id==true);
node_id = strMsh.nodes(pos,1);
dy = zeros(propStrDynamics.noTimeSteps, 1);
for ii = 1:propStrDynamics.noTimeSteps
    dy(ii, 1) = dHat(2*node_id, ii);
end
plot(propStrDynamics.T0+propStrDynamics.dt:propStrDynamics.dt:propStrDynamics.TEnd, dy);

%% END OF THE SCRIPT
