function damMtxRayleigh = computeDampMtxRayleighFEM ...
    (propAnalysis, massMtx, msh, numDOFs, propParameters, propGaussInt, ...
    propStrDynamics, tab, outMsg)
%% Licensing
%
% License:         BSD License
%                  cane Multiphysics default license: cane/license.txt
%
% Main authors:    Andreas Apostolatos
%
%% Function documentation
%
%% Function main body
if ~isfield(propStrDynamics.damping,'alpha')
    error('Undefined Rayleigh damping parameter propTransientAnalysis.damping.alpha');
end
if ~isfield(propStrDynamics.damping,'beta')
    error('Undefined Rayleigh damping parameter propTransientAnalysis.damping.beta');
end
if ischar(massMtx)
    error('Undefined mass matrix');
end
if strcmp(outMsg,'outputEnabled')
    fprintf(strcat(tab,'Rayleigh parameter alpha = %f \n'),propStrDynamics.damping.alpha);
    fprintf(strcat(tab,'Rayleigh parameter beta = %f \n\n'),propStrDynamics.damping.beta);
end

%% 0. Read input

% Define the dummy arrays
uSaved = 'undefined';
uDotSaved = 'undefined';
loadFactor = 'undefined';
precompStiffMtx = 'undefined';
uMeshALE = 'undefined';
precomResVct = 'undefined';
DOFNumbering = 'undefined';
F = zeros(numDOFs, 1);
computeBodyForces = @computeConstantVerticalStructureBodyForceVct;
t = 'undefined';
u = zeros(numDOFs,1);
uDot = zeros(numDOFs,1);

%% 1. Compute the linear stiffness matrix of the system
linStiffMtx = computeStiffMtxAndLoadVctFEMPlateInMembraneActionCST ...
    (propAnalysis, u, uSaved, uDot, uDotSaved, uMeshALE, ...
    precompStiffMtx, precomResVct, DOFNumbering, msh, F, ...
    loadFactor, computeBodyForces, propStrDynamics, t, ...
    propParameters, propGaussInt);

%% 2. Compute the damping matrix corresppnding to the Rayleigh approach
damMtxRayleigh = propStrDynamics.damping.alpha*massMtx + ...
    propStrDynamics.damping.beta*linStiffMtx;

end
