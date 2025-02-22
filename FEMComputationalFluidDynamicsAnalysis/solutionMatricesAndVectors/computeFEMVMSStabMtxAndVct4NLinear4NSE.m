function [K, resVct, minElEdgeSize] = ...
    computeFEMVMSStabMtxAndVct4NLinear4NSE ...
    (propAnalysis, up, upSaved, upDot, upDotSaved, uMeshALE, ...
    precompStiffMtx, precomResVct, DOFNumbering, fldMsh, F, ...
    loadFactor, computeBodyForces, propFldDynamics, t, ...
    propParameters, propGaussInt)
%% Licensing
%
% License:         BSD License
%                  cane Multiphysics default license: cane/license.txt
%
% Main authors:    Andreas Apostolatos
%
%% Function documentation
%
% Returns the tangent stiffness matrix and the residual vector
% corresponding to the nonlinear equation system occuring from the time
% discretization using the Bossak scheme and the space discretization using
% the classical finite element analysis of the Navier-Stokes equations.
%
% Reference :
%
% Ramon Codina, "A stabilized finite element method for generalized 
% stationary incompressible flows", Computer Methods in Applied Mechanics 
% and Engineering, Vol. 190 (2001), 2681-2706
%
% Implementation :
%
% KRATOS opensourse project, Riccardo Rossi
%
% Revision on the nonlinear equation system :
%                       
%                          G(u) * u = H                               (1)
%
% Nonlinear equation system (1) can be solved with the Newton method
% provided that the tangent of the system can be computed namely :
%
%               GTangent(u) = G(u) + dG(u)/du * u                     (2)
%
% In equation (2) the term dG(u)/du is obviously a third order tensor. By
% defining the residual of the equation system as :
%
%                   resVct(u) = G(u) * u - H                          (3)
%
% we iterate over all Newton iterations until convergence up to a given
% tolerance has been achieved, namely we solve iteratively the following 
% equation system:
%
%            GTangent(u^(i-1)) * du^i = - resVct(u^(i-1))            (4.1)
%                       u^i = u^(i-1) + du^i                         (4.2)
%
%             Input :
%      propAnalysis : Structure containing general properties of the
%                     analysis,
%                           .type : Analysis type
%                up : The discrete solution vector of the previous 
%                     nonlinear iteration step
%           upSaved : The discrete solution vector of the previous time 
%                     step
%             upDot : The rate of the discrete solution vector of the
%                     previous  nonlinear iteration step
%        upDotSaved : The rate of the discrete solution vector of the
%                     previous time step
%          uMeshALE : The mesh motion velocity field on the nodes of the
%                     mesh
%   precompStiffMtx : Precomputed part of the stiffness matrix
%      precomResVct : Precomputed part of the residual vector
%      DOFNumbering : The global numbering of the DOFs arranged in a
%                     3-dimentional array
%            fldMsh : Nodes and elements of the fluid mesh
%                 F : The boundary applied flux vector
%        loadFactor : Load factor in case more than one load steps are
%                     performed within each time step
% computeBodyForces : Function handle to the computation of the body force
%                     vector
%   propFldDynamics : Transient analysis parameters :
%                             .scheme : The time integration method
%                          .alphaBeta : (parameter for the Bossak scheme)
%                              .gamma : (parameter for the Bossak scheme)
%                                 .T0 : Start time of the simulation
%                               .TEnd : End time of the simulation
%                        .noTimeSteps : Number of time steps
%                                 .dt : Time step
%                 t : The current time of the transient simulation
%    propParameters : Flow parameters
%      propGaussInt : Structure containing information on the quadrature,
%                         .type : 'default', 'user'
%                   .domainNoGP : Number of Gauss Points for the domain 
%                                 integration
%                 .boundaryNoGP : Number of Gauss Points for the boundary 
%                                 integration
%
%            Output :
%                 K : The linearized tangent stiffness matrix
%          KNLinear : The nonlinear stiffness matrix
%            resVct : The residual vector corresponding to the Newton 
%                     linearization of the Bossak discrete equation system 
%                     applied to the transient Navier-Stokes equations
%     minElEdgeSize : The minimum element edge size in the mesh
%
% Function layout :
%
% 0. Read input
%
% 1. Get the index of the nodes of each element in the array of nodes
%
% 2. Create the element freedom tables for all elements at once
%
% 3. Get the element discrete solution vector of the previous Newton iteration step
%
% 4. Get the dicrete mesh velocity vector (0 since we don't do ALE) GOT TO BE FIXED
%
% 5. Get the coordinates of the nodes in a matrix form
%
% 6. Get the minimum element edge size
%
% 7. Choose an integration rule
%
% 8. Loop over all the quadrature points
% ->
%    8i. Transform the Gauss Point location from the parameter to the physical space
%
%   8ii. Compute the basis functions and their derivatives at the Gauss Point
%
%  8iii. Compute the determinant of the Jacobian transformation from the physical to the parent space
%
%   8iv. Compute the tangent stiffness, the mass matrix and the body force vector on the Gauss Point
%
%    8v. Compute the load vector corresponding to the body force vector of the system
%
%   8vi. Add the contributions from the Gauss point
% <-
%
% 9. Add the contribution from the Gauss Point and assemble to the global system
%
% 10. Compute the system matrix corresponding to the Bossak time integration scheme
%
% 11. Compute the right-hand side vector corresponding to the Bossak time integration scheme
%
%% Function main body

%% 0. Read input

% Total number of nodes in the mesh
noNodes = length(fldMsh.nodes(:, 1));

% Number of DOFs per node
if strcmp(propAnalysis.type, 'NAVIER_STOKES_2D')
    numDOFsPerNode = 3;
    numNodesEl = 3;
    isAnalysis3D = false;
elseif strcmp(propAnalysis.type, 'NAVIER_STOKES_3D')
    numDOFsPerNode = 4;
    numNodesEl = 4;
    isAnalysis3D = true;
else
    error('Wrong analysis type specified')
end

% Total number of elements in the mesh
numElmnts = length(fldMsh.elements(:, 1));

% Total number of degrees of freedom
numDOFs = numDOFsPerNode*noNodes;

% Number of degrees of freedom per element
numDOFsEl = numDOFsPerNode*numNodesEl;

% Initialize arrays
KLineaEl = zeros(numElmnts, numDOFsEl, numDOFsEl);
KNLineaEl = zeros(numElmnts, numDOFsEl, numDOFsEl);
massMtxEl = zeros(numElmnts, numDOFsEl, numDOFsEl);
FBodyEl = zeros(numDOFsEl, 1);

% Compute a nessecary pre-factor for the Bossak time integration scheme
preFactor = (1 - propFldDynamics.alphaBeta)/propFldDynamics.gamma/ ...
    propFldDynamics.dt;

% Initialize the global body force vector
FBody = zeros(numDOFs, 1);

%% 1. Get the index of the nodes of each element in the array of nodes
[~, idx] = ismember(fldMsh.elements(:, 2:numNodesEl + 1), fldMsh.nodes(:, 1));

%% 2. Create the element freedom tables for all elements at once
EFT = zeros(numDOFsEl, numElmnts);
for iEFT = 1:numNodesEl
    for iDOFsPerNode = 1:numDOFsPerNode - 1
        EFT(numDOFsPerNode*iEFT, :) = numDOFsPerNode*idx(:, iEFT)';
        EFT(numDOFsPerNode*iEFT - (numDOFsPerNode - iDOFsPerNode), :) = ...
            EFT(numDOFsPerNode*iEFT, :) - (numDOFsPerNode - iDOFsPerNode);
    end
end

%% 3. Get the element discrete solution vector of the previous Newton iteration step
upEl = up(EFT);

%% 4. Get the dicrete mesh velocity vector
if ~ischar(uMeshALE)
    uMeshALEEL = uMeshALE(EFT);
else
    uMeshALEEL = zeros(numDOFsEl, 1);
end

%% 5. Get the coordinates of the nodes in a matrix form

% define function to calculate euclidean norm
euclideanNorm = @(nodes) sqrt(nodes(:, 1, 1).^2 + nodes(:, 2, 1).^2 + nodes(:, 3, 1).^2);

% Minimum element edge size
if isAnalysis3D
    % Get the nodes of the mesh
    [~, idx] = ismember(fldMsh.elements(:, 2:5), fldMsh.nodes(:, 1));
    nodesI = fldMsh.nodes(idx(:, 1), 2:end);
    nodesJ = fldMsh.nodes(idx(:, 2), 2:end);
    nodesK = fldMsh.nodes(idx(:, 3), 2:end);
    nodesL = fldMsh.nodes(idx(:, 4), 2:end);
    
    % get element sizes
    h = min( [ euclideanNorm(nodesI - nodesJ) euclideanNorm(nodesI - nodesK) ...
               euclideanNorm(nodesJ - nodesK) euclideanNorm(nodesL - nodesI) ...
               euclideanNorm(nodesL - nodesJ) euclideanNorm(nodesL - nodesK)], ...
               [], 2);
else
    % Get the nodes of the mesh
    [~, idx] = ismember(fldMsh.elements(:, 2:4), fldMsh.nodes(:, 1));
    nodesI = fldMsh.nodes(idx(:, 1), 2:end);
    nodesJ = fldMsh.nodes(idx(:, 2), 2:end);
    nodesK = fldMsh.nodes(idx(:, 3), 2:end);
    
	% get element sizes
    h = min( [ euclideanNorm(nodesI - nodesJ) euclideanNorm(nodesI - nodesK) ...
               euclideanNorm(nodesJ - nodesK)], [], 2);
end

%% 6. Get the minimum element edge size
minElEdgeSize = min(h);

%% 7. Choose an integration rule
if strcmp(propGaussInt.type, 'default')
    noGP = 1;
elseif strcmp(propGaussInt.type, 'user')
    noGP = propGaussInt.domainNoGP;
end
if isAnalysis3D
    [GP, GW] = getGaussRuleOnCanonicalTetrahedron(noGP);
else
    [GP, GW] = getGaussRuleOnCanonicalTriangle(noGP);
end

%% 8. Loop over all the quadrature points
for iGP = 1:noGP
    %% 8i. Transform the Gauss Point location from the parameter to the physical space
    if isAnalysis3D
        xGP = GP(iGP, 1)*nodesI + GP(iGP, 2)*nodesJ + GP(iGP, 3)*nodesK + ...
        (1 - GP(iGP, 1) - GP(iGP, 2) - GP(iGP, 3))*nodesL;
    else
        xGP = GP(iGP, 1)*nodesI + GP(iGP, 2)*nodesJ + (1 - GP(iGP, 1) - GP(iGP, 2))*nodesK;
    end

    %% 8ii. Compute the basis functions and their derivatives at the Gauss Point
    if isAnalysis3D
        [dN, area] = computeCST3DBasisFunctionsAndFirstDerivatives ...
            (nodesI, nodesJ, nodesK, nodesL, xGP(:, 1, :), xGP(:, 2, :), xGP(:, 3, :));
    else
        [dN, area] = computeCST2DBasisFunctionsAndFirstDerivatives ...
            (nodesI, nodesJ, nodesK, xGP(:, 1, :), xGP(:, 2, :));
    end
    
    %% 8iii. Compute the determinant of the Jacobian transformation from the physical to the parent space
    if isAnalysis3D
        detJxxi = 1.*area;
    else
        detJxxi = 2.*area;
    end
    
    %% 8iv. Compute the tangent stiffness, the mass matrix and the body force vector on the Gauss Point
    [KLineaElOnGP, KNLineaElOnGP, massMtxElOnGP, FBodyElOnGP] = ...
        computeFEMVMSStabElTangentStiffMtxMassMtxLoadVctNLinear4NSE ...
        (xGP(1, 1), xGP(1, 2), xGP(1, 3), t, upEl, uMeshALEEL, dN, ...
        computeBodyForces, propParameters, h, propFldDynamics, isAnalysis3D);

    %% 8v. Compute the load vector corresponding to the body force vector of the system
    if norm(FBodyElOnGP) ~= 0
        FBody(EFT) = FBodyElOnGP(EFT) + FBodyElOnGP*GW(iGP)*detJxxi;
    end
    
    %% 8vi. Add the contributions from the Gauss point
    KLineaEl = KLineaEl + pstimes(KLineaElOnGP*GW(iGP), detJxxi);
    KNLineaEl = KNLineaEl + pstimes(KNLineaElOnGP*GW(iGP), detJxxi);
    if strcmp(propFldDynamics.timeDependence, 'TRANSIENT')
        massMtxEl = massMtxEl + pstimes(massMtxElOnGP*GW(iGP), detJxxi);
    end
    FBodyEl = FBodyEl + FBodyElOnGP;
end

%% 9. Add the contribution from the Gauss Point and assemble to the global system
if strcmp(propFldDynamics.timeDependence, 'TRANSIENT')
    [KLinear, KNLinear, massMtx] = assembleSparseMatricies ...
        (EFT, numDOFs, numDOFsEl, KLineaEl, KNLineaEl, massMtxEl);
elseif strcmp(propFldDynamics.timeDependence, 'STEADY_STATE')
    [KLinear, KNLinear] = assembleSparseMatricies ...
        (EFT, numDOFs, numDOFsEl, KLineaEl, KNLineaEl);
else
    error('wrong time dependence selected, see input file');
end

%% 10. Compute the system matrix corresponding to the Bossak time integration scheme
if strcmp(propFldDynamics.timeDependence, 'TRANSIENT')
    K = preFactor*massMtx + KLinear + KNLinear;
elseif strcmp(propFldDynamics.timeDependence, 'STEADY_STATE')
    K = KLinear + KNLinear;
end

%% 11. Compute the right-hand side vector corresponding to the Bossak time integration scheme
if strcmp(propFldDynamics.timeDependence, 'TRANSIENT')
    resVct = (preFactor*massMtx + KLinear)*up - (FBody + loadFactor*F) - ...
            ((1-propFldDynamics.alphaBeta)/propFldDynamics.gamma/ ...
            propFldDynamics.dt)*massMtx*upSaved - ...
            ((1-propFldDynamics.alphaBeta)/propFldDynamics.gamma - 1)*...
            massMtx*upDotSaved;
elseif strcmp(propFldDynamics.timeDependence, 'STEADY_STATE')
    resVct = KLinear*up - (FBody + loadFactor*F);
else
    error('wrong time dependence selected, see input file');
end

end