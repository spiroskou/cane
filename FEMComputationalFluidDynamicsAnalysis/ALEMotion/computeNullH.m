function [dx, dy, dz] = ...
    computeNullH(x, y, z, t, varargin)
%% Licensing
%
% License:         BSD License
%                  cane Multiphysics default license: cane/license.txt
%
% Main authors:    Andreas Apostolatos
%
%% Function documentation
%
% Applies null horizontal displacement for the selected node
%
%                    Input :
%                        x : x coordinate of the current node
%                        y : y coordinate of the current node
%                        z : z coordinate of the current node
%                        t : Current time step value
%
%                   Output :
%                       dx : movement in x coordinate of the current node
%                       dy : movement in y coordinate of the current node
%                       dz : movement in z coordinate of the current node
%
%% Function main body

dx = 0;
dy = NaN;
dz = 0;

end
