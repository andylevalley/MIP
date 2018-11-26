function [Ad,Bd] = BuildDiscreteHCWmatrices(sampleTime_s, n_rad_s)
%DISCRETECWMATRICIES Return the discrete A,B matrices associated with CWH
%
%  This function forms the discretized Clohessy-Wiltshire Relative
%  Equations of Motion

% INPUTS:
%  sampleTime_s Discrete sample time (sec)
%  n_rad_s      Mean-motion of chief satellite (rad/s)
%
% OUTPUTS:
%  Ad           Discrete form of state-space A matrix
%  Bd           Discrete form of state-space B matrix
%
% NOTE: The CWH state order used in this formulation is as follows:
%  1 - xd     (radial position)
%  2 - yd     (along-track position)
%  3 - xd_dot (radial velocity)
%  4 - xd_dot (along track velocity)
%  5 - zd     (out-of-plane position)
%  6 - zd_dot (out-of-lane velocity)
%
%  state space representation:
%   x_dot = Ad * x + B * u (x = 6x1 state, u = 3x1 control)
%
%  author:   1Lt Michael Tibbs & Dr. Raoul Rausch

%#codegen

% [xd, yd, zd, xd_dot, yd_dot, zd_dot] (continuous)
A=[          0     0           1          0        0       0;
              0     0           0          1        0       0;
              0     0           0          0        0       1;
    3*n_rad_s^2     0           0  2*n_rad_s        0       0;
              0     0  -2*n_rad_s          0        0       0
              0     0  -n_rad_s^2          0        0       0];

% Compute discrete A matrices for the specified samplTime_s
Ad = expm(A*sampleTime_s); % discretization method
Bd = [0 0 0;
      0 0 0;
      0 0 0;
      1 0 0;
      0 1 0;
      0 0 1];


end