yalmip('clear');
clear all
clc

% Model data
SampleTime = 60; % (secs)
MeanMotion = 7.291e-5; % (rad/sec)
[A,B] = BuildDiscreteHCWmatrices(SampleTime, MeanMotion);

T = 3600/SampleTime; % number of discrete time steps
nx = 6; % number of states
nu = 3; % number of control inputs

% Define decision variables
u = sdpvar(repmat(nu,1,T-1),repmat(1,1,T-1)); % state decision variables
x = sdpvar(repmat(nx,1,T),repmat(1,1,T)); % control decision variables
% d = binvar(repmat(2,1,T-2),repmat(1,1,T-2)); 5 integer decision variables


% constraints = [-100 <= x{:}(1) <= 100,
%                -100 <= x{:}(2) <= 100,
%                -100 <= x{:}(3) <= 100,
%                -1 <= x{:}(4) <= 1,
%                -1 <= x{:}(5) <= 1,
%                -1 <= x{:}(6) <= 1,];
           
constraints = [x{1}(1) == 0
               x{1}(2) == 0
               x{1}(3) == 0
               x{1}(4) == 0
               x{1}(5) == 0
               x{1}(6) == 0
               x{T}(1) == 10
               x{T}(2) == 10
               x{T}(3) == 0
               x{T}(4) == 0
               x{T}(5) == 0
               x{T}(6) == 0];
           
objective = 0;

for i = 1:T-1
  
  objective = objective + sum(abs(u{i}(1:3)));
  constraints = [constraints, x{i+1} == A*x{i} + B*u{i},... 
                 -100 <= x{i}(1) <= 100
                 -100 <= x{i}(2) <= 100
                 -100 <= x{i}(3) <= 100
                 -1 <= x{i}(4) <= 1
                 -1 <= x{i}(5) <= 1
                 -1 <= x{i}(6) <= 1];
  
end


optimize(constraints,objective)

for i = 1:T
    sol(1:6,i) = value(x{i});
    control(1:3,i) = value(u{i});
end

figure(1)
plot(sol(2,:),sol(1,:))

figure(2)
plot(2:T,control(1,:),2:T,control(2,:),2:T,control(3,:))






