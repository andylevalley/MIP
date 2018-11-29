yalmip('clear');
clear all
clc

SampleTime = 30; % (secs)
n = 7.291e-5; % (rad/sec)

A = [0,     0,      0,      1,      0,      0;
     0,     0,      0,      0,      1,      0;
     0,     0,      0,      0,      0,      1;
    3*n^2,  0,      0,      0,      2*n,    0;
     0,     0,      0,      -2*n,   0,      0;
     0,     0,      -n^2,   0,      0,      0];
 
 B = [0 0 0;
      0 0 0;
      0 0 0;
      1 0 0;
      0 1 0;
      0 0 1];
 
C = [1, 1,  1,  1,  1,  1];
 
sys = ss(A,B,C,0);
 
d_sys = c2d(sys,60);

A = d_sys.A;
B = d_sys.B;

%% 

T = 2*60*60/SampleTime; % number of discrete time steps
nx = 6; % number of states
nu = 3; % number of control inputs

% Define decision variables
u = sdpvar(nu,T-1); % state decision variables
x = sdpvar(nx,T); % control decision variables
h = binvar(T-1,3); % 5 integer decision variables

Mp = 20;
Ms = 10;
xw = [5 -5 5;
      5 -5 5;
      5 -5 -5;
      0   0 0;
      0   0 0;
      0   0 0];

constraints = [x(1,1) == 0
               x(2,1) == 0
               x(3,1) == 0
               x(4,1) == 0
               x(5,1) == 0
               x(6,1) == 0
               x(1,T) == 0
               x(2,T) == 0
               x(3,T) == 0
               x(4,T) == 0
               x(5,T) == 0
               x(6,T) == 0];
           

constraints = [constraints, -Mp <= x(1:3,:) <= Mp];
constraints = [constraints, -Ms <= x(4:6,:) <= Ms];

for i = 1:T-1
    
    constraints = [constraints, x(1:6,i+1) == A*x(1:6,i) + B*u(4:6,i)];
             
    constraints = [constraints
                   implies(h(i,1),x(1:6,i) == xw(1:6,1))
                   implies(h(i,2),x(1:6,i) == xw(1:6,2))
                   implies(h(i,3),x(1:6,i) == xw(1:6,3))];

    constraints = [constraints, sum(h(i,:)) <= 1];
               
                       
end

constraints = [constraints, sum(h(:,1)) >= 1];
constraints = [constraints, sum(h(:,2)) >= 1];
constraints = [constraints, sum(h(:,3)) >= 1];

objective = sum(sum(u,2));

options = sdpsettings('solver','intlinprog');
optimize(constraints,objective,options)

%% Plot solution

for i = 1:T
    sol(1:6,i) = value(x(1:6,i));
end

for i = 1:T-1
    control(1:3,i) = value(u(1:3,i));
    hval(i) = value(h(i));
end

figure(1)
scatter3(sol(2,:),sol(1,:),sol(3,:))

figure(2)
plot(1:T,sol(1,:),1:T,sol(2,:))

figure(3)
plot(2:T,control(1,:),2:T,control(2,:),2:T,control(3,:))

