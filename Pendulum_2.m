%%
clc
clear
close all

%% Parameters
M = 0.5;   % Cart mass
m = 0.2;   % Pendulum mass
b = 0.0;   % No friction
g = 9.81;  % Gravity 
l = 0.3;    % Length of pendulum
I= .006     % Moment of Interia
q = I*(M+m)+M*m*l^2;

%% Matrices
A = [0, 1, 0, 0; 
    0, 0, ((m^2*g*l^2)/q), 0;
    0, 0, 0, 1;
    0, 0, ((m*g*l*(m+M))/q), 0];

B = [0; (I+m*l^2)/q; 0; (m*l)/q];

%% Outputs
%C = [1 0 0 0; 0 0 1 0];
C = [1 0 0 0]; 
%C= [0 0 1 0]; %% Unobservable output (Rank 2)
D = 0;

%% Build System
sys = ss(A,B,C,D)
eig(A)
Sc = ctrb(sys)
So = obsv(sys)
X0 = [0; 3*pi/180; 0; 0];

%% Controller
Pole_Shift = [-3; -3; -3; -3];
k = acker(A,B,Pole_Shift)

Q = 5*eye(4);
R = .1;
k_lqr = lqr (A,B,Q,R);

%% Discrete Time
Ts = 1 ;
sys_d = c2d(sys,Ts)
Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;
Dd = sys_d.d;
Pole_Shift_d = [5; 5; 5; 5];
k_d = acker(Ad,Bd,Pole_Shift_d)

%% Observer
Pole_Shift = [5; 5; 5; 5];
Ob = acker(Ad',Cd',Pole_Shift_d)
