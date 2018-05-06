%% Unknown Input Observer Dynamic System Matricies
%  The coupled mass-spring system is a well studied mechanical setup in
%  control theory.  Here we develop an UIO for this system under 
%  disturbances.
%  Author: Sam Nazari
%  Date:  April 2015
clear,clc

%% 
%----------System Parameters---------%
k   = 1   % 
m   = 1   % kg
c   = 1 % 

%%
%----------State Space Formulation---------%
A   = [0 0 1 0;
       0 0 0 1;
       -2*k/m k/m -c/m 0;
       k/m -2*k/m 0 -c/m]
    
B   = [0;0;0;k/m]
C   = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]
D   = 0
E   = [0;0;0;1]

%%
%----------LQR Control---------%
Q=[100 0 0 0;
    0 100 0 0;
    0 0 100 0;
    0 0 0 100]
R=1
[Klqr ll ss]=lqr(A,B,Q,R)

%%
%----------Step 1: rank(CE) = rank(E) ?= 1---------%
rank(C*E)
rank(E)

%% 
%----------Step 2: Compute observer matrices---------%
H = E*inv((C*E)'*(C*E))*(C*E)'
T = eye(4)-H*C
A1 = T*A

%% 
%----------Step 3:Check (C,A1) rank---------%
rank(obsv(A1,C)) % Check observability of the pair (C,A1)

%%
%----------Step 4: Choose observer poles---------%
K1 = place(A',C',[-2,-10,-5,-3])'

%% 
%----------Step 5: Finish observer design---------%
F = A1-K1*C
K = K1 + F*H

%% Simulation results
%
% Set up simulation initial condiditons
x1_0       = -1;
x2_0       = -5;
x3_0       = 1;
x4_0       = 1; % theta = pi is vertically upward equilibrium

% sim time
TSIM = 30

d=5   % time that disturbance occurs

%% Sim the system
sim('massSpringMDL')
figure,
plot(tout,X(:,1)-X_hat(:,1),'r'),hold on,
plot(tout,X(:,2)-X_hat(:,2),'b'),hold on, 
plot(tout,X(:,3)-X_hat(:,3),'k'),hold on, 
plot(tout,X(:,4)-X_hat(:,4),'g'),
title('State Estimation Error for coupled mass spring system')
xlabel('time (sec)'),ylabel('m and m/s')
legend('q_1','q_2','dq_1','dq_2','Location','NorthEast')
xlim([0,10])
ylim([-1,1])