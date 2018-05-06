%% Unknown Input Observer Dynamic System Matricies
% Sam Nazari
% April 2015
% This example is form the book: 
% Robust Model Based Fault Diagnosis for Dynamic Systems by J. Chen
% Please see the associated ShareLaTeX pdf file "UIO and Examples"
A = [-1 1 0;-1 0 0;0 -1 -1];
B = [-1;0;0];
C = [1 0 0;0 0 1];
D = 0;
E = B;

%% Construct the state space system
% Let us construct the system as a state space object for simulation.
sys = ss(A,B,C,D);

%% Step one: Check rank
% We check to see if the rank(CE) = rank(E) = 1
rank(C*E)
rank(E)

%% Step two: Compute Observer Matricies
% Since we have no issues in the previous step, let us compute the observer
% matrices now:
H = E*inv((C*E)'*(C*E))*(C*E)'
T = eye(3)-H*C
A1 = T*A

%% Step three: check observability 
% We must check the system A1,C observability
rank(obsv(A1,C))

%% Pole Placement 
% Pole placement is used to assign the observer poles
K1 = place(A',C',[-2,-10,-5])'

%% Finish observer design
% Finally, the F and K matricies are computed
F = A1-K1*C
K = K1 + F*H

%% Simulation results
%
% Set up simulation initial condiditons
x1_0 = 100;
x2_0 = -100;
x3_0 = 1;

d = 10;


%% Sim the system
sim('exOne')

%% Plot the results
t = linspace(1,10,length(X_hat));
figure(1),plot(t,X),hold on
plot(t,X_hat(:,1),'xb'),hold on
plot(t,X_hat(:,2),'xg'),hold on
plot(t,X_hat(:,3),'xr')
xlabel('Time in seconds'),ylabel('State trajectory and estimate')
title('Dynamic system state and UIO estimate')
legend('x_1','x_2','x_3','x_1 hat','x_2 hat','x_3 hat')


