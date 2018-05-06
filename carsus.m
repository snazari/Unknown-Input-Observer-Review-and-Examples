%% Unknown Input Observer Dynamic Simulation for a car suspension system
% Please see the associated ShareLaTeX pdf file "UIO and Examples"
% Sam Nazari 
% April 2015
%----System Physical Constants----%
mb = 300;    % kg
mw = 60;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 190000; % N/m

% --------System Matrices(It is a SIMO system)---------%
A = [ 0 1 0 0; [-ks -bs ks bs]/mb ; ...
      0 0 0 1; [ks bs -ks-kt -bs]/mw]
B = [0 0; 0 10000/mb ; 0 0; [kt -10000]/mw]
C = eye(4)
%D = [0 0; 0 0; B(2,:)];

%---------Disturbance Matrix----------%
E=[1;1;1;1]

%---------LQR design----------%
Q=[.25 0 0 0;0 4 0 0;0 0 1 0;0 0 0 4]
R=50*eye(2)
[K1 l s]=lqr(A,B,Q,R)

%% Step one: Check rank
% We check to see if the rank(CE) = rank(E) = 1
rank(C*E)
rank(E)

%% Step two: Compute Observer Matricies
% Since we have no issues in the previous step, let us compute the observer
% matrices now:
H = E*inv((C*E)'*(C*E))*(C*E)'
T = eye(4)-H*C
A1 = T*A

f=zeros(4,4)
v=[-4 -4 -4 -4]
F=diag(v)

k1=inv(C)*(A-F-H*C*A)
k2=F*H
k=zeros
k=k1+k2

%% Step three: check observability 
% We must check the system A1,C observability
rank(obsv(A1,C))


%% Simulation results
%
% Set up simulation initial condiditons
X_0 = [-1;10;3;5]


%% Sim the system
sim('car_sus')
plot(tout,XhatErr)
legend('xHat_1 Error','xHat_2 Error','xHat_3 Error','xHat_4 Error')
title('Estimation Error for Car Suspension System with Disturbance')
xlabel('Time (sec)')
ylabel('Estimation Error in m and m/s')
xlim([0,1.5])


