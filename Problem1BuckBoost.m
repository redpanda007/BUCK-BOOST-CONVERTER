clear all
clc
Vg = 24;
R = 5;
L = 20e-6;
C = 80e-6;
D = 0.4;

% PLANT
% Np = [R*C*D*(D-1)];
% Dp = [L*R*C L (R*C*(D-1)^2)];

np = [Vg*D*L/R -Vg*(1-D)^2];
dp = [L*C*(1-D)^2 (L*(1-D)^2)/R (1-D)^4];

Gp = tf(np,dp);
rlocus(Gp);

%PI CONTROLLER
Kp = 0;
Ki = -5;
Kd = 0;
nc = [Kd Kp Ki];
dc = [0 1 0];

Gc = tf(nc,dc);

% Closed loop TF
Z = (1-D)^2;
A = Vg*D*L/R;
B = Vg*Z;


Nc = [Kp*A A*Ki-B*Kp -B*Ki];
Dc = [Z*L*C (Z*(L/R))-Kp*A A*Ki-B*Kp+Z^2 -B*Ki];

Gscl = tf(Nc,Dc);
subplot(3,1,2),rlocus(Gscl);
subplot(3,1,3),step(Gscl);


