function Xdot= segway_balance(t,X)
Xdot=zeros(size(X));

etta=1;  % gear ratio
Km=0.01;    % Motor constant
Kb=0.01;    % Back emf constant
Va=0;    % motor supply voltage also the control input
Ra=0.5;    % Armature resistance
r=0.5;     % Wheel radius
J=0.02479;     % Moment of inertial of the platform
Jw=0.00079;    % Moment of inertial of wheel
Cf=0.02;    % damping constant
mb=1;    % mass of the platform
g=-9.8;   % gravity
d=0.5;     % distance between CG and 

A=[0            1                0                  0             ;
   0 -(etta*Km*Kb+Cf*Ra)/(Ra*Jw) 0 -r*((etta*Km*Kb+Cf*Ra)/(Ra*Jw));
   0            0                0                  1             ;
   0 -(etta*Km*Kb+Cf*Ra)/(Ra*J*r) (mb*g*d/J) -((etta*Km*Kb+Cf*Ra)/(Ra*J))];

B= [0;etta*Km*r/Ra*Jw;0;etta*Km/Ra*r*J];

Xdot=A*X+B*Va;