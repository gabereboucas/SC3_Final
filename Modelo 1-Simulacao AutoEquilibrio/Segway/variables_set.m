%% parametros fisicos
g = 9.81;                       % aceleracao da gravidade [m/sec^2]
m = 0.05;						% peso da roda [kg]
R = 0.0275;						% raio da roda [m]
Jk = m * R^2 / 2;				% momento de inercia [kg*m^2]
M = 0.8;                        % massa do corpo [kg] // 0.8
h = 0.11;						% altura do corpo [m] // 0.11
l = h / 2;						% distância do centro de massa ao eixo da roda [m]
Jt =  M * l^2 / 3;              % momento de inercia do corpo [kg*m^2]
%% parametros do motor
Umax = 8.2;                     % voltagem motor DC [V]
J = 0.00237;					% momento de inercia motor DC [kgm^2]
r = 3;                          % resistencia motor DC [Om]
km = 0.274;                     % coeficiente do Motor
ke = 0.274;                     % coeficiente do Motor
