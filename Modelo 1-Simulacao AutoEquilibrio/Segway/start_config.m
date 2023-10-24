%% parametros para iniciar
Psi0 = pi/12;                   % ang inicial do corpo [Rad]
Psi_d0 = 0;                     % vel inicial do corpo [Rad/s]
Theta_d0 = 0;                   % ang inicial da roda [Rad/s]
%% parametros de ganho K
Tp = 0.9;                       % tempo transitorio real [s] 0.9
Tp_z = 7.4;                     % tempo transitorio teorico  (n=3, Tp_z=6.3) [sec]
K_mode = 0;                     % K : 0 auto mode   lqr:1 k>3 ackermod