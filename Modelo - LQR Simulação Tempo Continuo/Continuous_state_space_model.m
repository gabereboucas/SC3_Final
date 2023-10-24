% Parametros que definen al sistema
mc = 1.5; %massa rodas, motores e bateria.
mp = 0.5; %massa d a estrutura.
g = 9.81; %gravidade.
L = 1; %tamanho pendulo
d1 = 1e-2; 
d2 = 1e-2;
Ts= 0.1;
t = 0:Ts:300;
u = zeros(1,length(t));

%% matrizes que definen o sistema do diagrama de estados
%% sendo o sistema xppnto = Ax + Bu
A = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, (g*mp) /mc, -d1/mc, -d2/(L*mc) ;
    0, (g* (mp + mc) ) / (L*mc), -d1/ (L*mc), -(d2*mp + d2*mc) / (L^2*mc*mp) ];
B = [0; 0; 1/mc; 1/ (L*mc) ];

% Saida
%C = [0; 1; 0; 0]; %saida de q2
C = [1; 0; 0; 0];  %saida de q1
D = 0;

%Construir o sistema
%% definir sistema, mtz de contrabilidaade e alcance
sys = ss (A, B, C' , D)
Sc = ctrb(sys)  
AlcanceC = rank(Sc) 

So = obsv(sys)
AlcanceO = rank(So)

%% controlador
des_pole = [-3; -3; -3; -3]
K = acker(A,B,des_pole)

Q = 20*eye(4);
R = 0.1;
K_lqr = lqr (A,B,Q, R)

%% Sistema discreto
Ts= 0.1;
sys_d = c2d (sys, Ts)

Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;
Dd = sys_d.d; 

des_pole_d = [0.3; 0.3; 0.3; 0.3]*1;
Kd = acker (Ad, Bd, des_pole_d)

