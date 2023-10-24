%% Cálculo dos parâmetros
Clc; clear all; close all
% variação da distância entre placas
h=0:0.1:20;
% medidas
M_2motor = 0.242;
M1 = 0.155;
M2 = 0.245;
dM_fios =(25)*0.003;
M = 0.062;
M_4barras = 0.252;
M_placa = 0.160;
m = M_2motor+ M1 + M2 + 2*M_placa + M_4barras+ dM_fios;
h1 = 2.1;
d_h1 = 1.0;
d_h2 = 1.5;
d_hb = (3.8-1.5);
hp = 23;
e = 0.3;
R_motor = 1.15;
r = 6.3/2;
r_interno = 5.3/2;
d_eixos = 23;
%calculos
Io_2motor = (M_2motor)*(R_motor^2)/2;
Io_placa = M_placa*(e^2)/12;
Io_M1 = M1*(d_h1^2)/12;
Io_M2 = M2*(d_h2^2)/12;
Io_4barras = M_4barras*(hp).*(hp)/12;
Io_rodas = M*((r+r_interno)/2)^2;
I_2motor = Io_2motor;
I_placa1 = Io_placa + M_placa*(h1^2);
I_M1 = Io_M1 + M1*((h1+d_h1)^2);
I_4barras = Io_4barras + M_4barras*((hp/2)-d_hb).*((hp/2)-d_hb);
I_placa2 = Io_placa + M_placa*((h1+h).*(h1+h));
I_M2 = Io_M2 + M2*((h1+h+d_h2).*(h1+h+d_h2));
I_eixo = I_2motor+I_placa1+I_M1+I_4barras+I_placa2+I_M2;
m_x_l_cm = M_placa*h1 + M1*(h1+d_h1) + M_4barras*(hp/2 -d_hb) + M_placa*(h1+h) + M2*(h+h1+d_h2) + dM_fios*(h1+h/2);
l_cm = (m_x_l_cm)/m;

%% APÊNDICE II – Parâmetros Experimentais do PID
clear all clc close all
edit 'teste_ZN_T1_GB.txt';
ensaio = load('teste_ZN_T1_GB.txt');
theta = ensaio(:,3);
time = ensaio(:,1);
time = time - time(1,1);
n = length(time);
t_max = time(length(time));
dt = (t_max - time(1))/n;
tempo = [0:dt:(n-1)*dt]';
fs = 1/dt;
df = fs/n;
f = 0:df:(n/2-1)*df;
figure
%plot(t,xt);
XT= fft(theta,n)*2/n;
plot(f,abs(XT(1:n/2)));
K_crit = 2.8;
P_crit = 0.0771;
Kp = 0.6*K_crit;
Ki = Kp/(0.5*0.0771);
Kd = Kp*0.125*0.0771;
fid = fopen('Z-N_resultados_ensaio_tempo.txt','wt');
A0011 = [tempo';theta'];
fprintf(fid,'%f %f\n',A0011);
fclose(fid);
fid = fopen('Z-N_resultados_ensaio_frequencia.txt','wt');
A0011 = [f;abs(XT(1:n/2))'];
fprintf(fid,'%f %f\n',A0011);
fclose(fid);

%% APÊNDICE III – G(s) e projeto do controlador PID
clc; clear all; close all;
s = tf([1 0],1);
l = 0.0748; % dinstância entre o eixo e o centro de massa
g = 9.79; % valor aproximado da gravidade em Uberlândia
M = 0.062; % massa não suspensa, 2 rodas;
m = 1.304; % massa pendulo, estrutura e componentes;
I = 48*10^-4; % calculado aproximadamente
b = 0.002; % atrito dinâmico no motor;
Ct = 0.0001; % atrito dinâmico no pendulo;
R = 0.9246; % Resistência interna do motor
Km =0.54; % coeficiênte Km do motor
Ke =0.34; % coeficiente Ke do motor
r =0.0316; % raio da roda
Ic =(0.3096)*10^-4; % Inérica da roda;
Vcc = 6; % tensão de alimentação do motor [volts]
d_z = 0.05; %zona morta do motor
termo1 = (I+m*l^2)*s^2- m*g*l;
num1 = [-m*l*s];
den1 = minreal([(M+ m+ Ic/r^2)*s+(2*Km*Ke/R + 2*b)/r^2]);
termo2 = minreal((num1/den1)*m*l*s^2);
num2 = [num1/den1]*(2*Km*Vcc/(R*r));
den2 = minreal([termo1 + termo2]);
Gs = minreal(num2/den2)
Polo = pole(Gs)
zero = zero(Gs)
% rlocus(Gs);
K = minreal(Gs*(s-Polo(1))*(s-Polo(2))*(s-Polo(3))/(s-zero))
Gs = K*(s-zero)/[(s-Polo(1))*(s-Polo(2))*(s-Polo(3))]
% projeto do controlador
OS= 0.1;
Tac= 1.25;
qsi = (-log(OS))/sqrt(pi^2 + (log(OS))^2);
wn = 4/(qsi*Tac);
P1 = wn*(-qsi + sqrt(-1+qsi^2))
%Gc = (Kd*s^2 + Kp*s + Ki)/s
b=0.4*imag(P1);
% Gc = K(s^2 + 2as + (a^2+b^2))/s
%phase do controlador + phase da planta = -180
phase_sGc = phase_controlador + phase(P1);
tg_t = tan(phase_sGc)
Re= real(P1);
Im=imag(P1);
G_teste = 1/[tg_t*[Re^2+s^2+2*s*Re+b^2-Im^2]-[2*Re*Im+2*s*Im]]
a_t = pole(G_teste)
a=a_t(1)
Gc = (s^2 + 2*a*s + (a^2+b^2))/s
% rlocus(Gs*Gc)
K_total = K*abs(P1)*abs(P1+a-b*i)*abs(P1+a+b*i)/(abs(P1)*abs(P1-Polo(1))*abs(P1-Polo(2))*abs(P1-Polo(3)))
Gc = minreal((1/-K_total)*(s+a-b*i)*(s+a+b*i)/s)
rlocus(Gs*Gc)

%% APÊNDICE VI – Software em Matlab para a simulação do veículo de duas rodas: Função PRINCIPAL
% Programa que chama a função dinâmica e execura a solução numérica do
% sistema dinâmico
clear all; clc; close all
T_o=0; % Tempo inicial da simulação
T_f=2; % Tempo total de simulação
% definição de varávies Globais importantes
global Up
global Ta
Ta = 2; %(ms) % Tempo de amostragem
Up =0; % Ação de controle no instante passado
h=0.0001; % passo na solução numérica do sistema
n=(T_f-T_o)/h; % número total de pontos
wr=zeros(10,1); % vetor de estado passado com condições inicias nulas
w=zeros(10,1); % vetor de estado atual
resposta =zeros(10,n+1); % matriz que salva os valores da simulação
k1=zeros(10,1); % Coeficientes K1, K2, K3 e K4 do RK 4° ordem
k2=k1;
k3=k1;
k4=k1;
wr(3,1)= -5.2*pi/180; % condições iniciais;

% Solução numérica utilizando um RK de 4° Ordem
for i=1:1:(n+1)
 t= h*i;
 % RK 4° ordem

 Linear = 0; % escolhe entre o modelo linear e o modelo não linear

 if(Linear ==0)
 % Não LINEAR
 [k1 U1 i1]=(Pendulo3G(t,wr,h));
 Up = U1;
 [k2 U2 i2]=(Pendulo3G(t+h/2,wr+h*k1/2,h));
 Up = U2;
 [k3 U3 i3]=(Pendulo3G(t+h/2,wr+h*k2/2,h));
 Up = U3;
 [k4 U4 i4]=(Pendulo3G(t+h,wr+h*k3,h));
 end

 if(Linear ==1)
% %LINEAR
 [k1 U1 i1]=(Pendulo3G_L(t,wr,h));
 Up = U1;
 [k2 U2 i2]=(Pendulo3G_L(t+h/2,wr+h*k1/2,h));
 Up = U2;
 [k3 U3 i3]=(Pendulo3G_L(t+h/2,wr+h*k2/2,h));
 Up = U3;
 [k4 U4 i4]=(Pendulo3G_L(t+h,wr+h*k3,h));
 end

 U = (U1+2*U2+2*U3+ U4)/6; % Tensão normalizada

 i_m = (i1+2*i2+2*i3+ i4)/6; % Corrente normalizada
 Up = U; % Ação de controle em tensão passada
 w= wr+ (h/6)*(k1+2*k2+2*k3+k4); % Calculando o vetor de estado atual
 w(9,1)= wr(9,1)+h*w(3,1); % Variável auxiliar utilizada no cálculo da integral do Ângulo theta
 wr=w; % Salva o vetor de estado atual para o vetor passado do próximo ciclo
 % salvando os resultados e modificando as variáveis auxiliares
 resposta(1,i)=w(1,1);
 resposta(2,i)=w(2,1);
 resposta(3,i)=w(3,1);
 resposta(4,i)=w(4,1);
 resposta(5,i)=w(5,1);
 resposta(6,i)=w(6,1);
 resposta(7,i)=w(7,1);
 resposta(8,i)=i_m;
 resposta(9,i)=0;
 resposta(10,i)=U;
end
% Plotando os resultados
T=T_o:h:T_f;
figure %posição
hold on
plot(T,100*resposta(1,:))
title('Posição da base');
xlabel('tempo t_[_s_]');
ylabel('Posição x_[_cm_]');
hold off
% figure %Velocidade
% hold on
% plot(T,resposta(2,:))
% title('Velocidade do carrinho');
% xlabel('tempo_[_s_]');
% ylabel('velocidade v_[_m/s_]');
% hold off
figure %Velocidade angular
hold on
plot(T,resposta(4,:))
title('velocidade angular do pendulo');
xlabel('tempo_[_s_]');
ylabel('Velocidade Angular w_[_rad/s_]');
hold off
figure %ação de controle
hold on
plot(T,resposta(10,:)/7)
title('tensão');
xlabel('tempo_[_s_]');
ylabel('V');
hold off
figure %angulo
hold on
plot(T,(180/pi)*resposta(3,:))
title('Posição angular do pendulo');
xlabel('tempo_[_s_]');
ylabel('Posição Angular_[_graus_]');

hold off
% w_roda = (resposta(2,:))/0.0316; %[rad/s]
% w_roda = w_roda/(2*pi); %[Hz, RPS]
% w_roda = w_roda*60; % RPM
% figure %Velocidade angular da roda
% hold on
% plot(T,w_roda)
% title('velocidade angular da roda');
% xlabel('tempo_[_s_]');
% ylabel('Velocidade Angular da roda RPM');
% hold off
% Calculos estatísiticos do resultado
M_angulo = (180/pi)*sum(resposta(3,5/h:length(resposta(3,:))))/length(resposta(3,:))
EQM_angulo = (sum(resposta(3,:).*(resposta(3,:))))/length(resposta(3,:))
EQM_tensao = (sum(resposta(10,:).*(resposta(10,:))))/length(resposta(10,:))
% salvando em um arquivo txt os resultados da simulação
U_b = resposta(10,:)/6;
theta_b = resposta(3,:);
x_b = resposta(1,:);
T_p = T_o:0.001*Ta:T_f;
n = 0;
for i = 1:1:length(x_b);
 if( (i-1) >= (n*0.001*Ta/h))
 theta(n+1)=theta_b(i);
 U(n+1) = U_b(i);
 n = n+1;
 end
end

fid = fopen('NOVO_P_Resultados_PID_SS_teorico.txt','wt');
A0011 = [T_p; U ;theta];
fprintf(fid,'%f %f %f\n',A0011);
fclose(fid);

%% APÊNDICE VII – Software em Matlab para a simulação do veículo de duas rodas: Função Fpend NÃO LINEAR.
function [dXdt Q i_m] = Pendulo3G(t,X,h)
%UNTITLED3 Summary of this function goes here
% t - instante de tempo;
% X - Vetor coluna X= [x1,x2,x3,x4]';
Y = zeros(10,1);
% ação de controle
%V = fuzzyc(t,X,h);
V =PID(t,X,h);
%tensões de cada motor
Ve = V;
Vd = V;
Q = V;
% definição das constantes com valores em SI
l = 0.0748; % dinstância entre o eixo e o centro de massa
g = 9.79; % valor aproximado da gravidade em Uberlândia
M = 0.062; % massa não suspensa, 2 rodas;
m = 1.304; % massa pendulo, estrutura e componentes;
I = 48*10^-4; % calculado aproximadamente
Cx = 0; % coeficiente de atrito estático entre o pneu e o chão;
b = 0.002; % atrito dinâmico no motor;
Ct = 0.0001; % atrito dinâmico no pendulo;
R = 0.9246; % Resistência interna do motor
Km2 =0.54 ; % coeficiênte Km do motor 0.5373
Km1 =0.34; % coeficiente Ke do motor 34
r =0.0316; % raio da roda
Ic =(0.3096)*10^-4; % Inérica da roda;
Iz = 7.16*10^-3;
d = 0.23; % distância entre eixos
T=0; % declaração de um distúrbio externo interpretado em torque
Vcc = 6; % tensão de alimentação do motor [volts]
d_z = 0.05; %zona morta do motor, 3% no motor esquerdo e 3.5% no motor direito.
%definiçãso das variáveis de estado
x = X(1,1); % Posição da base
vx = X(2,1); % Velocidade da base
theta= X(3,1); % Ângulo com a vertical do pêndulo
w = X(4,1); % Velocidade angular do pêndulo
fi = X(5,1); % Rotação em torno da vertical de pêndulo
dfi = X(6,1); % Velocidade angular em torno da vertical
% dinâmica dos motores
% Velocidade angular do eixo das rodas
wd = (vx+d*dfi)/r;
we = (vx-d*dfi)/r;
% Calculo da corrente nos motores

if(abs(Vd) < d_z*Vcc)
 id = 0;
else
 id = (Vd - Km1*wd)/R;
end
if(abs(Ve) < d_z*Vcc)
 ie = 0;
else
 ie = (Ve - Km1*we)/R;
end
% Cálculo dos torques, considerando as inércias no modelo do pêndulo
i_m = id;
TRd = Km2*id;
TRe = Km2*ie;
% Dinâmica do pendulo
u = (TRd + TRe)/r;
term3 = (Iz + 2*Ic*(d/r)^2)*(r/d);
Y(5,1) = X(6,1);
Y(6,1) = (TRd - TRe)/term3;
% % Inserindo um distúrbio em forma de delta de Dirac no sistema no
tempo t
% if (t==10)
% T = 0*1/h;
% char = 'metade'
% end
T = m*g*l*1*(pi/180)*normrnd(0,1);
term1 = m + M + 2*Ic/(r^2);
term2 = I+m*l^2-((m*l*cos(theta))^2)/term1;
Y(1,1)=X(2,1);
Y(3,1)=X(4,1);
T_u = -(m*l*cos(theta))/term1;
T_w = -(2*Ct +(w*sin(2*theta)*(m*l)^2)/(2*term1));
T_t = m*l*g*sin(theta);
T_vx = m*l*cos(theta)*((2*b/r^2)+Cx)/term1 + (2*Ct/r);
T_ext = T;
Y(4,1) = (T_ext + T_t + T_u*u + T_w*w + T_vx*vx)/term2;
Y(2,1)=(u -((2*b/r^2)+Cx)*vx +sin(theta)*m*l*w^2 - m*l*cos(theta)*Y(4,1))/term1;
Y(9:10,1)=0;
dXdt = Y;
end

%% APÊNDICE VII – Software em Matlab para a simulação do veículo de duas rodas: Função Fpend LINEAR.

function [dXdt Q i_m] = Pendulo3G_L(t,X,h)
%UNTITLED3 Summary of this function goes here
% t - instante de tempo;
% X - Vetor coluna X= [x1,x2,x3,x4]';
Y = zeros(10,1);
% ação de controle
%V = fuzzyc(t,X,h);
V =PID(t,X,h);
%tensões de cada motor
Ve = V;
Vd = V;
Q = V;
% definição das constantes com valores em SI
l = 0.0748; % dinstância entre o eixo e o centro de massa
g = 9.79; % valor aproximado da gravidade em Uberlândia
M = 0.062; % massa não suspensa, 2 rodas;
m = 1.304; % massa pendulo, estrutura e componentes;
I = 48*10^-4; % calculado aproximadamente
Cx = 0; % coeficiente de atrito estático entre o pneu e o chão;
b = 0.002; % atrito dinâmico no motor;
Ct = 0.0000001; % atrito dinâmico no pendulo;
R = 0.9246; % Resistência interna do motor
Km2 =0.5373; % coeficiênte Km do motor
Km1 =0.34; % coeficiente Ke do motor
r =0.0316; % raio da roda
Ic =(0.3096)*10^-4; % Inérica da roda;
Iz = 7.16*10^-3;
d = 0.23; % distância entre eixos
T=0; % declaração de um distúrbio externo interpretado em torque
Vcc = 6; % tensão de alimentação do motor [volts]
d_z = 0.05; %zona morta do motor, 3% no motor esquerdo e 3.5% no motor direito.
%definiçãso das variáveis de estado
x = X(1,1); % Posição da base
vx = X(2,1); % Velocidade da base
theta= X(3,1); % Ângulo com a vertical do pêndulo
w = X(4,1); % Velocidade angular do pêndulo
fi = X(5,1); % Rotação em torno da vertical de pêndulo
dfi = X(6,1); % Velocidade angular em torno da vertical
% dinâmica dos motores
% Velocidade angular do eixo das rodas
wd = (vx+d*dfi)/r;
we = (vx-d*dfi)/r;
% Calculo da corrente nos motores

if(abs(Vd) < d_z*Vcc)
 id = 0;
else
 id = (Vd - Km1*wd)/R;
end
if(abs(Ve) < d_z*Vcc)
 ie = 0;
else
 ie = (Ve - Km1*we)/R;
end
% Cálculo dos torques, considerando as inércias no modelo do pêndulo
i_m = id;
TRd = Km2*id;
TRe = Km2*ie;
% Dinâmica do pendulo
u = (TRd + TRe)/r;
term3 = (Iz + 2*Ic*(d/r)^2)*(r/d);
Y(5,1) = X(6,1);
Y(6,1) = (TRd - TRe)/term3;
% % Inserindo um distúrbio em forma de delta de Dirac no sistema no tempo t
% if (t==10)
% T = 0*1/h;
% char = 'metade'
% end
T = m*g*l*1*(pi/180)*normrnd(0,1);
term1 = m + M + 2*Ic/(r^2);
term2 = I+m*l^2-((m*l*1)^2)/term1;
Y(1,1)=X(2,1);
Y(3,1)=X(4,1);
T_u = -(m*l*1)/term1;
T_w = -(2*Ct);
T_t = m*l*g*(theta);
T_vx = m*l*1*((2*b/r^2)+Cx)/term1 + (2*Ct/r);
T_ext = T;
Y(4,1) = (T_ext + T_t + T_u*u + T_w*w + T_vx*vx)/term2;
Y(2,1)=(u -((2*b/r^2)+Cx)*vx-m*l*1*Y(4,1))/term1;
Y(9:10,1)=0;
dXdt = Y;
end

%% APÊNDICE VIII – Software em Matlab para a simulação do veículo de duas rodas: Função Fcontrol PID.
function [U] = PID(t,X,h)
teta = X(3,1)+ 1*(pi/180)*normrnd(0,1);
omega = X(4,1);
integral = X(9,1);
global Up;
global Ta;
U = Up;
Tt = t/h; % garantindo que o Tt seja um número inteiro
Tam = (Ta/1000);
if mod(Tt,Tam/h)<= 0.05*Tam/h
uot = 1.68*teta + 43.6*integral + 0.0162*omega;
%uot = 1.655*teta + 21.98*integral + 0.0278*omega;
%uot = 1.675*teta + 25.07*integral + 0.023*omega;
U = 6*uot;
end
 if U > 6
 U = 6;
 end
 if U < -6
 U = -6;
 end
end

%% APÊNDICE IX – Software em Matlab para a simulação do veículo de duas rodas: Função Fcontrol FUZZY.
Function [U] = fuzzyc(t,X,h)
%UNTITLED2 Summary of this function goes here
% Detailed explanation goes her
teta = X(3,1)+ 1*(pi/180)*normrnd(0,1);
ttg = abs(teta*180/pi);% angulo em graus
omega = X(4,1);
omg = abs(omega*180/pi);% velocidade em graus/s
veloc = X(2,1);
a = zeros(1,7);
b = zeros(1,5);
integral = X(9,1);
global Up;
global Ta;
U = Up;
Tt = t/h; % garantindo que o Tt seja um número inteiro
Tam = Ta/1000;
if mod(Tt,Tam/h)<=0.05*Tam/h

if ttg<=1
 a(4)=100*(1-ttg);
 if teta >= 0
 a(5)=100-a(4);
 else
 a(3)=100-a(4);
 end
else
if ttg <=5
 if(teta>0)
 a(6)=25*(ttg-1);
 a(5)=100-a(6);
 else
 a(2)=25*(ttg-1);
 a(3)=100-a(2);
 end
 else
 if ttg<= 10
 if(teta>0)
 a(7)=20*(ttg-5);
 a(6)=100-a(7);
 else
 a(1)=20*(ttg-5);
 a(2)=100-a(1);

 end
 else
 if(teta>0)
 a(7)=100;
 else
 a(1)=100;
 end

 end
 end
end

if omg<=20
 b(3)=5*(20-omg);
 if omega>=0
 b(4)=100-b(3);
 else
 b(2)=100-b(3);
 end
else
 if omg<=100
 if omega>0
 b(5)=1.25*(omg-20);
 b(4)=100-b(5);
 else
 b(1)=1.25*(omg-20);
 b(2)=100-b(1);
 end
 else
 if omg>100
 if omega>0
 b(5)=100;
 else
 b(1)=100;
 end
 end
 end
end
c(1)= 1*(a(1)*(b(3)+b(2)+b(1))+a(2)*(b(2)+b(1))+a(3)*b(1))/10000;
c(2)= 1*(a(1)*b(4)+a(2)*b(3)+a(3)*b(2)+a(4)*b(1))/10000;
c(3)= 1*(a(1)*b(5)+a(2)*b(4)+a(3)*b(3)+a(4)*b(2)+a(5)*b(1))/10000;
c(4)= 1*(a(2)*b(5)+a(3)*b(4)+a(4)*b(3)+a(5)*b(2)+a(6)*b(1))/10000;
c(5)= 1*(a(3)*b(5)+a(4)*b(4)+a(5)*b(3)+a(6)*b(2)+a(7)*b(1))/10000;
c(6)= 1*(a(4)*b(5)+a(5)*b(4)+a(6)*b(3)+a(7)*b(2))/10000;
c(7)= 1*(a(5)*b(5)+a(6)*(b(4)+b(5))+a(7)*(b(3)+b(4)+b(5)))/10000;
fator_peso=1;
y=zeros(1,7);
h=0.01;
soma1=0;
soma2=0;
for ui=h:h:1
 if ui<=0.25
 y(5)=(ui)/0.25;
 y(3)=y(5);
 y(4)=1-y(5);
 else
 if ui<=0.5
 y(6)=(ui-0.25)/0.25;
 y(2)=y(6);
 y(5)=1-y(6);
 y(3)=y(5);
 else
 if ui<=0.75
 y(7)=(ui-0.5)/0.25;
 y(1)=y(7);
 y(6)=1-y(7);
 y(2)=y(6);
 else
 y(7)=1;
 y(1)=1;
 end
 end
 end

YP = max([c(4)*y(4),c(5)*y(5),c(6)*y(6),c(7)*y(7)])/fator_peso;
YN = max([c(4)*y(4),c(3)*y(3),c(2)*y(2),c(1)*y(1)])/fator_peso;
soma1 = soma1 + (ui+h/2)*YP + (-ui-h/2)*YN;
soma2 = soma2 + YP + YN;
end
 soma1 = soma1 + c(4)*h/2;
 soma2 = soma2 + c(4);
 u1 = soma1/(soma2+0.0001);
 fator_fuzzy = 0.5;
 C_PID = 1.68*teta + 43.6*integral + 0.0162*omega;
 u = fator_fuzzy*u1 +(1-fator_fuzzy)*C_PID;
 U = 6*u;
 if U>6
 U = 6;
 end
 if U < -6
 U = -6;
 end
 % 1.5956e-05
end
