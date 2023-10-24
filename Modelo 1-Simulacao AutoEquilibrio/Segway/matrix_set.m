%% variaveis complrmrntares
E = [M*l*R - 2*J                    M*l^2 + Jt;
    (2*m+M)*R^2 + 2*Jk + 2*J        M*l*R];
G = [0   -M*g*l;
    0    0];
H = 2*[-km/r;
    km/r];
%% estado das matrizes
A = [0 0 1;
    0 0 0;
    0 0 0];
A(2:3,1) = -E\G(:,2);
A(2:3,2) = -E\H;
B = [0;
    E\H];
%% inicialização
wn = Tp_z/Tp;
Y = [B A*B A^2*B];
    det_y = det(Y);
if K_mode == 0                          % manual
    
    K =[0       B(2)      B(3);
        B(3)    0           A(3,2)*B(2)-A(2,2)*B(3);
        A(3,2)*B(2)-A(2,2)*B(3)     A(2,1)*B(3)-A(3,1)*B(2) 0
        ]^(-1)*[3*wn+A(2,2); 3*wn^2+A(3,1);  wn^3-A(2,2)*A(3,1)+A(2,1)*A(3,2)];
elseif K_mode == 1                      % Ackerman mode
    F_A = (A+wn*eye(3))^3;
    K=[0 0 1]*(Y^-1)*F_A;
else                                    % Auto mode
    QQ = eye(3);
    RR = eye(1);
    K =lqr(A,B,QQ,RR);
end

