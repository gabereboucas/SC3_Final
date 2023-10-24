function plot_all(tout, m_volt, m_psi, m_theta_d, m_psi_d)
%CREATEFIGURE1(TOUT1, M_VOLT1, M_PSI1, M_THETA_D1, M_PSI_D1)
%  TOUT:  vector of x data
%  M_VOLT:  vector of y data
%  M_PSI1:  vector of y data
%  M_THETA_D1:  vector of y data
%  M_PSI_D1:  vector of y data


% Create figure
figure1 = figure;

% Create axes
axes1 = axes('Parent',figure1,...
    'Position',[0.13 0.583837209302326 0.334659090909091 0.341162790697675]);
hold(axes1,'on');

% Create plot
plot(tout,m_volt,'Parent',axes1);

% Create xlabel
xlabel('Time, seconds','FontSize',14,'Interpreter','latex');

% Create ylabel
ylabel('Voltage, volts','FontSize',14,'Interpreter','latex');

box(axes1,'on');
% Set the remaining axes properties
set(axes1,'XGrid','on','YGrid','on');
% Create axes
axes2 = axes('Parent',figure1,...
    'Position',[0.570340909090909 0.583837209302326 0.334659090909091 0.341162790697675]);
hold(axes2,'on');

% Create plot
plot(tout,m_psi,'Parent',axes2,'DisplayName','m_psi_d vs tout');

% Create xlabel
xlabel('Time, seconds','FontSize',14,'Interpreter','latex');

% Create ylabel
ylabel('$\psi$, degrees/second','FontSize',14,'Interpreter','latex');

box(axes2,'on');
% Set the remaining axes properties
set(axes2,'XGrid','on','YGrid','on');
% Create axes
axes3 = axes('Parent',figure1,...
    'Position',[0.13 0.11 0.334659090909091 0.341162790697674]);
hold(axes3,'on');

% Create plot
plot(tout,m_theta_d,'Parent',axes3,'DisplayName','m_theta_d vs tout');

% Create xlabel
xlabel('Time, seconds','FontSize',14,'Interpreter','latex');

% Create ylabel
ylabel('$\theta^*$, degrees/second','FontSize',14,'Interpreter','latex');

box(axes3,'on');
% Set the remaining axes properties
set(axes3,'XGrid','on','YGrid','on');
% Create axes
axes4 = axes('Parent',figure1,...
    'Position',[0.570340909090909 0.11 0.334659090909091 0.341162790697674]);
hold(axes4,'on');

% Create plot
plot(tout,m_psi_d,'Parent',axes4,'DisplayName','m_psi_d vs tout');

% Create xlabel
xlabel('Time, seconds','FontSize',14,'Interpreter','latex');

% Create ylabel
ylabel('$\psi^*$, degrees/second','FontSize',14,'Interpreter','latex');

box(axes4,'on');
% Set the remaining axes properties
set(axes4,'XGrid','on','YGrid','on');
