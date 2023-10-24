clc;
clear;
tspan=0:.01:10;
[t,x]=ode45('segway_balance',tspan,[0 0 0.3 0]);