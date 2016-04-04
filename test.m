clear all; close all; clc;

[dyn,inv_dyn]=simple_dyn();

Kp=10*ones(6,1);
Kv=10*ones(6,1);
c=Controller.ComputedTorqueF(inv_dyn,Kp,Kv);

x0=zeros(6,2);

t=sym('t','real');
p=Planner.fromSym(0.4*ones(6,1),t);
%p=Planner.fromSym(sin(t)*ones(6,1));
%p=Planner.trapezoid(x0,[0.4*ones(6,1),zeros(6,1)],1,1);

n=@(p,v,tau) normrnd(0.2, 2, [6 1]);

% ode=createODE(p,c,dyn);

t_range=0:0.01:5;

[pos,vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_range);

plot(t_range,pos);
figure
plot(t_range,vel);
