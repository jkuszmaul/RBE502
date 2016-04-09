% clear all; close all; clc;

%% Kinematics
[ret, sym_theta, diff_theta, plots,H] = inv_kin();

%% Dynamics
[dyn,inv_dyn]=simple_dyn();

%% Plan
% c: desired=@(t)

x0=zeros(6,2);

t=sym('t','real');
%p=Planner.fromSym(0.4*ones(6,1),t);
%p=Planner.fromSym(sin(t)*ones(6,1));
%p=Planner.trapezoid(x0,[0.4*ones(6,1),zeros(6,1)],1,1);

sym_d=[ 1   0   0   0
        0   1   0   0
        0   0   1   0.3+t/10
        0   0   0   1];
d_work=Planner.fromSym(reshape(sym_d,[],1));
p=Planner.toJointSpace(d_work,sym_theta,H,0:0.01:5);

%% Control
% c: tau=@(desired,actual)

Kp=10*ones(6,1);
Kv=10*ones(6,1);
c=Controller.ComputedTorque(inv_dyn,Kp,Kv);

%% Noise
% n: desired=@(t)
n=@(p,v,tau) normrnd(0.2, 2, [6 1]);

%% Simulation

t_span=0:0.01:5;

% ode=CreateODE(p,c,dyn,n);
% options = odeset('RelTol',1e-4,'AbsTol',1e-4.*ones(numel(x0),1));
% [T,Y]=ode45(ode,t_span,x0,options);
% pos=Y(:,1:size(Y,2)/2);
% vel=Y(:,1+size(Y,2)/2:end);

[pos,vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

%% Visualization

plot(t_span,pos);
figure
plot(t_span,vel);
