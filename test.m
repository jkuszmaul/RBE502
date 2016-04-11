%clear all; close all; clc;
close all;

%% Kinematics
[ret, sym_theta, diff_theta, plots,H] = inv_kin();

%% Dynamics
[dyn,inv_dyn]=simple_dyn();

%% Plan
% c: desired=@(t)

x0=zeros(6,2);

t=sym('t','real');
%p=Planner.fromSym(0.4*ones(6,1),t);
p=Planner.fromSym(sin(t)*ones(6,1));
%p=Planner.trapezoid(x0,[0.4*ones(6,1),zeros(6,1)],1,1);

%sym_d=[ 1   0   0   0
%        0   1   0   0
%        0   0   1   -0.3+t/10
%        0   0   0   1];
%d_work=Planner.fromSym(reshape(sym_d,[],1));
%p=Planner.toJointSpace(d_work,sym_theta,H,0:0.01:5);

%% Control
% c: tau=@(desired,actual)

Kp=.1*eye(6);
Kv=.1*eye(6);
Kpi=.1*eye(2);
dt = 0.01;
c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);

%% Noise
% n: desired=@(t)
n=@(p,v,tau) normrnd(1.5, 1, [6 1]);

%% Simulation

t_span=0:dt:5;

% ode=CreateODE(p,c,dyn,n);
% options = odeset('RelTol',1e-4,'AbsTol',1e-4.*ones(numel(x0),1));
% [T,Y]=ode45(ode,t_span,x0,options);
% pos=Y(:,1:size(Y,2)/2);
% vel=Y(:,1+size(Y,2)/2:end);

[pos,vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

% Try without adaptiveness.
Kpi=zeros(2);
c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);
[adaptive_pos,adaptive_vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

%% Visualization

traj_pos = [];
traj_vel = [];
for i = t_span
  traj = p(i);
  traj_pos(:, end+1) = traj(:, 1);
  traj_vel(:, end+1) = traj(:, 2);
end

plot(t_span,pos, t_span, traj_pos);
title('Positiion and Adaptive Control');
figure
plot(t_span,vel, t_span, traj_vel);
title('Velocity and Adaptive Control');
figure
plot(t_span,adaptive_pos, t_span, traj_pos);
title('Position without Adaptive Control');
figure
plot(t_span,adaptive_vel, t_span, traj_vel);
title('Velocity without Adaptive Control');
