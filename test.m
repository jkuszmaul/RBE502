%clear all; close all; clc;
close all;

%% Kinematics
%[ret, sym_theta, diff_theta, plots,H] = inv_kin();

%% Dynamics
[dyn,inv_dyn]=simple_dyn();

%% Plan
t=sym('t','real');
dt = 0.01;
t_span=0:dt:5;

% c: desired=@(t)

x0=zeros(6,2);


%p=Planner.fromSym(0.4*ones(6,1),t);
%p=Planner.fromSym(sin(t)*ones(6,1));
%p=Planner.trapezoid(x0,[0.4*ones(6,1),zeros(6,1)],1,1);

sym_d=[ 1   0   0   0
       0   1   0   0
       0   0   1   0.3+t/5
       0   0   0   1];
d_work=Planner.fromSym(sym_d,[]);
p=Planner.toJointSpace_func(d_work,t_span,[],@ik6_gen);

D=evalf(p,t_span.');
plot(t_span,D(:,:,1));
figure;
D=evalf(d_work,t_span.');
plot(t_span,D(:,3,1,4));
%% Control
% c: tau=@(desired,actual)

Krobust = 0.1*eye(6);
Lambda =0.5*eye(6); 
Kp=.1*eye(6);
Kv=.1*eye(6);
Kpi=.1*eye(2);

%c = Controller.ComputedTorque(inv_dyn,Kp,Kv);
c = Controller.RobustComputedTorque(Krobust,Lambda);
%c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);

%% Noise
% n: desired=@(t)
n=@(p,v,tau) normrnd(0,0, [6 1]);

%% Simulation



% ode=CreateODE(p,c,dyn,n);
% options = odeset('RelTol',1e-4,'AbsTol',1e-4.*ones(numel(x0),1));
% [T,Y]=ode45(ode,t_span,x0,options);
% pos=Y(:,1:size(Y,2)/2);
% vel=Y(:,1+size(Y,2)/2:end);

[pos,vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

% Try without adaptiveness.
%Kpi=zeros(2);
%c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);
%[adaptive_pos,adaptive_vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

%% Visualization

traj=evalf(p,t_span.');
traj_pos=traj(:,:,1);
traj_vel=traj(:,:,2);

% traj_pos = [];
% traj_vel = [];
% for i = t_span
%   traj = p(i);
%   traj_pos(:, end+1) = traj(:, 1);
%   traj_vel(:, end+1) = traj(:, 2);
% end

plot(t_span,pos, t_span, traj_pos);
title('Position and Robust Control');
figure
plot(t_span,vel, t_span, traj_vel);
title('Velocity and Robust Control');
%figure
%plot(t_span,adaptive_pos, t_span, traj_pos);
%title('Position without Adaptive Control');
%figure
%plot(t_span,adaptive_vel, t_span, traj_vel);
%title('Velocity without Adaptive Control');
