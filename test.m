%clear all; close all; clc;
close all;

%% Kinematics
%[ret, sym_theta, diff_theta, plots,H] = inv_kin();

%% Dynamics
[dyn,inv_dyn]=simple_dyn();

%% Plan
t=sym('t','real');
dt = 0.01;
t_span=0:dt:10;

% c: desired=@(t)

x0=zeros(6,2);


%p=Planner.fromSym(0.4*ones(6,1),t);
%p=Planner.fromSym(sin(t)*ones(6,1));
%p=Planner.trapezoid(x0,[0.4*ones(6,1),zeros(6,1)],1,1);

% sym_d=[ 1   0   0   0
%        0   1   0   0
%        0   0   1   -2.5-t/5
%        0   0   0   1];
   
sym_d=[ 1   0   0   .5*sin(t);
        0   1   0   .5*cos(t);
        0   0   1   -3;
        0   0   0   1]
d_work=Planner.fromSym(sym_d(1:3,:),[]);
p=Planner.toJointSpace_func(d_work,t_span,[],@ik6_gen);


trace_pts=evalf(d_work,t_span.');
trace_pts=trace_pts(:,1:3,1,4);
plot3(trace_pts(:,1),trace_pts(:,2),trace_pts(:,3))

% figure
% D=evalf(p,t_span.');
% plot(t_span,D(:,:,1));
% figure


%% Control
% c: tau=@(desired,actual)

Krobust = 0.1*eye(6);
Lambda =0.5*eye(6); 
Kp=10*eye(6);
Kv=10*eye(6);
Kpi=.1*eye(2);

c = Controller.ComputedTorque(inv_dyn,Kp,Kv);
str_c=' Computed Torque';

% c = Controller.RobustComputedTorque(Krobust,Lambda);
% str_c=' Robust Control';

% c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);
% str_c=' Adaptive Control';


%% Noise
% n: desired=@(t)
n=@(p,v,tau) normrnd(0.5,1, [6 1]);
% n=@(p,v,tau) normrnd(0,1, [6 1]);
% n=@(p,v,tau) normrnd(0,0, [6 1]);

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


plot(t_span,pos,'r', t_span, traj_pos,'b');
% plot(t_span,pos, t_span, traj_pos);
title(strcat('Position under ',str_c));

figure

plot(t_span,vel ,'r', t_span, traj_vel,'b');
% plot(t_span,vel, t_span, traj_vel);
title(strcat('Velocity under ',str_c));

