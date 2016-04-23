%clear all; close all; clc;
close all;
addpath Model;

%% Kinematics
% [ret, sym_theta, diff_theta, plots,H] = inv_kin();
% Get forward kinematics and function for jacobian.
[fwd_kin, jacfun, plotter] = kin3();

%% Dynamics
[dyn,inv_dyn]=simple_dyn();

t=sym('t','real');
dt = 0.01;
t_span=0:dt:10;


%% Plan
% p: desired=@(t)

x0=zeros(3,2);

%p=Planner.fromSym(0.4*ones(3,1),t);
% p=Planner.fromSym([sin(t)*ones(2,1);0]);
%p=Planner.trapezoid(x0,[0.4*ones(3,1),zeros(3,1)],1,1);

sym_d=[.5*sin(t);.5*cos(t);-3];
d_work=Planner.fromSym(sym_d,[]);
p=Planner.toJointSpace_func(d_work,t_span,[],@ik3_gen);

%% Control
% c: tau=@(desired,actual)

Krobust = 0.1*eye(3);
Lambda =0.5*eye(3);
Kp_adaptive=.2*eye(3);
Kv_adaptive=.2*eye(3);
Kpi=diag([.1 .1 2*ones(1, 3)]);

Kp=10*eye(3);
Kv=10*eye(3);

c = Controller.PD(Kp_adaptive,Kv_adaptive);
str_c=' PD Control';

% c = Controller.ComputedTorque(inv_dyn,Kp,Kv);
% str_c=' Computed Torque';

% c = Controller.RobustComputedTorque(Krobust,Lambda);
% str_c=' Robust Control';

% c=Controller.AdaptiveSimple(inv_dyn,Kv_adaptive,Kv_adaptive^-1 * Kp_adaptive, Kpi,dt);
% str_c=' Adaptive Control';

%% Noise
% n: noice=@(p,v,tau)

n=@(p,v,tau) normrnd(0,0, [3 1]);
% n=@(p,v,tau) normrnd(0,1, [3 1]);
% n=@(p,v,tau) normrnd(0.5,1, [3 1]);
% n=@(p,v,tau) normrnd(3,2, [3 1]) + 20 * [v(3);v(1);v(2)];


%% Simulation  

[pos,vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

% Do a animated plot.
[xdata, ydata, zdata] = plotter(pos(:, 1));
[gxdata, gydata, gzdata] = plotter(pos(:, 1));
TruePlot = plot3(xdata, ydata, zdata);
set(TruePlot, 'Color', 'green');
set(TruePlot, 'DisplayName', 'Actual');
hold on;
GoalPlot = plot3(gxdata, gydata, gzdata);
set(GoalPlot, 'DisplayName', 'Goal Trajectory');
legend('show');
zlim([-4 0]);
xlim([-2 2]);
ylim([-2 2]);
% Do calculations and drawing separately.
for i = 1:size(pos, 2)
  [xdata(i, :), ydata(i, :), zdata(i, :)] = plotter(pos(:, i));
  goal = p(dt * i);
  [gxdata(i, :), gydata(i, :), gzdata(i, :)] = plotter(goal(:, 1));
end
for i = 1:1
  % Use scale to only draw every Nth iteration.
  % If we try to draw too many, then it will just go slowly.
  scale = 2;
  for i = 1:size(pos, 2)/scale
    set(TruePlot, 'XData', xdata(i*scale, :));
    set(TruePlot, 'YData', ydata(i*scale, :));
    set(TruePlot, 'ZData', zdata(i*scale, :));
    set(GoalPlot, 'XData', gxdata(i*scale, :));
    set(GoalPlot, 'YData', gydata(i*scale, :));
    set(GoalPlot, 'ZData', gzdata(i*scale, :));
    drawnow;
    pause(scale * dt);
  end
end
figure

% Try without adaptiveness.
% Kpi=zeros(5);
% c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);
% [adaptive_pos,adaptive_vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

%% Visualization

traj=evalf(p,t_span.');
traj_pos=traj(:,:,1);
traj_vel=traj(:,:,2);


plot(t_span,pos,'r', t_span, traj_pos,'b');
% plot(t_span,pos, t_span, traj_pos);
title(strcat('Position under ',str_c));
xlabel('time (s)')
ylabel('position (radians)')

figure

plot(t_span,vel,'r', t_span, traj_vel,'b');
% plot(t_span,pos, t_span, traj_pos);
title(strcat('Velocity under ',str_c));
xlabel('time (s)')
ylabel('velocity (radians/s)')
