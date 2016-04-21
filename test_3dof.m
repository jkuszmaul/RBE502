%clear all; close all; clc;
close all;

%% Kinematics
% [ret, sym_theta, diff_theta, plots,H] = inv_kin();
% Get forward kinematics and function for jacobian.
[fwd_kin, jacfun, plotter] = kin3();

%% Dynamics
[dyn,inv_dyn]=simple_dyn();

%% Plan
% c: desired=@(t)

x0=zeros(3,2);

t=sym('t','real');
%p=Planner.fromSym(0.4*ones(3,1),t);
p=Planner.fromSym([sin(t)*ones(2,1);0]);
%p=Planner.trapezoid(x0,[0.4*ones(3,1),zeros(3,1)],1,1);

%% Control
% c: tau=@(desired,actual)

Krobust = 0.1*eye(3);
Lambda =0.5*eye(3);
Kp=.1*eye(3);
Kv=.1*eye(3);
Kpi=diag([.1 .1 3*ones(1, 3)]);
dt = 0.01;
%c = Controller.ComputedTorque(inv_dyn,Kp,Kv);
%c = Controller.RobustComputedTorque(Krobust,Lambda);
c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);

%% Noise
% n: desired=@(t)
n=@(p,v,tau) normrnd(3,2, [3 1]) + 20 * flipud(v);


%% Simulation

t_span=0:dt:5;

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
for i = 1:1
  for i = 1:size(pos, 2)
    [xdata, ydata, zdata] = plotter(pos(:, i));
    goal = p(dt * i);
    [gxdata, gydata, gzdata] = plotter(goal(:, 1));
    set(TruePlot, 'XData', xdata);
    set(TruePlot, 'YData', ydata);
    set(TruePlot, 'ZData', zdata);
    set(GoalPlot, 'XData', gxdata);
    set(GoalPlot, 'YData', gydata);
    set(GoalPlot, 'ZData', gzdata);
    drawnow;
  end
end
clf;
pause;

% Try without adaptiveness.
Kpi=zeros(5);
c=Controller.AdaptiveSimple(inv_dyn,Kv,Kv^-1 * Kp, Kpi,dt);
[adaptive_pos,adaptive_vel]=run_sim(x0(:,1),x0(:,2),p,c,n,t_span);

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
title('Position and Adaptive Control');
figure
plot(t_span,vel, t_span, traj_vel);
title('Velocity and Adaptive Control');
figure
plot(t_span,adaptive_pos, t_span, traj_pos);
title('Position without Adaptive Control');
figure
plot(t_span,adaptive_vel, t_span, traj_vel);
title('Velocity without Adaptive Control');
