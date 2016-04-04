% [dyn, inv_dyn] = simple_dyn()
% This returns functions for calculating the dynamics of a single joint arm..
% Because we are modelling the Delta robots as several independent joints,
% the dynamics are relatively trivial to calculate.
% Usage:
% For instance, if you want to calculate a single step of the system:
% [inv_dyn, dyn] = simple_dyn();
% a = dyn(p, v, tau);
% p1 = p + v * dt + 0.5 * a * dt^2;
% v1 = v + a * dt;
function [dyn, inv_dyn] = simple_dyn()
  control_mass = 0.2;
  control_radius = 0.2; % The distance the mass is from the center.
  %[ret_func ,inv_kin, fwd_kin, inv_vel] = inv_kin3();
  % We just treat each control arm as an independent joint.
  % As such, the dynamics are equivalent to that of a 1-link arm.
  I = control_mass * control_radius^2;
  B = 0.1; % Friction.
  g = 9.8;
  inv_dyn = @(pos, vel, accel) I * accel + B * vel + control_mass * g * control_radius * sin(pos);
  dyn = @(pos, vel, tau) (tau - B * vel - control_mass * g * control_radius * sin(pos)) / I;
end
