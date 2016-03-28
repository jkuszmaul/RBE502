function [dyn, inv_dyn] = simple_dyn()
  end_mass = 0.1;
  control_mass = 0.2;
  control_radius = 0.2; % The distance the mass is from the center.
  %[ret_func ,inv_kin, fwd_kin, inv_vel] = inv_kin3();
  % We just treat each control arm as an independent joint.
  % As such, the dynamics are equivalent to that of a 1-link arm.
  I = control_mass * control_radius^2;
  B = 0.1; % Friction.
  g = 0;%9.8;
  dyn = @(pos, vel, accel) I * accel + B * vel + end_mass * g * control_radius * sin(pos);
  inv_dyn = @(pos, vel, tau) (tau - B * vel - end_mass * g * control_radius * sin(pos)) / I;
end
