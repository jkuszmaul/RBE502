function [poses, vels] = run_sim(pi, vi, traj, control, noise, dt, num_iter)
  % control: function, should take (goalstate, curstate), state = [p; v].
  % noise: function, takes (p, v, tau).
  poses(:, 1) = pi;
  vels(:, 1) = vi;
  for i = 2:num_iter
    pos = poses(:, i-1);
    vel = vels(:, i-1);
    tau = control(traj(:, i-1), [pos; vel]);
    taus(:, i-1) = tau;
    [poses(:, i), vels(:, i)] = sim_step(pos, vel, tau, dt, noise);
  end
  taus
end
