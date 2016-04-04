function [poses, vels] = run_sim(pi, vi, traj, control, noise, t)
  % traj: function handle, returning row vector and taking a time.
  % control: function, should take (goalstate, curstate), state = [p; v].
  % noise: function, takes (p, v, tau).
  % t: A set of times to use (eg, pass in 0:0.01:2).
  poses(:, 1) = pi;
  vels(:, 1) = vi;
  for i = 2:size(t, 2)
    pos = poses(:, i-1);
    vel = vels(:, i-1);
    tau = control(traj(t(i)), [pos; vel]);
    taus(:, i-1) = tau;
    [poses(:, i), vels(:, i)] = sim_step(pos, vel, tau, t(i)-t(i-1), noise);
  end
end
