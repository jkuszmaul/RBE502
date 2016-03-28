function [p1, v1] = sim_step(p, v, tau, dt, noise)
  [dyn, inv_dyn] = simple_dyn();
  a = inv_dyn(p, v, tau) + noise(p, v, tau);
  p1 = p + v * dt + 0.5 * a * dt^2;
  v1 = v + a * dt;
end
