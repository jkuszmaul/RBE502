function [dyn, inv_dyn] = get_dyn()
  end_mass = 0.1;
  control_mass = 0.2;
  control_radius = 0.2; % The distance the mass is from the center.

  [ret_func ,inv_kin, inv_vel] = inv_kin3();
  inv_kin = inv_kin(:, 1);
  inv_vel = inv_vel(:, 1);
  t = sym('t');
  goal = sym('goal', [3 1]);
  sym(goal, 'real');
  sym_vel = sym('sym_vel', [3 1]);
  sym(sym_vel, 'real');
  p1(t) = sym('p1(t)');
  p2(t) = sym('p2(t)');
  p3(t) = sym('p3(t)');
  p = [p1(t); p2(t); p3(t)];
  v = diff(p, t);
  a = diff(v, t);
  inv_kin = subs(inv_kin, goal, p);
  inv_vel = subs(inv_vel, goal, p);
  inv_vel = subs(inv_vel, sym_vel, v);

  Kend = sum(v.*v) * end_mass;
  Kjoints = 0;
  for i = 1:3
    Kjoints = Kjoints + control_mass * control_radius^2 * inv_vel(i).*inv_vel(i);
  end
  P = control_mass * 9.8 * control_radius * sum(sin(inv_kin)) + end_mass * 9.8 * p(3);
  L = Kend + Kjoints - P;
  dLdp = jacobian(subs(L, p, goal), goal);
  dLdv = jacobian(subs(L, v, sym_vel), sym_vel);
  tau = subs(dLdp, goal, p) - diff(subs(dLdv, sym_vel, v), t);
  tau = tau.';
  atmp = sym('atmp', [3 1]);
  sym(atmp, 'real');
  %dtau = jacobian(subs(tau, diff(v, t), atmp), atmp);
  %dtau = subs(tau, a, atmp);
  %dtau = subs(dtau, v, sym_vel);
  %dtau = subs(dtau, p, goal);
  %dtau = tau - M * atmp;
  %symvar(dtau)
  dyn = @(pos, vel, accel) subs(subs(subs(tau, a, accel), v, vel), p, pos);
  inv_dyn = @calc_inv_dyn

  function accel = calc_inv_dyn(pos, vel, forces)
    tic
    eqs = forces == dyn(pos, vel, atmp);
    sol = solve(eqs, atmp(1), atmp(2), atmp(3))
    accel = [sol.atmp1; sol.atmp2; sol.atmp3];
    toc
  end
end
