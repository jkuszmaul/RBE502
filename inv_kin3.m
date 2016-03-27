function [ret, sym_theta, diff_theta, plots] = inv_kin3()
  top_r = 1.0;
  bot_r = 0.1;
  elbow_len = 1;
  link_len = 3;

  t = sym('t');
  goal = sym('goal', [3 1]);

  end_pts = link_ends(goal, bot_r);

  theta = sym('theta', [1, 3]);
  sym(theta, 'real');

  elbow_pts = start_constraints(top_r, elbow_len, theta);
  link_cts = simplify(link_constraints(elbow_pts, end_pts, link_len)) == 0;
  sym_theta = sym('sym_theta', [3 2]);
  for i = 1:3
    i
    res = solve(link_cts(i), theta(i), 'Real', 1);
    sym_theta(i, 1:2) = res.';
  end

  P1(t) = sym('P1(t)');
  P2(t) = sym('P2(t)');
  P3(t) = sym('P3(t)');
  post = [P1(t); P2(t); P3(t)];
  t2 = subs(sym_theta, goal, post);
  diff_theta = t2;
  for i = 1:3
    for j = 1:2
      diff_theta(i, j) = diff(t2(i, j), t);
    end
  end
  sym_vel = sym('sym_vel', [3 1]);
  diff_theta = subs(diff_theta, diff(P1(t), t), sym_vel(1, 1));
  diff_theta = subs(diff_theta, diff(P2(t), t), sym_vel(2, 1));
  diff_theta = subs(diff_theta, diff(P3(t), t), sym_vel(3, 1));
  diff_theta = subs(diff_theta, P1(t), goal(1));
  diff_theta = subs(diff_theta, P2(t), goal(2));
  diff_theta = subs(diff_theta, P3(t), goal(3));

  plots = @plotter;
  ret = @eval_ret;

  function [best, dt] = eval_ret(pos_act, vel_act)
    angles = eval(subs(sym_theta, goal, pos_act));
    % Evaluate derivative:
    deriv = diff_theta;
    deriv = subs(deriv, goal, pos_act);
    deriv = simplify(subs(deriv, sym_vel, vel_act));
    deriv = eval(deriv);
    best = angles(:, 1);
    dt = deriv(:, 1);
    % Now, always choose the values with the smaller absolute value.
    for i = 1:3
      % Only need to change anything if abs(2) < abs(1)
      if abs(angles(i, 2)) < abs(angles(i, 1))
        best(i) = angles(i, 2);
        dt(i) = deriv(i, 2);
      end
    end
  end

  function plotter(pos_act)
    joints = eval_ret(pos_act, [0 0 0]');
    % Do plotting.
    eval_elb = subs(elbow_pts, theta, joints.');
    eval_end = subs(end_pts, goal, pos_act);
    % Get locations of motor joints.
    base_hex = link_ends([0 0 0]', top_r);
    new_plot = zeros(3 * 3, 3);
    for i = 1:3
      new_plot(3*i-2, :) = eval_end(i, :);
      new_plot(3*i-1, :) = eval_elb(i, :);
      new_plot(3*i, :) = base_hex(i, :);
    end
    x_plot = new_plot(:, 1);
    y_plot = new_plot(:, 2);
    z_plot = new_plot(:, 3);
    eval_end(4, :) = eval_end(1, :); % Makes graph prettier :)
    plot3(x_plot, y_plot, z_plot, eval_end(:, 1), eval_end(:, 2), eval_end(:, 3),...
          'LineStyle', '-', 'Marker', 'o');
  end
end

function pts = link_ends(goal, bot_r)
  ptx = @(angle) goal(1, 1) + bot_r * cos(angle);
  pty = @(angle) goal(2, 1) + bot_r * sin(angle);
  ptz = goal(3);
  pts = [ptx(0) pty(0) ptz;
         ptx(2 * pi / 3) pty(2 * pi / 3) ptz;
         ptx(4 * pi / 3) pty(4 * pi / 3) ptz];
end

function pts = start_constraints(top_r, elbow_len, theta)
  rad = @(i) top_r + elbow_len * cos(theta(i));
  ptx = @(i) cos((i - 1) * 2 * pi / 3) * rad(i);
  pty = @(i) sin((i - 1) * 2 * pi / 3) * rad(i);
  ptz = @(i) sin(theta(i));
  pts = [ptx(1) pty(1) ptz(1);
         ptx(2) pty(2) ptz(2);
         ptx(3) pty(3) ptz(3)];
end

function cts = link_constraints(elbow_pts, end_pts, link_len)
  dist2 = sum((elbow_pts - end_pts).^2, 2);
  cts = dist2 - link_len^2;
end
