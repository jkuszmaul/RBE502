function [ret, sym_theta, diff_theta, plots] = inv_kin()
  % Returns the inverse kinematics function handle, which
  % takes an end-effector homogeneous transformation matrix.
  % Return values:
  %  ret: A function handle that takes a homogeneous transformation
  %       and returns the joint angles.
  %  sym_theta: A 6x2 symbolic matrix of the joint angles as a function
  %       of H (a homogeneous tranformation matrix. The two columns represent
  %       the two possible solutions for each joint.
  %  plots: A Function which takes a homogeneous transformation matrix and a set
  %       of joint variables and plots the arm.
  ret = 0;
  H = sym('H', [4, 4]);
  H(4, :) = [0 0 0 1];

  % Various Parameters.
  link_sep = .1; % Distance between links in pairs of link segments.
  long_side = .2; % Distance between pair sets.
  elbow_len = 1; % Length of first link.
  link_len = 3; % Length of second link.

  % Begin actually calculating.
  % Get the joint connection locations on the end effector.
  hex_pts = link_ends(H, link_sep, long_side);
  theta = sym('theta', [1, 6]);
  sym(theta, 'real');
  % Get the positions of the joint between the two links.
  elbow_pts = start_constraints(link_sep, long_side, elbow_len, theta);
  % Get the actual link constraints.
  link_cts = simplify(link_constraints(elbow_pts, hex_pts, link_len)) == 0;
  sym_theta = sym('sym_theta', [6, 2]);
  for i = 1:6
    i
    res = solve(link_cts(i), theta(i), 'Real', 1);
    sym_theta(i, 1:2) = res';
  end

  ret = @eval_ret;
  plots = @plotter;

  t = sym('t');
  R11(t) = sym('R11(t)');
  R12(t) = sym('R12(t)');
  R13(t) = sym('R13(t)');
  R21(t) = sym('R21(t)');
  R22(t) = sym('R22(t)');
  R23(t) = sym('R23(t)');
  R31(t) = sym('R31(t)');
  R32(t) = sym('R32(t)');
  R33(t) = sym('R33(t)');
  D1(t) = sym('D1(t)');
  D2(t) = sym('D2(t)');
  D3(t) = sym('D3(t)');
  H2 = [R11 R12 R13 D1;
        R21 R22 R23 D2;
        R31 R32 R33 D3;
        0   0   0   1]
  diff_theta = subs(sym_theta, H, H2);%diff(sym_theta, H);

  deriv = diff_theta;
  for i = 1:6
    for j = 1:2
      tmp(t) = diff_theta(i, j);
      deriv(i, j) = diff(tmp, t);
    end
  end

  Hdot = sym('Hdot', [3 4]);
  deriv = subs(deriv, diff(R11, t), Hdot(1, 1));
  deriv = subs(deriv, diff(R12, t), Hdot(1, 2));
  deriv = subs(deriv, diff(R13, t), Hdot(1, 3));
  deriv = subs(deriv, diff(R21, t), Hdot(2, 1));
  deriv = subs(deriv, diff(R22, t), Hdot(2, 2));
  deriv = subs(deriv, diff(R23, t), Hdot(2, 3));
  deriv = subs(deriv, diff(R31, t), Hdot(3, 1));
  deriv = subs(deriv, diff(R32, t), Hdot(3, 2));
  deriv = subs(deriv, diff(R33, t), Hdot(3, 3));
  deriv = subs(deriv, diff(D1, t), Hdot(1, 4));
  deriv = subs(deriv, diff(D2, t), Hdot(2, 4));
  deriv = subs(deriv, diff(D3, t), Hdot(3, 4));
  diff_theta = deriv;

  function [best, dtdH] = eval_ret(H_act, H_vel)
    tic
    angles = eval(subs(sym_theta, H, H_act));
    % Evaluate derivative:
    deriv = diff_theta;
    deriv = subs(deriv, Hdot, H_vel(1:3, 1:4));
    subs_pos = [R11(t) R12(t) R13(t) D1(t);
                R21(t) R22(t) R23(t) D2(t);
                R31(t) R32(t) R33(t) D3(t)];
    deriv = subs(deriv, subs_pos(1:3, 1:4), H_act(1:3, 1:4));
    deriv = eval(deriv);
    best = angles(:, 1);
    dtdH = deriv(:, 1);
    % Now, always choose the values with the smaller absolute value.
    for i = 1:6
      % Only need to change anything if abs(2) < abs(1)
      if abs(angles(i, 2)) < abs(angles(i, 1))
        best(i) = angles(i, 2);
        dtdH(i) = deriv(i, 2);
      end
    end
  end

  function plotter(H_act, joints)
    % Do plotting.
    eval_elb = subs(elbow_pts, theta, joints');
    eval_hex = subs(subs(hex_pts, theta, joints'), H, H_act);
    % Get locations of motor joints.
    base_hex = eval(link_ends(eye(4), link_sep, long_side));
    new_plot = zeros(6 * 3, 3);
    for i = 1:3
      new_plot(6*i-5, :) = base_hex(2*i-1, :);
      new_plot(6*i-4, :) = eval_elb(2*i-1, :);
      new_plot(6*i-3, :) = eval_hex(2*i-1, :);
      new_plot(6*i-2, :) = eval_hex(2*i, :);
      new_plot(6*i-1, :) = eval_elb(2*i, :);
      new_plot(6*i, :) = base_hex(2*i, :);
    end
    x_plot = new_plot(:, 1);
    y_plot = new_plot(:, 2);
    z_plot = new_plot(:, 3);
    eval_hex(7, :) = eval_hex(1, :); % Makes graph prettier :)
    plot3(x_plot, y_plot, z_plot, eval_hex(:, 1), eval_hex(:, 2), eval_hex(:, 3),...
          'LineStyle', '-', 'Marker', 'o');
  end

end

function pts = link_ends(H, short, long)
  rad = (long + 2 * abs(short)) / sqrt(3) - abs(short);
  ptx = @(i, off) rad * cos(2 * i * pi / 3) - off * sin(2 * i * pi / 3);
  pty = @(i, off) rad * sin(2 * i * pi / 3) + off * cos(2 * i * pi / 3);
  short = short / 2;
  end_frame_pts = [ptx(0, -short) pty(0, -short) 0 1;
                   ptx(0, short) pty(0, short) 0 1;
                   ptx(1, -short) pty(1, -short) 0 1;
                   ptx(1, short) pty(1, short) 0 1;
                   ptx(2, -short) pty(2, -short) 0 1;
                   ptx(2, short) pty(2, short) 0 1];
  %pts = zeros(6, 3);
  pts = sym('pts', [6, 3]);
  for row = 1:size(end_frame_pts, 1)
    tmp = (H * end_frame_pts(row, :)')';
    pts(row, :) = tmp(1, 1:3);
  end
end

function cts = link_constraints(elbow_pts, hex_pts, link_len)
  dist2 = sum((elbow_pts - hex_pts).^2, 2);
  cts = dist2 - link_len^2;
end

function pts = start_constraints(short, long, link_len, theta)
  pos = @(i, off) elbow_pos(off, long, link_len, floor(i / 2), theta(i+1));
  pts = [pos(0, -short);
         pos(1, short);
         pos(2, -short);
         pos(3, short);
         pos(4, -short);
         pos(5, short)];
end

function pos = elbow_pos(hexs, hexl, link_len, hex_pos, link_theta)
  pos = sym('pos', [1, 3]);
  rad = (hexl + 2 * abs(hexs)) / sqrt(3) - abs(hexs);
  pos(3) = link_len * sin(link_theta); % z
  out_dist = rad + link_len * cos(link_theta);
  inc = 2 * pi / 3;
  pos(1) = out_dist * cos(hex_pos * inc) - hexs / 2 * sin(hex_pos * inc);
  pos(2) = out_dist * sin(hex_pos * inc) + hexs / 2 * cos(hex_pos * inc);
end
