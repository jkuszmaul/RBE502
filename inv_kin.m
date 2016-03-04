function [ret, link_cts, elbow_pts, hex_pts] = inv_kin()
  ret = 0;
  %H = sym('H', [4, 4]);
  H = eye(4);
  H(4, :) = [0 0 0 1];

  % Translational component:
  H(1:3, 4) = [0; 1; -4];

  % Rotational component:
  theta = 0.0;
  H(1:3, 1:3) = [cos(theta) -sin(theta) 0;
                 sin(theta) cos(theta) 0;
                 0          0          1];
  % Various Parameters.
  link_sep = .1; % Distance between links in pairs of link segments.
  long_side = .2; % Distance between pair sets.
  elbow_len = 1; % Length of first link.
  link_len = 4; % Length of second link.

  % Begin actually calculating.
  % Get the joint connection locations on the end effector.
  hex_pts = link_ends(H, link_sep, long_side);
  theta = sym('theta', [1, 6]);
  sym(theta, 'real');
  % Get the positions of the joint between the two links.
  elbow_pts = start_constraints(link_sep, long_side, elbow_len, theta);
  % Get the actual link constraints.
  link_cts = simplify(link_constraints(elbow_pts, hex_pts, link_len)) == 0;
  ret = sym('ret', [6, 1]);
  for i = 1:6
    res = solve(link_cts(i), theta(i), 'Real', 1);
    % Choose the result that actually makes sense (abs(theta) < pi/2).
    if (abs(res(1)) < pi / 2)
      ret(i) = res(1)
    else
      ret(i) = res(2);
    end
  end

  % Do plotting.
  eval_elb = eval(subs(elbow_pts, theta, ret'));
  eval_hex = eval(subs(hex_pts, theta, ret'));
  % Get locations of motor joints.
  base_hex = eval(link_ends(eye(4), link_sep, long_side))
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
  plot3(x_plot, y_plot, z_plot, 'LineStyle', '-', 'Marker', 'o');
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
  dist2 = sum((elbow_pts - hex_pts).^2, 2)
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
