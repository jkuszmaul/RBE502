% fun: A forward kinematics functoins.
% jacfun: A function for computing the jacobian.
% Both functions take a vector of 3 joint angles and return something.
% fun Returns a [x;y;z] vector for position; jacfun returns a 3x3
% jacobian for computing x, y, and z velocities.
function [fun, jacfun, plot] = kin3()
  top_r = 1.0;
  bot_r = 0.1;
  elbow_len = 1;
  link_len = 3;
  fun = @fwd_ret;
  th = sym('th', [3 1]);
  jac = jacobian(fun(th), th);
  jacfun = matlabFunction(jac, 'Vars', {th});
  plot = @plotter;

  function i2 = fwd_ret(theta)
    elbow_pts = start_constraints(top_r - bot_r, elbow_len, theta.');
    p1 = elbow_pts(1, :).';
    p2 = elbow_pts(2, :).';
    p3 = elbow_pts(3, :).';
    r = link_len;
    % Faster code for intersecting sphere.
    p21 = p2-p1;
    p31 = p3-p1;
    c = cross(p21,p31);
    c2 = sum(c.^2);
    u1 = cross(((sum(p21.^2)+r^2-r^2)*p31 - ...
      (sum(p31.^2)+r^2-r^2)*p21)/2,c)/c2;
    v = sqrt(r^2-sum(u1.^2))*c/sqrt(c2);
    %i1 = p1+u1+v;
    i2 = p1+u1-v;
  end

  function [xdata, ydata, zdata] = plotter(theta)
    pos = fwd_ret(theta);
    end_tri = link_ends(pos, bot_r).';
    top_tri = link_ends([0;0;0], top_r).';
    elbows = start_constraints(top_r, elbow_len, theta).';
    xdata = [];
    ydata = [];
    zdata = [];
    for i = 1:3
      points = [];
      points(:, 1) = end_tri(:, 1+mod(i, 3));
      points(:, 2) = end_tri(:, i);
      points(:, 3) = elbows(:, i);
      points(:, 4) = top_tri(:, i);
      points(:, 5) = top_tri(:, 1+mod(i, 3));
      points(:, 6) = points(:, 4);
      points(:, 7) = points(:, 3);
      points(:, 8) = points(:, 2);
      xdata = [xdata points(1, :)];
      ydata = [ydata points(2, :)];
      zdata = [zdata points(3, :)];
    end
  end
end

function pts = link_ends(pos, bot_r)
  ptx = @(angle) pos(1, 1) + bot_r * cos(angle);
  pty = @(angle) pos(2, 1) + bot_r * sin(angle);
  ptz = pos(3);
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
