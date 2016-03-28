function [fun] = kin3()
  top_r = 1.0;
  bot_r = 0.1;
  elbow_len = 1;
  link_len = 3;
  fun = @fwd_ret;
  return

  t = sym('t');
  pos = sym('pos', [3 1]);

  end_pts = link_ends(pos, bot_r);

  theta = sym('theta', [1, 3]);

  elbow_pts = start_constraints(top_r, elbow_len, theta);
  link_cts = simplify(link_constraints(elbow_pts, end_pts, link_len)) == 0
  st = sym('st', [1 3]);
  ct = sym('ct', [1 3]);
  link_cts = subs(link_cts, sin(theta), st);
  link_cts = subs(link_cts, cos(theta), ct);
  ret = solve(link_cts, pos(1), pos(2), pos(3));
  ret = [ret.pos1(2); ret.pos2(2); ret.pos3(2)];
  %fun = @(th) solve(subs(link_cts, theta, th));
  %fun = @(th) subs(subs(ret, st, sin(th)), ct, cos(th));

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
