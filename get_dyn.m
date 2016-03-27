function dyn = get_dyn(inv_vel, inv_kin, H, Hdot)
  % Fixed parameters:
  end_mass = 0.1;
  end_I = 0.1;
  control_mass = 0.2;
  control_radius = 0.2; % The distance the mass is from the center.

  % S = dRdtheta * R^T
  % dRdtheta * dthetadt = dRdt
  %

  % Actual work:
  end_point = sym('end_point', [6 1]);
  end_vel = sym('end_vel', [6 1]);
  t = sym('t');
  tx(t) = sym('tx(t)');
  ty(t) = sym('ty(t)');
  tz(t) = sym('tz(t)');
  H_xyz = rotx(tx) * roty(ty) * rotz(tz);
  dH_xyz = diff(H_xyz, t);
  inv_kin = subs(inv_kin, H(1:3, 1:3), H_xyz);
  inv_vel = subs(inv_vel, H(1:3, 1:3), H_xyz);
  inv_vel = subs(inv_vel, Hdot(1:3, 1:3), dH_xyz);
  %S = [0 -end_vel(6) end_vel(5);
  %     end_vel(6) 0 -end_vel(4);
  %     -end_vel(5) end_vel(4) 0];
  %S = Hdot(1:3, 1:3) * H(1:3, 1:3)';
  %end_vel = [Hdot(1:3, 4);
  %           -S(2, 3); S(1, 3); -S(1, 2)]
  %Hdot(1:3, 4) = end_vel(1:3);
  %Hdot(4, 1:4) = [0 0 0 1];
  Kend = end_vel(1:3).*end_vel(1:3) * end_mass + ...
         end_vel(4:6)' * end_I * end_vel(4:6);
  Kjoints = 0;
  for i = 1:6
    Kjoint = control_mass * control_radius^2 * inv_vel(i).*inv_vel(i);
    Kjoints = Kjoints + Kjoint;
  end
  P = m * 9.8 * H(3, 4);
  L = (Kend + Kjoints) - P;
  tau = diff(L, end_point) - diff(diff(L, end_vel), end_point);
end

function [tx, ty, tz] = extract_angles(H)
  % First, get options for ty.
  ty1 = asin(H(1, 3))
  ty2 = pi - ty1

  tz1y1 = acos(H(1, 1) / cos(ty1))
  tz1y2 = acos(H(1, 1) / cos(ty2))
  tz2y1 = -acos(H(1, 1) / cos(ty1))
  tz2y2 = -acos(H(1, 1) / cos(ty2))
  % And now, test each possiblility.
  expr = @(ty, tz) -cos(ty) * sin(tz) == H(1, 2);
  if expr(ty1, tz1y1)
    disp '1, 1'
  end
  if expr(ty1, tz2y1)
    disp '1, 2'
  end
  if expr(ty2, tz1y2)
    disp '2, 1'
  end
  if expr(ty2, tz2y2)
    disp '2, 2'
  end
end
