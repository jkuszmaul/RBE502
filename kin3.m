function ret = kin()
pair_separation = 0.1;
%pair_radius = 3.0;
base_r = pair_separation; %sqrt(pair_radius^2 + (pair_separation / 2)^2);
%pair_dtheta = atan2(pair_separation / 2, pair_radius);
pair_dtheta = pi / 6;
link_length = 1.0;

elbows = @(t)[elbow_pos(base_r, link_length, 0, t(1));
              elbow_pos(base_r, link_length, 2*pi/3, t(2));
              elbow_pos(base_r, link_length, 4*pi/3, t(3))];
act_elbows = elbows([1 1 0])

pos = sym('pos', [3, 3]);
pos_all = pos;%[pos; pos; pos];
eqs = [0 == reg_tri_constraints(pair_separation, pos);
       0 == elbow_constraints(act_elbows, pos_all, 4)];
%solve(eqs)
%solve(0 == reg_hex_constraints(pair_separation, pos))
ret = eqs;

end

function constraints = reg_tri_constraints(side_len, points)
  constraints = sym('constraints', [3, 1]);
  con_idx = 1;
  for i = 1:2
    for j = i+1:3
      constraints(con_idx) = sum((points(i, 1:2) - points(j, 1:2)).^2) - side_len^2;
      %constraints(con_idx+1) = points(i, 3) - points(j, 3);
      con_idx = con_idx+1;
    end
  end
end

function constraints = elbow_constraints(elbows, points, link_length)
  dist2 = sum((points - elbows).^2, 2); % Compute norm for each difference.
  constraints = dist2 - link_length^2;
end

function pos = elbow_pos(base_r, link_r, base_angle, link_theta)
  pos = [0 0 0];
  pos(3) = link_r * sin(link_theta); % z
  horiz_dist = base_r + link_r * cos(link_theta);
  pos(1) = horiz_dist * cos(base_angle);
  pos(2) = horiz_dist * sin(base_angle);
end
