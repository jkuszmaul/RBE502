function test_pid()

pi = [0; 0; 0];
vi = [0; 0; 0];
t = 0:0.05:4.99
traj = @(t) [sin([t; t; t]); cos([t;t;t])];
[poses, vels] = run_sim(pi, vi, traj, @plaw, @noise, t);
poses
vels
act_traj = traj(t);
act_traj(1, :)
plot(t, poses, t, act_traj(1, :));
%axis([0 iter 0 0.5]);
xlabel('Iteration Number');
ylabel('Joint angle');
title('Simple P-Controller on each joint, with gaussian noise');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Goal trajectories');

function torques = plaw(goal, cur)
  p = 0.5;
  torques = p * (goal(1:3) - cur(1:3));
end

function n = noise(p, v, tau)
  n = normrnd(0.2, 2, [3 1]);
end
end
