function test_pid()

pi = [0; 0; 0];
vi = [0; 0; 0];
goalp = [0.4; 0.4; 0.4];
goalv = [0; 0; 0];
iter = 100;
traj = repmat([goalp; goalv], 1, iter);
tmps = 0:0.05:4.99
traj = [sin([tmps; tmps; tmps]); cos([tmps;tmps;tmps])];
[poses, vels] = run_sim(pi, vi, traj, @plaw, @noise, 0.01, iter);
poses
vels
plot(1:iter, poses, 1:iter, traj(1, :));
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
