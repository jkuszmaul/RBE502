% function theta = InverseKinematics(P, Lc, Lf, Rb, Re, Lo)
% 
% Calcualtes the motor angles reqired to position the end effector at the
% specified point P. P is defined as a column vector of X, Y, Z base co-ords.
% P has three rows, X, Y, Z.
% theta is also a column vector with three rows, theta1, theta2 and theta3
%
%
%   Parameters:
%       P       -   Column vector of base co-ordinates to perform inverse
%                   kinematics on.
%       Lc      -   Length of the contorl arm.
%       Lf      -   Length of the forearm.
%       Rb      -   Distance from center of base to motor axes.
%       Re      -   Distance form center of base to forearm joint.
%       Lo      -   Vector of contorl arm offsets along each motor axes.
%                   (m1, m2, m3)
%
%   Returns:
%       theta   -   Column vector of motor angles corresponding to the base
%                   co-ordinate in the corresponding column of P.
%                   -1 for out of range.
%       alpha   -   Angle that the forearm makes with its projection on the
%                   XY plane
% 

function [theta alpha] = DeltaInverseKinematics(P, Lc, Lf, Rb, Re, Lo)

% Set up motor axes. The matricies transform a point in motor co-ordinates
% to base co-ordinates

motorAxes = dhtrans(0, 0, Rb-Re, -pi/2); % The general motor axes
motor1 = motorAxes * trans(0, 0, -Lo(1));
motor2 = rotz(2*pi/3) * motorAxes * trans(0, 0, -Lo(2));
motor3 = rotz(4*pi/3) * motorAxes * trans(0, 0, -Lo(3));

% Need to obtain motor co-ordinates from base co=ordinates so need to
% invert these motor matricies. invht does this whilst still preserving the
% augmented homogenous dimention.

P(4, :) = 1;
Pmotor1 = invht(motor1) * P;
Pmotor2 = invht(motor2) * P;
Pmotor3 = invht(motor3) * P;

% Calculate phi for each motor: phi = atan2(Ly, Lx)
phi1 = atan2(Pmotor1(2, :), Pmotor1(1, :)); % * 180 / pi
phi2 = atan2(Pmotor2(2, :), Pmotor2(1, :));% * 180 / pi
phi3 = atan2(Pmotor3(2, :), Pmotor3(1, :)); % * 180 / pi

% phi1 = atan2(Pmotor1(1, :), Pmotor1(2, :)); % * 180 / pi
% phi2 = atan2(Pmotor2(1, :), Pmotor2(2, :));% * 180 / pi
% phi3 = atan2(Pmotor3(1, :), Pmotor3(2, :)); % * 180 / pi

% phi1 = atan2(Pmotor1(1, :), Pmotor1(2, :))
% phi2 = atan2(Pmotor2(1, :), Pmotor2(2, :))
% phi3 = atan2(Pmotor3(1, :), Pmotor3(2, :))

% phi1 = atan(Pmotor1(2, :) ./ Pmotor1(1, :)) *180 / pi
% phi2 = atan(Pmotor2(2, :) ./ Pmotor2(1, :)) *180 / pi
% phi3 = atan(Pmotor3(2, :) ./ Pmotor3(1, :)) *180 / pi

% Calculate psi for each motor
psi1 = acos((Pmotor1(1, :).^2 + Pmotor1(2, :).^2 + Pmotor1(3, :).^2 + Lc^2 - Lf^2) ...
            ./ (2 * Lc * sqrt(Pmotor1(1, :).^2 + Pmotor1(2, :).^2))); % *180 / pi
psi2 = acos((Pmotor2(1, :).^2 + Pmotor2(2, :).^2 + Pmotor2(3, :).^2 + Lc^2 - Lf^2) ...
            ./ (2 * Lc * sqrt(Pmotor2(1, :).^2 + Pmotor2(2, :).^2))); % *180 / pi
psi3 = acos((Pmotor3(1, :).^2 + Pmotor3(2, :).^2 + Pmotor3(3, :).^2 + Lc^2 - Lf^2) ...
            ./ (2 * Lc * sqrt(Pmotor3(1, :).^2 + Pmotor3(2, :).^2))); % *180 / pi

% Also calculate alphas
alpha(1, :) = asin(Pmotor1(3, :) / Lf);
alpha(2, :) = asin(Pmotor2(3, :) / Lf);
alpha(3, :) = asin(Pmotor3(3, :) / Lf);

theta(1, :) = phi1 - psi1;
theta(2, :) = phi2 - psi2;
theta(3, :) = phi3 - psi3;
%theta = theta*180/pi;
end

