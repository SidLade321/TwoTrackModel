% TEST_VEHICLEODE  Stage 4 checks: the car drives, turns, and settles.
clear; clc; close all;
p = params();
deg = pi/180;
% MaxStep stops ode45 taking huge steps on the straight and then overshooting
% when the steer input arrives (which gave spurious wheel-lift warnings).
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);

%% Check 1: straight line -> no yaw, no lateral motion
[~, X] = ode45(@(t,x) vehicleODE(t, x, p, @(t) 0, 15), [0 3], [15; 0; 0], opts);
assert(max(abs(X(:,2))) < 1e-6 && max(abs(X(:,3))) < 1e-6, 'Car drifts in a straight line');
fprintf('Check 1 passed: straight line stays straight.\n');

%% Check 2: low speed, small steer -> kinematic yaw rate r = v*delta/L
v = 5; d = 1*deg;
[~, X] = ode45(@(t,x) vehicleODE(t, x, p, @(t) d, v), [0 5], [v; 0; 0], opts);
r_kin = v * d / p.L;
err = abs(X(end,3) - r_kin) / r_kin * 100;
fprintf('Check 2: yaw rate %.4f rad/s vs kinematic %.4f rad/s (%.1f%% diff)\n', X(end,3), r_kin, err);
assert(err < 5, 'Low-speed yaw rate does not match kinematic value');

%% Check 3: symmetry -> steering right mirrors steering left
[~, XL] = ode45(@(t,x) vehicleODE(t, x, p, @(t)  2*deg, 15), [0 3], [15; 0; 0], opts);
[~, XR] = ode45(@(t,x) vehicleODE(t, x, p, @(t) -2*deg, 15), [0 3], [15; 0; 0], opts);
assert(abs(XL(end,3) + XR(end,3)) < 1e-6, 'Left/right asymmetry');
fprintf('Check 3 passed: left and right turns are mirror images.\n');

%% Demo: step steer at 15 m/s
v = 15;
% 4 deg steer, ramped in over 0.1 s from t = 0.5 s (fast driver input, as in
% ISO 7401). An instant step makes ode45 try rejected trial steps that
% spuriously predict wheel lift.
steer = @(t) 4*deg * min(max((t - 0.5)/0.1, 0), 1);
[t, X] = ode45(@(t,x) vehicleODE(t, x, p, steer, v), [0 4], [v; 0; 0], opts);

% Recompute outputs along the solution
n = numel(t); ay = zeros(n,1); beta = zeros(n,1); Fz = zeros(n,4);
for i = 1:n
    [~, o] = vehicleODE(t(i), X(i,:)', p, steer, v);
    ay(i) = o.ay; beta(i) = o.beta; Fz(i,:) = o.Fz;
end

assert(min(Fz(:)) > 0, 'A wheel lifted during the manoeuvre');
fprintf('Check 4 passed: no wheel lift (min load %.0f N).\n', min(Fz(:)));

% Trajectory: integrate heading and global position
psi = cumtrapz(t, X(:,3));
Xg  = cumtrapz(t, X(:,1).*cos(psi) - X(:,2).*sin(psi));
Yg  = cumtrapz(t, X(:,1).*sin(psi) + X(:,2).*cos(psi));

fig = figure('Name','Step steer');
subplot(2,2,1); plot(t, X(:,3), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Yaw rate [rad/s]'); title('Yaw rate');
subplot(2,2,2); plot(t, ay/p.g, 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('a_y [g]'); title('Lateral acceleration');
subplot(2,2,3); plot(t, beta/deg, 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('\beta [deg]'); title('Body slip angle');
subplot(2,2,4); plot(Xg, Yg, 'LineWidth', 1.5); grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); title('Trajectory');
sgtitle(sprintf('Step steer: %.0f deg at %.0f m/s', 4, v));
drawnow;
exportgraphics(fig, 'step_steer.png', 'Resolution', 200);   % saved next to the scripts

fprintf('\nStep steer 4 deg at 15 m/s, steady state:\n');
fprintf('  Yaw rate : %.3f rad/s\n', X(end,3));
fprintf('  Lat acc  : %.2f g\n', ay(end)/p.g);
fprintf('  Body slip: %.2f deg\n', beta(end)/deg);
fprintf('  Radius   : %.1f m\n', X(end,1)/X(end,3));
fprintf('  Corner loads [FL FR RL RR]: %.0f %.0f %.0f %.0f N\n', Fz(end,:));
fprintf('\nAll checks passed.\n');
