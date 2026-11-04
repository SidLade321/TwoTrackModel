% TEST_BICYCLE  Stage 7a: validate the two-track model against a bicycle model.
%   1) Linear range: yaw gain and eigenvalues should match exactly, because
%      about straight-line driving there is no load transfer.
%   2) Skidpad: the models should agree at low g and separate at high g.
%      The gap is what lateral load transfer adds.
clear; clc; close all;
p = params();
deg = pi/180;

fig = figure('Name', 'Bicycle validation', 'Position', [100 100 1100 750]);

%% 1) Linear range: yaw-rate gain vs speed
speeds = 5:1:30;
G2 = zeros(size(speeds)); G1 = G2;
E2 = zeros(2, numel(speeds)); E1 = E2;
for i = 1:numel(speeds)
    lin = linearise(p, speeds(i));    G2(i) = lin.Gr0; E2(:,i) = sort(real(lin.eig));
    bm  = bicycleModel(p, speeds(i)); G1(i) = bm.Gr0;  E1(:,i) = sort(real(bm.eig));
end
err = 100 * (G2 - G1) ./ G1;

subplot(2,2,1);
plot(speeds, G1, 'LineWidth', 2, 'DisplayName', 'Bicycle'); hold on;
plot(speeds, G2, 'o', 'DisplayName', 'Two-track (linearised)');
plot(speeds, speeds/p.L, '--k', 'DisplayName', 'Kinematic v/L');
grid on; xlabel('Speed [m/s]'); ylabel('r/\delta [1/s]');
title('Steady-state yaw gain'); legend('Location', 'northwest');

subplot(2,2,2);
plot(speeds, E1(1,:), 'LineWidth', 2, 'Color', [0 0.447 0.741], 'DisplayName', 'Bicycle'); hold on;
plot(speeds, E1(2,:), 'LineWidth', 2, 'Color', [0 0.447 0.741], 'HandleVisibility', 'off');
plot(speeds, E2(1,:), 'o', 'Color', [0.85 0.325 0.098], 'DisplayName', 'Two-track (linearised)');
plot(speeds, E2(2,:), 'o', 'Color', [0.85 0.325 0.098], 'HandleVisibility', 'off');
grid on; xlabel('Speed [m/s]'); ylabel('Eigenvalue [1/s]');
title('Yaw/lateral eigenvalues (all real, negative = stable)'); legend('Location', 'southeast');

v = 15;
lin = linearise(p, v);    e2 = sort(lin.eig);
bm  = bicycleModel(p, v); e1 = sort(bm.eig);
fprintf('Linear range\n');
fprintf('  Max yaw-gain difference over 5-30 m/s: %.2e %%\n', max(abs(err)));
fprintf('  Eigenvalues at 15 m/s: two-track [%.2f %.2f], bicycle [%.2f %.2f]\n', e2, e1);
fprintf('  Straight-line understeer gradient: %.3f deg/g (negative = slight oversteer)\n', ...
    bm.Kus * p.g / deg);

%% 2) Skidpad: steer vs lateral g, and grip limit
R = 9.125;
S  = sweepRadius(p, R);                                  % baseline roll split
q = p; q.Kphi_f = 12000; Sopt = sweepRadius(q, R);       % stage 5 optimum (40%)

vb = 3:0.02:20; ayb = []; db = [];
for vv = vb
    [d, ok] = bicycleSteadyState(p, R, vv);
    if ~ok, break; end
    ayb(end+1) = vv^2/R/p.g; db(end+1) = d/deg; %#ok<AGROW>
end
ay2 = [S.ay]/p.g;    d2 = [S.delta]/deg;
ay3 = [Sopt.ay]/p.g; d3 = [Sopt.delta]/deg;

subplot(2,1,2);
plot(ayb, db, 'LineWidth', 2, 'DisplayName', sprintf('Bicycle (no load transfer) - max %.2f g', ayb(end))); hold on;
plot(ay2, d2, 'LineWidth', 2, 'DisplayName', sprintf('Two-track, 53%% front roll - max %.2f g', ay2(end)));
plot(ay3, d3, 'LineWidth', 2, 'DisplayName', sprintf('Two-track, 40%% front roll - max %.2f g', ay3(end)));
plot([ayb(end) ay2(end) ay3(end)], [db(end) d2(end) d3(end)], 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; xlabel('a_y [g]'); ylabel('Steer angle \delta [deg]');
title(sprintf('Skidpad (R = %.3f m): agreement at low g, divergence at high g', R));
legend('Location', 'northwest');

drawnow;
exportgraphics(fig, 'bicycle_validation.png', 'Resolution', 200);

% Steer difference at a few lateral g levels
fprintf('\nSkidpad R = %.3f m\n', R);
fprintf('  %-8s %-14s %-14s %-10s\n', 'ay [g]', 'bicycle [deg]', 'two-track [deg]', 'diff [deg]');
for q_ay = [0.2 0.5 1.0 1.4]
    [~, i1] = min(abs(ayb - q_ay)); [~, i2] = min(abs(ay2 - q_ay));
    fprintf('  %-8.1f %-14.3f %-14.3f %-10.3f\n', q_ay, db(i1), d2(i2), d2(i2) - db(i1));
end
fprintf('\nGrip limit\n');
fprintf('  Bicycle (no load transfer): %.2f g\n', ayb(end));
fprintf('  Two-track, 53%% front roll : %.2f g  (%.1f%% lower)\n', ay2(end), 100*(1 - ay2(end)/ayb(end)));
fprintf('  Two-track, 40%% front roll : %.2f g  (%.1f%% lower)\n', ay3(end), 100*(1 - ay3(end)/ayb(end)));

%% Checks
assert(max(abs(err)) < 0.1, 'Linear yaw gain does not match bicycle model');
assert(max(abs(e2 - e1)) < 0.01*max(abs(e1)), 'Eigenvalues do not match bicycle model');
[~, i1] = min(abs(ayb - 0.5)); [~, i2] = min(abs(ay2 - 0.5));
assert(abs(d2(i2) - db(i1)) < 0.2, 'Models disagree at low g');
assert(ay2(end) < ayb(end), 'Load transfer should reduce the grip limit');
fprintf('\nAll checks passed.\n');
