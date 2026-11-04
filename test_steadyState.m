% TEST_STEADYSTATE  Stage 5: constant-radius cornering (ISO 4138 style).
%   Sweeps speed on the FS skidpad radius until the car can no longer hold
%   the circle. Gives the understeer gradient, max lateral g, which axle
%   saturates first, and how all of that changes with roll stiffness split.
clear; clc; close all;
p = params();
deg = pi/180;
R = 9.125;                          % FS skidpad centreline radius [m]

%% 1) Understeer curves for three roll stiffness splits
Kf_list = [10000 20000 40000];
fig = figure('Name', 'Steady state', 'Position', [100 100 1100 750]);
fprintf('Constant radius R = %.3f m\n', R);
fprintf('%-8s %-9s %-10s %-10s %-10s %-10s\n', 'Kphi_f', 'front %', 'K [deg/g]', 'max ay[g]', 'util F', 'util R');

for Kf = Kf_list
    q = p; q.Kphi_f = Kf;
    S = sweep(q, R);
    ay    = [S.ay] / p.g;
    delta = [S.delta] / deg;
    beta  = [S.beta] / deg;

    lin = ay < 0.5;                          % linear range for the gradient
    c = polyfit(ay(lin), delta(lin), 1);
    K = c(1);                                % understeer gradient [deg/g]

    lbl = sprintf('K_{\\phi f} = %d (%.0f%% front)', Kf, 100*Kf/(Kf + p.Kphi_r));
    subplot(2,2,1); hold on; plot(ay, delta, 'LineWidth', 1.5, 'DisplayName', lbl);
    subplot(2,2,2); hold on; plot(ay, beta,  'LineWidth', 1.5, 'DisplayName', lbl);

    fprintf('%-8d %-9.0f %-10.3f %-10.3f %-10.2f %-10.2f\n', Kf, 100*Kf/(Kf+p.Kphi_r), ...
        K, ay(end), S(end).utilF, S(end).utilR);
end

subplot(2,2,1); grid on; yline(p.L/R/deg, '--k', 'Kinematic L/R', 'HandleVisibility', 'off');
xlabel('a_y [g]'); ylabel('Steer angle \delta [deg]');
title('Understeer curve (rising = understeer)'); legend('Location', 'northwest');
subplot(2,2,2); grid on;
xlabel('a_y [g]'); ylabel('Body slip \beta [deg]'); title('Body slip angle');
legend('Location', 'southwest');

%% 2) Max lateral g vs roll stiffness distribution
Kf_sweep = 4000:2000:40000;
maxAy = zeros(size(Kf_sweep));
for i = 1:numel(Kf_sweep)
    q = p; q.Kphi_f = Kf_sweep(i);
    S = sweep(q, R);
    maxAy(i) = S(end).ay / p.g;
end
frontPct = 100 * Kf_sweep ./ (Kf_sweep + p.Kphi_r);
[best, ib] = max(maxAy);

subplot(2,1,2);
plot(frontPct, maxAy, '-o', 'LineWidth', 1.5); grid on; hold on;
plot(frontPct(ib), best, 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'auto');
xline(100*p.wf, '--k', 'Static front weight %');
xlabel('Front roll stiffness distribution [%]'); ylabel('Max a_y [g]');
title(sprintf('Grip limit vs roll stiffness split (best: %.0f%% front, %.2f g)', frontPct(ib), best));

drawnow;
exportgraphics(fig, 'steady_state.png', 'Resolution', 200);

fprintf('\nBest roll stiffness split: %.0f%% front (Kphi_f = %d) -> %.2f g\n', ...
    frontPct(ib), Kf_sweep(ib), best);

%% Checks
q = p; q.Kphi_f = 10000; K10 = gradientOf(sweep(q, R), p);
q = p; q.Kphi_f = 40000; K40 = gradientOf(sweep(q, R), p);
assert(K40 > K10, 'Stiffer front should increase understeer');
fprintf('Check passed: stiffer front -> more understeer (%.3f -> %.3f deg/g).\n', K10, K40);
fprintf('All checks passed.\n');

function K = gradientOf(S, p)
    ay = [S.ay] / p.g; d = [S.delta] * 180/pi;
    c = polyfit(ay(ay < 0.5), d(ay < 0.5), 1); K = c(1);
end

%% ---- Local functions ----
% Sweep speed up until no equilibrium exists
function S = sweep(p, R)
    S = [];
    guess = [];
    for v = 3:0.02:20
        ss = steadyState(p, R, v, guess);
        if ~ss.converged, break; end
        S = [S; ss]; %#ok<AGROW>
        guess = [ss.delta; ss.vy];  % warm start the next speed
    end
end
