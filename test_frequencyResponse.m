% TEST_FREQUENCYRESPONSE  Stage 6: transient response to steering.
%   Linearises the model about straight-line driving and computes the
%   frequency response of yaw rate and lateral acceleration to steer, at
%   several speeds. Then checks the linear result against a full nonlinear
%   sine-steer simulation.
clear; clc; close all;
p = params();
deg = pi/180;
speeds = [10 15 20 25];                   % [m/s]
fHz = logspace(-1, 1, 300);               % 0.1 - 10 Hz
w = 2*pi*fHz;

fig = figure('Name', 'Frequency response', 'Position', [100 100 1100 750]);
fprintf('%-8s %-10s %-10s %-8s %-14s %-14s %-12s\n', 'v [m/s]', 'eig 1', 'eig 2', 'zeta', 'r/delta [1/s]', 'kinematic v/L', 'r lag @1Hz');

for v = speeds
    lin = linearise(p, v);

    % Frequency response H(jw) = C (jwI - A)^-1 B + D
    Hr = zeros(size(w)); Hay = zeros(size(w));
    for k = 1:numel(w)
        X = (1i*w(k)*eye(2) - lin.A) \ lin.B;
        Hr(k)  = X(2);
        Hay(k) = lin.Cay * X + lin.Day;
    end

    lbl = sprintf('%d m/s', v);
    subplot(2,2,1); semilogx(fHz, abs(Hr)/lin.Gr0, 'LineWidth', 1.5, 'DisplayName', lbl); hold on;
    subplot(2,2,2); semilogx(fHz, angle(Hr)/deg,   'LineWidth', 1.5, 'DisplayName', lbl); hold on;
    subplot(2,2,3); semilogx(fHz, angle(Hay)/deg,  'LineWidth', 1.5, 'DisplayName', lbl); hold on;

    [~, i1] = min(abs(fHz - 1));
    fprintf('%-8d %-10.1f %-10.1f %-8.3f %-14.2f %-14.2f %-12.1f\n', v, ...
        real(lin.eig(1)), real(lin.eig(2)), lin.zeta, lin.Gr0, v/p.L, -angle(Hr(i1))/deg);
end

subplot(2,2,1); grid on; xlabel('Frequency [Hz]'); ylabel('|r/\delta| / steady-state');
title('Yaw rate gain (normalised)'); legend('Location', 'southwest');
subplot(2,2,2); grid on; xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
title('Yaw rate phase'); legend('Location', 'southwest');
subplot(2,2,3); grid on; xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
title('Lateral acceleration phase'); legend('Location', 'southwest');

%% Check: linear vs full nonlinear model, 1 Hz sine steer at 15 m/s
v = 15; f0 = 1; w0 = 2*pi*f0; amp = 0.5*deg;     % small input keeps it linear
lin = linearise(p, v);
X = (1i*w0*eye(2) - lin.A) \ lin.B;
Hlin = X(2);

steer = @(t) amp * sin(w0*t);
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.002);
[t, Xs] = ode45(@(t,x) vehicleODE(t, x, p, steer, v), [0 6], [v; 0; 0], opts);

% Fit r = a*sin + b*cos + c over the last 2 s (after transients die out)
k = t > 4;
M = [sin(w0*t(k)), cos(w0*t(k)), ones(nnz(k),1)];
c = M \ Xs(k,3);
gainNL  = hypot(c(1), c(2)) / amp;
phaseNL = atan2(c(2), c(1)) / deg;

subplot(2,2,4);
plot(t, Xs(:,3), 'LineWidth', 1.5, 'DisplayName', 'Nonlinear sim'); hold on;
plot(t, abs(Hlin)*amp*sin(w0*t + angle(Hlin)), '--', 'LineWidth', 1.5, 'DisplayName', 'Linear prediction');
yyaxis right; plot(t, steer(t)/deg, ':', 'LineWidth', 1.2, 'DisplayName', 'Steer'); ylabel('\delta [deg]');
yyaxis left; ylabel('Yaw rate [rad/s]'); xlim([3 6]); grid on;
xlabel('Time [s]'); title('1 Hz sine steer at 15 m/s'); legend('Location', 'southwest');

drawnow;
exportgraphics(fig, 'frequency_response.png', 'Resolution', 200);

fprintf('\n1 Hz sine steer at 15 m/s:\n');
fprintf('  Linear   : gain %.3f 1/s, phase %.2f deg\n', abs(Hlin), angle(Hlin)/deg);
fprintf('  Nonlinear: gain %.3f 1/s, phase %.2f deg\n', gainNL, phaseNL);
assert(abs(gainNL - abs(Hlin))/abs(Hlin) < 0.01 && abs(phaseNL - angle(Hlin)/deg) < 0.5, ...
    'Linear model does not match nonlinear simulation');
assert(all(real(lin.eig) < 0), 'Car is unstable in a straight line');
fprintf('Checks passed: linear matches nonlinear; car is stable.\n');
