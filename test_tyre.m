% TEST_TYRE  Tyre model checks + the key result: load transfer costs grip.
clear; clc; close all;
p = params();
deg = pi/180;

%% Plot 1: Fy vs slip angle at several loads
alpha = linspace(0, 15, 300) * deg;
loads = [300 700 1100 1500];

fig = figure('Name','Tyre curves');
subplot(1,2,1); hold on; grid on;
for Fz = loads
    plot(alpha/deg, tyreLateral(p, alpha, Fz), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('F_z = %d N', Fz));
end
xlabel('Slip angle [deg]'); ylabel('F_y [N]');
title('Lateral force vs slip angle'); legend('Location','southeast');

%% Plot 2: peak mu vs load (load sensitivity)
Fz_range = linspace(100, 1600, 100);
peakFy = arrayfun(@(F) max(tyreLateral(p, alpha, F)), Fz_range);
subplot(1,2,2); grid on;
plot(Fz_range, peakFy ./ Fz_range, 'LineWidth', 1.5);
xlabel('F_z [N]'); ylabel('Peak \mu = F_{y,max}/F_z');
title('Load sensitivity');
drawnow;
exportgraphics(fig, 'tyre_curves.png', 'Resolution', 200);  % saved next to the scripts

%% Check: peak grip grows LESS than proportionally with load
assert(max(tyreLateral(p, alpha, 1400)) < 2*max(tyreLateral(p, alpha, 700)), ...
    'Load sensitivity not working');

%% Key result: axle grip lost to lateral load transfer
% Skidpad case: 1.5 g left, 15 m/s
fprintf('Axle peak lateral grip at 1.5 g skidpad\n');
fprintf('%-12s %10s %10s %10s %10s\n', 'Kphi_f', 'Front [N]', 'loss [%]', 'Rear [N]', 'loss [%]');

for Kf = [10000 20000 40000]
    q = p; q.Kphi_f = Kf;
    Fz  = loadTransfer(q, 0, 1.5*q.g, 15);
    Fz0 = loadTransfer(q, 0, 0, 15);          % same speed, no lateral transfer

    peak = @(F) max(tyreLateral(q, alpha, F));
    front   = peak(Fz(1))  + peak(Fz(2));
    rear    = peak(Fz(3))  + peak(Fz(4));
    front0  = peak(Fz0(1)) + peak(Fz0(2));
    rear0   = peak(Fz0(3)) + peak(Fz0(4));

    fprintf('%-12d %10.0f %10.1f %10.0f %10.1f\n', Kf, ...
        front, 100*(1-front/front0), rear, 100*(1-rear/rear0));
end
fprintf('\nStiffer front -> front loses more grip -> more understeer.\n');
fprintf('All checks passed.\n');
