% TEST_LOADTRANSFER  Sanity checks for loadTransfer.m
clear; clc;
p = params();
names = {'FL','FR','RL','RR'};

cases = {
%   label                 ax          ay          v
    'Static',             0,          0,          0;
    'Skidpad 1.5g left',  0,          1.5*p.g,    15;
    'Braking 1.8g',      -1.8*p.g,    0,          20;
    'Accel 1.0g',         1.0*p.g,    0,          10;
};

for i = 1:size(cases,1)
    [lbl, ax, ay, v] = cases{i,:};
    Fz = loadTransfer(p, ax, ay, v);

    % Check 1: total load = weight + downforce
    expected = p.m*p.g + 0.5*p.rho*p.ClA*v^2;
    assert(abs(sum(Fz) - expected) < 1e-6, 'Total load mismatch');

    fprintf('\n%s (ax=%.1f, ay=%.1f m/s^2, v=%.0f m/s)\n', lbl, ax, ay, v);
    for k = 1:4
        fprintf('  %s: %7.1f N\n', names{k}, Fz(k));
    end
end

% Check 2: left turn should load the right wheels
Fz = loadTransfer(p, 0, 1.5*p.g, 15);
assert(Fz(2) > Fz(1) && Fz(4) > Fz(3), 'Lateral transfer sign wrong');

% Check 3: stiffer front roll -> more front lateral transfer
p2 = p; p2.Kphi_f = 2*p.Kphi_f;
Fz2 = loadTransfer(p2, 0, 1.5*p.g, 15);
assert((Fz2(2)-Fz2(1)) > (Fz(2)-Fz(1)), 'Roll stiffness split wrong');

fprintf('\nAll checks passed.\n');
