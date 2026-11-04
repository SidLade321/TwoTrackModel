function Fz = loadTransfer(p, ax, ay, v)
% LOADTRANSFER  Vertical load on each wheel, quasi-static.
%   Fz = loadTransfer(p, ax, ay, v)
%   p  : parameter struct from params()
%   ax : longitudinal accel [m/s^2], +ve = accelerating
%   ay : lateral accel [m/s^2], +ve = turning LEFT (load moves to right wheels)
%   v  : forward speed [m/s] (for aero downforce)
%   Fz : [FL FR RL RR] vertical loads [N]
%
% Conventions: x forward, y left, z up.
% Lateral transfer per axle = geometric part (through roll centre)
%                           + elastic part (through springs/ARBs),
% with the elastic part split by roll stiffness distribution.

m = p.m; g = p.g; L = p.L;

%% Static axle loads
Ff_static = m * g * p.b / L;
Fr_static = m * g * p.a / L;

%% Aero downforce
Fdown = 0.5 * p.rho * p.ClA * v^2;
Ff_aero = Fdown * p.aeroBalF;
Fr_aero = Fdown * (1 - p.aeroBalF);

%% Longitudinal transfer (accelerating moves load rearward)
dF_long = m * ax * p.h / L;

Ff = Ff_static + Ff_aero - dF_long;   % front axle total
Fr = Fr_static + Fr_aero + dF_long;   % rear axle total

%% Lateral transfer
Fy = m * ay;                                   % total lateral inertial force
h_ra = p.hrcf + (p.hrcr - p.hrcf) * p.a / L;   % roll axis height under CG
M_elastic = Fy * (p.h - h_ra);                 % sprung roll moment
Kf_frac = p.Kphi_f / (p.Kphi_f + p.Kphi_r);

dFlat_f = (Fy * (p.b / L) * p.hrcf + M_elastic * Kf_frac)       / p.tf;
dFlat_r = (Fy * (p.a / L) * p.hrcr + M_elastic * (1 - Kf_frac)) / p.tr;

%% Corner loads [FL FR RL RR]  (+ay = left turn -> right wheels gain)
Fz = [ Ff/2 - dFlat_f, ...
       Ff/2 + dFlat_f, ...
       Fr/2 - dFlat_r, ...
       Fr/2 + dFlat_r ];

if any(Fz < 0)
    warning('loadTransfer:wheelLift', ...
        'Wheel lift predicted (Fz < 0). Model is invalid beyond this point.');
end
end
