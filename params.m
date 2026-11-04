function p = params()
% PARAMS  Vehicle parameters for the two-track model.
%   All values are PLACEHOLDER estimates for a typical FS car.
%   Replace with team data as it becomes available. SI units throughout.

%% Mass and geometry
p.m      = 280;     % total mass incl. driver [kg]            PLACEHOLDER
p.L      = 1.55;    % wheelbase [m]                           PLACEHOLDER
p.wf     = 0.47;    % static front weight fraction [-]        PLACEHOLDER
p.tf     = 1.22;    % front track [m]                         PLACEHOLDER
p.tr     = 1.20;    % rear track [m]                          PLACEHOLDER
p.h      = 0.30;    % CG height [m]                           PLACEHOLDER
p.Izz    = 120;     % yaw inertia [kg m^2]                    PLACEHOLDER

% CG position (derived)
p.a = p.L * (1 - p.wf);   % CG to front axle [m]
p.b = p.L * p.wf;         % CG to rear axle [m]

%% Steering
p.ackermann = 1.0;  % 0 = parallel steer, 1 = 100% geometric Ackermann  PLACEHOLDER

%% Roll / suspension
p.hrcf   = 0.03;    % front roll centre height [m]            PLACEHOLDER
p.hrcr   = 0.05;    % rear roll centre height [m]             PLACEHOLDER
p.Kphi_f = 20000;   % front roll stiffness [Nm/rad]           PLACEHOLDER
p.Kphi_r = 18000;   % rear roll stiffness [Nm/rad]            PLACEHOLDER

%% Aero
p.rho    = 1.225;   % air density [kg/m^3]
p.ClA    = 3.0;     % downforce coefficient x area [m^2]      PLACEHOLDER
p.CdA    = 1.2;     % drag coefficient x area [m^2]           PLACEHOLDER
p.aeroBalF = 0.45;  % fraction of downforce on front axle [-] PLACEHOLDER

%% Tyre (simplified Pacejka, lateral only)                  ALL PLACEHOLDER
%  Fy = D*sin(C*atan(B*a - E*(B*a - atan(B*a))))
%  D  = mu(Fz)*Fz, with mu falling as load rises (load sensitivity)
p.tyre.B    = 17;     % stiffness factor [-]
p.tyre.C    = 1.4;    % shape factor [-]
p.tyre.E    = 0.0;    % curvature factor [-]
p.tyre.mu0  = 1.6;    % peak friction coeff at nominal load [-]
p.tyre.Fz0  = 700;    % nominal load [N]
p.tyre.kmu  = -0.15;  % load sensitivity: fractional mu change per unit (Fz-Fz0)/Fz0

%% Constants
p.g = 9.81;         % [m/s^2]
end
