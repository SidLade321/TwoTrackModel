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

%% Constants
p.g = 9.81;         % [m/s^2]
end
