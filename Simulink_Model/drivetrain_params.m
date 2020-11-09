function drivetrain = drivetrain_params()
    
drivetrain = struct();

    %Gearbox
    drivetrain.N_Ratio = 2.4;               % Motor to Driveshaft Gear Ratio
    drivetrain.J1 = 0.01;                   % Input shaft inertia, [kg*m^2]
    drivetrain.J2 = 0.01;                   % Output shaft inertia, [kg*m^2]
    drivetrain.G_b1 = 0.001;                % Input shaft damping, [N*m*s/rad]
    drivetrain.G_b2 = 0.001;                % Output shaft damping, [N*m*s/rad]
    drivetrain.w1_o = 0;                    % Input shaft initial velocity, [rad/s]
    drivetrain.eta = 1;                     % Constant efficiency factor
    
    %Rotational Inertia
    drivetrain.Jmotor = 0.1;                % Rotational Inertia, [kg*m^2]
    drivetrain.omega_o = 0;                 % Initial Velocity, [rad/s]
    
    %Driveshaft Compliance
    drivetrain.k = 1e4;                     % Torsional Stiffness, [Nm]
    drivetrain.b = 400;                     % Torsional Dampening, [Nms]
    drivetrain.theta_o = 0;                 % Initial Deflection, [rad]
    drivetrain.domega_o = 0;                % Initial velocity difference, [rad/s]
    drivetrain.omega_c = 100;               % Damping cutoff frequency, [rad/s]
    
    %All Wheel Drive
    drivetrain.J = 0.01;                    % Rotational Inertia, [kg*m^2]
    drivetrain.b_J = 0.001;                 % Torsional damping, [Nms/rad]  
    drivetrain.omega_o_J = 0;               % Initial velocity, [rad/s]
    drivetrain.k1 = 5e4;                    % Torsional stiffness, [Nm/rad]
    drivetrain.b1 = 100;                    % Torsional Damping, [Nms/rad]
    drivetrain.omega1_c = 3000;             % Damping cutoff frequency, [rad/s]
    
    % -Front Axle
    drivetrain.Ndiff = 7.94;                % Carrier to driveshaft ratio
    drivetrain.Jd = 0.025;                  % Carrier inertia, [kg*m^2]
    drivetrain.bd = 0.001;                  % Carrier  damping, [Nms/rad]
    drivetrain.Jw1 = 0.01;                  % Axle 1 inertia, [kg*m^2]
    drivetrain.bw1 = 0.001;                 % Axle 1 damping, [Nms/rad]
    drivetrain.Jw2 = 0.01;                  % Axle 2 inertia, [kg*m^2]
    drivetrain.bw2 = 0.001;                 % Axle 2 damping, [Nms/rad]
    drivetrain.omegaw1o = 0;                % Axle 1 velocity, [rad/s]
    drivetrain.omegaw2o = 0;                % Axle 2 velocity, [rad/s]
    
    % -Rear Axle
    drivetrain.Ndisks = 4;                                          % Number of disks
    drivetrain.Reff_LSD = 0.2;                                      % Effective radius, [m]
    drivetrain.Fc = 500;                                            % Nominal preload force, [N]
    drivetrain.muc = [0.16 0.13 0.115 0.11 0.105 0.1025 0.1013];    % Friction coefficient vector
    drivetrain.dw = [0 10 20 40 60 80 100];                         % Slip speed vector, [rad/s]
    drivetrain.tauC_LSD = 0.01;                                     % Coupling time constant, [s]
    
    %Wheels and Brakes
    drivetrain.lam_x = 1;                   % Longitudinal scaling factor
    
    % -Wheel Dynamics
    drivetrain.br = 0.001;                  % Axle viscous damping coefficient, [Nms/rad]
    drivetrain.Iyy_Whl = 0.8;               % Wheel inertia, [kg*m^2]
    drivetrain.omegao = 0;                  % Wheel initial angular velocity, [rad/s]
    drivetrain.Lrel = 0.15;                 % Relaxation length, [m]
    drivetrain.Re = 0.327;                  % Loaded radius, [m]
    drivetrain.UNLOADED_RADIUS = 0.336;     % Unloaded radius, [m]
    drivetrain.press = 2.344e5;             % Pressure, [Pa]
    
    % -Longitudinal
    drivetrain.Dx = 1;                      % Pure longitudinal peak factor
    drivetrain.Cx = 1.65;                   % Pure longitudinal shape factor
    drivetrain.Bx = 10;                     % Pure longitudinal stiffness factor
    drivetrain.Ex = 0.01;                   % Pure longitudinal curvature factor
    
    % -Rolling Resistance
    drivetrain.aMy = 0.01;                  % Velocity independent force coefficient
    drivetrain.bMy = 0;                     % Linear velocity force component, [s/m]
    drivetrain.cMy = 0;                     % Quadratic  velocity force component, [s^2/m^2]
    drivetrain.alphaMy = 0;                 % Tire pressure exponent
    drivetrain.betaMy = 1;                  % Normal force exponent
    
    % -Brake
    drivetrain.mu_static = 0.45;            % Static friction coefficient
    drivetrain.mu_kinetic = 0.35;           % Kinetic friction coefficient
    drivetrain.disk_abore = 0.05;           % Disc brake actuator bore, [m]
    drivetrain.Rm = 0.15;                   % Brake pad mean radius, [m]
    drivetrain.num_pads = 2;                % Number of brake pads
    
    % -Simulation Setup
    drivetrain.FZMIN = 0;                   % Minimum normal force, [N]
    drivetrain.FZMAX = 6570;                % Maximum normal force, [N]
    drivetrain.kappamax = 1.5;              % Max allowable slip ratio (absolute)
    drivetrain.VXLOW = 2;                   % Velocity tolerance used to handle low velocity situations, [m/s]
end
