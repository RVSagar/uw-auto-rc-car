function chassis = chassis_params()

chassis  = struct();
    %Longitudinal
    chassis.NF = 2;             % Number of wheels on front axle
    chassis.NR = 2;             % Number of wheels on rear axle

    chassis.m = 3;          % Vehicle mass, [kg]
    chassis.a = 0.1755;         % Longitudinal dist from CoM to front axle, [m]
    chassis.b = 0.1845;         % Longitudinal dist from CoM to rear axle, [m]
    chassis.h = 0.0098;         % Vertical dist from CoM to axle plane, [m]
    chassis.xdot_o = 0;         % Initial longitudinal velocity, [m/s]

    %Lateral
    chassis.d = 0;          % Lateral dist from geometric centerline to CoM, [m]
    chassis.w = 0.307.*[1 1];  % Track width, [m]
    chassis.Cy_f = 12e2;        % Front tire corner stiffness, [N/rad]
    chassis.Cy_r = 11e2;        % Rear tire axle corner stiffness, [N/rad]
    chassis.sigma_f = 0.1;      % Front tire(s) relaxation length, [m]
    chassis.sigma_r = 0.1;      % Rear tire(s) relaxation length, [m]
    chassis.ydot_o = 0;         % Initial lateral velocity, [m/s]
    
    %Yaw
    chassis.Izz = 0.00411;      % Yaw polar inertia, [kg*m^2]
    chassis.psi_o = 0;          % Initial yaw angle, [rad]
    chassis.r_o = 0;            % Initial yaw rate, [rad/s]

    %Aerodynamic
    chassis.Af = 0.046;             % Longitudinal drag area, [m^2]
    chassis.Cd = 0.3;               % Longitudinal drag coefficient
    chassis.Cl = 0.1;               % Longitudinal lift coefficient
    chassis.Cpm = 0.1;              % Longitudinal drag pitch moment
    chassis.beta_w = (0:0.1:0.3);   % Relative wind angle vector, [rad]
    chassis.Cs = (0:0.3:0.9);       % Side force coefficient vector
    chassis.Cym = (0:0.1:0.3);      % Yaw moment coefficient vector
    
    %Environment
    chassis.Pabs = 101325;          % Absolute pressure, [Pa]
    chassis.Tair = 298.15;          % Air temperature, [K]
    chassis.g = 9.81;               % Gravitational acceleration, [m/s^2]
    chassis.mu = 1;                 % Nominal friction scaling factor
    
    %Simulation
    chassis.xdot_tol = 1;           % Longitudinal velocity tolerance, [m/s]
    chassis.Fznom = 11.1834;        % Nominal normal force, [N]
    chassis.longOff = 0;            % Geometric longitudinal offset from axle plane, [m]
    chassis.latOff = 0;             % Geometric lateral offset from center plane, [m]
    chassis.vertOff = 0;            % Geometric vertical offset from axle plane, [m]
    
    %DriveTrain
    chassis.MotorPower = 0.7215;    % Motor Power, [kW]
    chassis.OA_DriveRatio = 11.82;  % Overall drive ratio
    chassis.TireRadius = 0.055;     % Tire radius, [m]
    chassis.GearRatio = 1;          % Gear ratio (of engaged gear)
    

