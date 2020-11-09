%% plant model parameters
function data = loadData_V2()

% powertrain
data.powertrain = powertrain_params();

% chassis
data.chassis = chassis_params();

% drivetrain
data.drivetrain = drivetrain_params();

if nargout == 0
    s = fieldnames(data);
    for i = 1 : length(s)
        assignin('base', s{i}, data.(s{i}));
    end
end
end
