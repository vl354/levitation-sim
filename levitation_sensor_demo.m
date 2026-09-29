% ============================================================
% HYPERLOOP LEVITATION SIM
% ============================================================

clear;
clc;
close all;

%% ============================================================
% POD DIMENSIONS
% ============================================================

podLength = 1.8288;       % 6 ft
podWidth  = 0.40;

% Coordinate convention:
% +X = forward
% +Y = right
% +Z = upward / away from rail

%% ============================================================
% SENSOR POSITIONS
% ============================================================

% Downward sensors
S1 = [ 0.70,  0.00];
S2 = [-0.70, -0.15];
S3 = [-0.70,  0.15];

% Side sensors
S4 = [ 0.70, -podWidth/2];
S5 = [-0.70, -podWidth/2];
S6 = [ 0.00,  podWidth/2];

%% ============================================================
% ELECTROMAGNET POSITIONS
% ============================================================

EM = [
     0.65, -0.15;     % EM1
     0.22, -0.15;     % EM2
    -0.22, -0.15;     % EM3
    -0.65, -0.15;     % EM4

     0.65,  0.15;     % EM5
     0.22,  0.15;     % EM6
    -0.22,  0.15;     % EM7
    -0.65,  0.15      % EM8
];

numEM = size(EM,1);

%% ============================================================
% SENSOR RECONSTRUCTION MATRIX
% ============================================================

% d = gap + x*tan(pitch) + y*tan(roll)

A = [
    1, S1(1), S1(2);
    1, S2(1), S2(2);
    1, S3(1), S3(2)
];

%% ============================================================
% SIMULATION SETTINGS
% ============================================================

nominalGap     = 0.006;      % target = 6 mm
nominalSideGap = 0.010;

% Start too far from rail so controller has something to fix
gap = 0.008;                 % 8 mm starting gap

% For first closed-loop test:
% keep attitude fixed and control vertical motion only
roll  = 0;
pitch = 0;
yaw   = 0;

lateralOffset = 0;

% Mechanical state
verticalVelocity = 0;

% PLACEHOLDER carrier mass
podMass = 20;                % kg

gGravity = 9.81;

% Number of electromagnets
numMagnets = 8;

% Control simulation
simulationTime = 3.0;
dt = 0.01;

time = 0:dt:simulationTime;
N = length(time);

%% ============================================================
% GAP CONTROLLER
% ============================================================

% This is the OUTER levitation controller.
%
% If gap > 6 mm:
% magnet is too far from rail
% -> increase duty/current
%
% If gap < 6 mm:
% magnet is too close
% -> decrease duty/current

% Starting controller values.
% These are simulation tuning parameters.

Kp = 12000;
Kd = 300;

% Starting duty needed to provide approximately the
% equilibrium current around the nominal gap.
baseDuty = 50;

minDuty = 0;
maxDuty = 100;

previousError = gap - nominalGap;

%% ============================================================
% PLACEHOLDER ELECTROMAGNET MODEL
% ============================================================

% Force model:
%
%       F = Kmag * I^2 / gap^2
%
% We choose Kmag so that eight magnets producing about
% 2.5 A each approximately support a 20 kg pod at 6 mm.
%
% THIS MUST EVENTUALLY BE REPLACED WITH REAL MAGNET DATA.

nominalCurrent = 2.5;

requiredForcePerMagnet = ...
    podMass*gGravity/numMagnets;

Kmag = requiredForcePerMagnet ...
       * nominalGap^2 ...
       / nominalCurrent^2;

%% ============================================================
% DATA STORAGE
% ============================================================

gapHistory       = zeros(N,1);
estimatedHistory = zeros(N,1);
dutyHistory      = zeros(N,1);
currentHistory   = zeros(N,1);
forceHistory     = zeros(N,1);
velocityHistory  = zeros(N,1);

%% ============================================================
% POD SURFACE
% ============================================================

[x,y] = meshgrid( ...
    linspace(-podLength/2,podLength/2,40), ...
    linspace(-podWidth/2,podWidth/2,20));

%% ============================================================
% RAIL POSITIONS
% ============================================================

leftRailY  = -podWidth/2 - nominalSideGap;
rightRailY =  podWidth/2 + nominalSideGap;

%% ============================================================
% CREATE FIGURE
% ============================================================

figure( ...
    'Color','black', ...
    'Name','Closed-Loop Hyperloop Levitation Simulation');

hold on;
grid on;

ax = gca;

ax.Color = 'black';
ax.XColor = 'white';
ax.YColor = 'white';
ax.ZColor = 'white';
ax.GridColor = 'white';
ax.GridAlpha = 0.20;

xlabel('Forward Position X (m)','Color','white');
ylabel('Lateral Position Y (m)','Color','white');
zlabel('Gap Above Rail (mm)','Color','white');

title( ...
    'Closed-Loop Electromagnetic Levitation', ...
    'Color','white');

%% ============================================================
% TOP RAIL
% ============================================================

[railX,railY] = meshgrid( ...
    linspace(-1.00,1.00,2), ...
    linspace(-0.25,0.25,2));

railZ = zeros(size(railX));

surf( ...
    railX,railY,railZ, ...
    'FaceAlpha',0.20, ...
    'EdgeColor','none');

%% ============================================================
% SIDE RAILS
% ============================================================

sideX = [
    -1 1;
    -1 1
];

sideZ = [
     0  0;
    15 15
];

leftY = leftRailY*ones(size(sideX));

surf( ...
    sideX,leftY,sideZ, ...
    'FaceAlpha',0.15, ...
    'EdgeColor','none');

rightY = rightRailY*ones(size(sideX));

surf( ...
    sideX,rightY,sideZ, ...
    'FaceAlpha',0.15, ...
    'EdgeColor','none');

%% ============================================================
% POD
% ============================================================

podPlot = surf( ...
    x,y,gap*1000*ones(size(x)), ...
    'FaceAlpha',0.75, ...
    'EdgeColor','none');

%% ============================================================
% SENSOR MARKERS
% ============================================================

verticalSensorPlot = scatter3( ...
    [S1(1) S2(1) S3(1)], ...
    [S1(2) S2(2) S3(2)], ...
    gap*1000*[1 1 1], ...
    120,'filled');

sideSensorPlot = scatter3( ...
    [S4(1) S5(1) S6(1)], ...
    [S4(2) S5(2) S6(2)], ...
    gap*1000*[1 1 1], ...
    120,'diamond','filled');

%% ============================================================
% ELECTROMAGNET MARKERS
% ============================================================

magnetPlot = scatter3( ...
    EM(:,1), ...
    EM(:,2), ...
    gap*1000*ones(numEM,1), ...
    100, ...
    'square', ...
    'filled');

%% ============================================================
% LASER BEAMS
% ============================================================

laser1 = plot3([0 0],[0 0],[0 0],'LineWidth',3);
laser2 = plot3([0 0],[0 0],[0 0],'LineWidth',3);
laser3 = plot3([0 0],[0 0],[0 0],'LineWidth',3);

%% ============================================================
% LIVE DATA PANEL
% ============================================================

measurementBox = annotation( ...
    'textbox', ...
    [0.70 0.30 0.27 0.55], ...
    'String','Starting...', ...
    'FitBoxToText','off', ...
    'BackgroundColor',[0.08 0.08 0.08], ...
    'EdgeColor',[0.4 0.4 0.4], ...
    'Color','white', ...
    'FontName','Consolas', ...
    'FontSize',10);

%% ============================================================
% VIEW SETTINGS
% ============================================================

xlim([-1 1]);
ylim([-0.27 0.27]);
zlim([0 12]);

view(40,25);

set(gca,'Position',[0.07 0.12 0.58 0.80]);

%% ============================================================
% CLOSED-LOOP SIMULATION
% ============================================================

for k = 1:N

    t = time(k);

    %% --------------------------------------------------------
    % 1. SIMULATED LASER READINGS
    % ---------------------------------------------------------

    d1 = gap ...
        + S1(1)*tan(pitch) ...
        + S1(2)*tan(roll);

    d2 = gap ...
        + S2(1)*tan(pitch) ...
        + S2(2)*tan(roll);

    d3 = gap ...
        + S3(1)*tan(pitch) ...
        + S3(2)*tan(roll);

    %% --------------------------------------------------------
    % 2. RECONSTRUCT POD POSE
    % ---------------------------------------------------------

    D = [d1; d2; d3];

    solution = A \ D;

    estimated_gap   = solution(1);
    estimated_pitch = atan(solution(2));
    estimated_roll  = atan(solution(3));

    %% --------------------------------------------------------
    % 3. CALCULATE GAP AT ALL 8 ELECTROMAGNETS
    % ---------------------------------------------------------

    EM_gap = estimated_gap ...
        + EM(:,1)*tan(estimated_pitch) ...
        + EM(:,2)*tan(estimated_roll);

    %% --------------------------------------------------------
    % 4. GAP CONTROLLER
    % ---------------------------------------------------------

    % Average EM gap for this first vertical-only controller

    measuredGap = mean(EM_gap);

    % Positive:
    % pod is too far from rail

    error = measuredGap - nominalGap;

    errorDerivative = ...
        (error - previousError)/dt;

    previousError = error;

    % PID/PD correction

    correction = ...
        Kp*error ...
        + Kd*errorDerivative;

    duty = baseDuty + correction;

    % Saturate duty cycle

    duty = max(minDuty, ...
           min(maxDuty,duty));

    %% --------------------------------------------------------
    % 5. SEND DUTY TO SIMULINK ELECTRICAL CIRCUIT
    % ---------------------------------------------------------

    % Your Pulse Generator uses the MATLAB workspace
    % variable "duty".

    % Run one short electrical simulation.

    simIn = Simulink.SimulationInput('levitation_model');

    simIn = simIn.setVariable('duty',duty);

    simIn = simIn.setModelParameter( ...
        'StopTime','0.02');

    simOut = sim(simIn);

    %% --------------------------------------------------------
    % 6. GET ACTUAL CURRENT FROM SIMULINK
    % ---------------------------------------------------------

    currentTS = simOut.coil_current;

    % Use the final current from the electrical simulation

    coilCurrent = currentTS.Data(end);

    % Current magnitude only

    coilCurrent = abs(coilCurrent);

    %% --------------------------------------------------------
    % 7. ELECTROMAGNET FORCE
    % ---------------------------------------------------------

    % For the first model, all eight magnets are assumed
    % to receive the same current.

    forcePerMagnet = ...
        Kmag * coilCurrent^2 ...
        / max(measuredGap,0.001)^2;

    totalMagneticForce = ...
        numMagnets * forcePerMagnet;

    %% --------------------------------------------------------
    % 8. POD VERTICAL DYNAMICS
    % ---------------------------------------------------------

    % IMPORTANT SIGN CONVENTION:
    %
    % gap increases DOWNWARD / away from upper rail.
    %
    % Gravity therefore increases gap.
    % Magnetic attraction decreases gap.

    gravityForce = podMass*gGravity;

    netForce = ...
        gravityForce - totalMagneticForce;

    acceleration = netForce/podMass;

    verticalVelocity = ...
        verticalVelocity ...
        + acceleration*dt;

    gap = ...
        gap ...
        + verticalVelocity*dt;

    %% --------------------------------------------------------
    % 9. SAFETY LIMITS FOR SIMULATION
    % ---------------------------------------------------------

    % Prevent the numerical model from going through the rail

    minimumGap = 0.002;
    maximumGap = 0.015;

    if gap < minimumGap

        gap = minimumGap;
        verticalVelocity = 0;

    elseif gap > maximumGap

        gap = maximumGap;
        verticalVelocity = 0;

    end

    %% --------------------------------------------------------
    % 10. STORE DATA
    % ---------------------------------------------------------

    gapHistory(k) = gap;

    estimatedHistory(k) = ...
        estimated_gap;

    dutyHistory(k) = duty;

    currentHistory(k) = ...
        coilCurrent;

    forceHistory(k) = ...
        totalMagneticForce;

    velocityHistory(k) = ...
        verticalVelocity;

    %% --------------------------------------------------------
    % 11. UPDATE POD VISUALIZATION
    % ---------------------------------------------------------

    z = estimated_gap ...
        + x*tan(estimated_pitch) ...
        + y*tan(estimated_roll);

    podZ = z*1000;

    set(podPlot, ...
        'ZData',podZ);

    %% --------------------------------------------------------
    % 12. UPDATE SENSOR POSITIONS
    % ---------------------------------------------------------

    verticalZ = ...
        [d1 d2 d3]*1000;

    set(verticalSensorPlot, ...
        'ZData',verticalZ);

    %% --------------------------------------------------------
    % 13. UPDATE ELECTROMAGNET POSITIONS
    % ---------------------------------------------------------

    EMz = EM_gap*1000;

    set(magnetPlot, ...
        'ZData',EMz);

    %% --------------------------------------------------------
    % 14. UPDATE LASERS
    % ---------------------------------------------------------

    set(laser1, ...
        'XData',[S1(1) S1(1)], ...
        'YData',[S1(2) S1(2)], ...
        'ZData',[d1*1000 0]);

    set(laser2, ...
        'XData',[S2(1) S2(1)], ...
        'YData',[S2(2) S2(2)], ...
        'ZData',[d2*1000 0]);

    set(laser3, ...
        'XData',[S3(1) S3(1)], ...
        'YData',[S3(2) S3(2)], ...
        'ZData',[d3*1000 0]);

    %% --------------------------------------------------------
    % 15. UPDATE LIVE DATA PANEL
    % ---------------------------------------------------------

    measurementText = sprintf([ ...
        'TIME: %5.2f s\n\n' ...
        'LEVITATION CONTROL\n' ...
        '-----------------------\n' ...
        'Target Gap:   %6.2f mm\n' ...
        'Actual Gap:   %6.2f mm\n' ...
        'Measured Gap: %6.2f mm\n' ...
        'Gap Error:    %6.2f mm\n\n' ...
        'ELECTRICAL SYSTEM\n' ...
        '-----------------------\n' ...
        'PWM Duty:     %6.1f %%\n' ...
        'Coil Current: %6.2f A\n\n' ...
        'FORCE MODEL\n' ...
        '-----------------------\n' ...
        'Mag Force:    %6.1f N\n' ...
        'Gravity:      %6.1f N\n' ...
        'Velocity:     %6.3f m/s'], ...
        t, ...
        nominalGap*1000, ...
        gap*1000, ...
        measuredGap*1000, ...
        error*1000, ...
        duty, ...
        coilCurrent, ...
        totalMagneticForce, ...
        gravityForce, ...
        verticalVelocity);

    set(measurementBox, ...
        'String',measurementText);

    drawnow;

end

hold off;

%% ============================================================
% RESULTS
% ============================================================

figure('Name','Levitation Controller Results');

plot( ...
    time, ...
    gapHistory*1000, ...
    'LineWidth',1.5);

hold on;

yline( ...
    nominalGap*1000, ...
    '--', ...
    'Target = 6 mm');

xlabel('Time (s)');
ylabel('Gap (mm)');
title('Levitation Gap');
grid on;

figure('Name','Electrical Response');

plot( ...
    time, ...
    currentHistory, ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Coil Current (A)');
title('Simulated Electromagnet Current');
grid on;

figure('Name','PWM Controller Output');

plot( ...
    time, ...
    dutyHistory, ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('PWM Duty (%)');
title('PWM Duty Command');
ylim([0 100]);
grid on;