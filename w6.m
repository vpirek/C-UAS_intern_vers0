clc;
clear;
close all;

rng('shuffle');

%% ==========================================================
% CONFIGURATION
%% ==========================================================

numTracks = 5;

objPos = [0 0];
protectionRadius = 500;

radar.maxRange = 20000;       % 20 km
radar.coverageAz = 360;       % idealized radar
radar.coverageEl = 20;

uavSpeed = 15;                % m/s
meanRCS = 0.01;               % -20 dBsm

MAX_MISSES = 5;               % hysteresis

%% ==========================================================
% RANDOM UAV TRAJECTORIES
%% ==========================================================

uav = struct();

for k = 1:numTracks

    startRange = 1500 + rand*2500;
    startBearing = rand*2*pi;

    startX = startRange*cos(startBearing);
    startY = startRange*sin(startBearing);

    startZ = 60 + rand*60;

    numWP = randi([3 6]);

    wp = zeros(numWP+2,3);

    wp(1,:) = [startX startY startZ];

    for n = 2:numWP+1

        r = 300 + rand*2500;
        phi = rand*2*pi;

        wp(n,1) = r*cos(phi);
        wp(n,2) = r*sin(phi);
        wp(n,3) = 60 + rand*60;

    end

    finalR = rand*100;
    finalPhi = rand*2*pi;

    wp(end,:) = [ ...
        finalR*cos(finalPhi), ...
        finalR*sin(finalPhi), ...
        80];

    uav(k).waypoints = wp;

    segLen = vecnorm(diff(wp),2,2);

    toa = [0; cumsum(segLen/uavSpeed)];

    uav(k).toa = toa;
    uav(k).totalTime = toa(end);

end

stopTime = ceil(max([uav.totalTime]));

%% ==========================================================
% FIGURE
%% ==========================================================

figure( ...
    'Name','Multi UAV Detection',...
    'Color','w');

hold on
grid on
axis equal

xlim([-4500 4500]);
ylim([-4500 4500]);

xlabel('X [m]');
ylabel('Y [m]');

title('360° Radar - Random UAV Scenario');

theta = linspace(0,2*pi,400);

protectionHandle = plot( ...
    protectionRadius*cos(theta),...
    protectionRadius*sin(theta),...
    'k','LineWidth',2);

radarHandle = plot( ...
    0,0,...
    'kp',...
    'MarkerSize',16,...
    'MarkerFaceColor','y');

colors = lines(numTracks);

uavHandle = gobjects(numTracks,1);
detHandle = gobjects(numTracks,1);
histHandle = gobjects(numTracks,1);

trajLegendHandles = gobjects(numTracks,1);

for k=1:numTracks

    trajLegendHandles(k) = plot( ...
        uav(k).waypoints(:,1), ...
        uav(k).waypoints(:,2), ...
        '--', ...
        'LineWidth',1.5,...
        'Color',colors(k,:));

    uavHandle(k) = plot( ...
        uav(k).waypoints(1,1), ...
        uav(k).waypoints(1,2), ...
        'o',...
        'MarkerSize',10,...
        'MarkerFaceColor',colors(k,:),...
        'MarkerEdgeColor','k');

    detHandle(k) = plot( ...
        nan,nan,...
        'rs',...
        'MarkerSize',10,...
        'LineWidth',2);

    histHandle(k) = plot( ...
        nan,nan,...
        '.',...
        'Color',[1 0 0],...
        'MarkerSize',8);

end

legendEntries = [{'Protection Zone';'Radar'}];

for k = 1:numTracks
    legendEntries{end+1} = sprintf('Track %d',k);
end

legend([protectionHandle radarHandle trajLegendHandles'],...
       legendEntries,...
       'Location','eastoutside');

%% ==========================================================
% STATISTICS
%% ==========================================================

for k=1:numTracks

    stats(k).firstDetectionTime = NaN;
    stats(k).firstDetectionRange = NaN;

    stats(k).detections = 0;
    stats(k).scans = 0;

    stats(k).trackDetected = false;

    stats(k).missCounter = 0;

    stats(k).trackLosses = 0;

    stats(k).acquiredLogged = false;
    stats(k).lostLogged = false;

    stats(k).breachTime = NaN;

    stats(k).warningTime = NaN;

    stats(k).rcsHistory = [];

    stats(k).detX = [];
    stats(k).detY = [];

    % Protection Zone

    stats(k).insideZone = false;

    stats(k).entryTime = NaN;
    stats(k).exitTime = NaN;

    stats(k).entryPosition = [NaN NaN NaN];
    stats(k).exitPosition = [NaN NaN NaN];

    stats(k).timeInsideZone = 0;

end

%% ==========================================================
% SIMULATION
%% ==========================================================

disp('========================================');
disp('SIMULATION STARTED');
disp('========================================');

dt = 0.5;

for t = 0:dt:stopTime

    for k = 1:numTracks

        stats(k).scans = stats(k).scans + 1;

        %% POSITION

        if t >= uav(k).totalTime

            currentPos = uav(k).waypoints(end,:);

        else

            currentPos = interp1( ...
                uav(k).toa,...
                uav(k).waypoints,...
                t);

        end

        %% UAV VISUALIZATION

        set(uavHandle(k),...
            'XData',currentPos(1),...
            'YData',currentPos(2));

        %% RANGE

        range_m = norm(currentPos);

        %% SWERLING I

        currentRCS = meanRCS*(-log(rand));

        stats(k).rcsHistory(end+1) = currentRCS;

        %% DETECTION MODEL

        rangeFactor = exp(-(range_m/3000)^2);

        rcsFactor = min(currentRCS/0.01,2);

        Pd = min(1,rangeFactor*rcsFactor);

        detected = rand < Pd;

        %% DETECTION PROCESS

        if detected

            stats(k).detections = ...
                stats(k).detections + 1;

            stats(k).missCounter = 0;

            stats(k).lostLogged = false;

            stats(k).detX(end+1) = currentPos(1);
            stats(k).detY(end+1) = currentPos(2);

            set(detHandle(k),...
                'XData',currentPos(1),...
                'YData',currentPos(2));

            set(histHandle(k),...
                'XData',stats(k).detX,...
                'YData',stats(k).detY);

        else

            stats(k).missCounter = ...
                stats(k).missCounter + 1;

            set(detHandle(k),...
                'XData',nan,...
                'YData',nan);

        end

        %% ACQUIRED

        if detected && ...
           ~stats(k).trackDetected

            stats(k).trackDetected = true;

            if isnan(stats(k).firstDetectionTime)

                stats(k).firstDetectionTime = t;
                stats(k).firstDetectionRange = range_m;

            end

            if ~stats(k).acquiredLogged

                fprintf('\n========================================\n');
                fprintf('TRACK %d ACQUIRED\n',k);
                fprintf('========================================\n');

                fprintf('Time      : %.1f s\n',t);

                fprintf('Position  : [%.0f %.0f]\n',...
                    currentPos(1),...
                    currentPos(2));

                fprintf('Range     : %.0f m\n',range_m);

                fprintf('Altitude  : %.0f m\n',currentPos(3));

                fprintf('RCS       : %.5f m2\n',currentRCS);

                fprintf('Pd        : %.3f\n',Pd);

                stats(k).acquiredLogged = true;

            end

        end

        %% LOST WITH HYSTERESIS

        if stats(k).trackDetected && ...
           stats(k).missCounter >= MAX_MISSES

            stats(k).trackDetected = false;

            stats(k).trackLosses = ...
                stats(k).trackLosses + 1;

            if ~stats(k).lostLogged

                fprintf('\n----------------------------------------\n');
                fprintf('TRACK %d LOST\n',k);
                fprintf('----------------------------------------\n');

                fprintf('Time      : %.1f s\n',t);

                fprintf('Position  : [%.0f %.0f]\n',...
                    currentPos(1),...
                    currentPos(2));

                fprintf('Range     : %.0f m\n',range_m);

                fprintf('Altitude  : %.0f m\n',currentPos(3));

                fprintf('RCS       : %.5f m2\n',currentRCS);

                fprintf('MissCount : %d\n',...
                    stats(k).missCounter);

                stats(k).lostLogged = true;

            end

        end

        %% BREACH

     %% PROTECTION ZONE ENTRY / EXIT

distanceToCenter = norm(currentPos(1:2));

% --- ENTRY ---

if distanceToCenter <= protectionRadius && ...
        ~stats(k).insideZone

    stats(k).insideZone = true;

    stats(k).entryTime = t;

    stats(k).entryPosition = currentPos;

    fprintf('\n*** TRACK %d ENTERED PROTECTION ZONE ***\n',k);

    fprintf('Time      : %.1f s\n',t);

    fprintf('Position  : [%.0f %.0f %.0f]\n',...
        currentPos(1),...
        currentPos(2),...
        currentPos(3));

    if ~isnan(stats(k).firstDetectionTime)

        stats(k).warningTime = ...
            t - stats(k).firstDetectionTime;

    end

end

% --- EXIT ---

if distanceToCenter > protectionRadius && ...
        stats(k).insideZone

    stats(k).insideZone = false;

    stats(k).exitTime = t;

    stats(k).exitPosition = currentPos;

    stats(k).timeInsideZone = ...
        stats(k).exitTime - ...
        stats(k).entryTime;

    fprintf('\n*** TRACK %d LEFT PROTECTION ZONE ***\n',k);

    fprintf('Time      : %.1f s\n',t);

    fprintf('Position  : [%.0f %.0f %.0f]\n',...
        currentPos(1),...
        currentPos(2),...
        currentPos(3));

end

    end

    drawnow;
    pause(0.02);

end
for k = 1:numTracks

    if ~isnan(stats(k).entryTime)

        if isnan(stats(k).exitTime)

            stats(k).timeInsideZone = ...
                stopTime - stats(k).entryTime;

        end

    end

end
%% ==========================================================
% REPORT
%% ==========================================================

disp(' ');
disp('========================================');
disp('SIMULATION REPORT');
disp('========================================');

totalDetections = 0;
totalScans = 0;

for k=1:numTracks

    availability = ...
        stats(k).detections / ...
        stats(k).scans;

    totalDetections = ...
        totalDetections + ...
        stats(k).detections;

    totalScans = ...
        totalScans + ...
        stats(k).scans;

    fprintf('\nTRACK %d\n',k);
    fprintf('----------------------------------------\n');

    fprintf('First Detection Time : %.2f s\n',...
        stats(k).firstDetectionTime);

    fprintf('First Detection Range: %.0f m\n',...
        stats(k).firstDetectionRange);

    fprintf('Detection Availability: %.1f %%\n',...
        availability*100);

    fprintf('Track Losses         : %d\n',...
        stats(k).trackLosses);

    fprintf('Mean RCS             : %.5f m2\n',...
        mean(stats(k).rcsHistory));
    fprintf('Entry Time          : %.2f s\n',...
        stats(k).entryTime);

    fprintf('Entry Position      : [%.0f %.0f %.0f]\n',...
        stats(k).entryPosition);

    fprintf('Exit Time           : %.2f s\n',...
        stats(k).exitTime);

    fprintf('Exit Position       : [%.0f %.0f %.0f]\n',...
        stats(k).exitPosition);

    fprintf('Time Inside Zone    : %.2f s\n',...
        stats(k).timeInsideZone);

    fprintf('Protection Breach    : %s\n',...
        string(~isnan(stats(k).breachTime)));

    fprintf('Warning Time         : %.2f s\n',...
        stats(k).warningTime);

end

disp(' ');
disp(' ');
disp('==============================================================');
disp('DETECTION SUMMARY TABLE');
disp('==============================================================');

fprintf('%4s %8s %8s %8s %8s %8s\n',...
    'ID',...
    'Det[m]',...
    'Entry',...
    'Exit',...
    'Inside',...
    'Pd');

for k = 1:numTracks

    PdTrack = ...
        stats(k).detections / ...
        stats(k).scans;

    fprintf('%4d %8.0f %8.1f %8.1f %8.1f %8.3f\n',...
        k,...
        stats(k).firstDetectionRange,...
        stats(k).entryTime,...
        stats(k).exitTime,...
        stats(k).timeInsideZone,...
        PdTrack);

end

disp('==============================================================');
disp('========================================');
disp('GLOBAL PERFORMANCE');
disp('========================================');

fprintf('Number of Tracks       : %d\n',numTracks);

fprintf('Overall Detection Pd  : %.3f\n',...
    totalDetections/totalScans);

fprintf('Total Detections      : %d\n',...
    totalDetections);

fprintf('Total Scans           : %d\n',...
    totalScans);

disp('========================================');