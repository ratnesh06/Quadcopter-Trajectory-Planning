close all;
clear all;
addpath('utils', 'trajectories');

trajhandle = @square;
controlhandle = @adaptive_controller;

% Initial condition setup
state_0 = trajhandle(0, 1);
R_0 = eye(3); % Identity matrix as initial rotation (no rotation)
x0{1} = init_state(state_0.pos, R_0);

% Simulation parameters
real_time = true;
nquad = 1;
time_tol = 18;
params = crazyflie();

% Figures initialization
fprintf('Initializing figures...\n');
h_fig = figure;
set(h_fig,'Renderer','OpenGL');
h_3d = gca;
set(gca, 'DataAspectRatio', [1 1 1], 'GridLineStyle', '-', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
quadcolors = lines(nquad);

% Initial conditions for simulation
fprintf('Setting initial conditions...\n');
max_iter = 5000;
starttime = 0;
tstep = 0.01;
cstep = 0.05;
nstep = cstep / tstep;
time = starttime;
err = [];
for qn = 1:nquad
    des_stop = trajhandle(inf, qn);
    stop{qn} = des_stop.pos;
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

% State and tolerances
x = x0;
pos_tol = 0.01;
vel_tol = 0.01;

% Simulation loop
fprintf('Simulation Running....\n');
for iter = 1:max_iter
    timeint = time:tstep:time+cstep;
    tic;
    for qn = 1:nquad
        % Initialize quad plot on first iteration
        if iter == 1
            QP{qn} = QuadPlot(qn, x{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
        end
        
        % Simulation dynamics and visualization
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn} = xsave(end, :)';
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
        QP{qn}.UpdateQuadPlot(x{qn}, [trajhandle(time + cstep, qn).pos; trajhandle(time + cstep, qn).vel], time + cstep);
    end
    
    time = time + cstep; % update simulation time
    t = toc;
    if(t > cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    if real_time
        pause(max(0, cstep - t));
    end

    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break;
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

% Plot the saved position and velocity of each robot
for qn = 1:nquad
    % Truncate saved variables
    QP{qn}.TruncateHist();
    % Plot position for each quad
    h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
    plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
    plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'est');
    % Plot velocity for each quad
    h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
    plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
    plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'est');
end



if ~isempty(err)
    error(err);
end

fprintf('Finished.\n');
