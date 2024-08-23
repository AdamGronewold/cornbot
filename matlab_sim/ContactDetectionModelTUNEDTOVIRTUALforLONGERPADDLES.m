clear all

% Initial sensor parameters
sensor.paddle_length = 0.3556;  % 6 inches in meters
sensor.spring_constant = 0.67;    % Nm/rad, adjust for stiffness
sensor.damping = 0.0003;          % Nm*s/rad, damping factor
sensor.sticky_region = [-deg2rad(25), deg2rad(10)];  % Sticky region in radians
sensor.sticky_friction_probability = 0.07;  % Probability of settling on a given angle
sensor.is_sticky = false;  % Flag to indicate if the sensor is in a sticky state

% Initial sensor position and orientation
sensor.pin_location = [0, 0];       % Place sensor at origin
sensor.home_angle = deg2rad(0);     % Home position angle (0 degrees)
sensor.span = [-90, 15];            % Active angle range in degrees, always define counterclockwise
sensor.in_contact(1) = false;       % Flag to indicate if sensor is in contact

% "Real"-Simulated State Histories
sensor.x_del_t = 0.008;             % Update rate of our paddle model
sensor.x_time_h(1) = 0;
sensor.x_angle_h(1) = 0;            % Actual angle history, initial angular value              
sensor.xdot_angle_h(1) = 0;         % Actual angular velocities histories
sensor.contact_location = [];
sensor.real_contact_count(1) = 0;      % Counter for real contacts

% Measurement Variables
sensor.y_del_t = sensor.x_del_t;    % Sample period for measurement
sensor.y_time_h = sensor.x_time_h;
sensor.y_angle_h = sensor.x_angle_h;
sensor.ydot_angle_h = sensor.xdot_angle_h;
sensor.yddot_angle_h = zeros(1, 1);  % Initial acceleration history
sensor.classifier_window = 5; % Each transition can only happen after this number times the sensor data rate
sensor.current_state = "Unknown State";
sensor.f_y_angle_classifier_h(1) = sensor.current_state;  % Initial state classifier history
sensor.classified_contact_count(1) = 0;  % Counter for classified contacts

% Robot stuff just to get by for now
robot.x = 0;
robot.y = 0;
robot.heading = 0;

% Plant construction
num_plants = 6;
sensor.intrarow_width = 0.2540; % 10 inches
for i = 1:num_plants
    plants(i).x = (sensor.paddle_length + 0.05) * rand(); % x values between 0 and one paddle length
    plants(i).y = 0.6 + (i-1) * sensor.intrarow_width + rand() * 0.1; % Spacing using intrarow width
    plants(i).speed = 0.2;  % Speed of the point moving downwards
end
%-------------------------------------

[local_plot, angle_plot] = init_figure(plants, sensor); % Initialize the plot
frames = struct('cdata', [], 'colormap', []);  

sensor.x_tk = 1; % This is saying time0=0 with matlab 1 indexing

while sensor.x_time_h(end) < 1800
    
    % Increase time
    sensor.x_tk = sensor.x_tk + 1;
    sensor.x_time_h(sensor.x_tk) = sensor.x_time_h(sensor.x_tk - 1) + sensor.x_del_t; % This is the simulation time

    % Move the plants downwards
    plants = move_plants(plants, sensor);

    % The "REAL" part of the sensor simulation
    % Check if any plant is within the active range and update the sensor behavior
    sensor = check_for_contact(sensor, plants);


    % Update the sensor's pin location and paddle end position
    sensor = update_sensor_dynamics(sensor, robot);  

    % Measurement part of the simulation
    sensor.y_tk = sensor.x_tk;
    sensor.y_time_h(sensor.y_tk) = sensor.x_time_h(sensor.y_tk);
    sensor.y_angle_h(sensor.y_tk) = sensor.x_angle_h(sensor.y_tk)+(2*randn()-1)/20;  % Measurement angle with a bit of noise

    % Estimate first and second derivatives
    if sensor.y_tk > 1
        sensor.ydot_angle_h(sensor.y_tk) = (sensor.y_angle_h(sensor.y_tk) - sensor.y_angle_h(sensor.y_tk - 1)) / sensor.y_del_t;
        if sensor.y_tk > 2
            sensor.yddot_angle_h(sensor.y_tk) = (sensor.ydot_angle_h(sensor.y_tk) - sensor.ydot_angle_h(sensor.y_tk - 1)) / sensor.y_del_t;
        else
            sensor.yddot_angle_h(sensor.y_tk) = 0;
        end
    else
        sensor.ydot_angle_h(sensor.y_tk) = 0;
        sensor.yddot_angle_h(sensor.y_tk) = 0;
    end

    %Classify the contact state from measurement
    sensor = classify_state(sensor);

    %Count the number of estimated contacts
    if sensor.f_y_angle_classifier_h(sensor.y_tk-1) ~= "In Contact" && sensor.f_y_angle_classifier_h(sensor.y_tk) == "In Contact"
        sensor.classified_contact_count(sensor.y_tk) = sensor.classified_contact_count(sensor.y_tk-1) + 1;
    else
        sensor.classified_contact_count(sensor.y_tk) = sensor.classified_contact_count(sensor.y_tk-1);
    end
    
    % Count the number of true contacts
    if sensor.in_contact(sensor.x_tk - 1)==false && sensor.in_contact(sensor.x_tk)==true
        sensor.real_contact_count(sensor.x_tk) = sensor.real_contact_count(sensor.x_tk-1) + 1;
    else
        sensor.real_contact_count(sensor.x_tk) = sensor.real_contact_count(sensor.x_tk-1);
    end

    %Plotting
    if mod(sensor.x_tk, 10) == 0  % Update plot at a reduced rate
        update_plot(local_plot, angle_plot, robot, sensor, plants);
        if isempty(frames(end).cdata)
            frames(end) = getframe(gcf);
        else
            frames(end + 1) = getframe(gcf);
        end
    end
end
%---------------------------------------------
v = VideoWriter('feeler_animation3', 'MPEG-4');                                                   
v.FrameRate = 25;
v.Quality = 100;
open(v);
for k = 1:length(frames)                                                                        % Write frames to the video
   writeVideo(v, frames(k));
end
close(v);

function [local_plot, angle_plot] = init_figure(plants, sensor)
    figure("position", [0, 0, 1800, 800]);
    
    % Main sensor plot
    subplot(2, 5, [1, 6]);
    hold on;
    local_plot.paddles = plot(nan, nan, 'black-', 'LineWidth', 3, 'DisplayName', 'Paddle Sensor');
    local_plot.active_range = fill(nan, nan, 'cyan', 'FaceAlpha', 0.3, 'DisplayName', 'Active Range');  % Add filled circle segment
    local_plot.plants = plot(nan, nan, 'LineStyle', 'none', 'Marker', '.', 'MarkerEdgeColor', [0.1 0.6 0.1], 'MarkerSize', 30, 'DisplayName', 'Moving Point');
    
    xlabel('X Coordinate (+x = east)');
    ylabel('Y Coordinate (+y = north)');
    title('Sticky Sensor Model');
    axis equal;
    legend('Location', 'northwest');
    set(gca, 'Color', [222/255, 184/255, 135/255]);
    
    % Angle plot
    subplot(2, 5, [2, 3]);
    hold on;
    angle_plot.angle_plot_contact = plot(nan, nan, 'r.-', 'DisplayName', 'In Contact'); % Red markers for contact
    angle_plot.angle_plot_no_contact = plot(nan, nan, 'b.-', 'DisplayName', 'No Contact'); % Blue markers for no contact
    angle_plot.angle_text = text(0, 0, '0', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    xlabel('Simulation Time (s)');
    ylabel('Feeler Angle (deg)');
    title({'Actual Feeler State, 𝜃','and a Perfect Classifier, g(x,k)'});
    grid on;
    legend('Location', 'northwest');
    local_plot.real_contact_text = text(0.7, 0.95, 'Real Contacts: 0', 'Units', 'normalized', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'BackgroundColor', 'white', 'EdgeColor', 'black');

    
    % Angular velocity plot
    subplot(2, 5, [7, 8]);
    hold on;
    angle_plot.angle_velocity_plot = plot(nan, nan, 'r.-', 'DisplayName', 'Feeler Angular Velocity');
    angle_plot.angle_vel_text = text(0, 0, '0', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    xlabel('Simulation Time (s)');
    ylabel('Angular Velocity (deg/s)');
    title('Approx. Actual 𝜃_{dot}');
    grid on;
    legend('Location', 'northwest');
    
    % Measured Angle plot
    subplot(2, 5, [4, 5]);
    hold on;
    angle_plot.y_angle_plot_contact = plot(nan, nan, 'Marker', 's', 'Color', 'r', 'LineStyle', 'none','DisplayName', 'In Contact'); % Red markers for contact
    angle_plot.y_angle_plot_nearcrash = plot(nan, nan, 'Marker', 'pentagram', 'Color', [1 0.7 0], 'LineStyle', 'none', 'DisplayName', 'Nearing Crash'); % Orange markers for near crash
    angle_plot.y_angle_plot_smhup = plot(nan, nan, 'Marker', '^', 'Color', 'b', 'LineStyle', 'none', 'DisplayName', 'SHM Increasing'); % Blue markers for SHM Increasing
    angle_plot.y_angle_plot_smhdown = plot(nan, nan, 'Marker', 'v', 'Color', 'b', 'LineStyle', 'none', 'DisplayName', 'SHM Decreasing'); % Blue markers for SHM Decreasing
    angle_plot.y_angle_plot_steady = plot(nan, nan, 'Marker', 'd', 'Color', 'g', 'LineStyle', 'none', 'DisplayName', 'Steady State Angle'); % Green markers for Steady State Angle
    angle_plot.y_angle_plot_unknown = plot(nan, nan, 'Marker', 'x', 'Color', [0.9 0 0.9], 'LineStyle', 'none', 'DisplayName', 'Unknown State'); % Purple markers for Unknown State
    angle_plot.y_angle_text = text(0, 0, '0', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    xlabel('Sensor Time (s)');
    ylabel('Measured Angle (deg)');
    title({'Noisey Measured State, y','and our analytical classifier, f(y,k)'});
    grid on;
    legend('Location', 'northwest');

    angle_plot.classified_contact_text = text(0.7, 0.95, 'Classified Contacts: 0', 'Units', 'normalized', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    % Measured Angular velocity plot
    subplot(2, 5, [9, 10]);
    hold on;
    angle_plot.y_angle_velocity_plot = plot(nan, nan, 'r.-', 'DisplayName', 'Feeler Angular Velocity');
    angle_plot.y_angle_vel_text = text(0, 0, '0', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    xlabel('Sensor Time (s)');
    ylabel('Angular Velocity (deg/s)');
    title('Approx. Measured Angular Velocity y_{dot}');
    grid on;
    legend('Location', 'northwest');
end

% Function to move plants
function plants = move_plants(plants, sensor)
    for i = 1:length(plants)
        plants(i).y = plants(i).y - plants(i).speed * sensor.x_del_t;

        % Reset plant if it moves out of the screen
        if plants(i).y < -0.2
            plants(i).x = (sensor.paddle_length + 0.05) * rand();
            plants(i).y = 0.6 + sensor.intrarow_width + rand() * 0.1;  % Reset y-coordinate using intrarow width
        end
    end
end

% check_for_contact Function
function sensor = check_for_contact(sensor, plants)
    % Initialize the maximum angle difference
    max_angle_diff = 0;  
    sensor.contact_angle = 0;
    sensor.in_contact(sensor.x_tk) = false;

    for i = 1:length(plants)
        dx = plants(i).x - sensor.pin_location(1);
        dy = plants(i).y - sensor.pin_location(2);

        if sqrt(dx^2 + dy^2) <= sensor.paddle_length % Check if the point is within paddle length
            % Convert span to radians and normalize to [0, 2*pi]
            span_rad = deg2rad(sensor.span);
            span_start = mod(span_rad(1), 2 * pi);
            span_end = mod(span_rad(2), 2 * pi);

            % Calculate the angle to the point and normalize to [0, 2*pi]
            angle_to_point = atan2(dy, dx) - sensor.home_angle;
            angle_to_point = mod(angle_to_point, 2 * pi);  

            % Check if the angle is within the active range
            if (span_start < span_end && angle_to_point >= span_start && angle_to_point <= span_end) || ...
               (span_start >= span_end && (angle_to_point >= span_start || angle_to_point <= span_end))
                % Check proximity along the paddle
                num_points = 500;  
                t = linspace(0, 1, num_points);
                for j = 1:num_points
                    paddle_x = sensor.pin_location(1) + t(j) * (sensor.paddle_end_x - sensor.pin_location(1));
                    paddle_y = sensor.pin_location(2) + t(j) * (sensor.paddle_end_y - sensor.pin_location(2));

                    if sqrt((paddle_x - plants(i).x)^2 + (paddle_y - plants(i).y)^2) < 0.005 % Proximity check
                        angle_to_point = atan2(plants(i).y - sensor.pin_location(2), plants(i).x - sensor.pin_location(1));
                        angle_diff = abs(rad2deg(angle_to_point - sensor.home_angle) - sensor.home_angle); 

                        if angle_diff > max_angle_diff
                            max_angle_diff = angle_diff;
                            sensor.contact_angle = rad2deg(angle_to_point - sensor.home_angle); 
                        end
                        sensor.in_contact(sensor.x_tk) = true;
                        sensor.is_sticky = false;  
                        sensor.contact_location = [plants(i).x, plants(i).y]; 
                        break
                    end
                end
            end
        end
    end

    if ~sensor.in_contact(sensor.x_tk)
        sensor.contact_location = [];
    end    
end

% update_sensor_dynamics Function
function sensor = update_sensor_dynamics(sensor, robot)
    if sensor.in_contact(sensor.x_tk) == true
        sensor.x_angle_h(sensor.x_tk) = sensor.contact_angle;
        sensor.xdot_angle_h(sensor.x_tk) = (sensor.x_angle_h(sensor.x_tk) - sensor.x_angle_h(sensor.x_tk - 1)) / sensor.x_del_t;  % Store angular velocity history

    elseif sensor.in_contact(sensor.x_tk) == false
        sensor = recenter_sensor(sensor);
    end

    if sensor.x_tk > 2
        sensor.xddot_angle_h(sensor.x_tk) = (sensor.xdot_angle_h(sensor.x_tk) - sensor.xdot_angle_h(sensor.x_tk - 1)) / sensor.x_del_t;  % Store angular accel history
    end

    % Logging the physical state
    R = [cos(robot.heading), -sin(robot.heading); sin(robot.heading), cos(robot.heading)];
    sensor.global_pin_location = [robot.x; robot.y] + R * [sensor.pin_location(1:2)]';  % Transform the sensor's pin location to the global frame
    sensor.global_angle = sensor.home_angle + deg2rad(sensor.x_angle_h(end));  % Calculate paddle end position
    sensor.paddle_end_x = sensor.global_pin_location(1) + sensor.paddle_length * cos(sensor.global_angle + robot.heading);
    sensor.paddle_end_y = sensor.global_pin_location(2) + sensor.paddle_length * sin(sensor.global_angle + robot.heading);
end

% recenter_sensor Function
function sensor = recenter_sensor(sensor)
    % The sensor paddle's motion is governed by the following differential equation for a spring-damper system:
    % Iθ¨ + bθ˙ + kθ = 0
    % 
    % where:
    % θ is the angular displacement from the home position (sensor.local_angle).
    % θ˙ is the angular velocity (sensor.angle_velocity).
    % θ¨ is the angular acceleration.
    % I is the moment of inertia (assumed to be 1 for simplicity).
    % b is the damping coefficient (sensor.damping).
    % k is the spring constant (sensor.spring_constant).


    % Estimating moment of inertia of plate
    wid = sensor.paddle_length; % width 6in
    hei = 0.0635; % height 2.5in
    thi = 0.005; % thickness 5mm
    volu = wid * hei * thi; % volume of plate
    rho = 7850; % density of steel
    mas = volu * rho;
    I = (mas / 3) * (hei^2 + wid^2);
    
    % Sticky friction behavior
    if sensor.x_angle_h(sensor.x_tk - 1) >= rad2deg(sensor.sticky_region(1)) && sensor.x_angle_h(sensor.x_tk - 1) <= rad2deg(sensor.sticky_region(2))
        if rand() < sensor.sticky_friction_probability
            % Stick to the current angle
            sensor.is_sticky = true;  % Enter sticky state
        end
    end

    if sensor.is_sticky == true
        % Update such that it sticks
        sensor.xdot_angle_h(sensor.x_tk) = 0;  % Stop the velocity
        sensor.x_angle_h(sensor.x_tk) = sensor.x_angle_h(sensor.x_tk - 1);
        return;  % Exit the function to keep the current angle
    end
    
    % Calculate the spring force and damping
    angle_diff = deg2rad(sensor.x_angle_h(sensor.x_tk - 1)) - deg2rad(sensor.home_angle);
    spring_force = -sensor.spring_constant * angle_diff / I;
    damping_force = -sensor.damping * deg2rad(sensor.xdot_angle_h(sensor.x_tk - 1)) / (sensor.x_del_t * I);
    total_force = spring_force + damping_force;

    % Update the local angle based on the forces
    sensor.xdot_angle_h(sensor.x_tk) = sensor.xdot_angle_h(sensor.x_tk - 1) + rad2deg(total_force * sensor.x_del_t);
    sensor.x_angle_h(sensor.x_tk) = sensor.x_angle_h(sensor.x_tk - 1) + sensor.xdot_angle_h(sensor.x_tk - 1) * sensor.x_del_t;
end

% update_plot Function
function update_plot(local_plot, angle_plot, robot, sensor, plants)
    % Update paddle position
    set(local_plot.paddles, 'XData', [sensor.global_pin_location(1), sensor.paddle_end_x], 'YData', [sensor.global_pin_location(2), sensor.paddle_end_y]);

    % Update plant positions
    x_points = arrayfun(@(p) p.x, plants);
    y_points = arrayfun(@(p) p.y, plants);
    set(local_plot.plants, 'XData', x_points, 'YData', y_points);

    % Calculate active range in global frame
    span_angles = linspace(sensor.span(1), sensor.span(2), 100);  % Discretize span into 100 points
    span_angles_rad = deg2rad(span_angles) + sensor.home_angle;  % Convert to radians and shift by home angle
    x_span = sensor.global_pin_location(1) + sensor.paddle_length * cos(span_angles_rad + robot.heading);
    y_span = sensor.global_pin_location(2) + sensor.paddle_length * sin(span_angles_rad + robot.heading);

    % Update active range display
    set(local_plot.active_range, 'XData', [sensor.global_pin_location(1); x_span'], 'YData', [sensor.global_pin_location(2); y_span']);

    % Update simulation time display in the annotation
    %set(local_plot.sim_time_box, 'String', sprintf('Sim Time: %.2f s', sensor.x_time_h(end)));

    secs_to_disp = 3;
    % Update angle plot
    x_data = sensor.x_time_h(sensor.x_time_h >= sensor.x_time_h(sensor.x_tk) - secs_to_disp);
    y_data = sensor.x_angle_h(end - length(x_data) + 1:end);

    % Separate data based on contact status
    in_contact_idx = find(sensor.in_contact);
    no_contact_idx = find(~sensor.in_contact);

    set(angle_plot.angle_plot_contact, 'XData', sensor.x_time_h(in_contact_idx), 'YData', sensor.x_angle_h(in_contact_idx), 'LineStyle', 'none');
    set(angle_plot.angle_plot_no_contact, 'XData', sensor.x_time_h(no_contact_idx), 'YData', sensor.x_angle_h(no_contact_idx), 'LineStyle', 'none');
    
    set(angle_plot.angle_text, 'String', sprintf('%.2f', sensor.x_angle_h(end)), 'Position', [sensor.x_time_h(end), sensor.x_angle_h(end)]);
    xlim_a = [sensor.x_time_h(end) - secs_to_disp, sensor.x_time_h(end) + 1];
    ylim_a = [-90, 90];
    set(get(angle_plot.angle_plot_contact, 'Parent'), 'XLim', xlim_a, 'YLim', ylim_a);

    % Update angular velocity plot
    y_data_vel = sensor.xdot_angle_h(end - length(x_data) + 1:end);
    set(angle_plot.angle_velocity_plot, 'XData', x_data, 'YData', y_data_vel);
    set(angle_plot.angle_vel_text, 'String', sprintf('%.2f', sensor.xdot_angle_h(end)), 'Position', [sensor.x_time_h(end), sensor.xdot_angle_h(end)]);
    xlim_av = [sensor.x_time_h(end) - secs_to_disp, sensor.x_time_h(end) + 1];
    ylim_av = [-1000, 1000];
    set(get(angle_plot.angle_velocity_plot, 'Parent'), 'XLim', xlim_av, 'YLim', ylim_av);

    % Update measured angle plot
    y_in_contact_idx = strcmp(sensor.f_y_angle_classifier_h, "In Contact");
    y_nearcrash_idx = strcmp(sensor.f_y_angle_classifier_h, "Nearing Crash");
    y_smhup_idx = strcmp(sensor.f_y_angle_classifier_h, "SHM Increasing");
    y_smhdown_idx = strcmp(sensor.f_y_angle_classifier_h, "SHM Decreasing");
    y_steady_idx = strcmp(sensor.f_y_angle_classifier_h, "Steady State Angle");
    y_unknown_idx = strcmp(sensor.f_y_angle_classifier_h, "Unknown State");

    set(angle_plot.y_angle_plot_contact, 'XData', sensor.y_time_h(y_in_contact_idx), 'YData', sensor.y_angle_h(y_in_contact_idx), 'LineStyle', 'none');
    set(angle_plot.y_angle_plot_nearcrash, 'XData', sensor.y_time_h(y_nearcrash_idx), 'YData', sensor.y_angle_h(y_nearcrash_idx), 'LineStyle', 'none');
    set(angle_plot.y_angle_plot_smhup, 'XData', sensor.y_time_h(y_smhup_idx), 'YData', sensor.y_angle_h(y_smhup_idx), 'LineStyle', 'none');
    set(angle_plot.y_angle_plot_smhdown, 'XData', sensor.y_time_h(y_smhdown_idx), 'YData', sensor.y_angle_h(y_smhdown_idx), 'LineStyle', 'none');
    set(angle_plot.y_angle_plot_steady, 'XData', sensor.y_time_h(y_steady_idx), 'YData', sensor.y_angle_h(y_steady_idx), 'LineStyle', 'none');
    set(angle_plot.y_angle_plot_unknown, 'XData', sensor.y_time_h(y_unknown_idx), 'YData', sensor.y_angle_h(y_unknown_idx), 'LineStyle', 'none');
    
    set(angle_plot.y_angle_text, 'String', sprintf('%.2f', sensor.y_angle_h(end)), 'Position', [sensor.y_time_h(end), sensor.y_angle_h(end)]);
    xlim_y_a = [sensor.y_time_h(end) - secs_to_disp, sensor.y_time_h(end) + 1];
    ylim_y_a = [-90, 90];
    set(get(angle_plot.y_angle_plot_contact, 'Parent'), 'XLim', xlim_y_a, 'YLim', ylim_y_a);

    % Update measured angular velocity plot
    y_data_vel = sensor.ydot_angle_h(end - length(x_data) + 1:end);
    set(angle_plot.y_angle_velocity_plot, 'XData', x_data, 'YData', y_data_vel);
    set(angle_plot.y_angle_vel_text, 'String', sprintf('%.2f', sensor.ydot_angle_h(end)), 'Position', [sensor.y_time_h(end), sensor.ydot_angle_h(end)]);
    xlim_y_av = [sensor.y_time_h(end) - secs_to_disp, sensor.y_time_h(end) + 1];
    ylim_y_av = [-1000, 1000];
    set(get(angle_plot.y_angle_velocity_plot, 'Parent'), 'XLim', xlim_y_av, 'YLim', ylim_y_av);

    % Update local plot limits
    xlim_local = [robot.x - 0.05, robot.x + 0.4];
    ylim_local = [robot.y - 0.4, robot.y + 0.6];
    set(get(local_plot.paddles, 'Parent'), 'XLim', xlim_local, 'YLim', ylim_local);
    
    % Update contact counters
    set(local_plot.real_contact_text, 'String', sprintf('Real Contacts: %d', sensor.real_contact_count(sensor.x_tk)));
    set(angle_plot.classified_contact_text, 'String', sprintf('Classified Contacts: %d', sensor.classified_contact_count(sensor.y_tk)));
    
    drawnow;
end

function sensor = classify_state(sensor)

    i=sensor.y_tk;
    if i <= sensor.classifier_window
        sensor.current_state="Unknown State";
    elseif i > sensor.classifier_window      
        % Determine state based on conditions
        switch sensor.current_state
            case "Unknown State"
                if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif sensor.ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -30000
                    sensor.current_state = "In Contact";
                elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) < 0) && all(sensor.y_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Decreasing";
                end
    
            case "Steady State Angle"
                if sensor.ydot_angle_h(i) < 1 && sensor.yddot_angle_h(i) < -15000
                    sensor.current_state = "In Contact";
                elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                end

           case "SHM Increasing"
                if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif sensor.y_angle_h(i) > 5 && abs(sensor.ydot_angle_h(i)) < 100 && sensor.yddot_angle_h(i) < 0
                    sensor.current_state= "SHM Decreasing";
                elseif sensor.ydot_angle_h(i) < -30 && sensor.yddot_angle_h(i) > 0
                    sensor.current_state = "In Contact";
                end
    
            case "SHM Decreasing"
                if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif sensor.ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -20000
                    sensor.current_state = "In Contact";
                elseif sensor.y_angle_h(i) < 0 && abs(sensor.ydot_angle_h(i)) < 100 && sensor.yddot_angle_h(i) > 0
                    sensor.current_state= "SHM Increasing";
                end
                
            case "In Contact"
                if all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                elseif all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 20) ...
                        && all(abs(sensor.yddot_angle_h(i-sensor.classifier_window:i)) < 1000)
                        sensor.current_state= "Steady State Angle";
                elseif sensor.y_angle_h(i)<-75
                        sensor.current_state= "Nearing Crash";
                end

            case "Nearing Crash"
                %if sensor.ydot_angle_h(i) < 0 && sensor.y_angle_h(i) < 0
                %    sensor.f_y_angle_classifier_h(i)h(i) = "In Contact";
                if all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                elseif all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 8) ...
                        && all(abs(sensor.yddot_angle_h(i-sensor.classifier_window:i)) < 1000)
                    sensor.current_state= "Steady State Angle";
                end
                
            otherwise
                sensor.current_state= "Unknown State";
        end
    end

    sensor.f_y_angle_classifier_h(i)=sensor.current_state;
    return
end

