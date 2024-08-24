%--------------------------------------------------------------------------
% INITIALIZATION
clear all
close all

% SIMULATION VARIABLES
sim.del_t = 0.008; % Simulation sample time
sim.simulation_time = 0; % Initialize simulation time
sim.last_plot_update_time = 0; 

% ROBOT INITIALIZATION
% Robot dimensions
robot.Wr = 0.497; % Robot width, 49.7 cm
robot.Lr = 0.508; % Robot length, 50.8 cm
robot.track = 0.381; % Track width of the robot wheel to wheel, in meters
robot.x_0 = -1; % Initial x position (wrt global frame)
robot.y_0 = 3; % Initial y position (wrt global frame)
robot.h_0 = -pi/2; % Initial heading (90 degrees in radians) (wrt global frame, east rhr counterclockwise)
% True robot state
robot.x = robot.x_0; % Current true x position (wrt global frame)
robot.y = robot.y_0; % Current true y position (wrt global frame)
robot.heading = robot.h_0; % Current true heading (wrt global frame)
% True robot state histories
robot.x_pos_h(1) = robot.x; % History of true x positions (gf)
robot.y_pos_h(1) = robot.y; % History of true y positions (gf)
robot.h_pos_h(1) = robot.heading; % History of true headings (gf)
% Measurement variables
robot.measurement_x = robot.x; % Measured x position (gf)
robot.measurement_y = robot.y; % Measured y position (gf)
robot.measurement_heading = robot.heading; % Measured heading (gf)
% Measurement histories
robot.measurement_x_h(1) = robot.measurement_x; % History of measured x positions (gf)
robot.measurement_y_h(1) = robot.measurement_y; % History of measured y positions (gf)
robot.measurement_h_h(1) = robot.measurement_heading; % History of measured headings (gf)
% Speed reference histories
robot.left_speed_ref_h(1) = 0; % History of left speed references 
robot.right_speed_ref_h(1) = 0; % History of right speed references
% Measurement time
robot.measurement_time = 0; % Initialize position measurement time
robot.del_t = 0.25; % Robot sample time in seconds

% PLANT CONSTRUCTION
row_width =0.83; % 0.762; % 30 inches
intrarow_width = 0.250; % 10 inches
plants_per_row = round(50 / intrarow_width); % 50 meter long rows
plants_1 = linspace(5, 55, plants_per_row);
num_rows = 4;
total_plants = plants_per_row * num_rows;
plants(total_plants).x = [];
plants(total_plants).y = [];
index = 1;
for i = 0:num_rows-1
    x_values = (row_width/2 * (2*i - 1)) * ones(size(plants_1)) + randn(size(plants_1)) / 35;
    y_values = plants_1 + randn(size(plants_1)) / 35;
    for j = 1:plants_per_row
        plants(index).x = x_values(j)+2*sin(j*4*pi/plants_per_row);
        plants(index).y = y_values(j);
        index = index + 1;
    end
end
% Remove some plants at random
avg_removals = 50; % Average number of plants to remove
probability_of_removal = avg_removals / numel(plants); % Probability of removal
mask = rand(1, numel(plants)) > probability_of_removal;
% Filter out removed plants
filtered_plants = struct('x', [], 'y', []);
index = 1; % Initialize index for filtered plants
for i = 1:numel(plants)
    if mask(i)
        filtered_plants(index).x = plants(i).x;
        filtered_plants(index).y = plants(i).y;
        index = index + 1;
    end
end
plants = filtered_plants; %plant positions (global frame)
clear x_values  y_values intrarow_width num_rows avg_removals i j index plants_1 plants_per_row filtered_plants total_plants probability_of_removal mask;

% NAVIGATION
% Goal Points, meters (global frame); Remember we only need to travel down every other row alley
robot.goal_points = [
        0, 4.9, 90; 
        0, 55, 90;
        0, 58, 90;
        row_width*2, 58, -90
        row_width*2, 54.9, -90;
        row_width*2, 5, -90; 
        row_width*4, 2, 0;
      ];
robot = cleanup_goal_points(robot);
robot.current_goal=robot.goal_points(1,:);
robot.current_goal_index = 1; % Assuming this starts at the first goal index
robot.planning_distance = 3; %only plan 2 meters ahead of the robot based on the goal_points
robot.obstactle_decay_buffer = -20; % Exponential padding away from plants in A*
robot.grid_size = 0.05; % A* grid size for search
robot.global_map = []; % Initialize the global map to store plant locations
robot.replan_path = false; %Should we replan the path?
robot.local_map_mask = [];
% Pure Pursuit Values
robot.Ld = 0.50; % Lookahead distance in meters
robot.goal_thresh = 0.5; % Goal threshold in meters
robot.v_ref = -0.2; % Nominal speed set for the robot in m/s
robot.buffer_size = 150; %only keep 150 timesteps at most


% SENSOR INITIALIZATION
% Front Right
sensor(1).pin_location = [3*robot.Lr / 2, -0.1143]; % Front right of the robot (translation in x, y in the robot frame which has x aligned with its heading)
sensor(1).mounting_angle = deg2rad(-90);     % Angle at which the sensor is mounted on the robot (robot frame)
R = [cos(robot.heading), -sin(robot.heading); sin(robot.heading), cos(robot.heading)]; %Rotation from robot frame to global frame
sensor(1).global_pin_location = [robot.x; robot.y] + R * [sensor(1).pin_location(1:2)]';  %(global pin location)
sensor(1).home_angle = 0;                    % Home position angle (in sensor frame)
sensor(1).span = [-90, 15];                  % Active angle range in degrees, always define counterclockwise (in the sensor frame)
sensor(1).paddle_length = 0.3556;  % 6 inches in meters
sensor(1).spring_constant = 0.67;    % Nm/rad, adjust for stiffness
sensor(1).damping = 0.0003;          % Nm*s/rad, damping factor
sensor(1).sticky_region = [-deg2rad(25), deg2rad(10)];  % Sticky region in radians (in the sensor frame)
sensor(1).sticky_friction_probability = 0.07;  % Probability of settling on a given angle 
sensor(1).is_sticky = false;  % Flag to indicate if the sensor is in a sticky state
% Back Right
% sensor(2).pin_location = [robot.Lr / 2, -robot.Wr / 2]; % Front right of the robot (translation in x, y in the robot frame which has x aligned with its heading)
% sensor(2).mounting_angle = deg2rad(-90);     % Angle at which the sensor is mounted on the robot (robot frame)
% R = [cos(robot.heading), -sin(robot.heading); sin(robot.heading), cos(robot.heading)]; %Rotation from robot frame to global frame
% sensor(2).global_pin_location = [robot.x; robot.y] + R * [sensor(2).pin_location(1:2)]';  %(global pin location)
% sensor(2).home_angle = 0;                    % Home position angle (in sensor frame)
% sensor(2).span = [-90, 15];                  % Active angle range in degrees, always define counterclockwise (in the sensor frame)
% sensor(2).paddle_length = 0.1524;  % 6 inches in meters
% sensor(2).spring_constant = 1.2* 0.67;    % Nm/rad, adjust for stiffness
% sensor(2).damping = 0.00029;          % Nm*s/rad, damping factor
% sensor(2).sticky_region = [-deg2rad(15), deg2rad(10)];  % Sticky region in radians (in the sensor frame)
% sensor(2).sticky_friction_probability = 0.05;  % Probability of settling on a given angle 
% sensor(2).is_sticky = false;  % Flag to indicate if the sensor is in a sticky state
% Front left
sensor(2).pin_location = [3*robot.Lr / 2, 0.1143]; % Front right of the robot (translation in x, y in the robot frame which has x aligned with its heading)
sensor(2).mounting_angle = deg2rad(90);     % Angle at which the sensor is mounted on the robot (robot frame)
R = [cos(robot.heading), -sin(robot.heading); sin(robot.heading), cos(robot.heading)]; %Rotation from robot frame to global frame
sensor(2).global_pin_location = [robot.x; robot.y] + R * [sensor(2).pin_location(1:2)]';  %(global pin location)
sensor(2).home_angle = 0;                    % Home position angle (in sensor frame)
sensor(2).span = [-15, 90];                  % Active angle range in degrees, always define counterclockwise (in the sensor frame)
sensor(2).paddle_length = 0.3556;  % 6 inches in meters
sensor(2).spring_constant = 1.2*0.673;    % Nm/rad, adjust for stiffness
sensor(2).damping = 0.0003;          % Nm*s/rad, damping factor
sensor(2).sticky_region = [-deg2rad(10), deg2rad(10)];  % Sticky region in radians (in the sensor frame)
sensor(2).sticky_friction_probability = 0.02;  % Probability of settling on a given angle 
sensor(2).is_sticky = false;  % Flag to indicate if the sensor is in a sticky state
% Back left
% sensor(4).pin_location = [robot.Lr / 2, robot.Wr / 2]; % Front right of the robot (translation in x, y in the robot frame which has x aligned with its heading)
% sensor(4).mounting_angle = deg2rad(90);     % Angle at which the sensor is mounted on the robot (robot frame)
% R = [cos(robot.heading), -sin(robot.heading); sin(robot.heading), cos(robot.heading)]; %Rotation from robot frame to global frame
% sensor(4).global_pin_location = [robot.x; robot.y] + R * [sensor(4).pin_location(1:2)]';  %(global pin location)
% sensor(4).home_angle = 0;                    % Home position angle (in sensor frame)
% sensor(4).span = [-15, 90];                  % Active angle range in degrees, always define counterclockwise (in the sensor frame)
% sensor(4).paddle_length = 0.1524;  % 6 inches in meters
% sensor(4).spring_constant = 0.672;    % Nm/rad, adjust for stiffness
% sensor(4).damping = 0.0003;          % Nm*s/rad, damping factor
% sensor(4).sticky_region = [-deg2rad(10), deg2rad(5)];  % Sticky region in radians (in the sensor frame)
% sensor(4).sticky_friction_probability = 0.05;  % Probability of settling on a given angle 
% sensor(4).is_sticky = false;  % Flag to indicate if the sensor is in a sticky state
for i = [1,2]
    % "Real"-Simulated State Histories
    sensor(i).x_del_t = 0.008;             % Update rate of our paddle model
    sensor(i).x_time = 0;
    sensor(i).x_angle_h(1) = 0;            % Actual angle history, initial angular value (in the sensor frame)             
    sensor(i).xdot_angle_h(1) = 0;         % Actual angular velocities histories
    sensor(i).xddot_angle_h(1) = 0; % Ensure xddot_angle_h is initialized
    sensor(i).contact_location = [];       % (in the sensor frame)
    sensor(i).in_contact = false;             % Flag to indicate if sensor is in contact

    % Measurement Variables
    sensor(i).y_del_t = sensor(i).x_del_t;    % Sample period for measurement
    sensor(i).y_time = sensor(i).x_time;
    sensor(i).y_angle_h = sensor(i).x_angle_h; %our angle measurement (in sensor frame)
    sensor(i).ydot_angle_h = sensor(i).xdot_angle_h;
    sensor(i).yddot_angle_h = zeros(1, 1);  % Initial acceleration history
    sensor(i).classifier_window = 5;       % Each transition can only happen after this number times the sensor data rate
    sensor(i).current_state = "Unknown State";
    sensor(i).f_y_angle_classifier_h(1) = sensor(i).current_state;  % Initial state classifier history

    sensor(i).estimated_global_pin_location = [robot.measurement_x; robot.measurement_y] + R * [sensor(i).pin_location(1:2)]'; % Initial estimated global pin location
    sensor(i).contact_angle = [];
    sensor(i).estimated_global_angle = sensor(i).mounting_angle + deg2rad(sensor(i).y_angle_h(end));  % Calculate paddle end position
    sensor(i).true_global_angle=[];
    sensor(i).paddle_end_x=[];
    sensor(i).paddle_end_y=[];
    % Localization
    sensor(i).D_hat = []; %(in sensor frame) %x distance from pin (0,0) to plant in contact
    sensor(i).P_hat = []; %(in sensor frame) %y distance from pin (0,0) to plant in contact
    sensor(i).H_hat = []; %(in sensor frame)
    sensor(i).plant_localization_in_sensor_frame ={};
    sensor(i).plant_localization_in_robot_frame={};
    sensor(i).plant_localization_in_global_frame={};

    %Time series buffer
    sensor(i).buffer_size=100; %timesteps
end

% PLOTTING
[true_plot, global_plot, est_plot] = init_figure(robot, plants, sensor); % initialize the plot
frames = struct('cdata', [], 'colormap', []); % initialize the frames we will save

%----------------------------------------------------------------------------------------------------------------------------------------------------
% MAIN LOOP
robot = plan_path(robot); %This is the initial path. It should be greedy
robot.time_since_planning = 0;

v = VideoWriter('astar_bumpSLAM_with_4_feelers4', 'MPEG-4');                                                   
v.FrameRate = 25;
v.Quality = 100;
open(v);

while true
    % Update physics every sim.del_t seconds
    robot = update_robot_position(robot, sim); % Update robot position and heading based on wheel speeds

    % SENSOR SECTION
    sensor = update_sensors(sensor, plants, robot); %True pose  
    sensor = measure_sensors(sensor, robot); %Measured pose
    [sensor, robot] = contact_mapping(sensor, robot); %Localization and Mapping

    % PATH PLANNING
    robot.time_since_planning = robot.time_since_planning + sensor(1).y_del_t; % Accumulate time since last planning
    if robot.time_since_planning> 0.125/2 %if a second has passed
        robot.replan_path=true;
    end
    if robot.replan_path==true %if a sec has passed or the map has change
         robot = plan_path(robot);
         robot.time_since_planning = 0;
        
    end
    robot = pure_pursuit(robot);

    % Measurement updates every robot.del_t seconds
    if sim.simulation_time >= robot.measurement_time+robot.del_t
        [robot, sensor] = robot_measurement(robot, sensor); %Pose measurement
        %robot = pure_pursuit(robot);
            % Check if goal is reached
        if sqrt((robot.goal_points(end, 1) - robot.x)^2 + (robot.goal_points(end, 2) - robot.y)^2) <= robot.goal_thresh
            disp('Made it to goal');
            break;
        end
        % Check if the robot has reached the current goal
        if norm([robot.measurement_x, robot.measurement_y] - robot.current_goal(1:2)) < robot.goal_thresh
            % Move to the next goal if available
            if robot.current_goal_index < size(robot.goal_points, 1)
                robot.current_goal_index = robot.current_goal_index + 1;
                robot.current_goal = robot.goal_points(robot.current_goal_index, :);
            end
        end
    end

    % Update plot
    if sim.simulation_time - sim.last_plot_update_time >= 0.008*5 % Update plot at a reduced rate
        update_plot(true_plot, global_plot, est_plot, robot, sensor);
        %frames(end + 1) = getframe(gcf);
        writeVideo(v, getframe(gcf));
        sim.last_plot_update_time=sim.simulation_time;
    end
    
    sim.simulation_time = sim.simulation_time + sim.del_t; % Update the simulation time

    sensor = timeseries_cleaning_buffer(sensor); %keep the timeseries information from exploding

    %robot = timeseries_cleaning_buffer_robot(robot);
end


% v = VideoWriter('astar_contactSLAM_with_4_feelers', 'MPEG-4');                                                   
% v.FrameRate = 25;
% v.Quality = 100;
% open(v);
% for k = 2:length(frames)
%     writeVideo(v, frames(k));
% end
close(v);
%----------------------------------------------------------------------------------------------------------------------------------------------------

%----------------------------------------------------------------------------------------------------------------------------------------------------
% FUNCTIONS
% Function to clean up the goal points list based on the starting robot position
function robot = cleanup_goal_points(robot)
    % Initialize variables
    p_min = []; % Closest point on the straight path between two adjacent goal points
    p_min_dist = inf; % Distance to closest point to be found
    p_seg = []; % The segment the closest point is on to be found

    % Loop through the goals to find the closest segment
    for j = 1:size(robot.goal_points, 1) - 1
        goal1 = robot.goal_points(j, :);                
        goal2 = robot.goal_points(j + 1, :);
        
        % Projection factor indicating how far along the segment the closest point is
        M = max(0, min(1, ((robot.measurement_x - goal1(1)) * (goal2(1) - goal1(1)) + ...
            (robot.measurement_y - goal1(2)) * (goal2(2) - goal1(2))) / ((goal2(1) - goal1(1))^2 + (goal2(2) - goal1(2))^2)));

        % X and Y coordinates of closest point on segment
        seg_min_x = goal1(1) + M * (goal2(1) - goal1(1)); 
        seg_min_y = goal1(2) + M * (goal2(2) - goal1(2)); 
        % Distance from robot to closest point of segment
        p_dist = sqrt((robot.measurement_x - seg_min_x)^2 + (robot.measurement_y - seg_min_y)^2); 
        if p_dist < p_min_dist % If this is the closest point so far...
            p_min_dist = p_dist; % Update the closest distance,
            p_min = [seg_min_x, seg_min_y]; % The closest point
            p_seg = [goal1; goal2]; % And the closest segment
        end
    end

    % Check if p_min is the beginning of the closest segment
    if isequal(p_min, p_seg(1, 1:2))
        % Set goal points from the beginning of the closest segment onward
        robot.goal_points = [p_seg(1, :); robot.goal_points(find(robot.goal_points(:, 1) == p_seg(2, 1) & robot.goal_points(:, 2) == p_seg(2, 2), 1):end, :)];
        return    
    % Check if p_min is the end of the closest segment
    elseif isequal(p_min, p_seg(2, 1:2))
        % Set goal points from the end of the closest segment onward
        robot.goal_points = robot.goal_points(find(robot.goal_points(:, 1) == p_seg(2, 1) & robot.goal_points(:, 2) == p_seg(2, 2), 1):end, :);
        return
    else
        % If p_min is somewhere in the middle of the closest segment, interpolate the heading and set the new goal list
        interpolated_heading = p_seg(1, 3) + M * (p_seg(2, 3) - p_seg(1, 3));
        new_goal_point = [p_min, interpolated_heading];
        robot.goal_points = [new_goal_point; robot.goal_points(find(robot.goal_points(:, 1) == p_seg(2, 1) & robot.goal_points(:, 2) == p_seg(2, 2), 1):end, :)];
    end
end

function robot = pure_pursuit(robot)
    robot = find_Ld_point(robot); % Find lookahead point
    robot = calculate_circle_and_speeds(robot); % Calculate the circle and wheel speeds
    
    % Update speed reference histories
    robot.left_speed_ref_h(end + 1) = robot.left_speed;
    robot.right_speed_ref_h(end + 1) = robot.right_speed;
end

function robot = find_Ld_point(robot)
    path_points=robot.planned_path(:,1:2);
    % Initialize the path segment to start with the first segment
    current_index = 1;
    
    while current_index < size(path_points, 1)
        % Define the start and end of the current segment
        x1 = path_points(current_index, 1);
        y1 = path_points(current_index, 2);
        x2 = path_points(current_index + 1, 1);
        y2 = path_points(current_index + 1, 2);

        % Calculate coefficients for quadratic equation
        a = (x2 - x1)^2 + (y2 - y1)^2;
        b = 2 * ((x1 - robot.measurement_x) * (x2 - x1) + (y1 - robot.measurement_y) * (y2 - y1));
        c = (x1 - robot.measurement_x)^2 + (y1 - robot.measurement_y)^2 - robot.Ld^2;

        discriminant = b^2 - 4 * a * c;

        if discriminant >= 0
            sqrt_discriminant = sqrt(discriminant);
            t1 = (-b + sqrt_discriminant) / (2 * a);
            t2 = (-b - sqrt_discriminant) / (2 * a);

            t = []; % Select the valid t in the range [0, 1]
            if 0 <= t1 && t1 <= 1
                t = t1;
            elseif 0 <= t2 && t2 <= 1
                t = t2;
            end

            if ~isempty(t) % Valid t found, calculate Ld point coordinates
                robot.Lx = x1 + t * (x2 - x1);
                robot.Ly = y1 + t * (y2 - y1);
                
                % Ensure Ld point is not behind the robot
                if dot([robot.Lx - robot.measurement_x, robot.Ly - robot.measurement_y], ...
                       [cos(robot.measurement_heading), sin(robot.measurement_heading)]) > 0
                    return;
                end
            end
        end

        % Move to the next segment if no valid Ld found on the current segment
        current_index = current_index + 1;
    end

    % If no valid Ld point found, set it to the last point in the path
    robot.Lx = path_points(end, 1);
    robot.Ly = path_points(end, 2);
end


function robot = calculate_circle_and_speeds(robot)

    % Calculate the tangential unit vector from the heading angle
    theta_h = robot.measurement_heading; % Current heading in radians
    T_x = cos(theta_h);
    T_y = sin(theta_h);

    % Calculate the vector from the current position to the Ld point
    dx = robot.Lx - robot.measurement_x;
    dy = robot.Ly - robot.measurement_y;

    % Calculate the perpendicular distance from the Ld point to the tangential vector
    cross = dx * T_y - dy * T_x;
    dot_product = dx * T_x + dy * T_y;
    mag_a = sqrt(dx^2 + dy^2);
    mag_b = sqrt(T_x^2 + T_y^2);
    angle_dif_rad = acos(dot_product / (mag_a * mag_b));
    angle_dif_deg = rad2deg(angle_dif_rad);

    % Calculate the radius of the circle
    perpendicular_distance = robot.Ld / 2;
    robot.turn_rad = perpendicular_distance / cos(abs(deg2rad(90) - angle_dif_rad));

    % Calculate the perpendicular vector to T_x, T_y
    if cross > 0
        perp_vector = [T_y, -T_x];
    else
        perp_vector = [-T_y, T_x];
    end

    robot.turn_center_x = robot.measurement_x + robot.turn_rad * perp_vector(1);
    robot.turn_center_y = robot.measurement_y + robot.turn_rad * perp_vector(2);

    % Determine left and right wheel speeds
    if robot.turn_rad ~= 0 && (0 <= angle_dif_deg && angle_dif_deg <= 30 || 330 <= angle_dif_deg && angle_dif_deg <= 360)
        % If it is facing forward, use normal pure pursuit
        if cross > 0
            robot.right_speed = robot.v_ref * (robot.turn_rad + robot.track / 2) / robot.turn_rad; % Turn right
            robot.left_speed = robot.v_ref * (robot.turn_rad - robot.track / 2) / robot.turn_rad;
        else
            robot.right_speed = robot.v_ref * (robot.turn_rad - robot.track / 2) / robot.turn_rad; % Turn left
            robot.left_speed = robot.v_ref * (robot.turn_rad + robot.track / 2) / robot.turn_rad;
        end
    elseif 30 < angle_dif_deg && angle_dif_deg < 330
        % If you are facing the wrong way, turn around
        robot.turn_rad = 0.2; % Turn around by making the radius small
        robot.turn_center_x = robot.measurement_x + robot.turn_rad * perp_vector(1);
        robot.turn_center_y = robot.measurement_y + robot.turn_rad * perp_vector(2);
        if cross > 0
            robot.right_speed = robot.v_ref * (robot.turn_rad + robot.track / 2) / robot.turn_rad; % Turn right
            robot.left_speed = robot.v_ref * (robot.turn_rad - robot.track / 2) / robot.turn_rad;
        else
            robot.right_speed = robot.v_ref * (robot.turn_rad - robot.track / 2) / robot.turn_rad; % Turn left
            robot.left_speed = robot.v_ref * (robot.turn_rad + robot.track / 2) / robot.turn_rad;
        end
    else
        robot.right_speed = 0;
        robot.left_speed = 0;
    end
end

function robot = plan_path(robot)
    robot = astar_plan(robot);
end

function robot = astar_plan(robot)

    % Get the segment to plan
    robot = seg_to_plan(robot);

    % Check the number of poses in the segment
    if size(robot.segment, 1) == 2
        % If there are two poses, plan directly between them
        robot.planned_path = astar_seg_plan(robot.segment(1, 1:2), robot.segment(2, 1:2), robot.global_map);
    elseif size(robot.segment, 1) == 3
        % If there are three poses, plan between the first two and the second two
        path1 = astar_seg_plan(robot.segment(1, 1:2), robot.segment(2, 1:2), robot.global_map);
        path2 = astar_seg_plan(robot.segment(2, 1:2), robot.segment(3, 1:2), robot.global_map);
        % Combine the paths into one overall path
        robot.planned_path = [path1; path2(2:end, :)]; % Avoid duplicating the middle pose
    end

    function robot = seg_to_plan(robot)
        distance_to_goal = norm([robot.current_goal(1) - robot.measurement_x, robot.current_goal(2) - robot.measurement_y]);
    
        if distance_to_goal > robot.planning_distance
            direction = (robot.current_goal(1:2) - [robot.measurement_x, robot.measurement_y]) / distance_to_goal;
            segment_end = [robot.measurement_x, robot.measurement_y] + robot.planning_distance * direction;
            segment_heading = robot.measurement_heading + (robot.current_goal(3) - robot.measurement_heading) * (robot.planning_distance / distance_to_goal);
            robot.segment = [robot.measurement_x, robot.measurement_y, robot.measurement_heading; segment_end, segment_heading];
        elseif robot.current_goal_index < size(robot.goal_points, 1)
            next_goal = robot.goal_points(robot.current_goal_index + 1, :);
            direction = (next_goal(1:2) - robot.current_goal(1:2)) / norm(next_goal(1:2) - robot.current_goal(1:2));
            segment_end = robot.current_goal(1:2) + (robot.planning_distance - distance_to_goal) * direction;
            segment_heading = robot.current_goal(3) + (next_goal(3) - robot.current_goal(3)) * ((robot.planning_distance - distance_to_goal) / norm(next_goal(1:2) - robot.current_goal(1:2)));
            robot.segment = [robot.measurement_x, robot.measurement_y, robot.measurement_heading; robot.current_goal(:)'; segment_end, segment_heading];
        else
            robot.segment = [robot.measurement_x, robot.measurement_y, robot.measurement_heading; robot.current_goal(:)'];
        end
    end

    function path = astar_seg_plan(start_pose, goal_pose, map)  
    
        % Calculate the direction vector from start to goal
        direction = (goal_pose - start_pose) / norm(goal_pose - start_pose);
        perpendicular = [-direction(2), direction(1)];
    
        % Initialize the central line with the start pose
        X(1) = start_pose(1);
        Y(1) = start_pose(2);
    
        % Step 1: Move along the direction vector and add grid points
        current_point = start_pose;
        while norm(current_point - start_pose) < norm(goal_pose - start_pose)
            % Move one grid size along the direction vector
            current_point = current_point + robot.grid_size * direction;
            X (end+1) = [current_point(1)];
            Y (end+1) = [current_point(2)];
        end
    
        % Step 2: Extend the central axis by padding on both ends
        padding = 0 * robot.grid_size; % Define padding distance
        % Pre-padding (before the start point)
        pre_padding_point(1,:) = start_pose - robot.grid_size * direction;
        while norm(pre_padding_point - start_pose) < padding
            pre_padding_point(end+1,:) = pre_padding_point(end,:) - robot.grid_size * direction;       
        end

        % Post-padding (after the goal point)
        post_padding_point(1,:) = current_point + robot.grid_size * direction;
        while norm(post_padding_point - goal_pose) < padding
            post_padding_point(end+1,:) = post_padding_point(end,:) + robot.grid_size * direction;
        end
        X = [flip(pre_padding_point(:,1)') X post_padding_point(:,1)'];
        Y = [flip(pre_padding_point(:,2)') Y post_padding_point(:,2)'];

        % Step 3: Create grid points perpendicular to the central line
        half_width = max(norm(goal_pose - start_pose)/4, 0.5);
    
        % Create grid points in the perpendicular direction
        X_full(:,:)=zeros(round(half_width/robot.grid_size),length(X));
        Y_full(:,:)=zeros(round(half_width/robot.grid_size),length(Y));
        X_full(1,:) = X;
        Y_full(1,:) = Y;
        for i = 1:round(half_width/robot.grid_size)
            X_full(i+1,:) = X_full(1,:) + i * robot.grid_size * perpendicular(1);
            Y_full(i+1,:) = Y_full(1,:) + i * robot.grid_size * perpendicular(2);
        end

        X_full2(:,:)=zeros(round(half_width/robot.grid_size),length(X));
        Y_full2(:,:)=zeros(round(half_width/robot.grid_size),length(Y));
        for i = 1:round(half_width/robot.grid_size)
            X_full2(i,:) = X - i * robot.grid_size * perpendicular(1);
            Y_full2(i,:) = Y - i * robot.grid_size * perpendicular(2);
        end
        X=[flip(X_full2(:,:),1); X_full];
        Y=[flip(Y_full2(:,:),1); Y_full];
        
        % Step 2: Find local map masked to the search space
        if ~isempty(map)
            % Define the boundaries of the grid
            min_x = min(X(:));
            max_x = max(X(:));
            min_y = min(Y(:));
            max_y = max(Y(:));
            
            % Create a logical mask for the points within the boundaries of X and Y
            inside_mask = (map(:,1) >= min_x & map(:,1) <= max_x) & ...
                          (map(:,2) >= min_y & map(:,2) <= max_y);
                      
            % Extract the local map based on the mask
            local_map = map(inside_mask, :);
        else
            local_map = [];
        end
    
        % Step 4: Initialize the open and closed lists
        open_list = [start_pose(1), start_pose(2), h_cost(start_pose, start_pose, goal_pose, local_map, robot.obstactle_decay_buffer), 0, h_cost(start_pose, start_pose, goal_pose, local_map, robot.obstactle_decay_buffer), nan, nan];
        closed_list = []; % Initialize an empty closed list
        path = [];
    
        %plot(neighbors(:,1), neighbors(:,2), 'color', 'cyan','marker','o','markerfacecolor','cyan')
        % Step 5: A* Search Loop
        while ~isempty(open_list)
            % Find the current node with the lowest f cost
            [~, idx_to_eval] = min(open_list(:,5)); 
            node_to_eval = open_list(idx_to_eval, :);
            
            % Check if the goal is reached
            if norm(node_to_eval(1:2) - goal_pose) < (robot.grid_size / 2)
                % Goal is reached, extract the path
                path = [];
                current_node = node_to_eval;
                
                % Construct the path by following parent links
                while ~isnan(current_node(6)) && ~isnan(current_node(7))
                    path = [current_node; path];
                    parent_idx = find(closed_list(:,1) == current_node(6) & closed_list(:,2) == current_node(7), 1);
                    current_node = closed_list(parent_idx, :);
                end
                if ~isempty(path)
                    path = [start_pose; path(:,1:2)];
                else
                    path = start_pose;
                end
                break;
            end
            
            % Update the open and closed lists
            open_list(idx_to_eval, :) = [];
            closed_list = [closed_list; node_to_eval]; 

            % Explore neighboring nodes and add them to the open list
            neighbors = [
                node_to_eval(1:2) + robot.grid_size * direction;  % Move in the direction of the path
                node_to_eval(1:2) - robot.grid_size * direction;  % Move in the opposite direction
                node_to_eval(1:2) + robot.grid_size * perpendicular;  % Move perpendicular to the path
                node_to_eval(1:2) - robot.grid_size * perpendicular  % Move in the opposite perpendicular direction
            ];

            
            % Define a small tolerance for floating-point comparisons
            tolerance = 1e-8;
            
            for i = 1:size(neighbors, 1)
                neighbor = neighbors(i, :);
    
                % Check if the neighbor is out of bounds using a custom comparison
                is_within_bounds = any(all(abs([X(:), Y(:)] - neighbor) < tolerance, 2));
    
                if is_within_bounds
                    % Calculate the costs for the neighbor
                    g_cost_neighbor = g_cost(node_to_eval(4), robot.grid_size);
                    h_cost_neighbor = h_cost(neighbor, start_pose, goal_pose, local_map, robot.obstactle_decay_buffer);
                    f_cost_neighbor = g_cost_neighbor + h_cost_neighbor;
                    
                    % Check if the neighbor is already in the closed list
                    in_closed_list = any(all(abs(closed_list(:,1:2) - neighbor) < tolerance, 2));
                    if in_closed_list
                        continue;
                    end
                    
                    % Check if the neighbor is already in the open list
                    in_open_list = any(all(abs(open_list(:,1:2) - neighbor) < tolerance, 2));
                    if in_open_list
                        existing_node_idx = find(all(abs(open_list(:,1:2) - neighbor) < tolerance, 2), 1);
                        if f_cost_neighbor < open_list(existing_node_idx, 5)
                            open_list(existing_node_idx, :) = [neighbor, h_cost_neighbor, g_cost_neighbor, f_cost_neighbor, node_to_eval(1:2)];
                        end
                    else
                        open_list = [open_list; [neighbor, h_cost_neighbor, g_cost_neighbor, f_cost_neighbor, node_to_eval(1:2)]];
                    end
                end
            end
        end
        
        if isempty(path)
            disp('No current path');
        end

    end

    % Helper Functions
    function h = h_cost(current_pose, start_pose, goal_pose, local_map, decay_buffer)
        distance_cost = norm(current_pose - goal_pose); 
        plant_cost = h_plant_cost(current_pose, local_map, decay_buffer); 
        row_cost = h_row_cost(current_pose, local_map);
        robot_turn_cost_at_start_cost = start_area_cost(current_pose, start_pose);
        h = distance_cost + plant_cost + row_cost+robot_turn_cost_at_start_cost;  
    end
    
    function cost = h_plant_cost(point, local_map, decay_buffer)
        cost = 0; 
        for i = 1:size(local_map, 1)
            dist_to_obstacle = norm(point - local_map(i, 1:2)); 
            if dist_to_obstacle<robot.grid_size+0.01
                cost=cost+70;
            else
                decay_factor = exp(decay_buffer * dist_to_obstacle) ;
                cost = cost + 70 * decay_factor ;
            end
        end
    end

    function cost = start_area_cost(current_pose, start_pose)
        heading=robot.measurement_heading;
        angle_to_point = atan2(current_pose(2) - start_pose(2), current_pose(1) - start_pose(1));
        relative_angle = wrapToPi(angle_to_point - heading);
        cup_angle_range = deg2rad(300);
        distance = sqrt((current_pose(1) - start_pose(1))^2 + (current_pose(2)- start_pose(2))^2);
        if abs(relative_angle) > (pi - cup_angle_range / 2) && distance < 0.5
            cost = 70;
        else
            cost = 0;
        end
    end

    function cost = h_row_cost(current_pose, local_map)
        cost = 0;  % Initialize the row cost
        if isempty(local_map)
            cost=0;
            return
        end
        % Get the unique row numbers
        unique_rows = unique(local_map(:, 3));
        
        for i = 1:length(unique_rows)
            % Get the indices of the current row
            row_idx = find(local_map(:, 3) == unique_rows(i));
            
            % Ensure there are at least two points in the row
            if length(row_idx) > 1
                % Iterate over each successive pair of points
                for j = 1:length(row_idx) - 1
                    plant1 = local_map(row_idx(j), 1:2);     % First plant coordinates
                    plant2 = local_map(row_idx(j + 1), 1:2); % Second plant coordinates
                    
                    % Generate points along the line connecting plant1 and plant2
                    num_points = 10; % Number of points along the line (can be adjusted)
                    line_points = [linspace(plant1(1), plant2(1), num_points)', linspace(plant1(2), plant2(2), num_points)'];
                    
                    % Elevate the cost along the line
                    for k = 1:num_points
                        point = line_points(k, :);
                        dist_to_line = norm(point - [current_pose(1), current_pose(2)]);
                        if dist_to_line<robot.grid_size
                            cost=cost+70;
                        else
                            cost=cost+0;
                        end
                    end
                end
            else
                cost=0;
            end
        end
    end

    function g = g_cost(parent_g_cost, grid_size)
        g = parent_g_cost + grid_size;
    end

end
% 
% function robot = timeseries_cleaning_buffer_robot(robot)
%     % Clean x_angle_h related histories
%     if length(sensor(i).x_angle_h) > robot.buffer_size
%         % Truncate histories to the most recent buffer_size entries
%         sensor(i).x_angle_h = sensor(i).x_angle_h(end-sensor(i).buffer_size+1:end);
%         sensor(i).xdot_angle_h = sensor(i).xdot_angle_h(end-sensor(i).buffer_size+1:end);
%         sensor(i).xddot_angle_h = sensor(i).xddot_angle_h(end-sensor(i).buffer_size+1:end);
%     end
% 
%     % Clean y_angle_h related histories
%     if length(sensor(i).y_angle_h) > robot.buffer_size
%         % Truncate histories to the most recent buffer_size entries
%         sensor(i).y_angle_h = sensor(i).y_angle_h(end-sensor(i).buffer_size+1:end);
%         sensor(i).ydot_angle_h = sensor(i).ydot_angle_h(end-sensor(i).buffer_size+1:end);
%         sensor(i).yddot_angle_h = sensor(i).yddot_angle_h(end-sensor(i).buffer_size+1:end);
%         sensor(i).f_y_angle_classifier_h = sensor(i).f_y_angle_classifier_h(end-sensor(i).buffer_size+1:end);
%         sensor(i).plant_localization_in_sensor_frame = sensor(i).plant_localization_in_sensor_frame(end-sensor(i).buffer_size+1:end);
%         sensor(i).plant_localization_in_robot_frame = sensor(i).plant_localization_in_robot_frame(end-sensor(i).buffer_size+1:end);
%         sensor(i).plant_localization_in_global_frame = sensor(i).plant_localization_in_global_frame(end-sensor(i).buffer_size+1:end);
%     end
% end

% Initialize the goals visualization
function [true_plot, global_plot, est_plot] = init_figure(robot, plants, sensor)
    figure("position",[0, 0, 2000, 1000]); % Increase the size of the entire plot
    
    % Global Plot
    subplot(1, 5, 1); % Make the global plot narrower
    hold on;
    global_plot.plants = plot([plants(:).x], [plants(:).y], 'Marker','o', 'MarkerSize', 5, 'LineStyle','none','MarkerEdgeColor',[0 0.6 0],'MarkerFaceColor',[0 0.6 0], 'DisplayName', 'Plants');
    
    global_plot.goal_points = gobjects(size(robot.goal_points, 1), 1); % Preallocate array for goal point handles
    global_plot.goal_arrows = gobjects(size(robot.goal_points, 1), 1); % Preallocate array for goal arrow handles
    global_plot.goal_texts = gobjects(size(robot.goal_points, 1), 1); % Preallocate array for goal text handles
    % Plot goal points, arrows, and texts
    for i = 1:size(robot.goal_points, 1)
        arrow_length = 2; % Length of the arrow
        arrow_x = [robot.goal_points(i,1), robot.goal_points(i,1) + arrow_length * cosd(robot.goal_points(i,3))];
        arrow_y = [robot.goal_points(i,2), robot.goal_points(i,2) + arrow_length * sind(robot.goal_points(i,3))];
        global_plot.goal_arrows(i) = plot(arrow_x, arrow_y, 'r-', 'LineWidth', 2, 'HandleVisibility', 'off');
        if i == 1
            global_plot.goal_points(i) = plot(robot.goal_points(i,1), robot.goal_points(i,2), 'ro', 'MarkerSize', 15, 'LineWidth', 2, 'MarkerFaceColor', 'w', 'DisplayName', 'Goal Points');
        else
            global_plot.goal_points(i) = plot(robot.goal_points(i,1), robot.goal_points(i,2), 'ro', 'MarkerSize', 15, 'LineWidth', 2, 'MarkerFaceColor', 'w', 'HandleVisibility', 'off');
        end
        global_plot.goal_texts(i) = text(robot.goal_points(i,1), robot.goal_points(i,2), num2str(i), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontWeight', 'bold', 'Color', 'k');
    end

    global_plot.robot_path = plot(robot.x_pos_h, robot.y_pos_h, 'b.-', 'DisplayName', 'True Robot Path');
    xlim([robot.x_pos_h(end)-2,3])
    ylim([0, 62])
    xlabel('X Coordinate (+x = east)');
    ylabel('Y Coordinate (+y = north)');
    title('Global Frame');
    axis equal
    legend Location southoutside
    set(gca, 'Color',[222/255,184/255,135/255])
    
    % True Robot State Plot
    subplot(1, 5, 2:3); % Make the true robot state plot larger
    hold on;
    %true_plot.robot_path = plot(robot.x_pos_h, robot.y_pos_h, 'b.-', 'DisplayName', 'True Robot Path');
    true_plot.robot = fill(nan, nan, 'cyan', 'DisplayName', 'True Robot'); % Initialize the robot rectangle
    true_plot.plants = plot([plants(:).x], [plants(:).y], 'Marker','o', 'MarkerSize', 5, 'LineStyle','none','MarkerEdgeColor',[0 0.6 0],'MarkerFaceColor',[0 0.6 0], 'DisplayName', 'Plants');
    true_plot.bracket1 = fill(nan, nan, 'cyan', 'Handlevisibility','off'); % Initialize the first bracket
    true_plot.bracket2 = fill(nan, nan, 'cyan', 'Handlevisibility','off'); % Initialize the second bracket
    for i = 1:4
        if i==1
            true_plot.sensor(i) = plot(nan, nan, 'black-', 'LineWidth', 3, 'DisplayName', 'Feeler Sensors'); % Initialize the sensor line
        else
            true_plot.sensor(i) = plot(nan, nan, 'black-', 'LineWidth', 3, 'HandleVisibility','off'); % Initialize the sensor line            
        end
    end
    xlabel('X Coordinate (+x = east)');
    ylabel('Y Coordinate (+y = north)');
    title('True Robot State');
    axis equal
    legend Location southwest
    set(gca, 'Color',[222/255,184/255,135/255])
    
    % Estimated Robot State Plot
    subplot(1, 5, 4:5); % Make the estimated robot state plot the same size as the true robot state plot
    hold on;
    est_plot.robot_path = plot(robot.measurement_x_h, robot.measurement_y_h, '.-', 'DisplayName', 'Estimated Robot Path','Color', [0.1 0.1 0.8]);
    est_plot.goal_points = plot(robot.goal_points(:,1), robot.goal_points(:,2), 'ro-', 'MarkerSize', 5, 'LineWidth', 2, 'DisplayName', 'Goal Points','LineStyle','--');
    %est_plot.plants = plot([plants(:).x], [plants(:).y], 'x', 'MarkerSize', 7, 'LineStyle','none', 'Color', [0.6, 0.8, 0.5], 'DisplayName', 'True Plant Locations'); % Light green 'x' markers
    est_plot.turning_arc = plot(nan, nan, 'Color', [1,230/255,0.2], 'Linewidth',2, 'DisplayName', 'Current Arc');
    est_plot.planned_path = plot(nan, nan, 'Color', [1, 0.5, 0], 'LineStyle', '--', 'LineWidth', 2, ...
        'Marker', '^', 'MarkerSize', 5, 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', 'w', ...
        'DisplayName', 'Planned Path');
    est_plot.lookahead = plot(nan, nan, 'mo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Lookahead Point');
    est_plot.robot = fill(nan, nan, 'cyan', 'FaceAlpha', 0.5, 'DisplayName', 'Estimated Robot'); % Initialize the robot rectangle with transparency
    est_plot.bracket1 = fill(nan, nan, 'cyan-', 'FaceAlpha', 0.5, 'Handlevisibility','off'); % Initialize the first bracket
    est_plot.bracket2 = fill(nan, nan, 'cyan-', 'FaceAlpha', 0.5, 'Handlevisibility','off'); % Initialize the second bracket
    est_plot.heading = quiver(robot.measurement_x, robot.measurement_y, cos(robot.measurement_heading), sin(robot.measurement_heading), 0.4, 'Color', [0.7 0 0.7], 'LineWidth', 2, 'DisplayName', 'Current Heading','MaxHeadSize',5);
    for i = 1:4
        if i==1
            est_plot.sensor_paddle(i) = plot(nan, nan, 'k-', 'LineWidth', 2, 'DisplayName', 'Estimated Paddle Location'); % Measured sensor paddle line
            %est_plot.contact_point_g(i) = plot(nan, nan, "Marker", "o", "Color",[0 0.3 0], 'MarkerSize', 10, 'DisplayName', 'Estimated Contact Point');
        else
            est_plot.sensor_paddle(i) = plot(nan, nan, 'k-', 'LineWidth', 2, 'HandleVisibility','off'); % Measured sensor paddle line
            %est_plot.contact_point_g(i) = plot(nan, nan, "Marker", "o", "Color",[0 0.3 0], 'MarkerSize', 10, 'HandleVisibility','off'); 
        end
    end
    est_plot.global_map = plot(nan, nan, '.', 'MarkerSize', 20, 'LineStyle','none', 'Color', [0 0.6 0], 'DisplayName', 'Mapped Plants'); % Light green 'x' markers

    xlabel('X Coordinate (+x = east)');
    ylabel('Y Coordinate (+y = north)');
    title('Estimated Robot State');
    axis equal
    legend Location southwest
    %set(gca, 'Color',[222/255,184/255,135/255])

    % LOCALIZATION
    % subplot(1, 7, 6:7);
    % hold on;
    % robot_corners = [
    %     -robot.Lr/2, -robot.Wr/2;
    %     -robot.Lr/2, robot.Wr/2;
    %     robot.Lr/2, robot.Wr/2;
    %     robot.Lr/2, -robot.Wr/2;
    %     -robot.Lr/2, -robot.Wr/2;
    % ];  
    % est_plot.robotframe = fill(robot_corners(:,1), robot_corners(:,2), 'cyan', 'FaceAlpha', 0.5, 'DisplayName', 'Estimated Robot'); % Initialize the robot rectangle with transparency
    % for i = 1:4
    %     if i==1
    %         est_plot.D(i) = plot(nan, nan, 'g-', 'LineWidth', 2, 'DisplayName', 'D_{hat}');
    %         est_plot.H(i) = plot(nan, nan, 'r-', 'LineWidth', 2, 'DisplayName', 'H_{hat}');
    %         est_plot.P(i) = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'P_{hat}');
    %         est_plot.pin_point(i) = plot(nan, nan, 'ko', 'MarkerSize', 10, 'DisplayName', sprintf('Sensor Pin Location %d', i));
    %         est_plot.contact_point(i) = plot(nan, nan, 'kx', 'MarkerSize', 10, 'DisplayName', sprintf('Sensor Contact Point %d', i));
    %     else
    %         est_plot.D(i) = plot(nan, nan, 'g-', 'LineWidth', 2,'HandleVisibility','off');
    %         est_plot.H(i) = plot(nan, nan, 'r-', 'LineWidth', 2,'HandleVisibility','off');
    %         est_plot.P(i) = plot(nan, nan, 'b-', 'LineWidth', 2,'HandleVisibility','off');
    %         est_plot.pin_point(i) = plot(nan, nan, 'ko', 'MarkerSize', 10,'HandleVisibility','off');
    %         est_plot.contact_point(i) = plot(nan, nan, 'kx', 'MarkerSize', 10,'HandleVisibility','off');
    %     end
    % end
    % xlim([-0.4, 0.9]);
    % ylim([-0.5, 0.5]);
    % axis equal
    % title('Localization in Robot Frame');
    % ylabel('Robot y axis')
    % xlabel('Robot x axis')
    % legend Location southoutside
end

function est_plot=update_plot(true_plot, global_plot, est_plot, robot, sensor)

    % GLOBAL PLOT UPDATES
    set(global_plot.robot_path, 'XData', robot.x_pos_h, 'YData', robot.y_pos_h);

    % TRUE ROBOT PLOT UPDATE
    %set(true_plot.robot_path, 'XData', robot.x_pos_h, 'YData', robot.y_pos_h);
    xlim(true_plot.robot.Parent, [robot.x - 1.2, robot.x + 1.2]);
    ylim(true_plot.robot.Parent, [robot.y - 1.2, robot.y + 1.2]);
    update_robot_area(true_plot.robot, true_plot.bracket1, true_plot.bracket2, robot.x, robot.y, robot.heading, robot.Lr, robot.Wr, sensor);

    % Update sensor plots
    for i = 1:2
        set(true_plot.sensor(i), 'XData', [sensor(i).global_pin_location(1), sensor(i).paddle_end_x], 'YData', [sensor(i).global_pin_location(2), sensor(i).paddle_end_y]);
    end

    % ESTIMATED ROBOT PLOT UPDATES
    set(est_plot.robot_path, 'XData', robot.measurement_x_h, 'YData', robot.measurement_y_h);
    update_robot_area(est_plot.robot, est_plot.bracket1, est_plot.bracket2, robot.measurement_x, robot.measurement_y, robot.measurement_heading, robot.Lr, robot.Wr, sensor); % Ensure this line is present
    set(est_plot.heading, 'XData', robot.measurement_x, 'YData', robot.measurement_y, 'UData', cos(robot.measurement_heading), 'VData', sin(robot.measurement_heading));

    if isfield(robot, 'turn_rad')
        theta = linspace(0, 2*pi, 100);
        arc_x = robot.turn_center_x + robot.turn_rad * cos(theta);
        arc_y = robot.turn_center_y + robot.turn_rad * sin(theta);
        set(est_plot.turning_arc, 'XData', arc_x, 'YData', arc_y);
        set(est_plot.lookahead, 'XData', robot.Lx, 'YData', robot.Ly);
    end

    %our map of the environment
    if ~isempty(robot.global_map) 
        set(est_plot.global_map, 'XData', robot.global_map(:,1), 'YData', robot.global_map(:,2));
    end

    % the planned path plot
    if isfield(robot, 'planned_path') && ~isempty(robot.planned_path)
        set(est_plot.planned_path, 'XData', robot.planned_path(:,1), 'YData', robot.planned_path(:,2));
    else
        set(est_plot.planned_path, 'XData', nan, 'YData', nan); % If no planned path, clear the plot
    end

    % Update sensor plots
    for i = 1:2
        set(est_plot.sensor_paddle(i), 'XData', [sensor(i).estimated_global_pin_location(1), sensor(i).estimated_paddle_end_x], 'YData', [sensor(i).estimated_global_pin_location(2), sensor(i).estimated_paddle_end_y]);
    end
    xlim(est_plot.robot_path.Parent, [robot.measurement_x - 3, robot.measurement_x + 3]);
    ylim(est_plot.robot_path.Parent, [robot.measurement_y - 3, robot.measurement_y + 3]);
    
    % LOCALIZATION PLOT UPDATES
    % for i = 1:4
    %     if (sensor(i).f_y_angle_classifier_h(end)=="In Contact" || sensor(i).f_y_angle_classifier_h(end)=="Nearing Crash") && length(sensor(i).y_angle_h) > 1
    %         % D, H, and P components in robot frame
    %         D_x = [sensor(i).pin_location(1), sensor(i).pin_location(1)];
    %         D_y = [sensor(i).pin_location(2), sensor(i).plant_localization_in_robot_frame{end}(2)];
    %         H_x = [sensor(i).pin_location(1), sensor(i).plant_localization_in_robot_frame{end}(1)];
    %         H_y = [sensor(i).pin_location(2), sensor(i).plant_localization_in_robot_frame{end}(2)];
    %         P_x = [sensor(i).pin_location(1), sensor(i).plant_localization_in_robot_frame{end}(1)];
    %         P_y = [sensor(i).plant_localization_in_robot_frame{end}(2), sensor(i).plant_localization_in_robot_frame{end}(2)];
    %         set(est_plot.D(i), 'XData', D_x, 'YData', D_y);
    %         set(est_plot.H(i), 'XData', H_x, 'YData', H_y);
    %         set(est_plot.P(i), 'XData', P_x, 'YData', P_y);
    %         set(est_plot.pin_point(i), 'XData', sensor(i).pin_location(1), 'YData', sensor(i).pin_location(2));
    %         set(est_plot.contact_point(i), 'XData', sensor(i).plant_localization_in_robot_frame{end}(1), 'YData', sensor(i).plant_localization_in_robot_frame{end}(2));
    %         set(est_plot.contact_point_g(i), 'XData', sensor(i).plant_localization_in_global_frame{end}(1), 'YData', sensor(i).plant_localization_in_global_frame{end}(2));
    %     else
    %         set(est_plot.D(i), 'XData', nan, 'YData', nan);
    %         set(est_plot.H(i), 'XData', nan, 'YData', nan);
    %         set(est_plot.P(i), 'XData', nan, 'YData', nan);
    %         set(est_plot.pin_point(i), 'XData', sensor(i).pin_location(1), 'YData', sensor(i).pin_location(2));
    %         set(est_plot.contact_point(i), 'XData', nan, 'YData', nan);
    %         set(est_plot.contact_point_g(i), 'XData', nan, 'YData', nan);
    %     end
    % end
    % xlim(est_plot.contact_point(1).Parent, [-0.4, 0.9]);
    % ylim(est_plot.contact_point(1).Parent, [-0.5, 0.5]);
    % 
    drawnow;
end


function update_robot_area(robot_patch, bracket1_patch, bracket2_patch, x, y, heading, Lr, Wr, sensor)
    % Update the robot rectangle
    robot_corners = [
        -Lr/2, -Wr/2;
        -Lr/2, Wr/2;
        Lr/2, Wr/2;
        Lr/2, -Wr/2;
        -Lr/2, -Wr/2;
    ];

    R = [cos(heading), -sin(heading); sin(heading), cos(heading)];
    rotated_corners = (R * robot_corners')';
    robot_x = rotated_corners(:,1) + x;
    robot_y = rotated_corners(:,2) + y;
    set(robot_patch, 'XData', robot_x, 'YData', robot_y);
    
    bracket1_corners=[
        sensor(1).pin_location(1)-0.015, sensor(1).pin_location(2);
        sensor(1).pin_location(1)+0.015, sensor(1).pin_location(2);
        sensor(2).pin_location(1)+0.015, sensor(2).pin_location(2);
        sensor(2).pin_location(1)-0.015, sensor(2).pin_location(2);
        sensor(1).pin_location(1)-0.015, sensor(1).pin_location(2);
        ];
    bracket2_corners=[
        Lr/2, -0.015
        Lr/2, 0.015
        sensor(1).pin_location(1), 0.015
        sensor(1).pin_location(1), -0.015
        Lr/2, -0.015
        ];

    bracket1= (R * bracket1_corners')';
    bracket2= (R * bracket2_corners')';
    b1_x = bracket1(:,1) + x;
    b1_y = bracket1(:,2) + y;
    b2_x = bracket2(:,1) + x;
    b2_y = bracket2(:,2) + y;
    set(bracket1_patch, 'XData', b1_x, 'YData', b1_y);
    set(bracket2_patch, 'XData', b2_x, 'YData', b2_y);
end


%------------------------------------------------------------------------
function robot = update_robot_position(robot, sim)
    % Update robot position and heading based on wheel speeds
    v_l = -robot.right_speed_ref_h(end) * sim.del_t; % Right wheel displacement (negative for forward motion)
    v_r = -robot.left_speed_ref_h(end) * sim.del_t; % Left wheel displacement (negative for forward motion)
    v_avg = (v_r + v_l) / 2; % Average displacement

    % Calculate the new heading
    new_heading = robot.heading + (v_r - v_l) / robot.track;

    % Add error
    new_heading = new_heading + sim.del_t * (pi / 64) * randn();

    % Update the position based on the new heading with some error
    robot.x = robot.x + v_avg * cos(new_heading) + sim.del_t * (randn() / 75);
    robot.y = robot.y + v_avg * sin(new_heading) + sim.del_t * (randn() / 75);
    robot.heading = new_heading;

    % Store true state
    robot.x_pos_h(end + 1) = robot.x;  
    robot.y_pos_h(end + 1) = robot.y;  
    robot.h_pos_h(end + 1) = robot.heading;
end

function [robot, sensor]=robot_measurement(robot, sensor)
    % Take measurements
    robot.measurement_x = robot.x+randn()/1000; % Measured x position (global frame)
    robot.measurement_y = robot.y+randn()/1000; % Measured y position (global frame)
    robot.measurement_heading = robot.heading+randn()/10000; % Measured heading (global frame)

    % Update measurement histories
    robot.measurement_x_h(end + 1) = robot.measurement_x; %(global frame)
    robot.measurement_y_h(end + 1) = robot.measurement_y; %(global frame)
    robot.measurement_h_h(end + 1) = robot.measurement_heading; %(global frame)

    %Update estimated pin locations of our sensors
    R_measurement = [cos(robot.measurement_heading), -sin(robot.measurement_heading);
             sin(robot.measurement_heading), cos(robot.measurement_heading)]; % Rotation from robot frame to global frame
    for i = 1:2
        sensor(i).estimated_global_pin_location = [robot.measurement_x; robot.measurement_y] + R_measurement * [sensor(i).pin_location(1:2)]';
    end

    % Update measurement time
    robot.measurement_time = robot.measurement_time + robot.del_t;
end

function sensor = update_sensors(sensor, plants, robot)
    for i=1:2
        sensor(i).x_time = sensor(i).x_time + sensor(i).x_del_t; % This is the sensor simulation time

        sensor(i) = check_for_contact(sensor(i), plants, robot);
        sensor(i) = sensor_dynamics(sensor(i), robot);      % Update the sensor's pin location and paddle end position
    
    end
end

function sensor = measure_sensors(sensor, robot)
    for i=1:2
        % Measurement part of the sensor
        sensor(i).y_time = sensor(i).x_time;
        sensor(i).y_angle_h(end+1) = sensor(i).x_angle_h(end)+(2*randn()-1)/100;  % Measurement angle with a bit of noise (sensor frame)
        sensor(i).estimated_global_angle = sensor(i).mounting_angle + deg2rad(sensor(i).y_angle_h(end));  % Calculate paddle end position
        sensor(i).estimated_paddle_end_x = sensor(i).estimated_global_pin_location(1) + sensor(i).paddle_length * cos(sensor(i).estimated_global_angle + robot.measurement_heading);
        sensor(i).estimated_paddle_end_y = sensor(i).estimated_global_pin_location(2) + sensor(i).paddle_length * sin(sensor(i).estimated_global_angle + robot.measurement_heading);

        % Estimate first and second derivatives
        if length(sensor(i).y_angle_h) > 1
            sensor(i).ydot_angle_h(end+1) = (sensor(i).y_angle_h(end) - sensor(i).y_angle_h(end-1)) / sensor(i).y_del_t;
            if length(sensor(i).y_angle_h) > 2
                sensor(i).yddot_angle_h(end+1) = (sensor(i).ydot_angle_h(end) - sensor(i).ydot_angle_h(end - 1)) / sensor(i).y_del_t;
            else
                sensor(i).yddot_angle_h(end+1) = 0;
            end
        else
            sensor(i).ydot_angle_h(end+1) = 0;
            sensor(i).yddot_angle_h(end+1) = 0;
        end
    end
end

function [sensor, robot] = contact_mapping(sensor, robot)
    for i=1:2
        % Classify the contact state from measurement
        sensor(i) = classify_state(sensor(i));

        % Per Timestep Localization
        sensor(i) = estimate_values(sensor(i), robot);
        sensor(i) = check_H_hat(sensor(i)); % Call the new function after localization
    end

    % Estimated Localization, Mapping
    for i = 1:2
        % if (sensor(i).f_y_angle_classifier_h(end) ~= "In Contact" || sensor(i).f_y_angle_classifier_h(end) ~= "Nearing Crash") && (sensor(i).f_y_angle_classifier_h(end-1) == "In Contact" || sensor(i).f_y_angle_classifier_h(end-1) == "Nearing Crash")
        %     [sensor(i), robot] = add_to_global_map(sensor(i), robot);
        %     robot.replan_path=true;
        % end
        if (sensor(i).f_y_angle_classifier_h(end) == "In Contact" || sensor(i).f_y_angle_classifier_h(end) == "Nearing Crash")
            [sensor(i), robot] = add_to_global_map(sensor(i), i, robot);
            %robot.replan_path=true;
        end
    end
end

% SENSOR FUNCTIONS
function sensor = check_for_contact(sensor, plants, robot) %none of this is measurement stuff, all "physical"

    % Initialize the maximum angle difference
    max_angle_diff = 0;  %(sensor frame)
    sensor.contact_angle = 0; %(sensor frame)
    sensor.in_contact = false;

    % Calculate the global to sensor frame transformation matrix
    R = [cos(robot.heading + sensor.mounting_angle), -sin(robot.heading + sensor.mounting_angle);
         sin(robot.heading + sensor.mounting_angle), cos(robot.heading + sensor.mounting_angle)];

    for i = 1:length(plants)
        % Transform plant coordinates to sensor frame
        plant_global = [plants(i).x; plants(i).y];
        plant_sensor = R' * (plant_global - sensor.global_pin_location);

        dx = plant_sensor(1); % distance in sensor frame x
        dy = plant_sensor(2); % distance in sensor frame y

        if sqrt(dx^2 + dy^2) <= sensor.paddle_length % Check if the point is within paddle length
            
            % Convert span to radians
            span_rad = deg2rad(sensor.span); % the angles we use the sensor in (sensor frame)
            span_start = span_rad(1);
            span_end = span_rad(2);

            % Calculate the angle to the point in the sensor frame
            angle_to_point = atan2(dy, dx);

            % Check if the angle is within the active range
            if (span_start < span_end && angle_to_point >= span_start && angle_to_point <= span_end) || ...
               (span_start >= span_end && (angle_to_point >= span_start || angle_to_point <= span_end))
                
                % Check proximity along the paddle
                num_points = 1000;  
                t = linspace(0, 1, num_points);
                for j = 1:num_points
                    paddle_x = sensor.global_pin_location(1) + t(j) * (sensor.paddle_end_x - sensor.global_pin_location(1));
                    paddle_y = sensor.global_pin_location(2) + t(j) * (sensor.paddle_end_y - sensor.global_pin_location(2));

                    if sqrt((paddle_x - plants(i).x)^2 + (paddle_y - plants(i).y)^2) < 0.0075 % Proximity check (global cords)
                        %angle_to_point = atan2(plants(i).y - sensor.global_pin_location(2), plants(i).x - sensor.global_pin_location(1));
                        angle_diff = abs(rad2deg(angle_to_point - sensor.home_angle)); 

                        if angle_diff > max_angle_diff
                            max_angle_diff = angle_diff;
                            sensor.contact_angle = rad2deg(angle_to_point - sensor.home_angle); 
                        end
                        sensor.in_contact = true;
                        sensor.is_sticky = false;  
                        sensor.contact_location = [plants(i).x, plants(i).y]; 
                        break
                    end
                end
            end
        end
    end

    if ~sensor.in_contact
        sensor.contact_location = [];
    end    
end

% sensor_dynamics Function
function sensor = sensor_dynamics(sensor, robot)
    % Logging the physical state
    R = [cos(robot.heading), -sin(robot.heading); sin(robot.heading), cos(robot.heading)];
    sensor.global_pin_location = [robot.x; robot.y] + R * [sensor.pin_location(1:2)]';  % Transform the sensor's pin location to the global frame
    sensor.true_global_angle = sensor.mounting_angle + deg2rad(sensor.x_angle_h(end));  % Calculate paddle end position
    sensor.paddle_end_x = sensor.global_pin_location(1) + sensor.paddle_length * cos(sensor.true_global_angle + robot.heading);
    sensor.paddle_end_y = sensor.global_pin_location(2) + sensor.paddle_length * sin(sensor.true_global_angle + robot.heading);

    if sensor.in_contact == true
        sensor.x_angle_h(end+1) = sensor.contact_angle;
        sensor.xdot_angle_h(end+1) = (sensor.x_angle_h(end) - sensor.x_angle_h(end - 1)) / sensor.x_del_t;  % Store angular velocity history

    elseif sensor.in_contact == false
        sensor = recenter_sensor(sensor);
    end

    if length(sensor.x_angle_h) > 2
        sensor.xddot_angle_h(end+1) = (sensor.xdot_angle_h(end) - sensor.xdot_angle_h(end - 1)) / sensor.x_del_t;  % Store angular accel history
    else
        sensor.xddot_angle_h(end+1) = 0; % Ensure xddot_angle_h is initialized
    end
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
    if sensor.x_angle_h(end) >= rad2deg(sensor.sticky_region(1)) && sensor.x_angle_h(end) <= rad2deg(sensor.sticky_region(2))
        if rand() < sensor.sticky_friction_probability
            % Stick to the current angle
            sensor.is_sticky = true;  % Enter sticky state
        end
    end

    if sensor.is_sticky == true
        % Update such that it sticks
        sensor.xdot_angle_h(end+1) = 0;  % Stop the velocity
        sensor.x_angle_h(end+1) = sensor.x_angle_h(end);
        return;  % Exit the function to keep the current angle
    end
    
    % Calculate the spring force and damping
    angle_diff = deg2rad(sensor.x_angle_h(end)) - deg2rad(sensor.home_angle);
    spring_force = -sensor.spring_constant * angle_diff / I;
    damping_force = -sensor.damping * deg2rad(sensor.xdot_angle_h(end)) / (sensor.x_del_t * I);
    total_force = spring_force + damping_force;

    % Update the local angle based on the forces
    sensor.xdot_angle_h(end+1) = sensor.xdot_angle_h(end) + rad2deg(total_force * sensor.x_del_t);
    sensor.x_angle_h(end+1) = sensor.x_angle_h(end) + sensor.xdot_angle_h(end) * sensor.x_del_t;
end

% classify_state Function
function sensor = classify_state(sensor)
    % Adjust values for left-side sensors
    if sensor.mounting_angle == deg2rad(90)
        y_angle_h = -sensor.y_angle_h;
        ydot_angle_h = -sensor.ydot_angle_h;
        yddot_angle_h = -sensor.yddot_angle_h;
    else
        y_angle_h = sensor.y_angle_h;
        ydot_angle_h = sensor.ydot_angle_h;
        yddot_angle_h = sensor.yddot_angle_h;
    end

    i = length(sensor.y_angle_h);
    if i <= sensor.classifier_window
        sensor.current_state = "Unknown State";
    elseif i > sensor.classifier_window      
        % Determine state based on conditions
        switch sensor.current_state
            case "Unknown State"
                if all(abs(ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -30000
                    sensor.current_state = "In Contact";
                elseif all(ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                elseif all(ydot_angle_h(i-sensor.classifier_window:i) < 0) && all(y_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Decreasing";
                end
    
            case "Steady State Angle"
                if ydot_angle_h(i) < 4 && yddot_angle_h(i) < -1000
                    sensor.current_state = "In Contact";
                elseif all(ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                end

           case "SHM Increasing"
                if all(abs(ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif y_angle_h(i) > 5 && abs(ydot_angle_h(i)) < 100 && yddot_angle_h(i) < 0
                    sensor.current_state= "SHM Decreasing";
                elseif ydot_angle_h(i) < -20 && yddot_angle_h(i) > 0
                    sensor.current_state = "In Contact";
                end
    
            case "SHM Decreasing"
                if all(abs(ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif ydot_angle_h(i) < 0 && yddot_angle_h(i) < -20000
                    sensor.current_state = "In Contact";
                elseif y_angle_h(i) < 0 && abs(ydot_angle_h(i)) < 100 && yddot_angle_h(i) > 0
                    sensor.current_state= "SHM Increasing";
                end
                
            case "In Contact"
                if all(ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                %elseif all(abs(ydot_angle_h(i-sensor.classifier_window:i)) < 20) ...
                %        && all(abs(yddot_angle_h(i-sensor.classifier_window:i)) < 1000)
                %        sensor.current_state= "Steady State Angle";
                elseif y_angle_h(i)<-75
                        sensor.current_state= "Nearing Crash";
                end

            case "Nearing Crash"
                %if sensor.ydot_angle_h(i) < 0 && sensor.y_angle_h(i) < 0
                %    sensor.f_y_angle_classifier_h(i)h(i) = "In Contact";
                if all(ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                %elseif all(abs(ydot_angle_h(i-sensor.classifier_window:i)) < 8) ...
                %        && all(abs(yddot_angle_h(i-sensor.classifier_window:i)) < 1000)
                %    sensor.current_state= "Steady State Angle";
                end
                
            otherwise
                sensor.current_state= "Unknown State";
        end
    end

    sensor.f_y_angle_classifier_h(i) = sensor.current_state;

    return
end


%LOCALIZATION
function sensor = estimate_values(sensor, robot)
    % Function to estimate D, P, and H during contacts
    % Continuous time equations:
    % D = constant perpendicular distance from the pin location to the point of contact
    % P(t) = D * tan(theta(t)) = travel distance along the paddle
    % H(t) = D / cos(theta(t)) = hypotenuse of the triangle formed by the paddle
    % Change in P(t) = point.speed * sensor.del_t

    % Discrete time equations:
    % theta_k = angle at time step k
    % theta_k-1 = angle at time step k-1
    % Change in travel distance: delta P = point.speed * sensor.del_t
    % Estimation of D:
    % D = point.speed * sensor.del_t / (tan(theta_k) - tan(theta_k-1))
    % Estimation of P and H:
    % P_hat_k = D_hat * tan(theta_k)
    % H_hat_k = D_hat / cos(theta_k)

    if (sensor.f_y_angle_classifier_h(end) == "In Contact" || sensor.f_y_angle_classifier_h(end) == "Nearing Crash") && length(sensor.y_angle_h) > 1
        theta_k = deg2rad(sensor.y_angle_h(end));
        theta_k_minus_1 = deg2rad(sensor.y_angle_h(end - 1));
        cos_theta_k = cos(theta_k);
        cos_theta_k_minus_1 = cos(theta_k_minus_1);
        sin_theta_diff = sin(theta_k - theta_k_minus_1);
    
        if sin_theta_diff ~= 0
            % Calculate the angular velocity of the robot
            omega = (robot.right_speed_ref_h(end) - robot.left_speed_ref_h(end)) / robot.track;

            % Position of the sensor relative to the robot's center
            d_x = sensor.pin_location(1);
            d_y = sensor.pin_location(2);

            % Calculate the velocity of the sensor in the robot frame
            v_robot = [robot.v_ref; 0]; % Robot's linear velocity in the robot frame
            omega_cross_d = [-omega * d_y; omega * d_x]; % Rotational component
            
            v_sensor = v_robot + omega_cross_d; % Total velocity at the sensor
            v_sensor_x = v_sensor(1); % X-component of the velocity
            %v_sensor_y = v_sensor(2);
            % Use v_sensor_x as the speed estimate for localization
            speed_estimate = -v_sensor_x;
            
            % Estimation of D, P, and H using the updated speed estimate
            D_hat = (speed_estimate * sensor.y_del_t * cos_theta_k * cos_theta_k_minus_1) / abs(sin_theta_diff);
            P_hat_k = -D_hat * tan(theta_k);
            H_hat_k = D_hat / cos(theta_k);

            % Store estimates
            sensor.D_hat = D_hat;
            sensor.P_hat = P_hat_k;
            sensor.H_hat = H_hat_k;

            sensor.plant_localization_in_sensor_frame{end+1} = [sensor.D_hat; sensor.P_hat; 1];

            % Transformation matrix T from sensor to robot
            T = [cos(sensor.mounting_angle), -sin(sensor.mounting_angle), sensor.pin_location(1);
                 sin(sensor.mounting_angle), cos(sensor.mounting_angle), sensor.pin_location(2);
                 0, 0, 1];
            sensor.plant_localization_in_robot_frame{end+1} = T * sensor.plant_localization_in_sensor_frame{end};
            
            % Transformation plant location from robot frame to global
            % frame using an extrapolated robot position
            %timesteps_to_estimate=(sensor.y_time-robot.measurement_time)/sensor.y_del_t;
            estimated_x = robot.measurement_x; % + robot.v_ref * cos(robot.measurement_heading) * timesteps_to_estimate* sensor.y_del_t;
            estimated_y = robot.measurement_y; % + robot.v_ref * sin(robot.measurement_heading) * timesteps_to_estimate * sensor.y_del_t;
            
            
            T2 = [cos(robot.measurement_heading), -sin(robot.measurement_heading), estimated_x;
                    sin(robot.measurement_heading), cos(robot.measurement_heading), estimated_y;
                    0, 0, 1];
            sensor.plant_localization_in_global_frame{end+1} = T2 * sensor.plant_localization_in_robot_frame{end};

        else
            % Store NaN when the estimation cannot be performed
            sensor.D_hat = NaN;
            sensor.P_hat = NaN;
            sensor.H_hat = NaN;
            sensor.plant_localization_in_sensor_frame{end+1} = NaN;
            sensor.plant_localization_in_robot_frame{end+1} = NaN;
            sensor.plant_localization_in_global_frame{end+1} = NaN;

        end
    else
        % Store NaN when not in contact
        sensor.D_hat = NaN;
        sensor.P_hat = NaN;
        sensor.H_hat = NaN;
        sensor.plant_localization_in_sensor_frame{end+1} = NaN;
        sensor.plant_localization_in_robot_frame{end+1} = NaN;
        sensor.plant_localization_in_global_frame{end+1} = NaN;
    end
end

% New function to check H_hat
function sensor = check_H_hat(sensor)
    if sensor.H_hat > sensor.paddle_length + 0.02
        sensor.current_state = "SHM Increasing";
        sensor.f_y_angle_classifier_h(end) = "SHM Increasing";
        sensor.D_hat = NaN;
        sensor.P_hat = NaN;
        sensor.H_hat = NaN;
        sensor.plant_localization_in_sensor_frame{end} = NaN;
        sensor.plant_localization_in_robot_frame{end} = NaN;
        sensor.plant_localization_in_global_frame{end} = NaN;
    end
end

function sensor = timeseries_cleaning_buffer(sensor)
    for i = 1:2
        % Clean x_angle_h related histories
        if length(sensor(i).x_angle_h) > sensor(i).buffer_size
            % Truncate histories to the most recent buffer_size entries
            sensor(i).x_angle_h = sensor(i).x_angle_h(end-sensor(i).buffer_size+1:end);
            sensor(i).xdot_angle_h = sensor(i).xdot_angle_h(end-sensor(i).buffer_size+1:end);
            sensor(i).xddot_angle_h = sensor(i).xddot_angle_h(end-sensor(i).buffer_size+1:end);
        end

        % Clean y_angle_h related histories
        if length(sensor(i).y_angle_h) > sensor(i).buffer_size
            % Truncate histories to the most recent buffer_size entries
            sensor(i).y_angle_h = sensor(i).y_angle_h(end-sensor(i).buffer_size+1:end);
            sensor(i).ydot_angle_h = sensor(i).ydot_angle_h(end-sensor(i).buffer_size+1:end);
            sensor(i).yddot_angle_h = sensor(i).yddot_angle_h(end-sensor(i).buffer_size+1:end);
            sensor(i).f_y_angle_classifier_h = sensor(i).f_y_angle_classifier_h(end-sensor(i).buffer_size+1:end);
            sensor(i).plant_localization_in_sensor_frame = sensor(i).plant_localization_in_sensor_frame(end-sensor(i).buffer_size+1:end);
            sensor(i).plant_localization_in_robot_frame = sensor(i).plant_localization_in_robot_frame(end-sensor(i).buffer_size+1:end);
            sensor(i).plant_localization_in_global_frame = sensor(i).plant_localization_in_global_frame(end-sensor(i).buffer_size+1:end);
        end
    end
end

function [sensor, robot] = add_to_global_map(sensor, sensor_index, robot)
    % Parameters
    row_threshold = 1;  % Threshold distance to consider the same row, in meters
    
    % Sensor tag based on the provided sensor index
    sensor_tag = sensor_index;

    % Find the start and end indices of the "In Contact" period
    contact_states = ["In Contact", "Nearing Crash"];
    start_idx = find(diff(ismember(sensor.f_y_angle_classifier_h, contact_states)) == 1, 1, 'last') + 1;
    end_idx = length(sensor.f_y_angle_classifier_h);
    
    if end_idx - start_idx < 4
        return
    end

    if ~isempty(start_idx) && ~isempty(end_idx) && start_idx < end_idx
        % Extract the localization data during the contact period
        valid_localizations = sensor.plant_localization_in_global_frame{start_idx:end_idx-1};
        current_point = [mean(valid_localizations(1, :)); mean(valid_localizations(2, :))];

        % Check if there is another point within 0.13 meters
        if ~isempty(robot.global_map)
            distances = sqrt(sum((robot.global_map(:, 1:2) - current_point').^2, 2));
            close_point_idx = find(distances < 0.13, 1);
        else
            close_point_idx = [];
        end

        if isempty(close_point_idx)
            % Determine the row number based on proximity to existing rows
            if isempty(robot.global_map)
                row_number = 1;
            else
                % Compute Euclidean distances from current point to all points in the global map
                euclidean_distances = sqrt(sum((robot.global_map(:, 1:2) - current_point').^2, 2));
                [min_distance, min_idx] = min(euclidean_distances);

                if min_distance < row_threshold && robot.global_map(min_idx, 4) == sensor_tag
                    row_number = robot.global_map(min_idx, 3);  % Assign to existing row
                else
                    row_number = max(robot.global_map(:, 3)) + 1;  % Create a new row
                end
            end

            % If no close point is found, add the current point to the global map
            robot.global_map(end+1, :) = [current_point', row_number, sensor_tag];
        else
            % If a close point is found, replace it with the average location
            robot.global_map(close_point_idx, 1:2) = (robot.global_map(close_point_idx, 1:2) + current_point') / 2;
            robot.global_map(close_point_idx, 3) = robot.global_map(close_point_idx, 3);  % Maintain the existing row number
            robot.global_map(close_point_idx, 4) = sensor_tag;  % Update the sensor tag
        end
    end
end
