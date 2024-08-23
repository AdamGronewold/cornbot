clear all
% Load the data
data = readtable('SimpleFeelerData_July25_2024_0.2mps.csv');

% Extract relevant columns
time_seconds = data.Time_seconds;
sensor.y_del_t = data.Del_t;
FL = data.FL; % Front Left feeler angle
FR = data.FR; % Front Right feeler angle
BL = data.BL; % Back Left feeler angle
BR = data.BR; % Back Right feeler angle

sensor.y_angle_h = -FR;
sensor.y_time_h = time_seconds;

% Initialize variables
num_samples = length(sensor.y_angle_h);
state_labels = strings(num_samples, 1);

sensor.classified_contact_count(1)=0;

% State Transition Threshold
sensor.classifier_window = 5; % Each transition can only happen after this number times the sensor data rate

% Initialize state
current_state = "Unknown State";

% Loop through each time step
for i = 2:num_samples
    sensor.y_tk=i;
    % Estimate first and second derivatives
    if sensor.y_tk > 1
        sensor.ydot_angle_h(sensor.y_tk) = (sensor.y_angle_h(sensor.y_tk) - sensor.y_angle_h(sensor.y_tk - 1)) / sensor.y_del_t(i);
        if sensor.y_tk > 2
            sensor.yddot_angle_h(sensor.y_tk) = (sensor.ydot_angle_h(sensor.y_tk) - sensor.ydot_angle_h(sensor.y_tk - 1)) / sensor.y_del_t(i);
        else
            sensor.yddot_angle_h(sensor.y_tk) = 0;
        end
    else
        sensor.ydot_angle_h(sensor.y_tk) = 0;
        sensor.yddot_angle_h(sensor.y_tk) = 0;
    end
    
    i=sensor.y_tk;
    if i <= sensor.classifier_window
        sensor.current_state="Unknown State";
    elseif i > sensor.classifier_window      
        % Determine state based on conditions
        switch sensor.current_state
            case "Unknown State"
                if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif sensor.ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -10000
                    sensor.current_state = "In Contact";
                elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) < 0) && all(sensor.y_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Decreasing";
                end
    
            case "Steady State Angle"
                if sensor.ydot_angle_h(i) < -25 && sensor.yddot_angle_h(i) < -500 
                    sensor.current_state = "In Contact";
                elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 1)
                    sensor.current_state= "SHM Increasing";
                end
                
            case "SHM Increasing"
                if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif sensor.y_angle_h(i) > 0 && abs(sensor.ydot_angle_h(i)) < 300 && sensor.yddot_angle_h(i) < 0
                    sensor.current_state= "SHM Decreasing";
                elseif sensor.ydot_angle_h(i) < -30 && sensor.yddot_angle_h(i) > 0
                    sensor.current_state = "In Contact";
                end
    
            case "SHM Decreasing"
                if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                    sensor.current_state= "Steady State Angle";
                elseif sensor.ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -10000
                    sensor.current_state = "In Contact";
                elseif sensor.y_angle_h(i) < 0 && abs(sensor.ydot_angle_h(i)) < 100 && sensor.yddot_angle_h(i) > 0
                    sensor.current_state= "SHM Increasing";
                end
                
            case "In Contact"
                if all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                    sensor.current_state= "SHM Increasing";
                elseif all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 8) ...
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
        %Count the number of estimated contacts
    if sensor.f_y_angle_classifier_h(sensor.y_tk-1) ~= "In Contact" && sensor.f_y_angle_classifier_h(sensor.y_tk) == "In Contact"
        sensor.classified_contact_count(sensor.y_tk) = sensor.classified_contact_count(sensor.y_tk-1) + 1;
    else
        sensor.classified_contact_count(sensor.y_tk) = sensor.classified_contact_count(sensor.y_tk-1);
    end
end

% Plot results
figure('Position', [0 0 400 400]);
%subplot(3,1,1)
plot(sensor.y_time_h, sensor.y_angle_h, 'Color', 'black', 'HandleVisibility', 'off');
hold on;
plot(sensor.y_time_h, sensor.y_angle_h, '-o', 'MarkerIndices', find(sensor.f_y_angle_classifier_h == "In Contact"), 'Marker', 's', 'Color', 'r', 'LineStyle', 'none');
plot(sensor.y_time_h, sensor.y_angle_h, '-o', 'MarkerIndices', find(sensor.f_y_angle_classifier_h == "Steady State Angle"), 'Marker', 'd', 'Color', 'g', 'LineStyle', 'none');
plot(sensor.y_time_h, sensor.y_angle_h, '-o', 'MarkerIndices', find(sensor.f_y_angle_classifier_h == "SHM Increasing"), 'Marker', '^', 'Color', 'b', 'LineStyle', 'none');
plot(sensor.y_time_h, sensor.y_angle_h, '-o', 'MarkerIndices', find(sensor.f_y_angle_classifier_h == "SHM Decreasing"), 'Marker', 'v', 'Color', 'b', 'LineStyle', 'none');
plot(sensor.y_time_h, sensor.y_angle_h, '-o', 'MarkerIndices', find(sensor.f_y_angle_classifier_h == "Nearing Crash"), 'Marker', 'pentagram', 'Color', [1 0.7 0], 'LineStyle', 'none');
plot(sensor.y_time_h, sensor.y_angle_h, '-o', 'MarkerIndices', find(sensor.f_y_angle_classifier_h == "Unknown State"), 'Marker', 'x', 'Color', [0.9 0 0.9], 'LineStyle', 'none');
title('Non-binary Contact States in The Real-world');
xlabel('Time (s)');
ylabel('Angle (rad)');
ylim([-100, 40]);
%xlim([8 29]);
legend('In Contact', 'Steady State Angle', 'SHM Increasing', 'SHM Decreasing', 'Nearing Crash', 'Unknown State');
hold off;

% subplot(3,1,2)
% plot(sensor.y_time_h, sensor.ydot_angle_h);
% 
% subplot(3,1,3)
% plot(sensor.y_time_h, sensor.yddot_angle_h);

