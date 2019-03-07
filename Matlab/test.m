clc;
% close all;
% clear;

%--------------------------------------------------------------------------
% Constant section
%--------------------------------------------------------------------------
car_speed = 0.0;
car_max_speed = 5;  % Km per hour;
dt=0.01;
wheel_base = 2.2;
wheel_angle = 0;
wheel_speed = 0;
look_ahead = 3;
frameStep = 24;  % update figures as every x steps
%--------------------------------------------------------------------------
% End of constant section
%--------------------------------------------------------------------------

%% First create map. Waypoints can be supplied

% use all default values? or select an optional name value
% map = CreateMap();
map = CreateMap('method','pchip');

% create waypoints using mouse or load from gpx
% map.create_waypoints();
map.load_gpx('./full_turn2-data.gpx');
map.interpolate_path();

%% Create or set initial car position and pose
[x0_init, yo_init, psi_init] = map.create_car();
% not needed anymore?
close(map.figure_handle);

%% create a steering controller
steering = SteeringController('method','pure pursuit');
steering.path_look_ahead = look_ahead;

%% create car plotter
show_car = CarPlotter();

%% set car initial position and load map on controller and also plotter
steering.load_car(x0_init, yo_init, psi_init);
steering.load_map(map.map_points);

show_car.prepare_figure(map.map_points, steering.car_position, steering.car_pose);


endpoint = steering.map_points(end,:);
steering.sampling_time = dt;


%% simulate movement of car and perform steering calculations 
% Also feed data into CarPlotter class to visualize movement.
counter = 1;
while(1)
    %----------------------------------------------------------------------
    % check termination condition
    %----------------------------------------------------------------------
    dist_to_end = norm(steering.car_position-endpoint);
    if dist_to_end<0.5 && (steering.num_points-steering.path_index)< look_ahead
        disp(['---simulation completed--- ', num2str(dist_to_end)])
        break
    end
    %----------------------------------------------------------------------
    % world frame position of car
    %----------------------------------------------------------------------
    steering.car_position(1) = steering.car_position(1)+ dt*car_speed*(cos(steering.car_pose));
    steering.car_position(2) = steering.car_position(2)+ dt*car_speed*(sin(steering.car_pose));
    steering.car_pose = steering.car_pose + dt*tan(wheel_angle(counter))*car_speed/wheel_base;
    
    
    %----------------------------------------------------------------------
    % update data into plotter class
    %----------------------------------------------------------------------
    show_car.update_data( steering.car_position, steering.car_pose);
    
    %----------------------------------------------------------------------
    % compute error between car position and target.
    %----------------------------------------------------------------------
    %     [wheel_angle(counter+1), wheel_speed(counter+1)] = steering.update();
    wheel_angle(counter+1)=steering.update();
    error_q = steering.map_points(steering.path_index, :) - steering.car_position;
    %----------------------------------------------------------------------
    % reference  value for the linear velocity
    %----------------------------------------------------------------------
    if norm(error_q)>10
        % increase smoothly
        car_speed = min([car_speed+0.1, car_max_speed/3.6]);
    else
        error_end = steering.map_points(end, :)-steering.car_position;
        % decrease at end
        car_speed = min([car_speed+0.1, norm(error_end)/4, car_max_speed/3.6]);
    end
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    if(mod(counter, frameStep)==0)
        % time to update plots
        show_car.update_plots();
        
    end
    counter=counter+1;
end

