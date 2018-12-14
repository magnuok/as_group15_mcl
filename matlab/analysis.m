clear all

number = 1544784062.24

information = load(['../run_monte_carlo_files/information',num2str(number),'.txt']);
mocap_pose_after_iteration = load(['../run_monte_carlo_files/_mocap_pose_after_iteration',num2str(number),'.txt']);
mocap_pose_before_iteration = load(['../run_monte_carlo_files/_mocap_pose_before_iteration',num2str(number),'.txt']);
particle_list_after_resampling = load(['../run_monte_carlo_files/_particle_list_after_resampling',num2str(number),'.txt']);
particle_list_before_resampling = load(['../run_monte_carlo_files/_particle_list_before_resampling',num2str(number),'.txt']);
weights = load(['../run_monte_carlo_files/_weights',num2str(number),'.txt']);
iteration_time = load(['../run_monte_carlo_files/iteration_time',num2str(number),'.txt']);

number_of_particles = information(1);
loop_time = information(3);

%% Define the estimated position from particle list 

% Position estimation based on the highest weight
estimated_position = [];
start = 1;
stop = number_of_particles;
for i = 1:length(weights) / number_of_particles 
    for j = start:stop
        [highest, index]  = max(weights(start:stop));
    end
estimated_position = [estimated_position ; particle_list_after_resampling(index,:)];
start = start + number_of_particles;
stop = stop + number_of_particles;
end 

% Position estimation based on hte wieghted average

estimated_position_ave = [];
start = 1;
stop = number_of_particles;
for i = 1:length(weights) / number_of_particles 
    for j = start:stop
        % Calculate weighted average here 
        pose = mean(weights.'*particle_list_after_resampling,1);

    end 
estimated_position_ave = [estimated_position_ave ; pose];
start = start + number_of_particles;
stop = stop + number_of_particles;
end 



%% Error calculation

error = abs(mocap_pose_after_iteration - estimated_position);
pose_error = sqrt((error(:,1).^2 + error(:,2).^2));
orientation_error = (error(:,3));

%% Error plot 

t = transpose(0:loop_time:1000);

figure
plot(t(1:length(error)),pose_error(1:length(error)),'LineWidth', 2)
box off
title('Pose error')
xlabel('time [s]')
ylabel ('error [m]')

figure
plot(t(1:length(error)),orientation_error(1:length(error)), 'LineWidth', 2)
box off
title('Orientation error')
xlabel('time [s]')
ylabel ('error [rad]')


%% plot iteration time, calculate min, maz and var

figure 
plot(t(1:length(iteration_time)),iteration_time(1:length(iteration_time)), 'LineWidth', 2)
box off
title('Iteration time')
xlabel('time [s]')
ylabel ('iteration time [s]')

iteration_time_min = min(iteration_time);
iteration_time_max = max(iteration_time);
iteration_time_var = var(iteration_time);



