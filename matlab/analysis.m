
%% EXPERIMENT 1
numbers = [1544953342.34, 1544953468.53, 1544953598.89, 1544953718.55, 1544953852.73, 1544953975.7, 1544954105.4, 1544954231.03, 1544954351.24, 1544954471.17];
folder = 'experiment_1';
%% EXPERIMENT 2 
numbers = [1544951877.49, 1544952006.27, 1544952129.57, 1544952252.39, 1544952380.38, 1544952580.46, 1544952838.03, 1544952975.95, 1544953099.8];
folder = 'experiment_2';
%% EXPERIMENT 3 
numbers = [1544957102.57, 1544957284.33, 1544957691.94, 1544957875.72, 1544958081.49, 1544958295.41, 1544958482.89, 1544958678.14, 1544958974.83, 1544959198.97];
folder = 'experiment_3';
%% EXPERIMENT 4
numbers = [1544954930.59, 1544955136.47, 1544955458.35, 1544955643.45, 1544955841.03, 1544956024.42, 1544956240.01, 1544956429.34, 1544956614.84, 1544956800.07];
folder = 'experiment_4';
%% 
% Load data
information = [];
mocap_pose_after_iteration = [];
mocap_pose_before_iteration = [];
particle_list_after_resampling = [];
particle_list_before_resampling = [];
weights = [];
iteration_time = [];

for i = 1:length(numbers)
    number = numbers(i);
    information = [information, load(['../run_monte_carlo_files/final_experiments/', folder, '/information',num2str(number),'.txt'])];
end
%    information = [information, load(['../run_monte_carlo_files/final_experiments/', folder, '/information1544952713.0.txt'])];

number_of_particles = information(1);
loop_time = information(3);

% Change here the min length of mocap and particles
min_len_mocap = 230; 
min_len_particles = min_len_mocap*number_of_particles;
start_iteration = 50;

% Read data
for i = 1:length(numbers)
    number = numbers(i);
    mocap_pose_after_iteration_load = load(['../run_monte_carlo_files/final_experiments/', folder, '/_mocap_pose_after_iteration',num2str(number),'.txt']);
    mocap_pose_after_iteration = [mocap_pose_after_iteration, mocap_pose_after_iteration_load(start_iteration+1:min_len_mocap,1:3)];
    
    mocap_pose_before_iteration_load = load(['../run_monte_carlo_files/final_experiments/', folder, '/_mocap_pose_before_iteration',num2str(number),'.txt']);
    mocap_pose_before_iteration = [mocap_pose_before_iteration, mocap_pose_before_iteration_load(start_iteration+1:min_len_mocap,1:3)];
    
    particle_list_after_resampling_load = load(['../run_monte_carlo_files/final_experiments/', folder, '/_particle_list_after_resampling',num2str(number),'.txt']);
    particle_list_after_resampling = [particle_list_after_resampling, particle_list_after_resampling_load(start_iteration*number_of_particles+1:min_len_particles,1:3)];
    
    particle_list_before_resampling_load = load(['../run_monte_carlo_files/final_experiments/', folder, '/_particle_list_before_resampling',num2str(number),'.txt']);
    particle_list_before_resampling = [particle_list_before_resampling, particle_list_before_resampling_load(start_iteration*number_of_particles+1:min_len_particles,1:3)];

    weights_load = load(['../run_monte_carlo_files/final_experiments/', folder, '/_weights',num2str(number),'.txt']);
    weights = [weights, weights_load(start_iteration*number_of_particles+1:min_len_particles)];
    
    iteration_time_load = load(['../run_monte_carlo_files/final_experiments/', folder, '/iteration_time',num2str(number),'.txt']);
    iteration_time = [iteration_time, iteration_time_load(start_iteration+1:min_len_mocap)];
end

% Position estimation based on the ten highest weights
estimated_position = [];
for i=1:number_of_particles:length(weights)
    [val, ind] = sort(weights(i:i+number_of_particles-1,:),'descend');
    pose = mean(particle_list_before_resampling(ind(1:10)+i-1,:));
    estimated_position = [estimated_position; pose];
end

%Remove the static error of estimated position. -0.2 for x and -0.1 for y
for i=1:3:length(numbers)*3
    estimated_position(:,i) = estimated_position(:,i) - 0.2;
end 
for i=2:3:length(numbers)*3 
    estimated_position(:,i) = estimated_position(:,i) - 0.1;
end

% Calculate error
error = mocap_pose_before_iteration - estimated_position;
pose_error_x = [];
pose_error_y = [];
orientation_error = [];

for i=1:3:length(numbers)*3
    pose_error_x = [pose_error_x, error(:,i)];
end
for i=2:3:length(numbers)*3
    pose_error_y = [pose_error_y, error(:,i)];
end    

for i=3:3:length(numbers)*3
    
    help = mod(error(:,i),(2*pi));
    for j = 1:length(help)
        if help(j) > pi
            help(j) = help(j) - 2*pi;
        end
    end         
    orientation_error = [orientation_error, help];

end

pose_error = sqrt((pose_error_x.^2 + pose_error_y.^2));

% Calculate mean of all errors
pose_error_x_mean = [];
pose_error_y_mean = [];
pose_error_mean = [];
orientation_error_mean = [];

for i=1:length(pose_error)
    pose_error_x_mean = [pose_error_x_mean, mean(pose_error_x(i,:))];
    pose_error_y_mean = [pose_error_y_mean, mean(pose_error_y(i,:))];
    pose_error_mean = [pose_error_mean, mean(pose_error(i,:))];
    orientation_error_mean = [orientation_error_mean, mean(orientation_error(i,:))];
end

%% Calculate meadian and mean of the x anf y mean errors 
e_pose_mean_mean = mean(pose_error_mean)
orientation_pose_mean_mean = mean(orientation_error_mean)
std_e_pose_mean = std(pose_error_mean)
std_e_orientation_mean = std(orientation_error_mean)

%% plot boxplot
t = transpose(1:1:10000);

figure
plot(t(1:length(error)),pose_error_mean(1:length(error)),'Color', 'r', 'LineWidth', 1.5)
set(gca,'fontsize',16,'box','off', 'TickDir', 'out', 'XTick', 1:10:100)
title('Pythagoras position error')
legend ('Mean position error')
ylabel ('error [m]')
xlabel ('number of iteration')
axis([0, 230, 0, 1])


hold on 

pose_error_boxplot = pose_error';
boxplot(pose_error_boxplot)
set(gca,'fontsize',16,'box','off', 'XTick', [0:10:250],'XTickLabel', [0:10:250])

%% plot historgram of iteration time
figure
histogram(iteration_time,0:0.05:0.7)
title('Iteration time of MCL')
xlabel('iteration time')
ylabel('count')
set(gca,'fontsize',16,'box','off','ygrid', 'on' )

%% polt all mean errors 
close all
t = transpose(0:loop_time*3:1000);
for i=1:length(numbers)
    figure
    plot(t(1:length(error)),pose_error(1:length(error),i))
    set(gca,'fontsize',16,'box','off')
    title('pose error [m], global')
    xlabel('time [s]')
    ylabel ('error [rad]')
    hold on
end
%% PLOT MEAN ERROR FOR ALL RUNS
% close all
% t = transpose(0:loop_time:10000);
% 
% % calculate the mean of the mean error 
% 
% figure
% plot(t(1:length(error)),pose_error_mean(1:length(error)),'+','LineWidth', 1.5)
% set(gca,'fontsize',16,'box','off')
% title('Mean pythagoras position error')
% xlabel('time [s]')
% ylabel ('error [m]')

%figure
% plot(t(1:length(error)),pose_error_x_mean(1:length(error)),'+','LineWidth', 1.5)
% set(gca,'fontsize',16,'box','off')
% title('Mean pose error to x directon over 10 runs')
% xlabel('time [s]')
% ylabel ('error [m]')
% 
% figure
% plot(t(1:length(error)),pose_error_y_mean(1:length(error)), '+','LineWidth', 1.5)
% set(gca,'fontsize',16,'box','off')
% title('Mean pose error to y directon over 10 runs')
% xlabel('time [s]')
% ylabel ('error [m]')
% 
% figure
% plot(t(1:length(error)),orientation_error_mean(1:length(error)), '+', 'LineWidth', 1.5)
% set(gca,'fontsize',16,'box','off')
% title('Mean orientation error')
% xlabel('time [s]')
% ylabel ('error [rad]')


%% 
% t = transpose(1:loop_time:10000);
% figure
% 
% for i=1:length(numbers)
%     plot(t(1:length(error)),pose_error(1:length(error),i))
%     set(gca,'fontsize',16,'box','off')
%     title('pose error 10 runs, 1m box')
%     xlabel('time [s]')
%     ylabel ('error [rad]')
%     hold on
% end
% 
% t = transpose(1:loop_time:10000);
% figure
% 
% for i=1:length(numbers)
%     plot(t(1:length(error)),pose_error_x(1:length(error),i))
%     set(gca,'fontsize',16,'box','off')
%     title('pose error x 10 runs, 1m box')
%     xlabel('time [s]')
%     ylabel ('error [rad]')
%     hold on
% end
% 
% t = transpose(1:loop_time:10000);
% figure
% 
% for i=1:length(numbers)
%     plot(t(1:length(error)),pose_error_y(1:length(error),i))
%     set(gca,'fontsize',16,'box','off')
%     title('pose error y 10 runs, 1m box')
%     xlabel('time [s]')
%     ylabel ('error [rad]')
%     hold on
% end
% 





% figure
% plot(t(1:length(error)),orientation_error_mean(1:length(error)),'Color', 'r', 'LineWidth', 1)
% set(gca,'fontsize',16,'box','off')
% title('Mean of the orientoration err, 5 runs, 1m box')
% xlabel('time [s]')
% ylabel ('error [m]')
% 
% axis([0, 300, 0, 1])
% 
% hold on 
% 
% orientation_error_boxplot = orientation_error';
% boxplot(orientation_error_boxplot)
% set(gca,'fontsize',16,'box','off')
% 


%% Calculate and plot error for multiple runs 

% GLOBAL INITIALIZATION 
clear all

% Load data
information = [];
mocap_pose_after_iteration = [];
mocap_pose_before_iteration = [];
particle_list_after_resampling = [];
particle_list_before_resampling = [];
weights = [];
iteration_time = [];

% Change here numbers
numbers = [1544879714.49, 1544879576.76, 1544879450.85, 1544878883.21, 1544878600.48, 1544878378.35, 1544878250.93, 1544878095.86, 1544877939.76, 1544877786.15, 1544877547.97, 1544877280.92, 1544877107.76, 1544876932.15, 1544876640.46, 1544876464.28, 1544876280.09, 1544876114.42, 1544875925.71, 1544875763.73, 1544875615.77, ];

for i = 1:length(numbers)
    number = numbers(i);
    information = [information, load(['../run_monte_carlo_files/experiment_x_1/information',num2str(number),'.txt'])];
end

number_of_particles = information(1);
loop_time = information(3);
% Change here the min length of mocap and particles
min_len_mocap = 300; 
min_len_particles = min_len_mocap*number_of_particles;

for i = 1:length(numbers)
    
    number = numbers(i);
    mocap_pose_after_iteration_load = load(['../run_monte_carlo_files/experiment_x_1/_mocap_pose_after_iteration',num2str(number),'.txt']);
    mocap_pose_after_iteration = [mocap_pose_after_iteration, mocap_pose_after_iteration_load(1:min_len_mocap,1:3)];
    
    mocap_pose_before_iteration_load = load(['../run_monte_carlo_files/experiment_x_1/_mocap_pose_before_iteration',num2str(number),'.txt']);
    mocap_pose_before_iteration = [mocap_pose_before_iteration, mocap_pose_before_iteration_load(1:min_len_mocap,1:3)];
    
    particle_list_after_resampling_load = load(['../run_monte_carlo_files/experiment_x_1/_particle_list_after_resampling',num2str(number),'.txt']);
    particle_list_after_resampling = [particle_list_after_resampling, particle_list_after_resampling_load(1:min_len_particles,1:3)];
    
    particle_list_before_resampling_load = load(['../run_monte_carlo_files/experiment_x_1/_particle_list_before_resampling',num2str(number),'.txt']);
    particle_list_before_resampling = [particle_list_before_resampling, particle_list_before_resampling_load(1:min_len_particles,1:3)];

    weights_load = load(['../run_monte_carlo_files/experiment_x_1/_weights',num2str(number),'.txt']);
    weights = [weights, weights_load(1:min_len_particles)];
    
    iteration_time_load = load(['../run_monte_carlo_files/experiment_x_1/iteration_time',num2str(number),'.txt']);
    iteration_time = [iteration_time, iteration_time_load(1:min_len_mocap)];
end

% Position estimation based on the ten highest weights
estimated_position = [];

for i=1:number_of_particles:length(weights)
    [val, ind] = sort(weights(i:i+49,:),'descend');
    pose = mean(particle_list_before_resampling(ind(1:10)+i-1,:));
    estimated_position = [estimated_position; pose];
end

% Remove the static error of estimated position. -0.2 for x and -0.1 for y
for i=1:3:length(numbers)*3
    estimated_position(:,i) = estimated_position(:,i) - 0.2;
end 
for i=2:3:length(numbers)*3
    estimated_position(:,i) = estimated_position(:,i) - 0.1;
end

% Calculate error
error = mocap_pose_before_iteration - estimated_position;
pose_error_x = [];
pose_error_y = [];
orientation_error = [];

for i=1:3:length(numbers)*3
    pose_error_x = [pose_error_x, error(:,i)];
end
for i=2:3:length(numbers)*3
    pose_error_y = [pose_error_y, error(:,i)];
end    
for i=3:3:length(numbers)*3
    orientation_error = [orientation_error, error(:,i)];
end    
pose_error = sqrt((pose_error_x.^2 + pose_error_y.^2));

% Calculate mean of all errors
pose_error_x_mean = [];
pose_error_y_mean = [];
pose_error_mean = [];
orientation_error_mean = [];


for i=1:length(pose_error)
    pose_error_x_mean = [pose_error_x_mean, mean(pose_error_x(i,:))];
    pose_error_y_mean = [pose_error_y_mean, mean(pose_error_y(i,:))];
    pose_error_mean = [pose_error_mean, mean(pose_error(i,:))];
    orientation_error_mean = [orientation_error_mean, mean(orientation_error(i,:))];
end

%% plot mean error of all runs 

t = transpose(0:loop_time:1000);

figure
plot(t(1:length(error)),pose_error_mean(1:length(error)),'+','LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Mean of the pose error, 5 runs, 1m box')
xlabel('time [s]')
ylabel ('error [m]')

figure
plot(t(1:length(error)),pose_error_x_mean(1:length(error)),'+','LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Mean of the pose error x, 5 runs, 1m box')
xlabel('time [s]')
ylabel ('error [m]')

figure
plot(t(1:length(error)),pose_error_y_mean(1:length(error)), '+','LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Mean of the pose error y, 5 runs, 1m box')
xlabel('time [s]')
ylabel ('error [m]')

figure
plot(t(1:length(error)),orientation_error_mean(1:length(error)), '+', 'LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Mean of the orientation error, 5 runs, 1m box')
xlabel('time [s]')
ylabel ('error [rad]')

%% 



%% plot mean error of all runs 

t = transpose(0:loop_time*3:1000);

figure
plot(t(1:length(error)),pose_error_mean(1:length(error)))
set(gca,'fontsize',16,'box','off')
title('Mean of the pose error, 5 runs, 1m box')
xlabel('time [s]')
ylabel ('error [m]')

axis([0, 80, 0, 1])

hold on 








