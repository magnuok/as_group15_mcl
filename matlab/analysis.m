close all;

number = 1544790720.89;

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

% Position estimation based on the ten highest weights
estimated_position = [];
stop = number_of_particles;
start = 1;
for i = 1:length(weights) / number_of_particles
   for j = start:stop
   [val, ind] = sort(weights(start:stop));
   val(1:10);
   ind(1:10);
   pose = mean(particle_list_after_resampling(ind(1:10),:));
   end
estimated_position = [estimated_position; pose];
start = start + number_of_particles;
stop = stop + number_of_particles;
end 

%
% %Position estimation based on the highest weight
% estimated_position = [];
% stop = number_of_particles;
% start = 1;
% for i = 1:length(weights) / number_of_particles
%    for j = start:stop
%        [highest, index]  = max(weights(start:stop));
%    end
% estimated_position = [estimated_position ; particle_list_after_resampling(index,:)];
% start = start + number_of_particles;
% stop = stop + number_of_particles;
% end

% % Position estimation based on the weighted average
%
% estimated_position = [];
% start = 1;
% stop = number_of_particles;
% for i = 1:length(weights) / number_of_particles
%    for j = start:stop
%        pose = sum((weights(start:stop).*particle_list_after_resampling(start:stop,:)))./sum(weights(start:stop));
%    end
% estimated_position = [estimated_position ; pose];
% start = start + number_of_particles;
% stop = stop + number_of_particles;
% end

%% Error calculation

error = mocap_pose_before_iteration - estimated_position;
pose_error_x = error(:,1);
pose_error_y = error(:,2);
pose_error = sqrt((error(:,1).^2 + error(:,2).^2));
orientation_error = mod(error(:,3),(2*pi));

%% Error plot 

t = transpose(0:loop_time:1000);

figure
plot(t(1:length(error)),pose_error(1:length(error)),'+','LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Pose error')
xlabel('time [s]')
ylabel ('error [m]')

figure
plot(t(1:length(error)),pose_error_x(1:length(error)),'+','LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Pose error x')
xlabel('time [s]')
ylabel ('error [m]')

figure
plot(t(1:length(error)),pose_error_y(1:length(error)), '+','LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Pose error y')
xlabel('time [s]')
ylabel ('error [m]')


figure
plot(t(1:length(error)),orientation_error(1:length(error)), '+', 'LineWidth', 2)
set(gca,'fontsize',16,'box','off')
title('Orientation error')
xlabel('time [s]')
ylabel ('error [rad]')


%% plot iteration time, calculate min, maz and var

figure;
plot(1:length(iteration_time),iteration_time(1:length(iteration_time)), '.', 'LineWidth', 1);
set(gca,'fontsize',16,'box','off')
title('Iteration time');
xlabel('iteration');
ylabel ('time [s]');


iteration_time_min = min(iteration_time);
iteration_time_max = max(iteration_time);
iteration_time_var = var(iteration_time);
%% box plot of timings
%boxplot(iteration_time)
%set(gca,'fontsize',16,'box','off')


%% historgram of iteration time
figure
histogram(iteration_time,0:0.1:2)
title('Iteration time of MCL')
xlabel('count')
ylabel('iteration time')
set(gca,'fontsize',16,'box','off','ygrid', 'on' )



%% plot the movement of the particles

figure;
h = animatedline('Marker', '.', 'LineStyle', 'none');
r = animatedline('Marker', '.', 'LineStyle', 'none', 'Color', 'r');

axis([12,20,11,17]);

for k = 1:length(estimated_position)
    addpoints(h,estimated_position(k,1),estimated_position(k,2));
    hold on;
    addpoints(r,mocap_pose_before_iteration(k,1), mocap_pose_before_iteration(k,2));
    drawnow;
    pause(0.1);
end
