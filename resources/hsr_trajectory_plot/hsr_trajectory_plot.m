%%% hsr_plot_trajectory.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Script to plot joint_state velocity and position
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% User inputs
clear all
close all

% Relative directory of calibration bagfile
%calibration_file = 'data/calibration.bag';
joint_states_file = 'joint_states.bag';

% plots
pos = true;
vel = true;
eff = true;


%% Make sure matlab_rosbag is installed

if ~exist('matlab_rosbag-0.5.0-linux64','file')
    websave('matlab_rosbag', ...
            ['https://github.com/bcharrow/matlab_rosbag/releases/' ...
             'download/v0.5/matlab_rosbag-0.5.0-linux64.zip']);
    unzip('matlab_rosbag');
    delete matlab_rosbag.zip
end
addpath('matlab_rosbag-0.5.0-linux64');

%% Load the tag detections bagfile

bag = ros.Bag.load(joint_states_file);
msg = bag.readAll('/hsrb/robot_state/joint_states');

N = numel(msg);

names = msg{1,1}.name;
%get rid of underscores since they are treated as subscript "operators"
for i=1:length(names)
    names(i) = regexprep(names(i), '_+', ' ');
end

for i = 1:N
    position(i,:) = msg{1,i}.position.';
    velocity(i,:) = msg{1,i}.velocity.';
    effort(i,:) = msg{1,i}.effort.';
    time(i) = msg{1,i}.header.stamp.time.';
end
time = time - time(1);
%relevant_ind = [1 2 3 15 16 17 18 19];
relevant_arm = [15 16 17 18 19];
relevant_base = [1 2 3];

%% Position Plot
if(pos)
    figure(1)
    hold on;
    subplot(2,1,1)
    plot(time(:),position(:,relevant_arm),'linewidth',2)
    legend(names(relevant_arm),'fontsize',14)
    title('Position of arm joints')
    xlabel('time(sec)','fontsize',14)
    ylabel('units','fontsize',14)
    set(gca,'fontsize',14)

    subplot(2,1,2)
    plot(time(:),position(:, relevant_base),'linewidth',2)
    legend(names(relevant_base),'fontsize',14)
    title('Position of base joints')
    xlabel('time(sec)','fontsize',14)
    ylabel('units','fontsize',14)
    set(gca,'fontsize',14)
    hold off;
end

%% Velocity Plot
if (vel)
    figure(2)
    hold on;
    subplot(2,1,1)
    plot(time(:),velocity(:,relevant_arm),'linewidth',2)
    legend(names(relevant_arm),'fontsize',14)
    title('Velocity of arm joints')
    xlabel('time (sec)','fontsize',14)
    ylabel('units/second','fontsize',14)
    set(gca,'fontsize',14)

    subplot(2,1,2)
    plot(time(:),velocity(:, relevant_base),'linewidth',2)
    legend(names(relevant_base),'fontsize',14)
    title('Velocity of base joints')
    xlabel('time(sec)','fontsize',14)
    ylabel('units/second','fontsize',14)
    set(gca,'fontsize',14)
end

%% Effort Plot
if(eff)
    figure(3)
    hold on;
    subplot(2,1,1)
    plot(time(:),effort(:,relevant_arm),'linewidth',2)
    legend(names(relevant_arm),'fontsize',14)
    title('Effort of arm joints')
    xlabel('time (sec)','fontsize',14)
    ylabel('units','fontsize',14)
    set(gca,'fontsize',14)

    subplot(2,1,2)
    plot(time(:),effort(:, relevant_base),'linewidth',2)
    legend(names(relevant_base),'fontsize',14)
    title('Effort of base joints')
    xlabel('time(sec)','fontsize',14)
    ylabel('units','fontsize',14)
    set(gca,'fontsize',14)
end
