clear all; close all; clc;rosshutdown;delete(instrfindall);

global zumoPose
global robotIds
global noOfRobots
global zumoSensors

robotIds = [314 528 871];
noOfRobots = size(robotIds,2);
zumoPose = zeros(noOfRobots,3);
zumoSensors = zeros(noOfRobots,4);

try
    % ROS Initialization BLOCK
    % rosinit, IP of ROS MASTER, IP of MATLAB SYSTEMS, Node Name
    % Get system IP: system('ipconfig'), confirm Interface, WiFi/Ethernet
    rosinit('http://192.168.100.201:11311', 'NodeHost', '192.168.100.105')
    poseSubscriber = rossubscriber('/robotsPose',@mubassirSubscriber);  
catch
    display('ROS initialization error')
    rosshutdown
end
% the variable zumoPose is UPDATED automatically via callback - USE that