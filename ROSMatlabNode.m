clear all; close all; clc
global robots
n = 3;
robots = zeros(n,4);

try
    % ROS Initialization BLOCK
    % rosinit, IP of ROS MASTER, IP of MATLAB SYSTEMS, Node Name
    % Get system IP: system('ipconfig'), confirm Interface, WiFi/Ethernet
    rosinit('http://192.168.100.200:11311', 'NodeHost', '192.168.100.105')
    pose = rossubscriber('/robotsPose',@mubassirSubscriber);
catch
    display('ROS initialization error')
    rosshutdown
end

% the variable robots is UPDATED automatically via callback - USE that
% To turn off the console OUTPUT of robots, comment last line of callback