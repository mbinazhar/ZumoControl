% clear all; close all; clc;
rosshutdown;delete(instrfindall);

global sonar;
global lwheel_pub;
global rwheel_pub;
global twist_pub;
sonar = zeros(1,5);

try
    % ROS Initialization BLOCK
    % rosinit, IP of ROS MASTER, IP of MATLAB SYSTEMS, Node Name
    % Get system IP: system('ipconfig'), confirm Interface, WiFi/Ethernet
    rosinit('http://192.168.100.123:11311', 'NodeHost', '192.168.100.105')
    sonar0_sub = rossubscriber('/sonar0',@sonar0Subscriber);
	sonar1_sub = rossubscriber('/sonar1',@sonar1Subscriber);
	sonar2_sub = rossubscriber('/sonar2',@sonar2Subscriber);
	sonar3_sub = rossubscriber('/sonar3',@sonar3Subscriber);
	sonar4_sub = rossubscriber('/sonar4',@sonar4Subscriber);
%     test_sub = rossubscriber('/lwheel_desired_vel',@testSubscriber);
    matlab_Wheelvelocity = robotics.ros.Node('/matlab_veloWheel');
    lwheel_pub = robotics.ros.Publisher(matlab_Wheelvelocity,'/lwheel_desired_vel','std_msgs/Int16');
    rwheel_pub = robotics.ros.Publisher(matlab_Wheelvelocity,'/rwheel_desired_vel','std_msgs/Int16');
    matlab_Twistvelocity = robotics.ros.Node('/matlab_veloTwist');
    twist_pub = robotics.ros.Publisher(matlab_Wheelvelocity,'/cmd_vel','geometry_msgs/Twist');
    
catch
    display('ROS initialization error')
    rosshutdown
end
% the variable zumoPose is UPDATED automatically via callback - USE that