function [ output_args ] = Transform_UC_DD( input_args )
%TRANSFORM_UC_DD Summary of this function goes here
%   Detailed explanation goes here

%Robot parameters
L=0.18/(2*pi);      %18 cm track length of Zumo
R=0.09;             %9  cm swing radius

w=input_args(1);
v=input_args(2);

vR=(2*v+w*L)/2*R;
vL=(2*v-w*L)/2*R;

output_args=[vR,vL];
end

