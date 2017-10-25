function [vLeft,vRight] = calcSpeedsWhileMoving( i,x,y,theta,x_g,y_g )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Kp =20;
Ki =10;
Kd =10;

dt=0.20; %200ms

distThresh = 0.1;

persistent E_k;
persistent e_k_1;

if(isempty(E_k))
    E_k    = [0;0;0];
    e_k_1 = [0;0;0];
end



u_x = x_g-x;
u_y = y_g-y;

dist = abs(sqrt( (y_g-y)^2 + (x_g-x)^2));

theta_g = atan2(u_y,u_x);

e_k = theta_g-theta;
e_k = atan2(sin(e_k),cos(e_k));


e_P = e_k;
e_I = E_k(i) + e_k*dt;
e_D = (e_k-e_k_1(i))/dt;

w = Kp*e_P + Ki*e_I + Kd*e_D;

E_k(i) = e_I;
e_k_1(i) = e_k;


if (dist>distThresh) % && (abs(e_k)>0.2) 
    vRot = w;
    vLeft =  40-vRot;
    vRight = 40+vRot;
else
    vLeft=0;
    vRight=0;
    E_k(i)=0;
end

if( vLeft > 127)
    vLeft = 127;
end
if( vLeft < -127)
    vLeft = -127;
end
if( vRight > 127)
    vRight = 127;
end
if( vRight < -127)
    vRight = -127;
end

if abs(vLeft) < 10
    vLeft=0;
end
if abs(vRight) < 10
    vLeft=0;
end


vLeft=round(vLeft);
vRight=round(vRight);
end
