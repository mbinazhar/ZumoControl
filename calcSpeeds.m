function [vLeft,vRight,hasReached] = calcSpeedsWhileMoving( i,x,y,theta,x_g,y_g )
% SET TURN FLAGS TO ZERO FOR TURN-WHILE-MOVING and 1 for TURN-THEN-MOVE
Kp =32;%12;
Ki =12;%5;
Kd =8;%5

dt=0.20; %200ms

distThresh = 0.08;

% defaultSpeed = 150;
defaultSpeed = 80;

persistent E_k;
persistent e_k_1;
persistent turnFlags;

if(isempty(E_k))
    E_k    = [0;0;0];
    e_k_1 = [0;0;0];
    turnFlags = [1;1;1];
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

vRot = w;
hasReached=0;

if abs (e_k)  > deg2rad(5) && turnFlags(i) == 1 && dist > distThresh
%     vLeft =  -e_k/abs(e_k)*50;
%     vRight = +e_k/abs(e_k)*50;
    vLeft =  -vRot;
    vRight = +vRot;
    
elseif (dist > distThresh)
    if turnFlags(i)==1
        turnFlags(i)=0;
        E_k(i)=0;e_k_1(i)=0;
        vLeft=0;
        vRight=0;
    else
    vLeft =  defaultSpeed-vRot;
    vRight = defaultSpeed+vRot;
    end
    
else
    vLeft=0;
    vRight=0;
    turnFlags(i)=1;
    hasReached=1;
end

if( vLeft > 300)
    vLeft = 300;
end
if( vLeft < -300)
    vLeft = -300;
end
if( vRight > 300)
    vRight = 300;
end
if( vRight < -300)
    vRight = -300;
end

% if abs(vLeft) < 10
%     vLeft=0;
% end
% if abs(vRight) < 10
%     vRight=0;
% end

% if vLeft == NaN || vRight == NaN
%     vLeft=0;vRight=0;
% end


% vLeft=round(vLeft);
% vRight=round(vRight);
end

