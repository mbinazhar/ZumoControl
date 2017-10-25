% clc; clear all; close all;
disp('Started');

global robots


clear calcSpeeds

turns=1; %The number of turns the spiral will have
x=[-1*pi*turns : 1: pi*turns];
r=[0:1/(length(x)-1):1];

X=sin(x).*r*1;  Y=cos(x).*r*1;
plot(X,Y,'-r','LineWidth',2)
axis square

x_g = X;
y_g = Y;
point = 1;
Kp =5;
Ki =5;
Kd =1;
E_k = 0;
e_k_1 = 0;
steps = 0;

theta_g=0;
e_k=0;
%=================================================
R = 41/2; % in mm
L = 88.41; % in mm
dt=0.05; %50ms
maxVel=70;
% xyz = zeros(1,3);
% eAngle = zeros(1,3);
% pose = zeros(1,3);
LOOP=1;
a=1;
i=3; % for 1 robot
vLeft = 0;
vRight = 0;
distThresh = 0.1;
angleThresh = pi/10;

v = maxVel; %constant velocity
%=================================================

position = animatedline;
axis([-2.5 2.5 -2.5 2.5])
grid on


% while abs(pose(1,1)-goal(1,1))>0.2 && abs(pose(1,2)-goal(1,2))>0.2
% while LOOP<10
flag = true;
turnFlag = true;
StraightFlag = false;
% k=1;
while flag
%     [res ir(a,:) Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontIR_function',[1],[],'',[],vrep.simx_opmode_blocking);
%     [res Ints us(a,:) Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getk3FrontSonar_function',[1],[],'',[],vrep.simx_opmode_blocking);
        k = find(robots(:,1)==871);
        if isempty(k)
            continue
        else
        
          
            
%         addpoints(robotTrajectory(i),robots(k(1,1),2),robots(k(1,1),3));

        x = robots(k(1,1),2)
        y = -robots(k(1,1),3)
        theta = robots(k(1,1),4)
        
        
        dist = abs(sqrt( (y_g(point)-y)^2 + (x_g(point)-x)^2));
        if dist>distThresh
%         [vLeft,vRight] = calcSpeeds(3,x,y,theta,x_g(point),y_g(point));
[vLeft,vRight] = calcSpeedsWhileMoving(3,x,y,theta,x_g(point),y_g(point));
sendSpeeds(s,3,vLeft,vRight);
        else
        point = point+1;
            formatSpec = 'POINT=%1d REACHED.\n';
            fprintf(formatSpec,point-1)
        end
%         
% %       theta = atan2(sin(theta), cos(theta));
% 
%     dist = abs(sqrt( (y_g(point)-y)^2 + (x_g(point)-x)^2));
% 
%     % 1. Calculate the heading (angle) to the goal.
%     % distance between goal and robot in x-direction
%     u_x = x_g(point)-x;     
%     % distance between goal and robot in y-direction
%     u_y = y_g(point)-y;
%     % angle from robot to goal. Hint: use ATAN2, u_x, u_y here.
%     theta_g = atan2(u_y,u_x);
% 
%     % 2. Calculate the heading error.
%     % error between the goal angle and robot's angle
%     % Hint: Use ATAN2 to make sure this stays in [-pi,pi].
%     e_k = theta_g-theta;
%     e_k = atan2(sin(e_k),cos(e_k));
% 
%     % 3. Calculate PID for the steering angle 
%     % error for the proportional term
%     e_P = e_k;
% 
%     % error for the integral term. Hint: Approximate the integral using
%     % the accumulated error, E_k, and the error for
%     % this time step, e_k.
%     e_I = E_k + e_k*dt;
% 
%     % error for the derivative term. Hint: Approximate the derivative
%     % using the previous error, e_k_1, and the
%     % error for this time step, e_k.
%     e_D = (e_k-e_k_1)/dt;
%     w = Kp*e_P + Ki*e_I + Kd*e_D;
% 
%     % 4. Save errors for next time step
%     E_k = e_I;
%     e_k_1 = e_k;
%         
%     if (abs(e_k)>angleThresh) && (dist>distThresh)
%         turnFlag = true;
%         StraightFlag = false;
%         vRight = (2*v+w*L)/(2*R);
%         vLeft  = (2*v-w*L)/(2*R);    
%     elseif abs(e_k)<angleThresh
%         turnFlag = false;
%         StraightFlag = true;
%     end
%     
%     if StraightFlag
%         if dist>distThresh
            vRight = v;
%             vLeft = v;
%         else
%             point = point+1;
%             formatSpec = 'POINT=%1d REACHED.\n';
%             fprintf(formatSpec,point-1)
%             if point>length(x_g)
%                 vRight = 0;
%                 vLeft = 0;
%                 flag = false;
%             end
%         end
%     end
    
    formatSpec = 'X=%4.2f, Y=%4.2f, Th=%4.2f, Thg=%4.2f, ThERR=%4.2f, DIST=%4.2f, vL=%4.2f, vR=%4.2f\n';
    fprintf(formatSpec,x,y,theta,theta_g,e_k,dist,vLeft,vRight)
    
    
%     
% if( vLeft > 127)
%     vLeft = 127;
% end
% if( vLeft < -127)
%     vLeft = -127;
% end
% if( vRight > 127)
%     vRight = 127;
% end
% if( vRight < -127)
%     vRight = -127;
% end
% 
% if abs(vLeft) < 10
%     vLeft=0;
% end
% if abs(vRight) < 10
%     vLeft=0;
% end
%     
    
%     sendSpeeds(s,3,round(vLeft),round(vRight));
    addpoints(position,x,y);
    drawnow
    LOOP = LOOP+1;
    steps = steps+1;
    pause(.2);
        end
end

formatSpec = 'TIME TAKEN=%4.2fsec\n';
fprintf(formatSpec,steps/20)


vLeft = 0;
vRight = 0;
sendSpeeds(s,3,vLeft,vRight);

%disp('Program ended');