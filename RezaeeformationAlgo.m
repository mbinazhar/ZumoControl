% close all;
% 
clear calcSpeeds
clear calcSpeedsWhileMoving

color = 'kbgrcmy'; colorVal=1;

robotIds = [528 314 871];
[~,numberOfRobots] =size(robotIds);


numberOfRobots=3;
robot=[ ([0, 0, pi/2])
        ([0, 0, pi/2])
        ([0, 0, pi/2])
        ];
robotT1 = robot;
%positions being input from VREP - these are just DUMMY values, but right
%ones

figure; 
hold on; grid on;
FLOORSIZE = 1.5; % in meters HALF
axis([-FLOORSIZE FLOORSIZE -FLOORSIZE FLOORSIZE]); axis square;

%Obstacle Info
x0 = 0;
y0 = 0;
v1=.3; v2=.4;
rectWidth =v1*2;
rectHeight=v2*2;

%Draw a rectangle
rectangle('Position',[x0-v1 y0-v2 rectWidth rectHeight]);%,'FaceColor',[1 0 0]);

A=sqrt(1/(2*((v1)^2)));
B=sqrt(1/(2*((v2)^2)));

ellipse(x0,y0,1/A,1/B);

ra = 0.5;
rk = zeros(numberOfRobots,1);

fxkdes = ones(numberOfRobots,1);
fykdes = ones(numberOfRobots,1);
% for figure 4
fxkdes = fxkdes.*8;
fykdes = fykdes.*8;

fxkOA = zeros(numberOfRobots,1);
fykOA = zeros(numberOfRobots,1);

xdot = zeros(numberOfRobots,1);
ydot = zeros(numberOfRobots,1);

psi = zeros(numberOfRobots,1);
chi = zeros(numberOfRobots,1);

M=1;
Bz=1;
kd=9;

maxVel=1000;
stopVel=0;


% Create Animated Line Objects for Each robot, different colors
robotTrajectory = animatedline('Color',color(colorVal),'LineWidth',2);
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end


t = 0;
MAX_TIME = 200;
zed = zeros(MAX_TIME,numberOfRobots*2);
% xdot1, ydot1, xdot2, ydot2
while t<MAX_TIME
% while robot(1,1)<1 && robot(2,1)<1 && robot(3,1)<1 && robot(1,2)<.9 && robot(2,2)<.9 && robot(3,2)<.9
    t = t+1;
    tic;
    for i=1:numberOfRobots
k = find(robots(:,1)==robotIds(1,i));
        if isempty(k)
            continue
        else
            x = robots(k(1,1),2);
            y = -robots(k(1,1),3);
            theta = robots(k(1,1),4);
            
            addpoints(robotTrajectory(i),x,y);
            if t==0
                circle(robot(i,1),robot(i,2),0.076,2);
            end
            
            robot(i,:) = [x y theta];
            robotT1(i,:) = robot(i,:);
            drawnow
        end

    end    
    
    for i=1:numberOfRobots
        xk = robot(i,1);
        yk = robot(i,2);
        
        chi(i)=atan2(y0-yk,x0-xk);
        psi(i) = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));

        if(mod(psi(i)-chi(i),2*pi)<=pi)
            fxkrc  =  (B/A)*(yk-y0);
            fykrc  = -(A/B)*(xk-x0);            
            fxkr   = fxkrc;
            fykr   = fykrc;
        else
            fxkrcc = -(B/A)*(yk-y0);
            fykrcc =  (A/B)*(xk-x0);
            fxkr   = fxkrcc;
            fykr   = fykrcc;
        end
        
%         if(i==4)
%             zed
%         end
        
        fxkrn = fxkr/norm([fxkr;fykr]);
        fykrn = fykr/norm([fxkr;fykr]);

        rk(i) =sqrt(((A^2*(xk-x0)^2+B^2*(yk-y0)^2-1)));
        if(rk(i)<=ra)
            fxkOA(i) = fxkdes(i) + ((g(fxkdes(i),fykdes(i))*fxkrn)/rk(i)^2)*((1/rk(i))-(1/ra));
            fykOA(i) = fykdes(i) + ((g(fxkdes(i),fykdes(i))*fykrn)/rk(i)^2)*((1/rk(i))-(1/ra));
        else
            fxkOA(i) = fxkdes(i);
            fykOA(i) = fykdes(i);
        end
        
%         if(abs(fxkOA(i))>FORCE_MAX)
%             fxkOA(i)=(fxkOA(i)/abs(fxkOA(i)))*FORCE_MAX;
%         end
%         if(abs(fykOA(i))>FORCE_MAX)
%             fykOA(i)=(fykOA(i)/abs(fykOA(i)))*FORCE_MAX;
%         end

        fx = @(t,x) [x(2); (fxkOA(i)-(Bz+kd)*x(2))/M];
%         [T,X]=ode45(fx,[0,0.05],[xk;xdot(i)]);
        [T,X]=ode45(fx,[0,0.05],[xk;0]);
        [m,z] = size(X);
        xk=real(X(m,1));
        xdot(i)=real(X(m,2));

        fy = @(t,y) [y(2); (fykOA(i)-(Bz+kd)*y(2))/M];
%         [T,Y]=ode45(fy,[0,0.05],[yk;ydot(i)]);
        [T,Y]=ode45(fy,[0,0.05],[yk;0]);
        [m,z] = size(Y);
        yk=real(Y(m,1));
        ydot(i)=real(Y(m,2));
        
        % xdot1, ydot1, xdot2, ydot2
        if(i==1)
            zed(t,1) = real(xdot(i));
            zed(t,2) = real(ydot(i));
        end
        if(i==2)
            zed(t,3) = real(xdot(i));
            zed(t,4) = real(ydot(i));
        end
        if(i==3)
            zed(t,5) = real(xdot(i));
            zed(t,6) = real(ydot(i));
        end
    end
    
    for i=1:numberOfRobots
        mult = 3;
        if(i==1)
            goalX = robot(i,1)+zed(t,1)*mult;
            goalY = robot(i,2)+zed(t,2)*mult;
            [vLeft,vRight] = calcSpeedsWhileMoving(i,robot(i,1),robot(i,2),robot(i,3),goalX,goalY);
%             sendSpeeds(s,i,vLeft,vRight);
        end
        if(i==2)
            goalX = robot(i,1)+zed(t,3)*mult;
            goalY = robot(i,2)+zed(t,4)*mult;
            [vLeft,vRight] = calcSpeedsWhileMoving(i,robot(i,1),robot(i,2),robot(i,3),goalX,goalY);
%             sendSpeeds(s,i,vLeft,vRight);
        end
        if(i==3)
            goalX = robot(i,1)+zed(t,5)*mult;
            goalY = robot(i,2)+zed(t,6)*mult;
            [vLeft,vRight] = calcSpeedsWhileMoving(i,robot(i,1),robot(i,2),robot(i,3),goalX,goalY);
%             sendSpeeds(s,i,vLeft,vRight);
        end
        
        if robot(i,1)<1 && robot(i,2)<.85  
        sendSpeeds(s,i,vLeft,vRight);
        else
            sendSpeeds(s,i,0,0);
        end
        
    end
    timeElapsed=toc;
    while(timeElapsed<0.2)
        timeElapsed=toc;
    end
%     pause(0.1);
    t
    zed
%     vX
%     vY
end

for i=1:numberOfRobots
    circle(robot(i,1),robot(i,2),0.076,4);
end

for i=1:numberOfRobots
    vX = stopVel;
    vY = stopVel;
  %  [res Ints Floats Strings Buffer]=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'setVelocity_function',[i vX vY],[],'',[],vrep.simx_opmode_blocking);
end