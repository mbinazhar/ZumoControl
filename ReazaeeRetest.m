% close all;
% 
clear calcSpeeds
clear calcSpeedsWhileMoving
initZigbee
global zumoSensors
global zumoPose

selectedRobot=[1 2 3];
numberOfRobots = 3;


color = 'kbgrcmy'; colorVal=1;

    fxvdes = .5;%.5
    fyvdes = .9;
    virtualYForceMultiplier=3;
    kp = 0;%.5; % Force on virtual bot
    
TIMESTEP = 0.2;

% robotIds = [528 314 871];


robot=[ ([0, 0, pi/2]) ] ;
robotT1 = robot;

robotDot = zeros(numberOfRobots,2);

FORCE_MAX = 20;
FORCE_MIN_THRESHOLD = 0;

obstacle = ([25,25]);
obstacleSize = ([4,3]);
ra = 1;

Tau = 4;


M = 1;
Bz = 1;
kd = 9;
ksk = 100;
kr = 10;
alpha = .2;
qk = .1;
q(1:numberOfRobots) = qk;
m(1:numberOfRobots) = M;

VirtualBot = [-.5,-0.5];
VirtualBot = [0,0];
VirtualBotDot = [0,0];


figure; 
hold on; grid on;
FLOORSIZE = 1.5; % in meters HALF
axis([-FLOORSIZE FLOORSIZE -FLOORSIZE FLOORSIZE]); axis square;

    
    
    robotTrajectory = animatedline('Color',color(colorVal),'LineWidth',2);
for i = 1: numberOfRobots-1
   colorVal = colorVal+1;
   if(colorVal>7)
       colorVal=1;
   end
   robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

circle(VirtualBot(1,1),VirtualBot(1,2),alpha,2);
circle(VirtualBot(1,1),VirtualBot(1,2),0.2,2);
VirtualTrajectory = animatedline('Color','r','LineWidth',1,'LineStyle','-.');

t = 0;
MAX_TIME = 200;
zed = zeros(MAX_TIME,numberOfRobots*2);

obstacleFlag = zeros(1,numberOfRobots);

v1 = obstacleSize(1,1);
v2 = obstacleSize(1,2);
A = sqrt(1/(2*((v1)^2)));
B = sqrt(1/(2*((v2)^2)));


FxkVST1 = zeros(1,numberOfRobots);
FykVST1 = zeros(1,numberOfRobots);

FxkVS = zeros(1,numberOfRobots);
FykVS = zeros(1,numberOfRobots);

FxkVSrkra = zeros(1,numberOfRobots);
FykVSrkra = zeros(1,numberOfRobots);

FxkObs = zeros(1,numberOfRobots);
FykObs = zeros(1,numberOfRobots);


while t<MAX_TIME
% while robot(1,1)<1 && robot(2,1)<1 && robot(3,1)<1 && robot(1,2)<.9 && robot(2,2)<.9 && robot(3,2)<.9
    t = t+1;
    tic;
     for i = selectedRobot

         x=zumoPose(i,1);
         y=zumoPose(i,2);
         theta=zumoPose(i,3);
            
            addpoints(robotTrajectory(i),x,y);
            if t==0
                circle(robot(i,1),robot(i,2),0.076,2);
            end
            
            robot(i,:) = [x y theta];
            robotT1(i,:) = robot(i,:);
            drawnow
        end

        
    
    for i = selectedRobot
        a = obstacle(1,:)-robot(i,1:2);
        rk(i) = sqrt(((A^2*a(1)^2+B^2*a(2)^2-1)));

        if(rk(i)<= ra)
            chi = atan2(a(2),a(1));
            psi = atan2(robot(i,2)-robotT1(i,2),robot(i,1)-robotT1(i,1));            
            if(mod(psi-chi,2*pi)<=pi)
                Fxkrc  =  (B/A)*(robot(i,2)-obstacle(1,2));
                Fykrc  = -(A/B)*(robot(i,1)-obstacle(1,1));
                Fxkr   = Fxkrc;
                Fykr   = Fykrc;
            else
                Fxkrcc = -(B/A)*(robot(i,2)-obstacle(1,2));
                Fykrcc =  (A/B)*(robot(i,1)-obstacle(1,1));
                Fxkr   = Fxkrcc;
                Fykr   = Fykrcc;
            end

            Fxkrn = Fxkr/norm([Fxkr;Fykr]);
            Fykrn = Fykr/norm([Fxkr;Fykr]);
            
            % To check if this (ith) robot has reached near obstacle before
            % If the robot is reaching for the first time, near obstacle
            % The FxkVSrkra is set to the previosu force.
            if(obstacleFlag(i)==0)
                FxkVSrkra(i) = FxkVST1(i);
                FykVSrkra(i) = FykVST1(i);
                obstacleFlag(i)=1;
            end
            % second part of eq 22, 
            FxkObs(i) = ((g(FxkVSrkra(i),FykVSrkra(i))*Fxkrn)/rk(i)^2)*((1/rk(i))-(1/ra));
            FykObs(i) = ((g(FxkVSrkra(i),FykVSrkra(i))*Fykrn)/rk(i)^2)*((1/rk(i))-(1/ra));
            
            FxkBS(i) = FxkVSrkra(i) + FxkObs(i);
            FykBS(i) = FykVSrkra(i) + FykObs(i);
        else
            for j = 1:numberOfRobots
                if i == j
                    r(i,j) = 0;
                    thetaX(i,j)	 = 0;
                    thetaY(i,j)	 = 0;
                    Fk(i,j) = 0;
                else
                    a = robot(i,:)-robot(j,:);
                    r(i,j) = sqrt(a(1)^2+a(2)^2);
                    
                    thetaX(i,j) = a(1)/abs(r(i,j));
                    thetaY(i,j) = a(2)/abs(r(i,j));
                    
                    % eq 6
                    Fk(i,j) = (kr*q(i)*q(j))/(r(i,j)^2);
                end
                Fxk_dist(i,j) = Fk(i,j)*thetaX(i,j);
                Fyk_dist(i,j) = Fk(i,j)*thetaY(i,j);
            end
            Fxk(i) = sum(Fxk_dist(i,1:numberOfRobots));
            Fyk(i) = sum(Fyk_dist(i,1:numberOfRobots));
            
            FxkVST1(i) = FxkVS(i);
            FykVST1(i) = FykVS(i);
            
            a = robot(i,1:2) - VirtualBot;
            % eq 8: FxkVS = Fxk - ksk*[attractive force b/w robot and virtual]
            FxkVS(i) = Fxk(i)-ksk*(a(1)*(a(1)^2+a(2)^2-alpha^2));
            FykVS(i) = Fyk(i)-ksk*(a(2)*(a(1)^2+a(2)^2-alpha^2));
            
            % eq 23
            FxkBS(i) = FxkVSrkra(i)*exp(-Tau*rk(i)) + FxkVS(i)*(1-exp(-Tau*rk(i)));
            FykBS(i) = FykVSrkra(i)*exp(-Tau*rk(i)) + FykVS(i)*(1-exp(-Tau*rk(i)));
            if(obstacleFlag(i)==1)
                obstacleFlag(i)=2;
            end
        end
    end
    
    xm = sum(robot(:,1))/numberOfRobots; 
    ym = sum(robot(:,2))/numberOfRobots; 
    rm = min(rk);
    
    fyvdes = virtualYForceMultiplier*sin((10)*VirtualBot(1,1));
%         fyvdes = 5;
    fyvdes=0.;fxvdes=0.;
        
    if(rm<=ra)
        fxvBS = fxvdes + kp*(xm-VirtualBot(1,1))*(1-exp(-Tau*rm));
        fyvBS = fyvdes + kp*(ym-VirtualBot(1,2))*(1-exp(-Tau*rm));
    else
        fxvBS = fxvdes;
        fyvBS = fyvdes;
    end
    
    for i=1:numberOfRobots
        [FxkBS(i),FykBS(i)] = forceConstrain(FxkBS(i),FykBS(i),FORCE_MAX);
        if(abs(FxkBS(i))>FORCE_MIN_THRESHOLD)
            fx = @(t,x) [x(2); (FxkBS(i)-(Bz+kd)*x(2))/M];
            [T,X] = ode45(fx,0:TIMESTEP/10:TIMESTEP,[robot(i,1);robotDot(i,1)]);
            robotT1(i,1) = robot(i,1);
            robot(i,1) = real(X(end,1));
            robotDot(i,1) = real(X(end,2));
        end
        
        if(abs(FykBS(i))>FORCE_MIN_THRESHOLD)
            fy = @(t,y) [y(2); (FykBS(i)-(Bz+kd)*y(2))/M];
            [T,Y] = ode45(fy,0:TIMESTEP/10:TIMESTEP,[robot(i,2);robotDot(i,2)]);
            robotT1(i,2) = robot(i,2);
            robot(i,2) = real(Y(end,1));
            robotDot(i,2) = real(Y(end,2));
        end
        
        
        if(i==1)
            zed(t,1) = robotDot(i,1);
            zed(t,2) = robotDot(i,2);
        end
        if(i==2)
            zed(t,3) = robotDot(i,1);
            zed(t,4) = robotDot(i,2);
        end
        if(i==3)
            zed(t,5) = robotDot(i,1);
            zed(t,6) = robotDot(i,2);
        end

    end
    
    [fxvBS,fyvBS] = forceConstrain(fxvBS,fyvBS,FORCE_MAX);
%     return
    if(abs(fxvBS)>FORCE_MIN_THRESHOLD)
        fx = @(t,x) [x(2); (fxvBS-(Bz+kd)*x(2))/M];
        [T,X] = ode45(fx,0:TIMESTEP/10:TIMESTEP,[VirtualBot(1,1);VirtualBotDot(1,1)]);
        VirtualBot(1,1) = real(X(end,1));
        VirtualBotDot(1,1) = real(X(end,2));
    end
    if(abs(fyvBS)>FORCE_MIN_THRESHOLD)
        fy = @(t,y) [y(2); (fyvBS-(Bz+kd)*y(2))/M];
        [T,Y] = ode45(fy,0:TIMESTEP/10:TIMESTEP,[VirtualBot(1,2);VirtualBotDot(1,2)]);
        VirtualBot(1,2) = real(Y(end,1));
        VirtualBotDot(1,2) = real(Y(end,2));
    end
    
     addpoints(VirtualTrajectory,VirtualBot(1,1), VirtualBot(1,2));
     
     
   
     for i = selectedRobot
        mult = 1;
        if(i==1)
            goalX = robot(i,1)+zed(t,1)*mult;
            goalY = robot(i,2)+zed(t,2)*mult;
            [vLeft,vRight] = calcSpeedsWhileMoving(i,robot(i,1),robot(i,2),robot(i,3),goalX,goalY);
            sendSpeedsCharacterWise(s,i,vLeft,vRight);
        end
        if(i==2)
            goalX = robot(i,1)+zed(t,3)*mult;
            goalY = robot(i,2)+zed(t,4)*mult;
            [vLeft,vRight] = calcSpeedsWhileMoving(i,robot(i,1),robot(i,2),robot(i,3),goalX,goalY);
            sendSpeedsCharacterWise(s,i,vLeft,vRight);
        end
        if(i==3)
            goalX = robot(i,1)+zed(t,5)*mult;
            goalY = robot(i,2)+zed(t,6)*mult;
            [vLeft,vRight] = calcSpeedsWhileMoving(i,robot(i,1),robot(i,2),robot(i,3),goalX,goalY);
            sendSpeedsCharacterWise(s,i,vLeft,vRight);
        end
        
        sprintf('Robot=%.1f , x=%.2f , y=%.2f, t=%.2f, vLeft=%.1f , vRight=%.1f',i,robot(i,1),robot(i,2),robot(i,3),vLeft,vRight)
        
%         if robot(i,1)<1 && robot(i,2)<.85  
%         sendSpeeds(s,i,vLeft,vRight);
%         else
%             sendSpeeds(s,i,0,0);
%         end
        
     end
        
        
        
        timeElapsed=toc;
    while(timeElapsed<0.2)
        timeElapsed=toc;
    end
%     pause(0.1);
    t
    zed
     
     
end