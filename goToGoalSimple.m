global zumoSensors
global zumoPose

color = 'kbgrcmy'; colorVal=1;

X = 1;
Y = 2;
THETA = 3;
THETA_TOLERANCE = pi/20.0;
GOAL_TOLERANCE  = 0.05;
GOAL_PAUSE = 2;
point = 1;
dt = 0.2;



% goal = [ 0      0      0
%          0.4    0      0
%          0      0.4    0
%         -0.4    0      0
%          0     -0.4    0
%          0.4    0      0];
t=linspace(-.8,+.8,15);
curve=0.2*sin(15*t);
goal=[t' , curve'];
plot(goal(:,1),goal(:,2),'+-');
     
MAXPOINTS=size(goal,1);

% robotIds = [314 528 871];
% noOfRobots = size(robotIds,2);
% zumoPose = zeros(noOfRobots,3);
% zumoSensors = zeros(noOfRobots,9);

clear calcSpeeds
clear calcSpeedsWhileMoving

% figure; 
hold on; grid on;
axis([-1 1 -1 1]); axis square;



robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];

for i = 1: noOfRobots-1
    colorVal = colorVal+1;
    if(colorVal>7)
        colorVal=1;
    end
    robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end



t = 2;
while t>0
    t = t+1;
    
    
%     for i = 1:noOfRobots
    for i = 2:2
        
        x = zumoPose(i,X);
        y = zumoPose(i,Y);
        theta = zumoPose(i,THETA);
        
        addpoints(robotTrajectory(i),x,y);

        [vLeft,vRight,hasReached] = calcSpeeds(i,x,y,theta,goal(point,1),goal(point,2)) % TURNS THEN MOVES WHILE TURNING
        sendSpeedsCharacterWise(s,i,round(vLeft),round(vRight));
        
%         sprintf('Robot=%.1f , x=%.2f , y=%.2f, t=%.2f, vLeft=%.1f , vRight=%.1f',i,x,y,theta,vLeft,vRight)
                    
    end
    
    if hasReached == 1
        point = point+1
        pause(3);
    end
    if point>MAXPOINTS
        return
    end
    
    
    pause(dt);
    if(mod(t,10)==0)
        drawnow
    end
    
end
    
    
        
        