close all; color = 'kbgrcmy'; colorVal=1;

global robots

robotIds = [528 314 871];
[~,numberOfRobots] =size(robotIds);

clear calcSpeeds

figure; hold on; grid on;
axis([-3 3 -3 3]); axis square;

% Create Animated Line Objects for Each robot, different colors
robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',2)];
for i = 1: numberOfRobots-1
    colorVal = colorVal+1;
    if(colorVal>7)
        colorVal=1;
    end
    robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',2)];
end

t = 2;
while t>0
    t = t+1;
    for i = 1:numberOfRobots
        k = find(robots(:,1)==robotIds(1,i));
        if isempty(k)
            continue
        else
            addpoints(robotTrajectory(i),robots(k(1,1),2),-robots(k(1,1),3));
            x = robots(k(1,1),2);
            y = -robots(k(1,1),3);
            theta = robots(k(1,1),4);
            
            if i==1
                [vLeft,vRight] = calcSpeeds(i,x,y,theta,0,0);
            elseif i==3
                [vLeft,vRight] = calcSpeeds(i,x,y,theta,0,1);
            end
            
            sendSpeeds(s,i,vLeft,vRight);
            sprintf('Robot=%.1f , x=%.2f , y=%.2f, t=%.2f, vLeft=%.1f , vRight=%.1f',i,x,y,theta,vLeft,vRight)
            
        end
    end
    pause(0.2);
    if(mod(t,10)==0)
        drawnow
    end
end