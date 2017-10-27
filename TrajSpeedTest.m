global robots

clear calcSpeeds
clear calcSpeedsWhileMoving

color = 'kbgrcmy'; colorVal=1;

robotIds = [528 314 871];
[~,numberOfRobots] =size(robotIds);

x_g = [-.5,-.3,0,-.1,-.1,-.1];
x_g = [x_g ; x_g+.4 ; x_g+0.8];


y_g = [-.5,-.2,.1,.5,.7,.7];
figure;
plot(x_g(1,:),y_g,'-.r','LineWidth',2);
hold on
plot(x_g(2,:),y_g,'-.c','LineWidth',2);
plot(x_g(3,:),y_g,'-.y','LineWidth',2);

axis square
axis([-1.1 1.1 -1.1 1.1])
grid on


robotTrajectory = [animatedline('Color',color(colorVal),'LineWidth',1)];
for i = 1: numberOfRobots-1
    colorVal = colorVal+1;
    if(colorVal>7)
        colorVal=1;
    end
    robotTrajectory = [robotTrajectory;animatedline('Color',color(colorVal),'LineWidth',1)];
end
drawnow

tic;
wayPoint=2;
while (wayPoint<=length(y_g))
    for i = 1:numberOfRobots
        k = find(robots(:,1)==robotIds(1,i));
%         k = find(robots(:,1)==871);
        if isempty(k)
            continue
        else
            x = robots(k(1,1),2);
            y = -robots(k(1,1),3);
            theta = robots(k(1,1),4);
            
            addpoints(robotTrajectory(i),x,y);
            
            [vLeft,vRight] = calcSpeedsWhileMoving(i,x,y,theta,x_g(i,wayPoint),y_g(wayPoint));
%             [vLeft,vRight] = calcSpeedsWhileMoving(i,x,y,theta,0,-1);
            formatSpec = 'Robot=%1.0f X=%4.2f, Y=%4.2f, Th=%4.2f, Xg=%4.2f, Yg=%4.2f\n';
            
            
%             if i==3
            sendSpeeds(s,i,vLeft,vRight);
            fprintf(formatSpec,i,x,y,theta,x_g(wayPoint),y_g(wayPoint));
%             end
            
            
        end
        end
        drawnow
        pause(0.2);
            
        timeCheck=toc;
        if timeCheck >=3
            tic;
            wayPoint=wayPoint+1;
        end
        
    
    
end