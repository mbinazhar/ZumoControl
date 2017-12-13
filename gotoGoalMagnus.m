global zumoSensors
global zumoPose

initZigbee

color = 'kbgrcmy'; colorVal=1;

X = 1;
Y = 2;
THETA = 3;
THETA_TOLERANCE = pi/20.0;
GOAL_TOLERANCE  = 0.05;
GOAL_PAUSE = 2;
point = 1;
dt = 0.2;



goal = [ .65     .25      0
          -.5     -.2      0];

      MAXPOINTS=size(goal,1);

% robotIds = [314 528 871];
% noOfRobots = size(robotIds,2);
% zumoPose = zeros(noOfRobots,3);
% zumoSensors = zeros(noOfRobots,9);

clear calcSpeeds
clear calcSpeedsWhileMoving

% figure; 
hold on; grid on;
axis([-1.1 1.1 -1.1 1.1]); axis square;



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
        
        %         intermediateGoal = [goal(point,1) , goal(point,2)];
       
        [vLeft,vRight,hasReached] = calcSpeedsWhileMoving(i,x,y,theta,goal(point,1),goal(point,2));
        %% ADD SENSOR CODE HERE
        

        
        if ( zumoSensors(i,1) >= 1)
            vRight = vRight - 75 ;
        end
        

     
        if ( zumoSensors(i,2) == 1)
            vRight = vRight - 85 ;
        end
        if ( zumoSensors(i,2) >= 2)
            vRight = vRight - 105;
        end
        
        
        if ( zumoSensors(i,3) == 1)
            vLeft= vLeft - 85;
            markX = x + cos(theta-deg2rad(12) )*.18
            markY = y + sin(theta-deg2rad(12))*.18
            plot(markX,markY,'bx')
        end
        if ( zumoSensors(i,3) >= 2)
            vLeft= vLeft - 105;
            markX = x + cos(theta-deg2rad(5))*.09
            markY = y + sin(theta-deg2rad(5))*.09
            plot(markX,markY,'rx')
        end
        % 1 = 0.18 m
        % 2 = 0.11 m
        % 3 = 0.07 m
        
        
        
          if ( zumoSensors(i,4) >= 1)
            vLeft = vLeft - 75;
            
            if ( zumoSensors(i,4) == 1)
            markX = x + cos(theta-deg2rad(5))*.13
            markY = y + sin(theta-deg2rad(5))*.13
            plot(markX,markY,'bo')
            elseif ( zumoSensors(i,4) >= 2)
            markX = x + cos(theta-deg2rad(5))*.07
            markY = y + sin(theta-deg2rad(5))*.07
            plot(markX,markY,'ro')
            end
            
          end
        
        % 1 = 0.13 m
        % 2 = 0.10 m
        % 3 = 0.05 m

        %%
        sendSpeedsCharacterWise(s,i,round(vLeft),round(vRight));
        
        sprintf('Robot=%.1f , x=%.2f , y=%.2f, t=%.2f, vLeft=%.1f , vRight=%.1f',i,x,y,theta,vLeft,vRight)
        disp(zumoSensors(i,:));
                    
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
    
    
        
        