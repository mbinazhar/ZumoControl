dt = 0.2;

t = 2;
while t>0
    t = t+1;
    
%         figure(2);
%         grid on;
%         for sensorNo = 1:1:5
%             sensorValue = 
% 
%             plot(markX,markY,strcat(sensorColors(sensorValue),sensorMarkers(sensorNo)));
%         end
%         v=1;w=0;

%         speedInSonarDirection = (sonar - .4)/3;
        
%         v = sonarCloseness(3) + .5*sonarCloseness(2) + .5*sonarCloseness(4);

% 
%         v = speedInSonarDirection(3)
%         if(v>.5)
%             v=0.5;
%         end
        fixedW=0.5;
        fixedV=0.2;
        
        if(sonar(3) < 0.45)
            v=0;
        else
            v=fixedV;
        end

        if(sonar(2) < 0.35) && (sonar(4) < 0.35) % LEFT DIAGONAL SENSOR
            w=0;v=-0.08;
        elseif (sonar(2) < 0.45)
            w= -fixedW;v=0;
        elseif (sonar(4) < 0.45)
            w= fixedW;v=0;
        else
            w=0;
        end
%         w = sonarCloseness(1) - .5*sonarCloseness(2) - .5*sonarCloseness(4);
        sprintf('Sonar=%.1f,%.1f,%.1f,%.1f,%.1f , v=%.2f, w=%.1f',sonar,v,w)
        TwistvelocityPublish(v,w);
        
    pause(dt); 
end
    
    
        
        