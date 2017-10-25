global robots

clear calcSpeeds

x_g = [-.6,-.4,-.1,-.2,-.2,-.2];
y_g = [-.7,-.5,-.1,.2,.5,.5];
figure;
plot(x_g,y_g,'-r','LineWidth',2);
axis square

position = animatedline;
axis([-1.5 1.5 -1.5 1.5])
grid on

tic;
i=2;
while (i<=length(x_g))
   k = find(robots(:,1)==871);
        if isempty(k)
            continue
        else                     
%         addpoints(robotTrajectory(i),robots(k(1,1),2),robots(k(1,1),3));

        x = robots(k(1,1),2);
        y = -robots(k(1,1),3);
        theta = robots(k(1,1),4);
        
[vLeft,vRight] = calcSpeedsWhileMoving(3,x,y,theta,x_g(i),y_g(i));
    formatSpec = 'X=%4.2f, Y=%4.2f, Th=%4.2f, Xg=%4.2f, Yg=%4.2f\n';
    fprintf(formatSpec,x,y,theta,x_g(i),y_g(i));
    
sendSpeeds(s,3,vLeft,vRight);

addpoints(position,x,y);
    drawnow
pause(0.2);
        end

    timeCheck=toc;
    if timeCheck >=3
        tic;
        i=i+1;
    end
    
end