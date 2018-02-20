UT_theta=[-90 -35 0 35 90];
UT_theta=deg2rad(UT_theta);

while(1)
pause(0.2);

for UTsensor = 1:1:5
        if UTsensor ==1
            plot([0 lidar(UTsensor)*cos(UT_theta(UTsensor)+pi/2)], [0 lidar(UTsensor)*sin(UT_theta(UTsensor)+pi/2)] ,'r' );
        axis([-5 5 -.1 10]); axis square;hold on;
        else
        plot([0 lidar(UTsensor)*cos(UT_theta(UTsensor)+pi/2)], [0 lidar(UTsensor)*sin(UT_theta(UTsensor)+pi/2)] ,'r' );
        end
        end
        hold off;
        
end