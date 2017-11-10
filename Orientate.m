
global robots

robotIds = [528 314 871];
[~,numberOfRobots] =size(robotIds);

Kp =12*3;
Ki =8*3;
Kd =5*3;

    E_k    = [0;0;0];
    e_k_1 = [0;0;0];
    
dt=0.20; %200ms

t=1;
while t<5*5
    t = t+1;
    for i = 1:numberOfRobots
        k = find(robots(:,1)==robotIds(1,i));
        if isempty(k)
            continue
        else

            x = robots(k(1,1),2);
            y = -robots(k(1,1),3);
            theta = robots(k(1,1),4);
            
            rad2deg(theta)
            
            e_k = deg2rad(45) - theta;
            
            e_P = e_k;
            e_I = E_k(i) + e_k*dt;
            e_D = (e_k-e_k_1(i))/dt;
            
            w = Kp*e_P + Ki*e_I + Kd*e_D;
            
            E_k(i) = e_I;
            e_k_1(i) = e_k;
            
            if abs (e_k)  > deg2rad(3)
                vRot = w;
                vLeft =  round(-vRot);
                vRight = round(vRot);
            else
                vLeft =0;vRight =  0;
            end
            
            if( vLeft > 127)
    vLeft = 127;
end
if( vLeft < -127)
    vLeft = -127;
end
if( vRight > 127)
    vRight = 127;
end
if( vRight < -127)
    vRight = -127;
end

if abs(vLeft) < 10
    vLeft=0;
end
if abs(vRight) < 10
    vLeft=0;
end

                
            if i==1
                sendSpeedsCharacterWise(s,i,vLeft,vRight);
            end
            if i==2
                sendSpeedsCharacterWise(s,i,vLeft,vRight);
            end
            if i==3
                sendSpeedsCharacterWise(s,i,vLeft,vRight);
            end

            
        end
    end
    pause(0.2);

end