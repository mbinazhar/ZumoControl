global zumoSensors
global zumoPose

selectedRobot=3;

initZigbee

color = 'kbgrcmy'; colorVal=1;

clear calcSpeeds
clear calcSpeedsWhileMoving

%     x_goal=.65;     y_goal=.25;
    
    x_goal=-.5;    y_goal= -.2 ;
    

figure(2);grid on;
axis([-1 1 -1 1]); axis square;
figure(1);
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



X = 1;
Y = 2;
THETA = 3;
THETA_TOLERANCE = pi/20.0;
GOAL_TOLERANCE  = 0.05;
GOAL_PAUSE = 2;
point = 1;
dt = 0.2;


%Parameters to be initialized
number_of_UT_sensors=4; 
simulation_time=3000;
v=zeros(1,simulation_time)*5;   %constant linear velocity
u=zeros(1,simulation_time);
u_Vec=[];
u_bar=3;

%New variable definition in ver # 4
PROGRESS_POINT_x=zumoPose(selectedRobot,X); %Initial x-postion of P3DX in VREP
PROGRESS_POINT_y=zumoPose(selectedRobot,Y);  %Initial y-postion of P3DX in VREP
PROGRESS_MADE=1;

%all the angles are in degrees
UT_theta=[90 , 5 , -5 ,  -90];
UT_theta=UT_theta.*180/pi;  %conversion from degrees to radians

UT_x_loc=[-.05 , 0  , 0 , .05];
UT_y_loc=[ 0   , .05,.05,  0 ];

d0=2;

distance_to_obstacle_UT=zeros(number_of_UT_sensors,simulation_time);
distance_to_obstacle_UT_for_OA=ones(number_of_UT_sensors,simulation_time);

% distance_to_obstacle=zeros(1,simulation_time);
% distance_to_obstacle_Vec=[];
distance_derivative=zeros(1,simulation_time);
error=zeros(1,simulation_time);
dt=0.25;   %sampling time is considered to be 0.25 second


for i=1:simulation_time
%     i
%     PROGRESS_MADE
    %Transformation from unicycle to differntial drive
    if i<simulation_time
        velocity=Transform_UC_DD([u(i) v(i)]); %
        vR=velocity(1);
        vL=velocity(2);
    elseif i==simulation_time
        vR=0;
        vL=0;
    end

    %% SCALE VELOCITIES HERE FIRST
    z = selectedRobot;
    
    scalingFactor = 1000;
    vLeft = scalingFactor * vL;
    vRight = scalingFactor * vR;
    
    sendSpeedsCharacterWise(s,z,round(vLeft),round(vRight));
    
    %% ADD SENSOR CODE HERE
        sensorAngles =          [90 , 5 , -5 ,  -90;...
                                 90 , 7 , -7 ,  -90;...
                                 90 , 12 , -12 ,-90]; 
        sensorMarkers =         ['<' , 'o' , 'x' , '>'];
        sensorColors =          ['g' , 'b' , 'r'];
        sensorDistances =       [0.15 , 0.10 , 0.06;...
                                 0.18 , 0.11 , 0.07;...
                                 0.18 , 0.11 , 0.07;...
                                 0.15 , 0.10 , 0.06];
                             sensorDistances = sensorDistances';
                             sensorDistances = sensorDistances + 0.04;
                            
        for sensorNo = 1:1:4
            sensorValue = zumoSensors(z,sensorNo);
            if sensorValue == 0
                continue;
            end
            
            x=zumoPose(z,X);
            y=zumoPose(z,Y);
            theta=zumoPose(z,THETA);
            
            markX = x + sensorDistances(sensorValue,sensorNo) * cos(theta+deg2rad(sensorAngles(sensorValue,sensorNo)));
            markY = y + sensorDistances(sensorValue,sensorNo) * sin(theta+deg2rad(sensorAngles(sensorValue,sensorNo)));
            plot(markX,markY,strcat(sensorColors(sensorValue),sensorMarkers(sensorNo)));
        end
        
        %%
%     distance_to_obstacle_UT(1,i)= 4-zumoSensors(z,1);
%     distance_to_obstacle_UT(2,i)= 4-zumoSensors(z,2);
%     distance_to_obstacle_UT(3,i)= 4-zumoSensors(z,3);
%     distance_to_obstacle_UT(4,i)= 4-zumoSensors(z,4);

    for UTsensor = 1:1:4
        temp= zumoSensors(z,UTsensor);
        if temp == 0
            distance_to_obstacle_UT(UTsensor,i)= 0;
        else
            distance_to_obstacle_UT(UTsensor,i)= 4-temp;
        end
    end
    

    
    robot_x_pos(i)= zumoPose(z,X);
    robot_y_pos(i)= zumoPose(z,Y);
    robot_theta(i)= zumoPose(z,THETA);

    addpoints(robotTrajectory(z),robot_x_pos(i),robot_y_pos(i));

    sprintf('Robot=%.1f , x=%.2f , y=%.2f, t=%.2f, vLeft=%.1f , vRight=%.1f , Sens=%s',z,robot_x_pos(i),robot_y_pos(i),robot_theta(i),vLeft,vRight,num2str(zumoSensors(z,:)))
    

    
    if (i>2)
        DISTANCE_TO_GOAL_FROM_PROGRESS_POINT=sqrt((PROGRESS_POINT_x-x_goal)^2+(PROGRESS_POINT_y-y_goal)^2);
        DISTANCE_TO_GOAL_FROM_CURRENT_POSITION=sqrt((robot_x_pos(i)-x_goal)^2+(robot_y_pos(i)-y_goal)^2);
        
        if DISTANCE_TO_GOAL_FROM_CURRENT_POSITION <= DISTANCE_TO_GOAL_FROM_PROGRESS_POINT
            PROGRESS_POINT_x=robot_x_pos(i);
            PROGRESS_POINT_y=robot_y_pos(i);
        elseif DISTANCE_TO_GOAL_FROM_CURRENT_POSITION > DISTANCE_TO_GOAL_FROM_PROGRESS_POINT %if no progress is made then don't update the progress point
            PROGRESS_POINT_x=PROGRESS_POINT_x;
            PROGRESS_POINT_y=PROGRESS_POINT_y;
        end
        
        if DISTANCE_TO_GOAL_FROM_CURRENT_POSITION<DISTANCE_TO_GOAL_FROM_PROGRESS_POINT
            PROGRESS_MADE=1;
        elseif DISTANCE_TO_GOAL_FROM_CURRENT_POSITION>=DISTANCE_TO_GOAL_FROM_PROGRESS_POINT
            PROGRESS_MADE=0;
        end
    end
    
    
%-----------------------------------------------------------------------
    %       Coding the FSM for navigation
    %------------------------------------------------------------------------
    %     sum(b                       (b                      (:,4) ~= 0,4)<0.3)
    %     sum (distance_to_obstacle_UT(distance_to_obstacle_UT (:,i)~=0,i)<0.3)
     % Thresholds
    
    DISTANCE_FOR_ACTIVATING_OA_GTG=2.5;
    DISTANCE_FOR_ACTIVATING_DANGER_OA=1.5;
    %DISTANCE_FOR_ACTIVATING_DANGER_OA<0.5;
    
    
    
    if  (abs(robot_x_pos(i)-x_goal)<0.05) && (abs(robot_y_pos(i)-y_goal)<0.05)
        token=-1;
        disp('I am at the goal');
    elseif sum(distance_to_obstacle_UT(:,i))==0  %i.e. there is no obstacle
        token=0; %'GTG'
        disp('I am in GTG');
    elseif sum(distance_to_obstacle_UT(:,i))~=0 && sum (distance_to_obstacle_UT(distance_to_obstacle_UT (:,i)~=0,i)<DISTANCE_FOR_ACTIVATING_OA_GTG)==0
        %         %The first condition implies that if any one of the UT sensors have
        %         %detetcted an obstacle then the condition is TRUE. The second
        %         %condition says that remove all the ZEROS from UT sensors data
        %         %array and make sure that the robot's minimum distance to the obstacle is greater than DISTANCE_FOR_ACTIVATING_OA_GTG. If
        %         %the robot move closer (i.e.<DISTANCE_FOR_ACTIVATING_OA_GTG) to the obstacle then this condition is FALSE
        token=1; %'OA_GTG';
        disp('I am in OA_GTG');
    elseif sum(distance_to_obstacle_UT(:,i))~=0 && sum (distance_to_obstacle_UT(distance_to_obstacle_UT (:,i)~=0,i)<DISTANCE_FOR_ACTIVATING_OA_GTG)~=0  && sum (distance_to_obstacle_UT(distance_to_obstacle_UT (:,i)~=0,i)<DISTANCE_FOR_ACTIVATING_DANGER_OA)==0
        %The first condition is the same as in the previous case. 
        % The second consition says that 
        % the robot has approached near the obstacle i.e. any one of the UT sensors reports a
        % DISTANCE_FOR_ACTIVATING_DANGER_OA<distance < DISTANCE_FOR_ACTIVATING_OA_GTG
%         if PROGRESS_MADE==1
%             token=2; %'OA';
%             disp('I am in OA');
%         elseif PROGRESS_MADE==0
%             token=4;
%             disp('I am in Follow_Wall');
%         end
             token=2; %'OA';
             disp('I am in OA');
    elseif sum(distance_to_obstacle_UT(:,i))~=0 && sum (distance_to_obstacle_UT(distance_to_obstacle_UT (:,i)~=0,i)<DISTANCE_FOR_ACTIVATING_DANGER_OA)~=0
        %The first condition is the same as in the first case.
        % the robot has approached very near to the obstacle i.e. any one of the UT sensors reports a 
        % distance< DISTANCE_FOR_ACTIVATING_DANGER_OA
        
        
        if PROGRESS_MADE==1
            token=3; %'Danger_OA';
            disp('I am in Danger_OA');
        elseif PROGRESS_MADE==0
            token=4;
            disp('I am in Follow_Wall');
        end
          
    end

    %--------------------------------------------------------------------
    % Go-to-Goal Code
    %--------------------------------------------------------------------
    %     if sum(distance_to_obstacle_UT(:,i))==0  %i.e. there is no obstacle
    
    
    %Go to Goal Vector
    
    GTG_x_resultant(i)=-robot_x_pos(i)+x_goal;
    GTG_y_resultant(i)=-robot_y_pos(i)+y_goal;
    GTG_vector(:,i)=[GTG_x_resultant(i);GTG_y_resultant(i)];
    
    %Normalizing GTG vectors
    GTG_vector_magnitude=norm(GTG_vector(:,i));
    GTG_norm_vector(:,i)=GTG_vector(:,i)./GTG_vector_magnitude;
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    %                Obstacle Avoidance Code
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    %     elseif sum(distance_to_obstacle_UT(:,i))~=0 %there is some obstacle on my way
    
    %take all the distances from the UT sensor and transform them from:
    %a) sensor's frame to robot's frame
    %b) from robot's frame to world frame
    
    %Transform from sensor's frame to robot's frame
    
    %         UT_x_loc
    %         UT_y_loc
    
    %     distance_to_obstacle_UT_for_OA=distance_to_obstacle_UT;
    
    for k=1:number_of_UT_sensors
        R1=[cos(robot_theta(i)) -sin(robot_theta(i))  robot_x_pos(i);
            sin(robot_theta(i))  cos(robot_theta(i))   robot_y_pos(i);
            0                 0                1];
        R2=[cos(UT_theta(k)) -sin(UT_theta(k)) UT_x_loc(k);
            sin(UT_theta(k))  cos(UT_theta(k)) UT_y_loc(k);
            0                 0                1];
        
        
        OA_UT_vectors_WF(:,k)=R1*R2*[distance_to_obstacle_UT(k,i),0,1]';
        
    end
    
    %Calculate a vector from the transformed point to the robot's center
    UT_sensor_gains = [1 0.5 0.5 1];
    u_i = (OA_UT_vectors_WF(1:2,:)-repmat([ robot_x_pos(i); robot_y_pos(i)],1,number_of_UT_sensors))*diag(UT_sensor_gains);
    %      u_ao = sum(u_i,2);
    
    
    
    %     OA_UT_vectors_WF
    OA_x_resultant(i)=sum(u_i(1,:));
    OA_y_resultant(i)=sum(u_i(2,:));
    
    OA_vector(:,i)=[OA_x_resultant(i);OA_y_resultant(i)];
    
    %Normalizing OA vectors
    OA_vector_magnitude=norm(OA_vector(:,i));
    OA_norm_vector(:,i)=OA_vector(:,i)./OA_vector_magnitude;
    
    
    
    
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    %                      Blended Controller
    %-------------------------------------------------------------------
    %--------------------------------------------------------------------
    alpha=0.75;
   
    
    OA_GTG_vector(:,i)=alpha*OA_norm_vector(:,i)+(1-alpha)*GTG_norm_vector(:,i);
    
    
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    %                Follow Wall (version # 4) complete navigation scheme
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    ufw_RIGHT=zeros(2,1);
    ufw_LEFT=zeros(2,1);
    
    %%
    if token==4
        index=1;
        
        %Computing p_vector for ufw_l
        %         if ((cond1 && cond2) || (cond2 && cond3))
        %the wall is on the left
        %             disp('I am in case 1 token 4');
        % Calculating p1 and p2 vector
        for k=1:1:2
            
            R1=[cos(robot_theta(i)) -sin(robot_theta(i))  robot_x_pos(i);
                sin(robot_theta(i))  cos(robot_theta(i))   robot_y_pos(i);
                0                 0                1];
            R2=[cos(UT_theta(k)) -sin(UT_theta(k)) UT_x_loc(k);
                sin(UT_theta(k))  cos(UT_theta(k)) UT_y_loc(k);
                0                 0                1];
            
            p_vector_LEFT(:,index)=R1*R2*[distance_to_obstacle_UT(k,i),0,1]'; %p_vector=[p1 p2];
            index=index+1;
        end
        
        
        %         elseif ((cond4 && cond5) || (cond5 && cond6))
        %the wall is on the right
        % Calculating p1 and p2 vector
        %             disp('I am in case  2 token 4');
        for k=3:1:4
            R1=[cos(robot_theta(i)) -sin(robot_theta(i))  robot_x_pos(i);
                sin(robot_theta(i))  cos(robot_theta(i))   robot_y_pos(i);
                0                 0                1];
            R2=[cos(UT_theta(k)) -sin(UT_theta(k)) UT_x_loc(k);
                sin(UT_theta(k))  cos(UT_theta(k)) UT_y_loc(k);
                0                 0                1];
            
            p_vector_RIGHT(:,index)=R1*R2*[distance_to_obstacle_UT(k,i),0,1]'; %p_vectro=[p1 p2];
            index=index+1;
        end
        
        %         end
        p1_LEFT=p_vector_LEFT(1:2,1);
        p2_LEFT=p_vector_LEFT(1:2,2);
        p1_RIGHT=p_vector_RIGHT(1:2,1);
        p2_RIGHT=p_vector_RIGHT(1:2,2);
        
        
        %Calculating p2-p1 for the left hand side
        ufw_t_LEFT=p2_LEFT-p1_LEFT;
        ufw_t_magnitude_LEFT=norm(ufw_t_LEFT);
        ufw_t_norm_vector_LEFT=ufw_t_LEFT/ufw_t_magnitude_LEFT;
        
        %Calculating p2-p1 for the right hand side
        ufw_t_RIGHT=p2_LEFT-p1_RIGHT;
        ufw_t_magnitude_RIGHT=norm(ufw_t_RIGHT);
        ufw_t_norm_vector_RIGHT=ufw_t_RIGHT/ufw_t_magnitude_RIGHT;
        
        
        
        ua_WF_LEFT=p1_LEFT; %in World Frame
        ua_WF_RIGHT=p1_RIGHT; %in World Frame
        
        %Converting the robot's x,y location in World Frame
        R1=[cos(robot_theta(i)) -sin(robot_theta(i))  robot_x_pos(i);
            sin(robot_theta(i))  cos(robot_theta(i))   robot_y_pos(i);
            0                 0                1];
        up=R1*[robot_x_pos(i);robot_y_pos(i);1];
        
        up_WF_LEFT=up(1:2,1);
        up_WF_RIGHT=up(1:2,1);
        
        ufw_p_LEFT=(ua_WF_LEFT-up_WF_LEFT)-((ua_WF_LEFT-up_WF_LEFT)*ufw_t_norm_vector_LEFT')*ufw_t_norm_vector_LEFT;
        ufw_p_RIGHT=(ua_WF_RIGHT-up_WF_RIGHT)-((ua_WF_RIGHT-up_WF_RIGHT)*ufw_t_norm_vector_RIGHT')*ufw_t_norm_vector_RIGHT;
        
        
        dfw=0.5 ;  %minimum distance we want to maintain from the wall
        alphafw=0.5;
        
        ufw_p_magnitude_LEFT=norm(ufw_p_LEFT);
        ufw_p_magnitude_RIGHT=norm(ufw_p_RIGHT);
        
        ufw_p_norm_vector_LEFT=ufw_p_LEFT/ufw_p_magnitude_LEFT;
        ufw_p_norm_vector_RIGHT=ufw_p_RIGHT/ufw_p_magnitude_RIGHT;
        
        
        ufw_p_opp_LEFT=ufw_p_LEFT-dfw*ufw_p_norm_vector_LEFT;
        ufw_p_opp_RIGHT=ufw_p_RIGHT-dfw*ufw_p_norm_vector_RIGHT;
        
        %My modification in the code, Magnus is not normalizing ufw_p_opp
        ufw_p_opp_magnitude_LEFT=norm(ufw_p_opp_LEFT);
        ufw_p_opp_magnitude_RIGHT=norm(ufw_p_opp_RIGHT);
        
        
        ufw_p_opp_norm_vector_LEFT=ufw_p_opp_LEFT/ufw_p_opp_magnitude_LEFT;
        ufw_p_opp_norm_vector_RIGHT=ufw_p_opp_RIGHT/ufw_p_opp_magnitude_RIGHT;
        
        ufw_LEFT=alphafw*ufw_t_norm_vector_LEFT+(1-alphafw)*ufw_p_opp_norm_vector_LEFT;
        ufw_RIGHT=alphafw*ufw_t_norm_vector_RIGHT+(1-alphafw)*ufw_p_opp_norm_vector_RIGHT;
        
        if sum(isnan(ufw_LEFT))>0
            ufw_LEFT=[0;0];
        end
        if sum(isnan(ufw_RIGHT))>0
            ufw_RIGHT=[0;0];
        end
      
        
        sigma_LEFT=inv([GTG_norm_vector(1,i) OA_norm_vector(1,i); GTG_norm_vector(2,i) OA_norm_vector(2,i)])*ufw_LEFT;
        sigma_RIGHT=inv([GTG_norm_vector(1,i) OA_norm_vector(1,i); GTG_norm_vector(2,i) OA_norm_vector(2,i)])*ufw_RIGHT;
        
        if (sigma_LEFT(1)>0 && sigma_LEFT(2)>0)
            ufw=ufw_LEFT;
            disp('I am in follow wall LEFT');
        elseif (sigma_RIGHT(1)>0 && sigma_RIGHT(2)>0)
            ufw=ufw_RIGHT;
            disp('I am in follow wall RIGHT');
        else
            token=3;
            disp('I am not following wall either on the RIGHT or LEFT');
        end
        
    end
    
    %-----------------------------------------------------------------
        %                  Code for plotting the vectors in separate figure
        %-----------------------------------------------------------------

        
        %First vector will be drawn in RED colour [OA_Vector]
        %Fourth vector will be drawn in GREEN colour [GTG_Vector]
        
        figure(2);
        grid on;
        plot([0 OA_norm_vector(1,i)], [0 OA_norm_vector(2,i)] ,'r' );
        axis([-1 1 -1 1]); axis square;hold on;
        plot([0 ufw_RIGHT(1)],[0  ufw_RIGHT(2)], 'k-.'); 
        plot([0 ufw_LEFT(1)] ,[0  ufw_LEFT(2)] , 'k:');     
        plot([0 GTG_norm_vector(1,i)],[0  GTG_norm_vector(2,i)], 'g');
        hold off;
       
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    %          Calculating the error
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    
    %Checking desired robot's pose      %% ****** QUESTION - ARE ALL GOALS
    %NORMALIZED GTG VECTORS ??? CAN BE HANDLED WITH LOW LEVEL GTG ? THIS
    %THETA IS IN ROBOT FRAME ?
    if token==-1
        theta_desired(i)=robot_theta(i);
    elseif token==0
        %         theta_desired(i)=atan2(GTG_y_resultant(i),GTG_x_resultant(i));
        theta_desired(i)=atan2(GTG_norm_vector(2,i),GTG_norm_vector(1,i));
    elseif token==1
        theta_desired(i)=atan2(OA_GTG_vector(2,i),OA_GTG_vector(1,i));
    elseif token==2 
        theta_desired(i)=atan2(OA_norm_vector(2,i),OA_norm_vector(1,i));
     elseif token==3
        theta_desired(i)=robot_theta(i)+pi/4;
        
    elseif token==4
        theta_desired(i)=atan2(ufw(2),ufw(1));
    end
    
    
    theta_desired*pi/180;
    %Calculating errors
    theta_error(i)=theta_desired(i)-robot_theta(i);
    theta_error(i) = atan2(sin(theta_error(i)),cos(theta_error(i)));
    
    %     theta_error_modified(i)=atan2(cos(theta_error(i)),sin(theta_error(i)));
    distance_error(i)=sqrt((robot_y_pos(i)-y_goal)^2+(robot_x_pos(i)-x_goal)^2); % *** COMMENT - BECAUSE THIS IS FINAL GOAL ??? PROGRESS POINT?
    
    
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    %          PID Controller
    %--------------------------------------------------------------------
    %--------------------------------------------------------------------
    
    if abs(theta_error(i))>0.11 && distance_error(i)>5e-2   %If the orientation error is large keep on rotating
        %              disp('I am in case 1');
        u(i+1)=scalingFactor/500 *   6*theta_error(i) ;   %*exp(-abs(theta(error))*i);
        v(i+1)=0.2;
    elseif abs(theta_error(i))<0.11 && distance_error(i)>10e-2
        %             u(i+1)=1*theta_error(i);
        %             v(i+1)=5*(distance_error(i));
        %             disp('I am in case 2');
        u(i+1)=scalingFactor/500 *   4*theta_error(i);
        v(i+1)=0.2;
    elseif abs(theta_error(i))<0.11 && distance_error(i)<8e-2
        %             disp('I am in case 3');
        u(i+1)=0;
        v(i+1)=0;
    end
    
    %--------------------------------------------------------------------
    
    if i==simulation_time-1
        u(i+1)=0;
        v(i+1)=0;
    end
    
    
    pause(dt);
    if(mod(i,10)==0)
        figure(1);
        drawnow
    end
    
end  %end of the for loop
    
    



