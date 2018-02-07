% function must be called after every dt = sample time = 2s , i.e, at the GPS data rate.

function [x,y] = myfunction2(lat,long,hgt,lat0,lon0,hgt0,ax,ay,az,phi,theta,psi,x_od,y_od,vl,vr,r_wheel,angle,time,dt,k)

% MATLAB Code for online Robot Localization:

%   Obtain the values for all positioning sensors:
%   ==============================================

% 1. Accelerometers:
% =================

% All accelerometer values are in units of m/s^2.So no conversion module is
% needed

acc_X = ax;
acc_Y = ay;
acc_Z = az;

% 2. Gyroscope:
% =============

% Euler Angles
R = phi;   % roll angle
P = theta; % pitch angle
Y = psi;   %yaw angle

% Odometer :
x_odom = x_od;
y_odom = y_od;
theta_od = angle;
v_left= vl;
v_right = vr;
wheel_radius = r_wheel;

% GPS Based Positions

% convert from geodetic to ENU co-ordinates using wgs84Ellipsoidal model:

[xEast,yNorth,zUp] = geodetic2enu(lat,long,hgt,lat0,lon0,hgt0,wgs84Ellipsoid);
x_gps = xEast;
y_gps = yNorth;
z_gps = zUp;


                                        %  IMU
                                        %  ===
                                        
% Converting IMU Data from BODY frame to ENU frame:

% Euler Angles:
% =============
Y = psi;  %
R =phi;
P = theta;
yaw_angle = psi;

% Calculate the DCM/Rotation (Transformation) Matrix:
% ==================================================
R11 =cos(Y)*cos(R)-sin(Y)*sin(P)*sin(R);
R12 =-sin(Y)*cos(P);
R13 = cos(Y)*sin(R)+sin(Y)*sin(P)*cos(R)
R21 = sin(Y)*cos(R)+cos(Y)*sin(P)*sin(R)
R22 = cos(Y)*cos(P)
R23 =sin(Y)*sin(R)-cos(Y)*sin(P)*cos(R)
R31 =-cos(P)*sin(R) 
R32 = sin(P) 
R33 =cos(P)*cos(R) ;

% Accelerometer velocities converted to ENU

aE1 = [R11 R12 R13]*[acc_X acc_Y acc_Z]';
aN1 = [R21 R22 R23]*[acc_X acc_Y acc_Z]';
aU1 = [R31 R32 R33]*[acc_X acc_Y acc_Z]' ;  


% IMU Velocity:
% ============
          
          vE1= aE1*dt;
          vN1= aN1*dt;
          vU1= aU1*dt;


V_acc = sqrt(vE1.^2+vN1.^2);
 
                                    % Odometer:
                                    % =========
 
V_od = (vl+vr)/2;
Vx = V_od*cos(theta_od);
Vy = V_od*sin(theta_od);
Vz = 0;

% Convert body frame velocities to ENU co-ordinates

 vE2 =[R11 R12 R13]*[Vx Vy Vz]';
 vN2 = [R21 R22 R23]*[Vx Vy Vz]';
 vU2 =[R31 R32 R33]*[Vx Vy Vz]' ;
%  
%  V_od = sqrt(vE2.^2+vN2.^2);
                                    
                                    
                                % Kalman Filters:
                                % ==============
                        
                    % Design kalman filter: (on GPS & IMU data):
                    % ========================================
                    
% Designing of the 11-state kalman filter:
% =======================================

% Set the Matrices Prior To Starting the Filter:
% Q,R,H

% 1. Q = Process Noise Covariance Matrix:
% ======================================

Q1 = 0.0001* eye(11);

% 2. R = Measurement Noise Covariance Matrix:
% ==========================================

% var_aN = var(aN);
% var_aE = var(aE);
% var_aD = var(aD);
% var_lambda = var(lambda);
% var_mu = var(mu);
% D = [var_aN var_aE var_aD var_lambda var_mu];
% R = diag(D)

R1 = 0.001*eye(5); % because total sensor readings = 5.

% 3. H = Observation Matrix:
% =========================

H1 = [1 0 0 0 0 0 0 0 0 0 0  ;
     0 1 0 0 0 0 0 0 0 0 0  ;
     0 0 0 0 0 0 1 0 0 0 0  ;
     0 0 0 0 0 0 0 1 0 0 0  ;
     0 0 0 0 0 0 0 0 0 1 0  ];
 
% Initialize the matrix Z

Z1 = [aE1 aN1  x_gps y_gps yaw_angle];
 
 % 1. Initialize the Filter:

x_01 = [0 0 0 0 0 0 0 0 0 0 0]';
P_01 = Q1;

% Run the Kalman Filter To Fuse The Data:

% dt = 0.1; % sample time for the data

% Compute matrix A:(The 11x11 system matrix)

% A1= [1           0           0           0   0   0   0 0 0 0     0;
%     0           1            0           0   0   0   0 0 0 0     0;
%     0           0            1           0   0   0   0 0 0 0     0;
%     dt          0            0           1   0   0   0 0 0 0     0;
%     0           dt           0           0   1   0   0 0 0 0     0;
%     0           0          dt            0   0   1   0 0 0 0     0;
%     0.5*dt^2    0           0            dt  0   0   1 0 0 0     0;
%     0        0.5*dt^2       0            0   dt  0   0 1 0 0     0;
%     0    0                 0.5*dt^2      0   0  dt   0 0 1 0     0;
%     0    0                  0            0   0   0   0 0 0 1     0;
%     0    0                  0            0   0   0   0 0 0 1/dt  1]
    
                    % Design kalman filter: (for Odometer & IMU data):
                    % ==============================================
                    
% Designing of the 7-state kalman filter:
% =======================================

% Set the Matrices Prior To Starting the Filter:
% Q,R,H

% 1. Q = Process Noise Covariance Matrix:
% ======================================

Q2 = 0.001* eye(7);

% 2. R = Measurement Noise Covariance Matrix:
% ==========================================

% var_aN = var(aN);
% var_aE = var(aE);
% var_aD = var(aD);
% var_lambda = var(lambda);
% var_mu = var(mu);
% D = [var_aN var_aE var_aD var_lambda var_mu];
% R = diag(D)

R2 = 0.01*eye(5); % because total sensor readings = 4.

% 3. H = Observation Matrix:
% =========================

H2 = [1 0 0 0 0 0 0   ;
      0 1 0 0 0 0 0   ;
      0 0 0 0 1 0 0   ;
      0 0 0 0 0 1 0   ;
      0 0 0 0 0 0 1  ];
 
% Initialize the matrix Z

Z2 = [vE1 vN1 yaw_angle x_gps y_gps];
 
 % 1. Initialize the Filter:

x_02 = [0 0 0 0 0 0 0 ]';
P_02 = Q2;

% Run the Kalman Filter To Fuse The Data:

%dt = 0.1; % sample time for the data

% Compute matrix A:(The 11x11 system matrix)

% A2= [1           0           0           0            0   0   0;
%     0           1           0           0            0   0   0;
%     1/dt        0           1           0            0   0   0;
%     0           dt          0           1            0   0   0;
%     0           0           0           0            1   0   0;
%     dt          0          0.5*dt^2     0            0   1   0;
%     0          dt            0         0.5*dt^2      0   0   1];
   
                    % Design the Neural Networks:
                    % ==========================

% MLP Training :
% ==============

% x_MLP = 0;
% y_MLP = 0;
% Number of Hidden Layers Neurons:
HL1 = 17;
HL2 = 15;
% Number of Output Layers:
OL = 1;

            % Pre-allocation of MLP input and output variables:
            % ================================================
% aE_test = zeros();          % Accelerometer (INS) : input
% aN_test = zeros();          % Accelerometer (INS) : input
% aU_test = zeros();          % Accelerometer (INS) : input
% cum_aE_test =zeros();       % Accelerometer (INS) : input
% cum_aN_test =zeros();       % Accelerometer (INS) : input
% cum_aU_test=zeros();        % Accelerometer (INS) : input
% vE_test = zeros();          % Odometer : input
% vN_test =zeros();           % Odometer : input
% vU_test =zeros();           % Odometer : input
% cum_vE_test =zeros();       % Odometer : input
% cum_vN_test =zeros();       % Odometer : input
% cum_vU_test=zeros();        % Odometer : input
% theta_test=zeros();         % Gyroscope: input
% cum_theta_test = zeros();   % Gyroscope: input
% x_saved = zeros();          % Complementary filter : output
% y_saved = zeros();          % Complementary filter : output

% =========================================================

                        % Values of the fusion parameters:
                        % ===============================
 % For GPS on,
 % since GPS is on assign the similar weights to the final output:

a1= 0.5; 
a2 = 1-a1;

                        % Design of HPF for velocity data:
                        % ===============================

% Accelerometer:                        
[b,a] =butter(2,55/100, 'high');

% Start the two Kalman Filters:

% KF1 = GPS+INS        ------- 1
% KF2 = GPS+Odometer   ------- 2

% % for INS
% x_hat_minus_KF1 = x_01;  
% P_hat_minus_KF1 = P_01;
% % for odometer
% x_hat_minus_KF2 = x_02;  
% P_hat_minus_KF2 = P_02;


 if x_gps~=0 % if GPS is on
     if k==1 % if it is first iteration
         
% for INS
x_hat_minus_KF1 = x_01;  
P_hat_minus_KF1 = P_01;
% for odometer
x_hat_minus_KF2 = x_02;  
P_hat_minus_KF2 = P_02;
 
 % 1. Predict State:
 A1= [1           0           0           0   0   0   0 0 0 0     0;
    0           1            0           0   0   0   0 0 0 0     0;
    0           0            1           0   0   0   0 0 0 0     0;
    dt          0            0           1   0   0   0 0 0 0     0;
    0           dt           0           0   1   0   0 0 0 0     0;
    0           0          dt            0   0   1   0 0 0 0     0;
    0.5*dt^2    0           0            dt  0   0   1 0 0 0     0;
    0        0.5*dt^2       0            0   dt  0   0 1 0 0     0;
    0    0                 0.5*dt^2      0   0  dt   0 0 1 0     0;
    0    0                  0            0   0   0   0 0 0 1     0;
    0    0                  0            0   0   0   0 0 0 1/dt  1]

 x_hat_minus_KF1 = A1*x_hat_minus_KF1;% INS:
 A2= [1           0           0           0            0   0   0;
    0           1           0           0            0   0   0;
    1/dt        0           1           0            0   0   0;
    0           dt          0           1            0   0   0;
    0           0           0           0            1   0   0;
    dt          0          0.5*dt^2     0            0   1   0;
    0          dt            0         0.5*dt^2      0   0   1];
 x_hat_minus_KF2 = A2*x_hat_minus_KF2;% Odometer:     
 
 % 2. Predict Error Covariance:
 P_hat_minus_KF1 = A1*P_hat_minus_KF1*A1'+Q1;% INS:
 P_hat_minus_KF2 = A2*P_hat_minus_KF2*A2'+Q2;% Odometer: 
 
 % 3. Compute Kalman Gain:
 K1 = (P_hat_minus_KF1*H1')/(H1*P_hat_minus_KF1*H1'+R1);% INS:
 K2 = (P_hat_minus_KF2*H2')/(H2*P_hat_minus_KF2*H2'+R2);% Odometer: 
 
 % 4. Compute the Estimate:
 z1 = [aE1 aN1  x_gps y_gps yaw_angle]';% INS:
 x_hat1 = x_hat_minus_KF1+K1*(z1-H1*x_hat_minus_KF1);
 z2 = [vE1 vN1  yaw_angle x_gps y_gps ]';% Odometer: 
 x_hat2 = x_hat_minus_KF2+K2*(z2-H2*x_hat_minus_KF2);
 
 % Compute the Error Covarance:
 P_hat1 = P_hat_minus_KF1-K1*H1*P_hat_minus_KF1;% INS:
 P_hat2 = P_hat_minus_KF2-K2*H2*P_hat_minus_KF2;% Odometer:
 
 % Preserve the values for next pass
 
 x_hat_minus_KF1 = x_hat1; % INS:
 P_hat_minus_KF1 = P_hat1;
 
 x_hat_minus_KF2 = x_hat2; % Odometer: 
 P_hat_minus_KF2 = P_hat2;
 
     else
         
         % 1. Predict State:
 A1= [1           0           0           0   0   0   0 0 0 0    0;
    0           1            0           0   0   0   0 0 0 0     0;
    0           0            1           0   0   0   0 0 0 0     0;
    dt          0            0           1   0   0   0 0 0 0     0;
    0           dt           0           0   1   0   0 0 0 0     0;
    0           0          dt            0   0   1   0 0 0 0     0;
    0.5*dt^2    0           0            dt  0   0   1 0 0 0     0;
    0        0.5*dt^2       0            0   dt  0   0 1 0 0     0;
    0    0                 0.5*dt^2      0   0  dt   0 0 1 0     0;
    0    0                  0            0   0   0   0 0 0 1     0;
    0    0                  0            0   0   0   0 0 0 1/dt  1]

 x_hat_minus_KF1 = A1*x_hat_minus_KF1;% INS:
 A2= [1           0           0           0            0   0   0;
    0           1           0           0            0   0   0;
    1/dt        0           1           0            0   0   0;
    0           dt          0           1            0   0   0;
    0           0           0           0            1   0   0;
    dt          0          0.5*dt^2     0            0   1   0;
    0          dt            0         0.5*dt^2      0   0   1];
 x_hat_minus_KF2 = A2*x_hat_minus_KF2;% Odometer:     
 
 % 2. Predict Error Covariance:
 P_hat_minus_KF1 = A1*P_hat_minus_KF1*A1'+Q1;% INS:
 P_hat_minus_KF2 = A2*P_hat_minus_KF2*A2'+Q2;% Odometer: 
 
 % 3. Compute Kalman Gain:
 K1 = (P_hat_minus_KF1*H1')/(H1*P_hat_minus_KF1*H1'+R1);% INS:
 K2 = (P_hat_minus_KF2*H2')/(H2*P_hat_minus_KF2*H2'+R2);% Odometer: 
 
 % 4. Compute the Estimate:
 z1 = [aE1 aN1  x_gps y_gps yaw_angle]';% INS:
 x_hat1 = x_hat_minus_KF1+K1*(z1-H1*x_hat_minus_KF1);
 z2 = [vE1 vN1  yaw_angle x_gps y_gps ]';% Odometer: 
 x_hat2 = x_hat_minus_KF2+K2*(z2-H2*x_hat_minus_KF2);
 
 % Compute the Error Covarance:
 P_hat1 = P_hat_minus_KF1-K1*H1*P_hat_minus_KF1;% INS:
 P_hat2 = P_hat_minus_KF2-K2*H2*P_hat_minus_KF2;% Odometer:
 
 % Preserve the values for next pass
 
 x_hat_minus_KF1 = x_hat1; % INS:
 P_hat_minus_KF1 = P_hat1;
 
 x_hat_minus_KF2 = x_hat2; % Odometer: 
 P_hat_minus_KF2 = P_hat2;
     end
 
 % Save Values for Estimated X position and Y position
 x_E1 = x_hat1(7);% INS:
 y_N1 = x_hat1(8);
 
 x_E2 = x_hat2(6);% Odometer: 
 y_N2 = x_hat2(7);
 
% Fuse the KF1 & KF2 Outputs Using Complementary Filter
 
FOx_GPS_on = a1*x_E1+a2*x_E2; % Final Output x when GPS on,
FOy_GPS_on = a1*y_N1+a2*y_N2; % Final Output y when GPS on,

x = FOx_GPS_on;
y = FOy_GPS_on;

 % save the values in database:
 
if k==1
    
aE_test(k)=aE1;
aN_test(k)=aN1;
aU_test(k)=aU1;
cum_aE_test(k) =aE1;
cum_aN_test(k) =aN1;
cum_aU_test(k) =aU1;
vE_test(k) = vE1;
vN_test(k) =vN1;
vU_test(k) =vU1;
cum_vE_test(k) =vE1;
cum_vN_test(k) =vN1;
cum_vU_test(k) =vU1;
theta_test(k)=psi;
cum_theta_test(k) = psi;
x_saved(k)=FOx_GPS_on;
y_saved(k)=FOy_GPS_on;
%     cum_aN = cum_aN+aN(k);
%     cum_aE = cum_aE+aE(k);
%     cum_aU = cum_aU+aU(k);
    else
    aE_test(k)=aE1;
    aN_test(k)=aN1;
    aU_test(k)=aU1;
    cum_aE_test(k) =cum_aE_test(k-1)+aE1;
    cum_aN_test(k) =  cum_aN_test(k-1)+aN1;
    cum_aU_test(k) =  cum_aU_test(k-1)+aU1;
    cum_vE_test(k) = cum_vE_test(k-1)+vE1;
    cum_vN_test(k) =  cum_vN_test(k-1)+vN1;
    cum_vU_test(k) = cum_vU_test(k-1)+vU1;
    cum_theta_test(k) = cum_theta_test(k-1)+psi;
    vE_test(k) =vE1;
    vN_test(k) =vN1;
    vU_test(k) =vU1;
    theta_test(k)= psi;
    x_saved(k)=FOx_GPS_on;
    y_saved(k)= FOy_GPS_on;
% last saved GPS values :
x_gps_last = FOx_GPS_on;
y_gps_last = FOy_GPS_on;
end
  flag =0; 
else
% Train the MLPs
 if flag == 0
    % train the network for x-position:    
    p1 = ([aE_test; aN_test;aU_test;cum_aE_test;cum_aN_test; cum_aU_test; vE_test;vN_test;vU_test;cum_vE_test;cum_vN_test;cum_vU_test;theta_test; cum_theta_test]);
    t1 = x_saved; 
    net1 = newff(p1,t1,[HL1 HL2 OL],{'logsig','logsig','purelin'},'trainlm','learngd');
    net1.trainParam.goal = 0.001;
    net1.trainParam.lr=0.15;
    % train the network
    MLP1_trained = train(net1,p1,t1);
    % train the network for y-position :
    
    p2 = ([aE_test; aN_test;aU_test;cum_aE_test;cum_aN_test; cum_aU_test; vE_test;vN_test;vU_test;cum_vE_test;cum_vN_test;cum_vU_test;theta_test; cum_theta_test]);
    t2 = y_saved; 
    net2 = newff(p2,t2,[HL1 HL2 OL],{'logsig','logsig','purelin'},'trainlm','learngd');
    net2.trainParam.goal = 0.001;
    net2.trainParam.lr=0.15;

    % train the network
    MLP2_trained = train(net2,p2,t2);
    flag = 1;
    
    aE_test(k)=aE1;
    aN_test(k)=aN1;
    aU_test(k)=aU1;
    cum_aE_test(k) =cum_aE_test(k-1)+aE1;
    cum_aN_test(k) =  cum_aN_test(k-1)+aN1;
    cum_aU_test(k) =  cum_aU_test(k-1)+aU1;
    cum_vE_test(k) = cum_vE_test(k-1)+vE1;
    cum_vN_test(k) =  cum_vN_test(k-1)+vN1;
    cum_vU_test(k) = cum_vU_test(k-1)+vU1;
    cum_theta_test(k) = cum_theta_test(k-1)+psi;
    vE_test(k) =vE1;
    vN_test(k) =vN1;
    vU_test(k) =vU1;
    theta_test(k)= psi;
    
 else
  % Make Predction Using Trained Neural Networks  
    aE_test(k)=aE1;
    aN_test(k)=aN1;
    aU_test(k)=aU1;
    cum_aE_test(k) = cum_aE_test(k-1)+aE1;
    cum_aN_test(k) = cum_aN_test(k-1)+aN1;
    cum_aU_test(k) = cum_aU_test(k-1)+aU1;
    cum_vE_test(k) = cum_vE_test(k-1)+vE1;
    cum_vN_test(k) = cum_vN_test(k-1)+vN1;
    cum_vU_test(k) = cum_vU_test(k-1)+vU1;
    cum_theta_test(k) = cum_theta_test(k-1)+psi;
    vE_test(k) =vE1;
    vN_test(k) =vN1;
    vU_test(k) =vU1;
    theta_test(k)= psi;
  
  p1 =([aE_test(k); aN_test(k); aU_test(k); cum_aE_test(k);cum_aN_test(k); cum_aU_test(k);vE_test(k);vN_test(k);vU_test(k);cum_vE_test(k);cum_vN_test(k);cum_vU_test(k);theta_test(k); cum_theta_test(k)]);
  % Output of MLP1:
  o1= sim(MLP1_trained,p1)
  x_MLP(k)  = o1;
    
  % test the trained network for y-position
%      p1 = [aE(k) ; aN(k);aU(k); cum_aN ; cum_aE; cum_aU;vE(k);vN(k);vU(k);cum_vE;cum_vN;cum_vU];
 p2 =([aE_test(k); aN_test(k); aU_test(k); cum_aE_test(k);cum_aN_test(k); cum_aU_test(k);vE_test(k);vN_test(k);vU_test(k);cum_vE_test(k);cum_vN_test(k);cum_vU_test(k);theta_test(k); cum_theta_test(k)]);
    % Output of MLP1:
 o2= sim(MLP2_trained,p2)
  y_MLP(k)  = o2;  
  
 
 x_E1(k) = x_MLP(k); % NN
 y_N1(k) = y_MLP(k);   
%     x_E1(k) = 0; % NN
%     y_N1(k) = 0;   
     
 
 % KF for Odometer:
 
 % 1. Predict State: 
x_hat_minus_KF2 = A2*x_hat_minus_KF2;% Odometer:     

% 2. Predict Error Covariance:
P_hat_minus_KF2 = A2*P_hat_minus_KF2*A2'+Q2;% Odometer: 
 
 % 3. Compute Kalman Gain:
 K2 = (P_hat_minus_KF2*H2')/(H2*P_hat_minus_KF2*H2'+R2);% Odometer: 
 
 % 4. Compute the Estimate:
 
 z2 = [vE1 vN1  yaw_angle x_gps_last y_gps_last ]';% Odometer: 
 x_hat2 = x_hat_minus_KF2+K2*(z2-H2*x_hat_minus_KF2);
 
 % Compute the Error Covarance:
 
 P_hat2 = P_hat_minus_KF2-K2*H2*P_hat_minus_KF2;% Odometer: 
 
 % Preserve the values for next pass
 
x_hat_minus_KF2 = x_hat2; % Odometer: 
 P_hat_minus_KF2 = P_hat2;
 
 % Save Values for Estimated X position and Y position
 
 x_E2(k) = x_hat2(6);% Odometer: 
 y_N2(k) = x_hat2(7);
 
 % Apply HPF to velocities:

% 1. FOR ODOMETER:

% [b,a] =butter(2,25/100, 'high');
y1 = filter(b,a,V_od);
  

 
 % 2. FOR ACCELEROMETER:
 
%  [b,a] =butter(2,25/100, 'high');
 y2 = filter(b,a,V_acc);
 
 
 % Calculate the error:
 error =(y1-y2);
 e(k)= error;

 % Fuzzy Logic
 
input = double(error) ;
fismat = readfis('new_alpha');   
b1 = evalfis(input,fismat);
b2 = 1-b1;

% b1=0; 

%  x_E2(k) = 0;        % Odometer: 
%  y_N2(k) = 0;
 
 % Fuse the KF1 & KF2 Outputs Using Complementary Filter
 
FOx_GPS_on = b1*x_E1(k)+b2*x_E2(k); % Final Output x when GPS on,
FOy_GPS_on =b1*y_N1(k)+b2*y_N2(k); % Final Output y when GPS on,

x = smooth(FOx_GPS_on,20,'sgolay');
y = smooth(FOy_GPS_on,20,'sgolay');
 end
end
end



                               



