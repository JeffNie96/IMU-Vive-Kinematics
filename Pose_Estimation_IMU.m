% % Pose estimation script using IMUs
% % Written by Jeffrey Z. Nie and James W. Nie 2/27/21

clc
clear

%% Data preprocessing step
% % Sensor sampling rate
Ts_acc = 6.75e-3; % Accelerometer sampling period
Ts_gyro = 6.75e-3; % Gyroscope sampling period 
Ts_mag = 13.5e-3; % Magnetometer sampling period 
Fs_acc = 1/Ts_acc; % Accelerometer sampling frequency
Fs_gyro = 1/Ts_gyro; % Gyroscope sampling frequency 
Fs_mag = 1/Ts_mag; % Magnetometer sampling frequency 

% % Extracting raw data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('nh_sral_1-25-2021_IMU_Vicon_5cyclereps.mat') % Test data taken in RIC 17th floor

% % IMU IDs for plotting purposes
IMU_13_ID = ' IMU 13';
IMU_14_ID = ' IMU 14';
IMU_15_ID = ' IMU 15';
IMU_16_ID = ' IMU 16';

% % For debugging purposes
% Acc_raw_total = cell2mat(temp_IMU(1,1));
% Gyro_raw_total = cell2mat(temp_IMU(2,1));
% Mag_raw_total = cell2mat(temp_IMU(3,1));

% % Extract the raw data 
% If vectors IMU_0s are not empty matrices, then there were skipped samples in the data (this usually occurs if the antenna wasn't screwed in)
% Valid_start -> if returns 1, then start indices line up
% Valid_end -> if returns 1, then end indices line up
% Valid_ind -> if returns 1, then both start and end indices line up (this is good)
[Acc_IMU_13, Acc_IMU_14, Acc_IMU_15, Acc_IMU_16, Gyro_IMU_13, Gyro_IMU_14, Gyro_IMU_15, Gyro_IMU_16, Mag_IMU_13, Mag_IMU_14, Mag_IMU_15, Mag_IMU_16, time, flag_IMU_13, flag_IMU_14, flag_IMU_15, flag_IMU_16, IMU_13_0s, IMU_14_0s, IMU_15_0s, IMU_16_0s, Valid_start, Valid_end, Valid_ind] = ExtractIMUData(temp_IMU);

% % Visualize the raw data
% VisualizeRawData(Acc_IMU_13, Gyro_IMU_13, Mag_IMU_13, IMU_13_ID, flag_IMU_13) % IMU 13
% VisualizeRawData(Acc_IMU_14, Gyro_IMU_14, Mag_IMU_14, IMU_14_ID, flag_IMU_14) % IMU 14
% VisualizeRawData(Acc_IMU_15, Gyro_IMU_15, Mag_IMU_15, IMU_15_ID, flag_IMU_15) % IMU 15
% VisualizeRawData(Acc_IMU_16, Gyro_IMU_16, Mag_IMU_16, IMU_16_ID, flag_IMU_16) % IMU 16

% % % % Magnetometer calibration % % % %
% % b_bias and A_inv were were obtained using least-squares ellipsoid specific fitting
% % R_IMU is the rotation matrix correcting for any cross-axis misalignment between the accelerometer and magnetometer
% % If no cross-axis misalignment, then R_IMU_# should be a 3x3 identity matrix
b_bias_IMU_13 = [-390.4241; -455.9203; 118.4820];
A_inv_IMU_13 = [ 0.0203,  0.0002,  0.0007;
                 0.0002,  0.0207,       0;
                 0.0007,       0,  0.0200];   
R_IMU_13 = [  0.9998,   -0.0091,   -0.0158;
              0.0096,    0.9993,    0.0356;
              0.0155,   -0.0358,    0.9992]; 

b_bias_IMU_14 = [-328.3380; -316.1681; -28.9159];
A_inv_IMU_14 = [ 0.0203, -0.0004,  0.0007;
                -0.0004,  0.0202,  0.0007;
                 0.0007,  0.0007,  0.0208];      
R_IMU_14 = [  1.0000,   -0.0055,    0.0061;
              0.0057,    0.9992,   -0.0398;
             -0.0058,    0.0399,    0.9992]; % This one should minimize the std

b_bias_IMU_15 = [90.9772; 46.1747; -93.6837];
A_inv_IMU_15 = [ 0.0207, -0.0010,  0.0005;
                -0.0010,  0.0202,  0.0006;
                 0.0005,  0.0006,  0.0203];      
R_IMU_15 = [  0.9998,   -0.0044,   -0.0195;
              0.0048,    0.9997,    0.0221;
              0.0194,   -0.0221,    0.9996]; % This one should minimize the std

b_bias_IMU_16 = [-44.9684; -30.4124; 22.0082];
A_inv_IMU_16 = [ 0.0201, -0.0008,  0.0003;
                -0.0008,  0.0203, -0.0002;
                 0.0003, -0.0002,  0.0209];   
R_IMU_16 = [  0.9998,   -0.0172,   -0.0028;
              0.0172,    0.9998,    0.0083;
              0.0027,   -0.0083,    1.0000]; % This one should minimize the std        
             
% % Determines whether or not the calibrated Mag data should be visualized
% % 0 -> no ; 1 -> yes
flag_visualize_mag_IMU_13 = 0;
flag_visualize_mag_IMU_14 = 0;
flag_visualize_mag_IMU_15 = 0;
flag_visualize_mag_IMU_16 = 0;

if flag_IMU_13 == 1 % IMU 13 active
    [Mag_cal_IMU_13, Mag_cal_norm_IMU_13, Mag_debiased_IMU_13, Mag_raw_normalized_IMU_13] = CalibrateMagEF(Mag_IMU_13, A_inv_IMU_13, b_bias_IMU_13);
    Mag_cal_IMU_13 = (R_IMU_13*Mag_cal_IMU_13')';
    % % Visualize the calibrated magnetometer data 
    VisualizeMagCal(Mag_cal_IMU_13, Mag_cal_norm_IMU_13, Mag_debiased_IMU_13, Mag_raw_normalized_IMU_13, Mag_IMU_13, IMU_13_ID, flag_visualize_mag_IMU_13) 
end

if flag_IMU_14 == 1 % IMU 14 active
    [Mag_cal_IMU_14, Mag_cal_norm_IMU_14, Mag_debiased_IMU_14, Mag_raw_normalized_IMU_14] = CalibrateMagEF(Mag_IMU_14, A_inv_IMU_14, b_bias_IMU_14);
    Mag_cal_IMU_14 = (R_IMU_14*Mag_cal_IMU_14')';
    % % Visualize the calibrated magnetometer data 
    VisualizeMagCal(Mag_cal_IMU_14, Mag_cal_norm_IMU_14, Mag_debiased_IMU_14, Mag_raw_normalized_IMU_14, Mag_IMU_14, IMU_14_ID, flag_visualize_mag_IMU_14) 
end

if flag_IMU_15 == 1 % IMU 15 active
    [Mag_cal_IMU_15, Mag_cal_norm_IMU_15, Mag_debiased_IMU_15, Mag_raw_normalized_IMU_15] = CalibrateMagEF(Mag_IMU_15, A_inv_IMU_15, b_bias_IMU_15);
    Mag_cal_IMU_15 = (R_IMU_15*Mag_cal_IMU_15')';
    % % Visualize the calibrated magnetometer data 
    VisualizeMagCal(Mag_cal_IMU_15, Mag_cal_norm_IMU_15, Mag_debiased_IMU_15, Mag_raw_normalized_IMU_15, Mag_IMU_15, IMU_15_ID, flag_visualize_mag_IMU_15)
end

if flag_IMU_16 == 1 % IMU 16 active
    [Mag_cal_IMU_16, Mag_cal_norm_IMU_16, Mag_debiased_IMU_16, Mag_raw_normalized_IMU_16] = CalibrateMagEF(Mag_IMU_16, A_inv_IMU_16, b_bias_IMU_16);
    Mag_cal_IMU_16 = (R_IMU_16*Mag_cal_IMU_16')';
    % % Visualize the calibrated magnetometer data 
    VisualizeMagCal(Mag_cal_IMU_16, Mag_cal_norm_IMU_16, Mag_debiased_IMU_16, Mag_raw_normalized_IMU_16, Mag_IMU_16, IMU_16_ID, flag_visualize_mag_IMU_16)
end

% Plot IMU data to determine the initial resting data
% Use this figure to determine where the start_index for the resting data should be
figure;
subplot(3,1,1)
plot(Acc_IMU_13);
ylabel('Acc IMU 13')

subplot(3,1,2)
plot(Gyro_IMU_13);
ylabel('Gyro IMU 13')

subplot(3,1,3)
plot(Mag_cal_IMU_13);
ylabel('Mag IMU 13')

figure;
subplot(3,1,1)
plot(Acc_IMU_14);
ylabel('Acc IMU 14')

subplot(3,1,2)
plot(Gyro_IMU_14);
ylabel('Gyro IMU 14')

subplot(3,1,3)
plot(Mag_cal_IMU_14);
ylabel('Mag IMU 14')

% figure;
% subplot(3,1,1)
% plot(Acc_IMU_15);
% ylabel('Acc IMU 15')
% 
% subplot(3,1,2)
% plot(Gyro_IMU_15);
% ylabel('Gyro IMU 15')
% 
% subplot(3,1,3)
% plot(Mag_cal_IMU_15);
% ylabel('Mag IMU 14')

% figure;
% subplot(3,1,1)
% plot(Acc_IMU_16);
% ylabel('Acc IMU 16')
% 
% subplot(3,1,2)
% plot(Gyro_IMU_16);
% ylabel('Gyro IMU 16')
% 
% subplot(3,1,3)
% plot(Mag_cal_IMU_16);
% ylabel('Mag IMU 16')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rest indices
start_index = 1; % This should be the start of the resting index (found from figure)
end_index = 500; % This should be the end of the resting index (found from figure)
restAcc_IMU_13 = Acc_IMU_13(start_index:end_index,:);
restAcc_IMU_14 = Acc_IMU_14(start_index:end_index,:); 
restGyro_IMU_13 = Gyro_IMU_13(start_index:end_index,:); 
restGyro_IMU_14 = Gyro_IMU_14(start_index:end_index,:); 
restMag_cal_IMU_13 = Mag_cal_IMU_13(start_index:end_index,:); 
restMag_cal_IMU_14 = Mag_cal_IMU_14(start_index:end_index,:); 

% restAcc_IMU_15 = Acc_IMU_15(start_index:end_index,:);
% restAcc_IMU_16 = Acc_IMU_16(start_index:end_index,:); 
% restGyro_IMU_15 = Gyro_IMU_15(start_index:end_index,:); 
% restGyro_IMU_16 = Gyro_IMU_16(start_index:end_index,:); 
% restMag_cal_IMU_15 = Mag_cal_IMU_15(start_index:end_index,:); 
% restMag_cal_IMU_16 = Mag_cal_IMU_16(start_index:end_index,:); 

% Concatenate the resting and task IMU data to obtain a longer resting time
% to allow the complementary filter more time to converge.
num_iterations = 80;
[RestTask_Acc13,RestTask_Gyro13,RestTask_Mag13] = concatenateIMU(start_index,restAcc_IMU_13,restGyro_IMU_13,restMag_cal_IMU_13,Acc_IMU_13,Gyro_IMU_13,Mag_cal_IMU_13,num_iterations); % Concatenate IMU 13 for longer resting data
[RestTask_Acc14,RestTask_Gyro14,RestTask_Mag14] = concatenateIMU(start_index,restAcc_IMU_14,restGyro_IMU_14,restMag_cal_IMU_14,Acc_IMU_14,Gyro_IMU_14,Mag_cal_IMU_14,num_iterations); % Concatenate IMU 14 for longer resting data

% [RestTask_Acc15,RestTask_Gyro15,RestTask_Mag15] = concatenateIMU(start_index,restAcc_IMU_15,restGyro_IMU_15,restMag_cal_IMU_15,Acc_IMU_15,Gyro_IMU_15,Mag_cal_IMU_15,num_iterations); % Concatenate IMU 15 for longer resting data
% [RestTask_Acc16,RestTask_Gyro16,RestTask_Mag16] = concatenateIMU(start_index,restAcc_IMU_16,restGyro_IMU_16,restMag_cal_IMU_16,Acc_IMU_16,Gyro_IMU_16,Mag_cal_IMU_16,num_iterations); % Concatenate IMU 16 for longer resting data

%% IMU attitude estimation 
% % Flags specifying whether to display the ZYX (yaw, pitch, roll) Euler angles (yes -> 1 ; no -> 0)
flag_ZYX_Visualize_IMU_13 = 0;
flag_ZYX_Visualize_IMU_14 = 0; 
flag_ZYX_Visualize_IMU_15 = 0;
flag_ZYX_Visualize_IMU_16 = 0;

% % Constants specifying the yaw biases (in degrees) - Leave at 0 if good magnetometer calibration
yaw_bias_IMU_13 = 0;
yaw_bias_IMU_14 = 0;
yaw_bias_IMU_15 = 0;
yaw_bias_IMU_16 = 0;

% % Use this section if importing concatenated patient data (otherwise comment out) 
Acc_IMU_13 = RestTask_Acc13;
Gyro_IMU_13 = RestTask_Gyro13;
Mag_cal_IMU_13 = RestTask_Mag13;
Acc_IMU_14 = RestTask_Acc14;
Gyro_IMU_14 = RestTask_Gyro14;
Mag_cal_IMU_14 = RestTask_Mag14;
% Acc_IMU_15 = RestTask_Acc15;
% Gyro_IMU_15 = RestTask_Gyro15;
% Mag_cal_IMU_15 = RestTask_Mag15;
% Acc_IMU_16 = RestTask_Acc16;
% Gyro_IMU_16 = RestTask_Gyro16;
% Mag_cal_IMU_16 = RestTask_Mag16;

% % The quaternion estimations express the sensor frame w/ respect to the world frame (NED) (i.e. rotates a vector expressed in the sensor frame to the world frame
% % Extract_ZYX_EulerAngles basically does the same thing as quat2angle -> type 'help quat2angle' to see more
if flag_IMU_13 == 1 % IMU 13 active
    % Obtain the sensor attitude estimation
    [q_e_b_CF_IMU_13, w_bias_IMU_13, norm_acc13, norm_mag13, error_mag13, error_acc13, count13_1, count13_2, count13_3] = AttitudeEstimation_IECF(Acc_IMU_13, Gyro_IMU_13, Mag_cal_IMU_13);
    
    % Correct for the yaw bias (only going to do this for the FCF output, so the fused grav/mag estimate has no yaw bias correction)
    [q_e_b_CF_IMU_13] = Correct_Yaw_Bias(q_e_b_CF_IMU_13, yaw_bias_IMU_13);
    
    % Obtain the ZYX Euler angles in degrees (yaw pitch roll)
    % u_ZYX = [psi, theta, phi] = [yaw, pitch, roll] <-> [Z, Y, X]
    u_ZYX_CF_IMU_13 = Extract_ZYX_EulerAngles(q_e_b_CF_IMU_13); % ZYX Euler angles from the full FCF
end

if flag_IMU_14 == 1 % IMU 14 active
    % Obtain the sensor attitude estimation
    [q_e_b_CF_IMU_14, w_bias_IMU_14, norm_acc14, norm_mag14, error_mag14, error_acc14, count14_1, count14_2, count14_3] = AttitudeEstimation_IECF(Acc_IMU_14, Gyro_IMU_14, Mag_cal_IMU_14);
    
    % Correct for the yaw bias (only going to do this for the FCF output, so the fused grav/mag estimate has no yaw bias correction)
    [q_e_b_CF_IMU_14] = Correct_Yaw_Bias(q_e_b_CF_IMU_14, yaw_bias_IMU_14);
    
    % Obtain the ZYX Euler angles in degrees (yaw pitch roll)
    % u_ZYX = [psi, theta, phi] = [yaw, pitch, roll] <-> [Z, Y, X]
    u_ZYX_CF_IMU_14 = Extract_ZYX_EulerAngles(q_e_b_CF_IMU_14); % ZYX Euler angles from the full FCF
end

if flag_IMU_15 == 1 % IMU 15 active
    % Obtain the sensor attitude estimation
    [q_e_b_CF_IMU_15, w_bias_IMU_15, norm_acc15, norm_mag15, error_mag15, error_acc15, count15_1, count15_2, count15_3] = AttitudeEstimation_IECF(Acc_IMU_15, Gyro_IMU_15, Mag_cal_IMU_15);
    
    % Correct for the yaw bias (only going to do this for the FCF output, so the fused grav/mag estimate has no yaw bias correction)
    [q_e_b_CF_IMU_15] = Correct_Yaw_Bias(q_e_b_CF_IMU_15, yaw_bias_IMU_15);
    
    % Obtain the ZYX Euler angles in degrees (yaw pitch roll)
    % u_ZYX = [psi, theta, phi] = [yaw, pitch, roll] <-> [Z, Y, X]
    u_ZYX_CF_IMU_15 = Extract_ZYX_EulerAngles(q_e_b_CF_IMU_15); % ZYX Euler angles from the full FCF
end

if flag_IMU_16 == 1 % IMU 16 active
    % Obtain the sensor attitude estimation
    [q_e_b_CF_IMU_16, w_bias_IMU_16, norm_acc16, norm_mag16, error_mag16, error_acc16, count16_1, count16_2, count16_3] = AttitudeEstimation_IECF(Acc_IMU_16, Gyro_IMU_16, Mag_cal_IMU_16);
    
    % Correct for the yaw bias (only going to do this for the FCF output, so the fused grav/mag estimate has no yaw bias correction)
    [q_e_b_CF_IMU_16] = Correct_Yaw_Bias(q_e_b_CF_IMU_16, yaw_bias_IMU_16);
    
    % Obtain the ZYX Euler angles in degrees (yaw pitch roll)
    % u_ZYX = [psi, theta, phi] = [yaw, pitch, roll] <-> [Z, Y, X]
    u_ZYX_CF_IMU_16 = Extract_ZYX_EulerAngles(q_e_b_CF_IMU_16); % ZYX Euler angles from the full FCF
end

%% Baseline orientation removal
% % Arm IMU
q_e_A = q_e_b_CF_IMU_13;
Acc_IMU_A = Acc_IMU_13; 
Gyro_IMU_A = Gyro_IMU_13; 

% % Forearm IMU
q_e_FA = q_e_b_CF_IMU_14; 
Acc_IMU_FA = Acc_IMU_14; 
Gyro_IMU_FA = Gyro_IMU_14; 

% % Select period for baseline orientation removal
end_ind = 45630;
start_ind = end_ind - 200; 

% % If desired, rotate the forearm sensor to align a different sensor axis with the length of the forearm (this rotated FA frame is termed FAi)
% % If not, multiply q_e_FA with the identity quaternion ([1 0 0 0]) to keep the original forearm sensor alignment (i.e. frame FA = frame FAi)
q_FA_z_neg90 = [cosd(-45), 0, 0, sind(-45)]; % Rotate -90 degrees about the forearm sensor's z axis
q_e_FAi = quatmultiply(q_e_FA, q_FA_z_neg90); 
% q_e_FAi = quatmultiply(q_e_FA, [1 0 0 0]); % Uncomment this if do not want to rotate the initial sensor frame 

% % Baseline orientation removal 
q_A_e = quatconj(q_e_A); 
q_A_FAi = quatmultiply(q_A_e, q_e_FAi);
q_FAi_A = quatconj(q_A_FAi); 
q_A_FAi_init = mean(q_A_FAi(start_ind:end_ind,:)); % Need to adjust the starting samples on a case by case basis 
q_e_Ai = quatmultiply(q_e_A,q_A_FAi_init); % Initially, q_e_13i = q_e_14i
q_Ai_FAi = quatmultiply(quatconj(q_e_Ai),q_e_FAi); 
q_FAi_Ai = quatconj(q_Ai_FAi);
u_ZYX_FAi_Ai = Extract_ZYX_EulerAngles(q_FAi_Ai);  

% % If rotated the original forearm sensor frame to align a different sensor axis with the length of the forearm, need to rotate all sensor measurements to the FA frame
% % If did not rotate the original forearm sensor frame, then R_FA_FAi is the 3x3 identity (i.e. will not rotate the sensor measurements)
R_FA_FAi = quat2rotm(q_FA_z_neg90); % Uncomment this if rotated the original forearm sensor frame
% R_FA_FAi = eye(3); % Uncomment this if did not originally rotate the initial forearm sensor frame

R_Ai_A = quat2rotm(quatconj(q_A_FAi_init)); 
Gyro_IMU_Ai = (R_Ai_A*Gyro_IMU_A')'; 
Gyro_IMU_FAi = (R_FA_FAi'*Gyro_IMU_FA')';
GyroAi = Gyro_IMU_Ai*diag([pi/180, pi/180, pi/180]); % In rad/s
GyroFAi = Gyro_IMU_FAi*diag([pi/180, pi/180, pi/180]); % In rad/s

%% Find the FE axis of rotation
% % Theory is directly from Seel et al 2012 "Joint Axis and Position Estimation from Intertial Measurement Data by Exploiting Kinematic Constraints"
% % Find FE axis of rotation using gyroscope measurements (Skip this if no calibration motion)
% % If did NOT rotate the forearm frame in the above block, then frame FA = frame FAi

% % Use GyroFAi to find the indices
Li = 45570;
Lf = 46370;  
Interval = Lf - Li;

IT = 400; % Number of iterations
x = zeros(4,IT); % phi 1, theta 1, phi 2, theta 2 in rad
error = zeros(Interval+1,IT); 
x(1,1) = pi;
x(2,1) = pi;
x(3,1) = pi;
x(4,1) = pi;
de_dx = zeros(Interval+1,4);

j1 = zeros(IT,3); 
j2 = zeros(IT,3); 
count = 0;

for o = 2:IT+1
    p1 = x(1,o-1); % phi 1
    t1 = x(2,o-1); % theta 1
    p2 = x(3,o-1); % phi 2
    t2 = x(4,o-1); % theta 2
    j1(o-1,:) = [cos(p1)*cos(t1), cos(p1)*sin(t1), sin(p1)];
    j2(o-1,:) = [cos(p2)*cos(t2), cos(p2)*sin(t2), sin(p2)];
    
    for t = Li:Lf

        error(t,o-1) = norm(cross(GyroAi(t,:), j1(o-1,:))) - norm(cross(GyroFAi(t,:), j2(o-1,:)));

        de1_dj1 = cross(cross(GyroAi(t,:), j1(o-1,:)), GyroAi(t,:))/norm(cross(GyroAi(t,:),j1(o-1,:)));
        de2_dj2 = cross(cross(GyroFAi(t,:), j2(o-1,:)), GyroFAi(t,:))/norm(cross(GyroFAi(t,:),j2(o-1,:)));
        de_dj = [de1_dj1, -1*de2_dj2];


        dj1x_dp1 = -sin(p1)*cos(t1);
        dj1x_dt1 = -cos(p1)*sin(t1);
        dj1x_dp2 = 0;
        dj1x_dt2 = 0;

        dj1y_dp1 = -sin(p1)*sin(t1);
        dj1y_dt1 = cos(p1)*cos(t1);
        dj1y_dp2 = 0;
        dj1y_dt2 = 0;

        dj1z_dp1 = cos(p1);
        dj1z_dt1 = 0;
        dj1z_dp2 = 0;
        dj1z_dt2 = 0;

        dj2x_dp1 = 0;
        dj2x_dt1 = 0;
        dj2x_dp2 = -sin(p2)*cos(t2);
        dj2x_dt2 = -cos(p2)*sin(p2);

        dj2y_dp1 = 0;
        dj2y_dt1 = 0;
        dj2y_dp2 = -sin(p2)*sin(t2);
        dj2y_dt2 = cos(p2)*cos(t2);

        dj2z_dp1 = 0;
        dj2z_dt1 = 0;
        dj2z_dp2 = cos(p2);
        dj2z_dt2 = 0;

        dj_dx = [ dj1x_dp1,  dj1x_dt1,  dj1x_dp2,  dj1x_dt2;
                  dj1y_dp1,  dj1y_dt1,  dj1y_dp2,  dj1y_dt2;
                  dj1z_dp1,  dj1z_dt1,  dj1z_dp2,  dj1z_dt2;
                  dj2x_dp1,  dj2x_dt1,  dj2x_dp2,  dj2x_dt2;
                  dj2y_dp1,  dj2y_dt1,  dj2y_dp2,  dj2y_dt2;
                  dj2z_dp1,  dj2z_dt1,  dj2z_dp2,  dj2z_dt2 ];

        de_dx(t,:) = de_dj*dj_dx;
    end
    x(:,o) = x(:,o-1) - pinv(de_dx)*error(:,o-1);
end

figure
subplot(411)
plot(j1)
title('j1') 
subplot(412)
plot(GyroAi(Li:Lf,:))
title('Gyro Ai')
subplot(413)
plot(j2)
title('j2')
subplot(414)
plot(GyroFAi(Li:Lf,:))
title('Gyro FAi')

%% Compute the change in elbow angle
q = q_Ai_FAi;
N = length(q); 

% % Use j2(end,:) to fill in j1_t, which is the FE axis of rotation in frame Ai
% % j2 is the FE axis in frame FAi (which is frame FA if didn't rotate the FA sensor frame two blocks above)
% % With baseline orientation removal, j2 = j1_t
j1_t = [-0.0778    0.0244    0.9967]; % This is the estimated FE axis (j2) of rotation from the above block "Find the FE axis of rotation" 

% % Convert to rotation vector notation
alpha_q = 2*acos(q(:,1));
n_q = zeros(N,3); 
v_q = zeros(N,3); 
EF_angle = zeros(N,1); 

for t = 1:N 
    n_q(t,:) = q(t,2:4)/sqrt(1-q(t,1)^2); 
    v_q(t,:) = alpha_q(t)*n_q(t,:);   
    EF_angle(t) = rad2deg(dot(v_q(t,:), j1_t)); 
end

delta_elbow_angle = EF_angle;

%% Position estimation from the shoulder
% % Four parameters needed: q_e_A, q_A_FAi_init, j1_t, delta_elbow_angle
% q_e_A = q_e_b_CF_IMU_13; % Comment out if already defined 
arm_tested = 0; % 0 = Right UE tested, 1 = Left UE tested (can actually use any number) 

% % EPP
index_start = end_ind;
index_end = length(delta_elbow_angle);

length_A = 29; % cm
length_FA = 23; % cm
wristEPP = WristPosition_IMU(arm_tested, q_e_A(index_start:index_end,:), delta_elbow_angle(index_start:index_end), q_A_FAi_init, j1_t, length_A, length_FA);

norm_2Dxy_EPP = zeros(length(wristEPP),1); 
norm_3D_EPP = zeros(length(wristEPP),1); 

for t = 1:length(wristEPP)
    norm_2Dxy_EPP(t) = norm(wristEPP(t,1:2));
    norm_3D_EPP(t) = norm(wristEPP(t,:)); 
end

figure('units', 'normalized', 'outerposition', [0 0 1 1])
subplot(611)
plot(delta_elbow_angle(index_start:index_end))
title('IMU Wrist Position Estimation'); ylabel('Degrees')
subplot(612)
plot(wristEPP(:,1))
title('x position'); ylabel('cm')
subplot(613)
plot(wristEPP(:,2))
title('y position'); ylabel('cm')
subplot(614)
plot(wristEPP(:,3))
title('z position'); ylabel('cm')
subplot(615)
plot(norm_2Dxy_EPP)
title('2D norm - xy (horizontal) plane'); ylabel('cm')
subplot(616)
plot(norm_3D_EPP)
title('3D norm'); xlabel('Sample'); ylabel('cm') 


