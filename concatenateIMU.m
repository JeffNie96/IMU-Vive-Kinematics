function [RestTask_Acc,RestTask_Gyro,RestTask_Mag ] = concatenateIMU(start_index,restAcc,restGyro,restMag,taskAcc,taskGyro,taskMag, num_iterations)
% % Written by James W. Nie 2/7/21
% concatenateIMU is to take advantage of all the data being processed
% offline. This function combines multiple instances of the resting data
% set with the task data set in the IMU accelerometer, gyroscope, and
% magnetometer values to allow more time for the complementary filter to
% converge. This is particularly useful given small gains for the
% magnetometer and accelerometer in the complementary filter.
%%% Defining inputs:
    % start_index - the start of the resting index (in the original data set) 
    % restAcc - resting accelerometer data
    % restGyro - resting gyroscope data
    % restMag - resting magnetometer data
    % taskAcc - task accelerometer data
    % taskGyro - task gyroscope data
    % taskMag - task magnetometer data
    % num_iterations - number of times resting data is concatenated
    

% Labeling the resting and task data from the input of the concatenateIMU
% function.
Resting_Acc = restAcc;
Resting_Gyro = restGyro; 
Resting_Mag = restMag; 

Reaching_Acc = taskAcc;
Reaching_Gyro = taskGyro; 
Reaching_Mag = taskMag; 


%%% Gyro bias removal
ind_gyro1 = 1; % Start of resting gyro
ind_gyro2 = length(Resting_Gyro); % End of resting gyro to determine gyro bias

gyro_bias = mean(Resting_Gyro(ind_gyro1:ind_gyro2,:));
restingGyroBias = zeros(length(Resting_Gyro(:,1)),3);
reachingGyroBias = zeros(length(Reaching_Gyro(:,1)),3);
for i = 1:3
    restingGyroBias(:,i) = gyro_bias(i);
    reachingGyroBias(:,i) = gyro_bias(i);
end
% Determine new resting and reaching gyro values to eliminate gyro bias. 
Resting_Gyro = Resting_Gyro - restingGyroBias;
Reaching_Gyro = Reaching_Gyro - reachingGyroBias;

%%% Baseline correction (removes baseline between reaching and resting) 
ind_reach1 = start_index; % start of resting data set
ind_reach2 = ind_reach1+100;

% Take the mean of the accelerometer and magnetometer to eliminate the
% difference in resting and reaching values
mean_rest_Acc = mean(Resting_Acc);
mean_rest_Mag = mean(Resting_Mag);
mean_reach_Acc = mean(Reaching_Acc(ind_reach1:ind_reach2,:)); 
mean_reach_Mag = mean(Reaching_Mag(ind_reach1:ind_reach2,:)); 

diff_Acc = mean_rest_Acc - mean_reach_Acc; 
diff_Mag = mean_rest_Mag - mean_reach_Mag; 

restingDiffAcc = zeros(length(Resting_Acc), 3);
restingDiffMag = zeros(length(Resting_Mag), 3);

for i = 1:3
    restingDiffAcc(:,i)= diff_Acc(i);
    restingDiffMag(:,i) = diff_Mag(i);
end
% Determine the new resting accelerometer and magnetometer resting baseline
% values.
Resting_Acc = Resting_Acc - restingDiffAcc; 
Resting_Mag = Resting_Mag - restingDiffMag; 


%%% Concatenate the resting and reaching data to allow more time for the
%%% complementary filter to converge.
ind1 = 1; % Start of resting index
ind2 = ind_reach1; % Start of reaching index

% Concentated data set.
temp_Rest_Acc = cat(1, Resting_Acc(ind1:end,:), Resting_Acc(ind2:end,:));
temp_Rest_Gyro = cat(1, Resting_Gyro(ind1:end,:), Resting_Gyro(ind2:end,:));
temp_Rest_Mag = cat(1, Resting_Mag(ind1:end,:), Resting_Mag(ind2:end,:));

for ind = 1:(num_iterations-2)
    temp_Rest_Acc = cat(1, Resting_Acc(ind1:end,:), temp_Rest_Acc);
    temp_Rest_Gyro = cat(1, Resting_Gyro(ind1:end,:), temp_Rest_Gyro);
    temp_Rest_Mag = cat(1, Resting_Mag(ind1:end,:), temp_Rest_Mag);

end

RestTask_Acc = cat(1,temp_Rest_Acc, Reaching_Acc(ind2:end,:)); 
RestTask_Gyro = cat(1,temp_Rest_Gyro, Reaching_Gyro(ind2:end,:)); 
RestTask_Mag = cat(1,temp_Rest_Mag, Reaching_Mag(ind2:end,:)); 

end

