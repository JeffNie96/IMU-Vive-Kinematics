function [Acc_IMU_13, Acc_IMU_14, Acc_IMU_15, Acc_IMU_16, Gyro_IMU_13, Gyro_IMU_14, Gyro_IMU_15, Gyro_IMU_16, Mag_IMU_13, Mag_IMU_14, Mag_IMU_15, Mag_IMU_16, time, flag_IMU_13, flag_IMU_14, flag_IMU_15, flag_IMU_16, IMU_13_0s, IMU_14_0s, IMU_15_0s, IMU_16_0s, Valid_start, Valid_end, Valid_ind] = ExtractIMUData(temp_IMU)
% Written by Jeff Nie 7/4/19

% This function takes in the N by 13 Acc, Gyro, and Mag data arrays and outputs 4 Acc, Gyro, and Mag arrays (one for each IMU) 
% Doesn't matter if the IMU is active or inactive

% % Cleaning up the raw data (getting rid of unwanted 0's and invalid data at the beginning and end of the data set)
% Find the magnitudes of the measured acceleration. They should be close to 1 at
% the start of the true data (since the sensor isn't moving initially), and should nearly always
% be greater than exactly 0 (b/c the sensor measures gravity) - only time it will be 0 is if an exact
% -g acts on the sensor (thus, will check the array to see if returned indices make sense in
% the context of the raw data


% % For the raw patient data in the 'MyoCI2_raw_reach_IMU_data' folder
%     Acc_raw_total = cell2mat(temp_IMU.IMU(1));
%     Gyro_raw_total = cell2mat(temp_IMU.IMU(2));
%     Mag_raw_total = cell2mat(temp_IMU.IMU(3));    


    Acc_raw_total = cell2mat(temp_IMU(1,1));
    Gyro_raw_total = cell2mat(temp_IMU(2,1));
    Mag_raw_total = cell2mat(temp_IMU(3,1));

    Ts_acc = 6.75e-3; % Accelerometer sampling period
    Ts_gyro = 6.75e-3; % Gyroscope sampling period
    Ts_mag = 13.5e-3; % Magnetometer sampling period

    % Columns of raw data array corresponding to each IMU
    x_col_IMU_13 = 2;
    z_col_IMU_13 = 4;
    x_col_IMU_14 = 5;
    z_col_IMU_14 = 7;
    x_col_IMU_15 = 8;
    z_col_IMU_15 = 10;
    x_col_IMU_16 = 11;
    z_col_IMU_16 = 13;
    
    % Flags for identifying active IMUs 
    flag_IMU_13 = 0;
    flag_IMU_14 = 0;
    flag_IMU_15 = 0;
    flag_IMU_16 = 0;
    
    % Variables to identify dropped samples
    IMU_13_0s = []; 
    IMU_14_0s = [];
    IMU_15_0s = [];
    IMU_16_0s = []; 

    % Raw acc, gyro, and mag readings for IMU 13 -> x, y, z are 1st, 2nd, 3rd columns
    Acc_raw_IMU_13 = Acc_raw_total(:,x_col_IMU_13:z_col_IMU_13); % Raw acc readings for IMU 13
    Gyro_raw_IMU_13 = Gyro_raw_total(:,x_col_IMU_13:z_col_IMU_13); % Raw gyro readings for IMU 13
    Mag_raw_IMU_13 = Mag_raw_total(:,x_col_IMU_13:z_col_IMU_13); % Raw mag readings for IMU 13

    % Raw acc, gyro, and mag readings for IMU 14 -> x, y, z are 1st, 2nd, 3rd columns
    Acc_raw_IMU_14 = Acc_raw_total(:,x_col_IMU_14:z_col_IMU_14); % Raw acc readings for IMU 14
    Gyro_raw_IMU_14 = Gyro_raw_total(:,x_col_IMU_14:z_col_IMU_14); % Raw gyro readings for IMU 14
    Mag_raw_IMU_14 = Mag_raw_total(:,x_col_IMU_14:z_col_IMU_14); % Raw mag readings for IMU 14

    % Raw acc, gyro, and mag readings for IMU 15 -> x, y, z are 1st, 2nd, 3rd columns
    Acc_raw_IMU_15 = Acc_raw_total(:,x_col_IMU_15:z_col_IMU_15); % Raw acc readings for IMU 15
    Gyro_raw_IMU_15 = Gyro_raw_total(:,x_col_IMU_15:z_col_IMU_15); % Raw gyro readings for IMU 15
    Mag_raw_IMU_15 = Mag_raw_total(:,x_col_IMU_15:z_col_IMU_15); % Raw mag readings for IMU 15

    % Raw acc, gyro, and mag readings for IMU 16 -> x, y, z are 1st, 2nd, 3rd columns
    Acc_raw_IMU_16 = Acc_raw_total(:,x_col_IMU_16:z_col_IMU_16); % Raw acc readings for IMU 16
    Gyro_raw_IMU_16 = Gyro_raw_total(:,x_col_IMU_16:z_col_IMU_16); % Raw gyro readings for IMU 16
    Mag_raw_IMU_16 = Mag_raw_total(:,x_col_IMU_16:z_col_IMU_16); % Raw mag readings for IMU 16

    % % Active IMUs should be synchronized, so same number of 0's and invalid samples (i.e. samples w/ acc magnitudes much greater than 1)
    % % Find an active IMU
    if norm(Acc_raw_IMU_13) ~= 0
        x_col = x_col_IMU_13;
        z_col = z_col_IMU_13;
    elseif norm(Acc_raw_IMU_14)~= 0
        x_col = x_col_IMU_14;
        z_col = z_col_IMU_14;
    elseif norm(Acc_raw_IMU_15) ~= 0
        x_col = x_col_IMU_15;
        z_col = z_col_IMU_15;
    elseif norm(Acc_raw_IMU_16) ~= 0
        x_col = x_col_IMU_16;
        z_col = z_col_IMU_16;
    else
        fprintf('Error: invalid data set (all samples are 0) \n')
        return
    end

    Raw_acc_norm = zeros(1, length(Acc_raw_total));

    for t = 1:length(Acc_raw_total)
        Raw_acc_norm(t) = norm(Acc_raw_total(t,x_col:z_col)); % The magnitudes of the acceleration for every sample
    end

    arbitrary_ceiling = 1.5; % Sets an arbitrary upper bound for the max initial acceleration magnitudes (shouldn't be exactly 1 b/c of noise and errors)
    acc_ind = find(((Raw_acc_norm(:) > 0) & (Raw_acc_norm(:) < arbitrary_ceiling)));

    % The Fs of the Acc/Gyro are 2x the Fs of the Mag

    % This is to ensure that the samples line up
    if mod((acc_ind(1)), 2) == 1 % if starting index is odd
        ind_start_acc_gyro = acc_ind(1); % Start index for the true raw data for accelerometer and gyro (same Ts)
        ind_start_mag = (ind_start_acc_gyro+1)/2;
    else % if starting index is even
        ind_start_acc_gyro = acc_ind(1) + 1; % drop the first accelerometer sample
        ind_start_mag = (ind_start_acc_gyro+1)/2;
    end

    if mod((acc_ind(end)), 2) == 1 % if ending index is odd
        ind_end_acc_gyro = acc_ind(end); % End index for the true raw data for accelerometer and gyro (same Ts)
        ind_end_mag = (ind_end_acc_gyro+1)/2;
    else % if starting index is even
        ind_end_acc_gyro = acc_ind(end) - 1; % Drop the last accelerometer reading
        ind_end_mag = (ind_end_acc_gyro+1)/2; % The same formulation as acc_ind(end)/2
    end

    % % Verify the indices are correct (1 -> Acc/Gyro samples line up w/ the Mag sample )
    Valid_start = (Acc_raw_total(ind_start_acc_gyro,1) == Mag_raw_total(ind_start_mag,1)); % If returns 1, then start indices line up
    Valid_end = (Acc_raw_total(ind_end_acc_gyro, 1) == Mag_raw_total(ind_end_mag,1)); % If returns 1, then end indices line up
    Valid_ind = Valid_start && Valid_end; % If returns 1, then both start and end indices line up (this is good)

    % % Extracting raw data along sensor axes (local frame)
    x_acc_raw_IMU_13 = Acc_raw_IMU_13(ind_start_acc_gyro:ind_end_acc_gyro,1); % Accelerometer IMU 13
    y_acc_raw_IMU_13 = Acc_raw_IMU_13(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_acc_raw_IMU_13 = Acc_raw_IMU_13(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_acc_raw_IMU_14 = Acc_raw_IMU_14(ind_start_acc_gyro:ind_end_acc_gyro,1); % Accelerometer IMU 14
    y_acc_raw_IMU_14 = Acc_raw_IMU_14(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_acc_raw_IMU_14 = Acc_raw_IMU_14(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_acc_raw_IMU_15 = Acc_raw_IMU_15(ind_start_acc_gyro:ind_end_acc_gyro,1); % Accelerometer IMU 15
    y_acc_raw_IMU_15 = Acc_raw_IMU_15(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_acc_raw_IMU_15 = Acc_raw_IMU_15(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_acc_raw_IMU_16 = Acc_raw_IMU_16(ind_start_acc_gyro:ind_end_acc_gyro,1); % Accelerometer IMU 16
    y_acc_raw_IMU_16 = Acc_raw_IMU_16(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_acc_raw_IMU_16 = Acc_raw_IMU_16(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_gyro_raw_IMU_13 = Gyro_raw_IMU_13(ind_start_acc_gyro:ind_end_acc_gyro,1); % Gyroscope IMU 13
    y_gyro_raw_IMU_13 = Gyro_raw_IMU_13(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_gyro_raw_IMU_13 = Gyro_raw_IMU_13(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_gyro_raw_IMU_14 = Gyro_raw_IMU_14(ind_start_acc_gyro:ind_end_acc_gyro,1); % Gyroscope IMU 14
    y_gyro_raw_IMU_14 = Gyro_raw_IMU_14(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_gyro_raw_IMU_14 = Gyro_raw_IMU_14(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_gyro_raw_IMU_15 = Gyro_raw_IMU_15(ind_start_acc_gyro:ind_end_acc_gyro,1); % Gyroscope IMU 15
    y_gyro_raw_IMU_15 = Gyro_raw_IMU_15(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_gyro_raw_IMU_15 = Gyro_raw_IMU_15(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_gyro_raw_IMU_16 = Gyro_raw_IMU_16(ind_start_acc_gyro:ind_end_acc_gyro,1); % Gyroscope IMU 16
    y_gyro_raw_IMU_16 = Gyro_raw_IMU_16(ind_start_acc_gyro:ind_end_acc_gyro,2);
    z_gyro_raw_IMU_16 = Gyro_raw_IMU_16(ind_start_acc_gyro:ind_end_acc_gyro,3);

    x_mag_raw_IMU_13 = Mag_raw_IMU_13(ind_start_mag:ind_end_mag,1); % Magnetometer IMU 13
    y_mag_raw_IMU_13 = Mag_raw_IMU_13(ind_start_mag:ind_end_mag,2);
    z_mag_raw_IMU_13 = Mag_raw_IMU_13(ind_start_mag:ind_end_mag,3);

    x_mag_raw_IMU_14 = Mag_raw_IMU_14(ind_start_mag:ind_end_mag,1); % Magnetometer IMU 14
    y_mag_raw_IMU_14 = Mag_raw_IMU_14(ind_start_mag:ind_end_mag,2);
    z_mag_raw_IMU_14 = Mag_raw_IMU_14(ind_start_mag:ind_end_mag,3);

    x_mag_raw_IMU_15 = Mag_raw_IMU_15(ind_start_mag:ind_end_mag,1); % Magnetometer IMU 15
    y_mag_raw_IMU_15 = Mag_raw_IMU_15(ind_start_mag:ind_end_mag,2);
    z_mag_raw_IMU_15 = Mag_raw_IMU_15(ind_start_mag:ind_end_mag,3);

    x_mag_raw_IMU_16 = Mag_raw_IMU_16(ind_start_mag:ind_end_mag,1); % Magnetometer IMU 16
    y_mag_raw_IMU_16 = Mag_raw_IMU_16(ind_start_mag:ind_end_mag,2);
    z_mag_raw_IMU_16 = Mag_raw_IMU_16(ind_start_mag:ind_end_mag,3);

    % Gyro has one extra garbage sample, so set that equal to the 1st valid gyro sample 
    
    x_gyro_raw_IMU_13(1) = x_gyro_raw_IMU_13(2); % Gyroscope IMU 13
    y_gyro_raw_IMU_13(1) = y_gyro_raw_IMU_13(2);
    z_gyro_raw_IMU_13(1) = z_gyro_raw_IMU_13(2);

    x_gyro_raw_IMU_14(1) = x_gyro_raw_IMU_14(2); % Gyroscope IMU 14
    y_gyro_raw_IMU_14(1) = y_gyro_raw_IMU_14(2);
    z_gyro_raw_IMU_14(1) = z_gyro_raw_IMU_14(2);

    x_gyro_raw_IMU_15(1) = x_gyro_raw_IMU_15(2); % Gyroscope IMU 15
    y_gyro_raw_IMU_15(1) = y_gyro_raw_IMU_15(2);
    z_gyro_raw_IMU_15(1) = z_gyro_raw_IMU_15(2);

    x_gyro_raw_IMU_16(1) = x_gyro_raw_IMU_16(2); % Gyroscope IMU 16
    y_gyro_raw_IMU_16(1) = y_gyro_raw_IMU_16(2);
    z_gyro_raw_IMU_16(1) = z_gyro_raw_IMU_16(2);

    % Form the time arrays (should be the same across all IMUs since they're synchronized)
    t_acc_OG = (0:Ts_acc:((length(x_acc_raw_IMU_13)-1)*Ts_acc))';
    t_gyro_OG = (0:Ts_gyro:((length(x_gyro_raw_IMU_13)-1)*Ts_gyro))';
    t_mag = (0:Ts_mag:((length(x_mag_raw_IMU_13)-1)*Ts_mag))';
    t_acc = t_mag; % Downsampled time array
    t_gyro = t_mag; % Downsampled time array
    time = t_mag;

    % Storing the values in a new array (New raw data) in the nonintuitive sensor frames
    % 1st column -> sensor frame x
    % 2nd column -> sensor frame y
    % 3rd column -> sensor frame z
    Acc_ni_IMU_13 = zeros(length(t_mag), 3);
    Gyro_ni_IMU_13 = zeros(length(t_mag), 3);
    Mag_ni_IMU_13 = zeros(length(t_mag), 3);

    Acc_ni_IMU_14 = zeros(length(t_mag), 3);
    Gyro_ni_IMU_14 = zeros(length(t_mag), 3);
    Mag_ni_IMU_14 = zeros(length(t_mag), 3);

    Acc_ni_IMU_15 = zeros(length(t_mag), 3);
    Gyro_ni_IMU_15 = zeros(length(t_mag), 3);
    Mag_ni_IMU_15 = zeros(length(t_mag), 3);

    Acc_ni_IMU_16 = zeros(length(t_mag), 3);
    Gyro_ni_IMU_16 = zeros(length(t_mag), 3);
    Mag_ni_IMU_16 = zeros(length(t_mag), 3);

    % IMU 13
    Acc_ni_IMU_13(:,1) = x_acc_raw_IMU_13(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Acc_ni_IMU_13(:,2) = y_acc_raw_IMU_13(1:2:end);
    Acc_ni_IMU_13(:,3) = z_acc_raw_IMU_13(1:2:end);
    Gyro_ni_IMU_13(:,1) = x_gyro_raw_IMU_13(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Gyro_ni_IMU_13(:,2) = y_gyro_raw_IMU_13(1:2:end);
    Gyro_ni_IMU_13(:,3) = z_gyro_raw_IMU_13(1:2:end);
    Mag_ni_IMU_13(:,1) = x_mag_raw_IMU_13;
    Mag_ni_IMU_13(:,2) = y_mag_raw_IMU_13;
    Mag_ni_IMU_13(:,3) = z_mag_raw_IMU_13;

    % IMU 14
    Acc_ni_IMU_14(:,1) = x_acc_raw_IMU_14(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Acc_ni_IMU_14(:,2) = y_acc_raw_IMU_14(1:2:end);
    Acc_ni_IMU_14(:,3) = z_acc_raw_IMU_14(1:2:end);
    Gyro_ni_IMU_14(:,1) = x_gyro_raw_IMU_14(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Gyro_ni_IMU_14(:,2) = y_gyro_raw_IMU_14(1:2:end);
    Gyro_ni_IMU_14(:,3) = z_gyro_raw_IMU_14(1:2:end);
    Mag_ni_IMU_14(:,1) = x_mag_raw_IMU_14;
    Mag_ni_IMU_14(:,2) = y_mag_raw_IMU_14;
    Mag_ni_IMU_14(:,3) = z_mag_raw_IMU_14;

    % IMU 15
    Acc_ni_IMU_15(:,1) = x_acc_raw_IMU_15(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Acc_ni_IMU_15(:,2) = y_acc_raw_IMU_15(1:2:end);
    Acc_ni_IMU_15(:,3) = z_acc_raw_IMU_15(1:2:end);
    Gyro_ni_IMU_15(:,1) = x_gyro_raw_IMU_15(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Gyro_ni_IMU_15(:,2) = y_gyro_raw_IMU_15(1:2:end);
    Gyro_ni_IMU_15(:,3) = z_gyro_raw_IMU_15(1:2:end);
    Mag_ni_IMU_15(:,1) = x_mag_raw_IMU_15;
    Mag_ni_IMU_15(:,2) = y_mag_raw_IMU_15;
    Mag_ni_IMU_15(:,3) = z_mag_raw_IMU_15;

    % IMU 16
    Acc_ni_IMU_16(:,1) = x_acc_raw_IMU_16(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Acc_ni_IMU_16(:,2) = y_acc_raw_IMU_16(1:2:end);
    Acc_ni_IMU_16(:,3) = z_acc_raw_IMU_16(1:2:end);
    Gyro_ni_IMU_16(:,1) = x_gyro_raw_IMU_16(1:2:end); % Taking every other sample (i.e. downsampling by 2) to match the # of mag samples
    Gyro_ni_IMU_16(:,2) = y_gyro_raw_IMU_16(1:2:end);
    Gyro_ni_IMU_16(:,3) = z_gyro_raw_IMU_16(1:2:end);
    Mag_ni_IMU_16(:,1) = x_mag_raw_IMU_16;
    Mag_ni_IMU_16(:,2) = y_mag_raw_IMU_16;
    Mag_ni_IMU_16(:,3) = z_mag_raw_IMU_16;

    % % Correct the sensor frames such that they are intuitive
    % % i.e. +x pointing in same direction of arrow, +z pointing down (i.e. NED
    % % for gyro/mag/acc when the sensor is right-side up, like an aircraft)

    Rot_mag = [1 0 0; 0 -1 0; 0 0 1]'; % Rotates all of the mag measurements in the nonintuitive mag frame (see trigno manual) to the intuitive sensor frame
    Rot_acc_gyro = [0 -1 0; -1 0 0; 0 0 -1]'; % Rotates all of acc/gyro measurements in the nonintuitive acc/gyro frame (see trigno manual) to the intuitive sensor frame

    % Storing the values in a new array (New raw data) in the intuitive sensor frames
    % 1st column -> sensor frame x
    % 2nd column -> sensor frame y
    % 3rd column -> sensor frame z
    Acc_IMU_13 = (Rot_acc_gyro*Acc_ni_IMU_13')'; % IMU 13
    Gyro_IMU_13 = (Rot_acc_gyro*Gyro_ni_IMU_13')';
    Mag_IMU_13 = (Rot_mag*Mag_ni_IMU_13')';

    Acc_IMU_14 = (Rot_acc_gyro*Acc_ni_IMU_14')'; % IMU 14
    Gyro_IMU_14 = (Rot_acc_gyro*Gyro_ni_IMU_14')';
    Mag_IMU_14 = (Rot_mag*Mag_ni_IMU_14')';

    Acc_IMU_15 = (Rot_acc_gyro*Acc_ni_IMU_15')'; % IMU 15
    Gyro_IMU_15 = (Rot_acc_gyro*Gyro_ni_IMU_15')';
    Mag_IMU_15 = (Rot_mag*Mag_ni_IMU_15')';

    Acc_IMU_16 = (Rot_acc_gyro*Acc_ni_IMU_16')'; % IMU 16
    Gyro_IMU_16 = (Rot_acc_gyro*Gyro_ni_IMU_16')';
    Mag_IMU_16 = (Rot_mag*Mag_ni_IMU_16')';
    
    % % Uncomment this if the raw data skipped samples (i.e. 0's for x, y, z) for Acc/Mag/Gyro
    % % Shouldn't do anything if there were no skipped samples
    % IMU 13
%     if norm(Acc_IMU_13) ~= 0
%         Mag_uncal_norm = zeros(length(Mag_IMU_13),1); %Using the Mag to find the samples b/c unlikely to measure 0 (Acc would work well too)
%         for t = 1:length(Mag_IMU_13)
%             Mag_uncal_norm(t) = norm(Mag_IMU_13(t,:));
%         end
% 
%         IMU_13_0s = find(Mag_uncal_norm == 0); % Finds the samples that were skipped (should be the same for Mag/Gyro/Acc)
% 
%         for t = 1:length(IMU_13_0s)
%             count = IMU_13_0s(t); % This is the sample # of the skipped sample
%             flag0 = 0; % Sets whether a previous nonzero sample has been found
%             subtract = 1; % How many samples to go back to a nonzero sample (starts at 1)
%             while flag0 == 0
%                 temp_Mag_norm = norm(Mag_IMU_13((count-subtract),:)); % Check whether the previous sample is 0 norm
%                 if temp_Mag_norm == 0 % If the previous sample is also 0 norm
%                     subtract = subract + 1; % Increment subtract to check the next previous sample (i.e. go backwards more)
%                 else
%                     Mag_IMU_13(count,:) = Mag_IMU_13((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Acc_IMU_13(count,:) = Acc_IMU_13((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Gyro_IMU_13(count,:) = Gyro_IMU_13((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     flag0 = 1; % Exit the while loop
%                 end
%             end
%         end
%     end
% 
% %     % IMU 14
%     if norm(Acc_IMU_14) ~= 0
%         Mag_uncal_norm = zeros(length(Mag_IMU_14),1); %Using the Mag to find the samples b/c unlikely to measure 0 (Acc would work well too)
%         for t = 1:length(Mag_IMU_14)
%             Mag_uncal_norm(t) = norm(Mag_IMU_14(t,:));
%         end
% 
%         IMU_14_0s = find(Mag_uncal_norm == 0); % Finds the samples that were skipped (should be the same for Mag/Gyro/Acc)
% 
%         for t = 1:length(IMU_14_0s)
%             count = IMU_14_0s(t); % This is the sample # of the skipped sample
%             flag0 = 0; % Sets whether a previous nonzero sample has been found
%             subtract = 1; % How many samples to go back to a nonzero sample (starts at 1)
%             while flag0 == 0
%                 temp_Mag_norm = norm(Mag_IMU_14((count-subtract),:)); % Check whether the previous sample is 0 norm
%                 if temp_Mag_norm == 0 % If the previous sample is also 0 norm
%                     subtract = subract + 1; % Increment subtract to check the next previous sample (i.e. go backwards more)
%                 else
%                     Mag_IMU_14(count,:) = Mag_IMU_14((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Acc_IMU_14(count,:) = Acc_IMU_14((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Gyro_IMU_14(count,:) = Gyro_IMU_14((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     flag0 = 1; % Exit the while loop
%                 end
%             end
%         end
%     end
% 
% %     % IMU 15
%     if norm(Acc_IMU_15) ~= 0
%         Mag_uncal_norm = zeros(length(Mag_IMU_15),1); %Using the Mag to find the samples b/c unlikely to measure 0 (Acc would work well too)
%         for t = 1:length(Mag_IMU_15)
%             Mag_uncal_norm(t) = norm(Mag_IMU_15(t,:));
%         end
% 
%         IMU_15_0s = find(Mag_uncal_norm == 0); % Finds the samples that were skipped (should be the same for Mag/Gyro/Acc)
% 
%         for t = 1:length(IMU_15_0s)
%             count = IMU_15_0s(t); % This is the sample # of the skipped sample
%             flag0 = 0; % Sets whether a previous nonzero sample has been found
%             subtract = 1; % How many samples to go back to a nonzero sample (starts at 1)
%             while flag0 == 0
%                 temp_Mag_norm = norm(Mag_IMU_15((count-subtract),:)); % Check whether the previous sample is 0 norm
%                 if temp_Mag_norm == 0 % If the previous sample is also 0 norm
%                     subtract = subract + 1; % Increment subtract to check the next previous sample (i.e. go backwards more)
%                 else
%                     Mag_IMU_15(count,:) = Mag_IMU_15((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Acc_IMU_15(count,:) = Acc_IMU_15((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Gyro_IMU_15(count,:) = Gyro_IMU_15((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     flag0 = 1; % Exit the while loop
%                 end
%             end
%         end
%     end
% 
% %     % IMU 16
%     if norm(Acc_IMU_16) ~= 0
%         Mag_uncal_norm = zeros(length(Mag_IMU_16),1); %Using the Mag to find the samples b/c unlikely to measure 0 (Acc would work well too)
%         for t = 1:length(Mag_IMU_16)
%             Mag_uncal_norm(t) = norm(Mag_IMU_16(t,:));
%         end
% 
%         IMU_16_0s = find(Mag_uncal_norm == 0); % Finds the samples that were skipped (should be the same for Mag/Gyro/Acc)
% 
%         for t = 1:length(IMU_16_0s)
%             count = IMU_16_0s(t); % This is the sample # of the skipped sample
%             flag0 = 0; % Sets whether a previous nonzero sample has been found
%             subtract = 1; % How many samples to go back to a nonzero sample (starts at 1)
%             while flag0 == 0
%                 temp_Mag_norm = norm(Mag_IMU_16((count-subtract),:)); % Check whether the previous sample is 0 norm
%                 if temp_Mag_norm == 0 % If the previous sample is also 0 norm
%                     subtract = subract + 1; % Increment subtract to check the next previous sample (i.e. go backwards more)
%                 else
%                     Mag_IMU_16(count,:) = Mag_IMU_16((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Acc_IMU_16(count,:) = Acc_IMU_16((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     Gyro_IMU_16(count,:) = Gyro_IMU_16((count-subtract),:); % Set the skipped sample to the value of the most recent nonzero sample
%                     flag0 = 1; % Exit the while loop
%                 end
%             end
%         end
%     end
    
    % Set the flags to identify the active IMUs (Acc should always be nonzero if IMU is active b/c of gravity)
    if norm(Acc_IMU_13) ~= 0 % IMU 13
        flag_IMU_13 = 1;
    end
    
    if norm(Acc_IMU_14) ~= 0 % IMU 14
        flag_IMU_14 = 1;
    end
    
    if norm(Acc_IMU_15) ~= 0 % IMU 15
        flag_IMU_15 = 1;
    end
    
    if norm(Acc_IMU_16) ~= 0 % IMU 16
        flag_IMU_16 = 1;
    end

    
    % % Remove gyro static bias (sb)
    % % This can be done by simply turning on the IMUs and leaving them alone, then recording the measured gyroscope readings
    Gyro_sb_IMU_13 = [6.5475, -0.0192, 0.2720]; 
    Gyro_sb_IMU_14 = [-0.5006, 7.8772, -0.5653];
    Gyro_sb_IMU_15 = [-0.4336, 0.9384, -0.0688];
    Gyro_sb_IMU_16 = [-2.0169, 3.3919, -0.9027];
    
    % % Calibrate the accelerometers
    % % This does not need to be calibrated nearly as frequently as the magnetometers
    % % Hard-coded calibration parameters for IMU 13
    A_acc_IMU_13 = [ 0.9944,   -0.0069,   -0.0039;
                     0.0093,    0.9981,    0.0246;
                     0.0022,   -0.0279,    0.9908 ];             
    b_acc_IMU_13 = [0.0153; -0.0005; -0.0448];
    
    % % Hard-coded calibration parameters for IMU 14
    A_acc_IMU_14 = [ 0.9940,   -0.0112,    0.0060;
                     0.0118,    0.9972,   -0.0206;
                    -0.0113,    0.0147,    0.9916 ];             
    b_acc_IMU_14 = [-0.0334; -0.0614; -0.0627];
    
    % % Hard-coded calibration parameters for IMU 15
    A_acc_IMU_15 = [ 0.9902,   -0.0211,   -0.0007;
                     0.0238,    0.9944,    0.0416;
                    -0.0057,   -0.0631,    0.9906 ];             
    b_acc_IMU_15 = [0.1538; -0.1792; -0.0475];
    
    % % Hard-coded calibration parameters for IMU 16
    A_acc_IMU_16 = [ 1.0023,   -0.0126,    0.0163;
                     0.0148,    0.9965,    0.0235;
                    -0.0194,   -0.0246,    0.9944 ];             
    b_acc_IMU_16 = [-0.0363; -0.0194; -0.0235];

    for t = 1:length(Acc_IMU_13) 
        Gyro_IMU_13(t,:) = Gyro_IMU_13(t,:) - Gyro_sb_IMU_13;
        Gyro_IMU_14(t,:) = Gyro_IMU_14(t,:) - Gyro_sb_IMU_14;
        Gyro_IMU_15(t,:) = Gyro_IMU_15(t,:) - Gyro_sb_IMU_15;
        Gyro_IMU_16(t,:) = Gyro_IMU_16(t,:) - Gyro_sb_IMU_16;
 
        Acc_IMU_13(t,:) = A_acc_IMU_13*Acc_IMU_13(t,:)' + b_acc_IMU_13; 
        Acc_IMU_14(t,:) = A_acc_IMU_14*Acc_IMU_14(t,:)' + b_acc_IMU_14; 
        Acc_IMU_15(t,:) = A_acc_IMU_15*Acc_IMU_15(t,:)' + b_acc_IMU_15; 
        Acc_IMU_16(t,:) = A_acc_IMU_16*Acc_IMU_16(t,:)' + b_acc_IMU_16; 
        
%         % Normalize all of the accelerometer measurements (disable this if want to set thresholds for external acceleration)
% %         Acc_IMU_13(t,:) = Acc_IMU_13(t,:)/norm(Acc_IMU_13(t,:));
% %         Acc_IMU_14(t,:) = Acc_IMU_14(t,:)/norm(Acc_IMU_14(t,:));
% %         Acc_IMU_15(t,:) = Acc_IMU_15(t,:)/norm(Acc_IMU_15(t,:));
% %         Acc_IMU_16(t,:) = Acc_IMU_16(t,:)/norm(Acc_IMU_16(t,:));
    end
    


end

