function [Mag_cal, Mag_cal_norm, Mag_debiased, Mag_raw_normalized] = CalibrateMagEF(Mag, A_inv, b_bias)
% Written by Jeff Nie 7/4/19
% Takes in the mag samples (N by 3 array) and calibration parameters (A_inv is a 3 by 3 array and b_bias is a 3 by 1 vector)
% Outputs the calibrated mag data, normalized calibrated mag data, debiased raw mag data, and the normalized raw data
% h_true = R*A_inv*(h_measured - b_bias)

    % % Magnetic field in Chicago IL
    % % Taken from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
    % % Address entered is 303 E Superior St, Chicago, IL 60611 for
    % % NED (North-East-Down convention; actually is local NED) 
    % % N -> global x
    % % E -> global y
    % % D -> global z
    Earth_mag_N = 19152.1; % in nT (N is +, S is -), This is w/ reference to the global N
    Earth_mag_E = -1318.8; % in nT (E is +, W is -), This is w/ reference to the global E
    Earth_mag_vert = 49940.0; % in nT (D is +, U is -), z axis

    Earth_mag = [Earth_mag_N, Earth_mag_E, Earth_mag_vert]/1000; % In uT now
    Earth_mag_norm = norm(Earth_mag); % in uT 
    
    Mag_debiased = zeros(size(Mag)); % Debiased raw magnetometer readings
    Mag_cal_norm = zeros(size(Mag)); % Raw ST calibrated magnetometer readings

    for t = 1:length(Mag)
        Mag_debiased(t,:) = Mag(t,:)' - b_bias; % Removes the bias of the magnetometer readings
        Mag_cal_norm(t,:) = A_inv*Mag_debiased(t,:)';
        % % Don't do the following if want to use the magnitude of the mag field to determine when a magnetic disturbance ocurred (b/c then everything, including disturbances, is normalized to the same magnitude) 
%         Mag_cal_norm(t,:) = Mag_cal_norm(t,:)/norm(Mag_cal_norm(t,:)); % Normalize the calibrated mag values (doesn't affect computed heading angle when doing this b/c this scales both components equally)
    end
    
    % % Normalizing raw magnetometer readings
    Mag_raw_normalized = zeros(size(Mag));
    % Not sure if normalizing the raw data to the earth's magnetic norm is
    % a correct way of representing the uncalibrated data
    for t = 1:length(Mag)
        Mag_raw_normalized(t, 1) = Mag(t,1)/Earth_mag_norm;
        Mag_raw_normalized(t, 2) = Mag(t,2)/Earth_mag_norm;
        Mag_raw_normalized(t, 3) = Mag(t,3)/Earth_mag_norm;
    end
    
    % % Converting the calibrated magnetometer data to be non-normalized
    % Multiplying by the theoretical magnetic field (the Earth's) to compare
    % with the raw data set (same concern as above)
    
    Mag_cal = zeros(size(Mag));
    
    for t = 1:length(Mag)
        Mag_cal(t, :) = Mag_cal_norm(t, :) * Earth_mag_norm;
    end


end

