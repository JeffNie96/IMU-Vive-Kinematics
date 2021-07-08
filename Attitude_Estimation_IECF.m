function [ q_e_b_CF, w_bias, norm_acc, norm_mag, mag_disturb_error, acc_disturb_error, count1, count2, count3] = Attitude_Estimation_IECF( Acc_IMU, Gyro_IMU, Mag_IMU )
% Written by Jeff Nie on 7/25/19
% The Improved Explicit Complementary Filter as described by "How magnetic disturbances influences the attitude..."
% Similar to JC's CF 

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
    Earth_mag_unit = Earth_mag/Earth_mag_norm;

    Ts_mag = 13.5e-3; % Magnetometer sampling period 

    Acc = Acc_IMU; 
    Gyro = Gyro_IMU*diag([pi/180, pi/180, pi/180]); 
    Mag = Mag_IMU; 
    N = length(Acc); 
    mag_north = [sqrt(Earth_mag_unit(1)^2 +  Earth_mag_unit(2)^2), 0, Earth_mag_unit(3)];
    mag_north = mag_north/norm(mag_north);
    Mag_u = zeros(N,3); 

    a_ref = [0 0 -1]; % Points in the up direction 
    m_ref = cross(a_ref, mag_north)/norm(cross(a_ref, mag_north)); % Points in the west direction 

    a_base = zeros(N,3); 
    m_base = zeros(N,3); 

    error_acc = zeros(N,3); 
    error_mag = zeros(N,3); 
    
    norm_acc = zeros(N,1); 
    norm_mag = zeros(N,1);
    
    % % To initialize the sensor fusion faster
    b1 = Acc_IMU(1,:)'/norm(Acc_IMU(1,:)); 
    b2 = Mag_IMU(1,:)'/norm(Mag_IMU(1,:)); 
    r1 = a_ref';
    r2 = mag_north'; 
    q_start = OQE(b1, b2, r1, r2); % OQE - optimal quaternion estimator
%     q_start = [1 0 0 0]; % If do not want to use the OQE, use this for the initial condition
    
    
    q_e_b_CF = zeros(N,4); 
%     q_e_b_CF(1,:) = [1 0 0 0]; 
    q_e_b_CF(1,:) = q_start; 
    q_e_b_CF(1,:) = q_e_b_CF(1,:)/norm(q_e_b_CF(1,:)); 
    q_e_b_gyro_dot = zeros(N,4); 
    w_bias = zeros(N,3);
    
    % % Gains
    K_a = 0.025;  
    K_m = 0.025;
    K_bias_a = 0.00000025;
    K_bias_m = 0.00000025;
    
  %  % Switch to this to determine the yaw bias (basically trust yaw and acc a lot more)
%     K_a = 50;
%     K_m = 50;
   
    
    acc_ref = 1; % 1g
    
    
    % % Error compensation -> turning off the magnetometer/accelerometer contributions to the final orientation estimation if the magnetometer/accelerometer distortion (disturbance) exceeds a certain threshold
    % % Disturbance is estimated by the difference in magnitude of the measured acceleration/magnetic field and the reference acceleration/magnetic field
    threshold_mag = 0.15; % Mag error threshold
    threshold_acc = 0.15;
%     threshold_mag = 0.99; % Mag error threshold (raise this value to turn off the error compensation, as this can sometimes screw up the convergence)
%     threshold_acc = 0.99;
    
    mag_disturb_error = zeros(N,1); 
    acc_disturb_error = zeros(N,1); 
    
    count1 = 0; 
    count2 = 0;
    count3 = 0;
    
    for t = 2:N
        norm_acc(t-1) = norm(Acc(t-1,:)); 
        norm_mag(t-1) = norm(Mag(t-1,:)); 
        
        if t == 106 % Computing the mean of the magnetic field vector during the very initial portions of the resting data 
            mag_ref = mean(norm_mag(1:t-1)); 
        end
        
        if t <= 106 % Begin estimating the orientation 
            a_base(t-1,:) = Acc(t-1,:)/norm_acc(t-1);
            Mag_u(t-1,:) = Mag(t-1,:)/norm_mag(t-1);
            m_base(t-1,:) = cross(a_base(t-1,:), Mag_u(t-1,:))/norm(cross(a_base(t-1,:), Mag_u(t-1,:)));
            
            R_b_e = quat2rotm(quatconj(q_e_b_CF(t-1,:)));
            temp_acc = R_b_e*a_ref';
            temp_mag = R_b_e*m_ref';
            
            error_acc(t-1,:) = cross(a_base(t-1,:), temp_acc);
            error_mag(t-1,:) = cross(m_base(t-1,:), temp_mag);

            
            q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) - w_bias(t-1,:) + K_a*error_acc(t-1,:) + K_m*error_mag(t-1,:))]);
%             q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) + K_a*error_acc(t-1,:) + K_m*error_mag(t-1,:))]);
            q_e_b_CF(t,:) = q_e_b_CF(t-1,:) + q_e_b_gyro_dot(t-1,:)*Ts_mag;
            q_e_b_CF(t,:) = q_e_b_CF(t,:)/norm(q_e_b_CF(t,:));
            
            w_bias(t,:) = w_bias(t-1,:) - K_bias_a*error_acc(t-1,:) - K_bias_m*error_mag(t-1,:);
        else
            
            mag_disturb_error(t-1) = abs((norm_mag(t-1) - mag_ref)/mag_ref); 
            acc_disturb_error(t-1) = abs((norm_acc(t-1) - acc_ref)/acc_ref); 
            
            a_base(t-1,:) = Acc(t-1,:)/norm_acc(t-1);
            Mag_u(t-1,:) = Mag(t-1,:)/norm_mag(t-1);
            m_base(t-1,:) = cross(a_base(t-1,:), Mag_u(t-1,:))/norm(cross(a_base(t-1,:), Mag_u(t-1,:)));
            
            R_b_e = quat2rotm(quatconj(q_e_b_CF(t-1,:)));
            temp_acc = R_b_e*a_ref';
            temp_mag = R_b_e*m_ref';
            
            error_acc(t-1,:) = cross(a_base(t-1,:), temp_acc);
            error_mag(t-1,:) = cross(m_base(t-1,:), temp_mag);
            
            if (mag_disturb_error(t-1) >= threshold_mag) && (acc_disturb_error(t-1) >= threshold_acc) % Mag and Acc disturbances
                q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) - w_bias(t-1,:))]);
%                 q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 Gyro(t-1,:)]);
                q_e_b_CF(t,:) = q_e_b_CF(t-1,:) + q_e_b_gyro_dot(t-1,:)*Ts_mag;
                q_e_b_CF(t,:) = q_e_b_CF(t,:)/norm(q_e_b_CF(t,:));
                
                w_bias(t,:) = w_bias(t-1,:);
                
                count1 = count1 + 1;
            
            elseif (mag_disturb_error(t-1) >= threshold_mag) && (acc_disturb_error(t-1) < threshold_acc) % Only mag disturbed
                q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) - w_bias(t-1,:) + K_a*error_acc(t-1,:))]);
%                 q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) + K_a*error_acc(t-1,:))]);
                q_e_b_CF(t,:) = q_e_b_CF(t-1,:) + q_e_b_gyro_dot(t-1,:)*Ts_mag;
                q_e_b_CF(t,:) = q_e_b_CF(t,:)/norm(q_e_b_CF(t,:));
                
                w_bias(t,:) = w_bias(t-1,:) - K_bias_a*error_acc(t-1,:);
                
                count2 = count2 + 1;
            elseif (mag_disturb_error(t-1) < threshold_mag) && (acc_disturb_error(t-1) >= threshold_acc) % Only acc disturbed
                q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) - w_bias(t-1,:) + K_m*error_mag(t-1,:))]);
%                 q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) + K_m*error_mag(t-1,:))]);
                q_e_b_CF(t,:) = q_e_b_CF(t-1,:) + q_e_b_gyro_dot(t-1,:)*Ts_mag;
                q_e_b_CF(t,:) = q_e_b_CF(t,:)/norm(q_e_b_CF(t,:));
                
                w_bias(t,:) = w_bias(t-1,:) - K_bias_m*error_mag(t-1,:);
                
                count3 = count3 + 1;
            else % Neither mag nor acc disturbed
                q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) - w_bias(t-1,:) + K_a*error_acc(t-1,:) + K_m*error_mag(t-1,:))]);
%                 q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) + K_a*error_acc(t-1,:) + K_m*error_mag(t-1,:))]);
                q_e_b_CF(t,:) = q_e_b_CF(t-1,:) + q_e_b_gyro_dot(t-1,:)*Ts_mag;
                q_e_b_CF(t,:) = q_e_b_CF(t,:)/norm(q_e_b_CF(t,:));
                
                w_bias(t,:) = w_bias(t-1,:) - K_bias_a*error_acc(t-1,:) - K_bias_m*error_mag(t-1,:);
            end
            
%             % % Uncomment this and comment the above to avoid using the mag error rejection 
%             q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) - w_bias(t-1,:) + K_a*error_acc(t-1,:) + K_m*error_mag(t-1,:))]);
% %             q_e_b_gyro_dot(t-1,:) = 0.5*quatmultiply(q_e_b_CF(t-1,:), [0 (Gyro(t-1,:) + K_a*error_acc(t-1,:) + K_m*error_mag(t-1,:))]);
%             q_e_b_CF(t,:) = q_e_b_CF(t-1,:) + q_e_b_gyro_dot(t-1,:)*Ts_mag;
%             q_e_b_CF(t,:) = q_e_b_CF(t,:)/norm(q_e_b_CF(t,:));
%             
%             w_bias(t,:) = w_bias(t-1,:) - K_bias_a*error_acc(t-1,:) - K_bias_m*error_mag(t-1,:);
            
        end


    end


end

