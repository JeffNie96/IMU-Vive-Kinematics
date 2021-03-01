function [u_ZYX] = Extract_ZYX_EulerAngles(q_ZYX)
% Written by Jeff Nie 7/5/19
% Obtains the yaw (psi), pitch (theta), and roll (phi) angles from quaternion representations of attitude
% Input is the best orientation estimate as a N by 4 array of the quaternions 
% Outputs an N by 3 array u_ZYX -> u_ZYX = [psi, theta, phi] = [yaw, pitch, roll] <-> [Z, Y, X]
    % The outputted angles are in degrees 

    
    % % quat2angle function does the exact same thing that the following code does (i.e. the following method to calculate the Euler angles from the quaternion)
    % % quat2angle function -> default sequence of roation is ZYX (yaw pitch roll, which is what I'm doing)
    
    q0_ZYX = q_ZYX(:,1); % 1st element of q estimation
    q1_ZYX = q_ZYX(:,2); % 2nd element of q estimation
    q2_ZYX = q_ZYX(:,3); % 3rd element of q estimation
    q3_ZYX = q_ZYX(:,4); % 4th element of q estimation
    
    r32_ZYX = 2*q1_ZYX.*q2_ZYX + 2*q0_ZYX.*q3_ZYX; % For yaw calculation
    r33_ZYX = q0_ZYX.^2 + q1_ZYX.^2 - q2_ZYX.^2 - q3_ZYX.^2; % For yaw calculation
    r31_ZYX = 2*q1_ZYX.*q3_ZYX - 2*q0_ZYX.*q2_ZYX; % For pitch calculation
    r21_ZYX = 2*q2_ZYX.*q3_ZYX + 2*q0_ZYX.*q1_ZYX; % For roll calculation
    r11_ZYX = q0_ZYX.^2 - q1_ZYX.^2 - q2_ZYX.^2 + q3_ZYX.^2; % For roll cacluation
    
    
    psi_ZYX = real(atan2d(r32_ZYX,r33_ZYX)); % Yaw angle from best estimation
    theta_ZYX = real(-1*asind(r31_ZYX)); % Pitch angle from best estimation
    phi_ZYX = real(atan2d(r21_ZYX,r11_ZYX)); % Roll angle from best estimation
    u_ZYX = [psi_ZYX, theta_ZYX, phi_ZYX];
    
end

