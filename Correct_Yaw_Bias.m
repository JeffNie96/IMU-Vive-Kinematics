function [q_e_b_corrected] = Correct_Yaw_Bias(q_e_b, yaw_bias)
% Written by Jeff Nie on 7/13/19
% Corrects for the empirically determined yaw angle bias that persists after mag calibration 
% Inputs are
    % The biased quaternion estimate from the CF expressing the sensor frame w/ respect to the world frame as an N by 4 array
    % The estimated yaw bias angle in degrees
% Output is the corrected quaternion estimated expressing the sensor frame w/ respect to the world frame


correct_yaw = -yaw_bias;
N = length(q_e_b);
q_rotate = [cosd(correct_yaw/2)*ones(N,1), zeros(N,1), zeros(N,1), sind(correct_yaw/2)*ones(N,1)]; % Only rotate about the z axis

q_e_b_corrected = quatmultiply(q_rotate, q_e_b); 


end

