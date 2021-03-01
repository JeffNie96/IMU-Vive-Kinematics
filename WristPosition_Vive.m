function [pos_earthShoulder2Wrist] = WristPosition_Vive(q_e_A, q_A_FA, Length_A, Length_FA)
% Written by Jeff Nie 7/8/19
% Determines the position of the wrist w/ respect to the greater tubercle of the humerus (which is approximately the location of IMU 16) 
% Uses the Denavit Hartenberg convention
% Inputs are:
    % An N by 3 array containing FE elbow angles (delta elbow angles)
    % An N by 4 array containing the quaternion estimations of the arm's orientation 
    % A 1 by 4 quaternion containing the initial baseline orientation difference
    % A 3D vector of the FE axis of rotation in the FAi frame
    % The forearm and arm lengths
% Outputs are: 
    % The position vectors of the wrist as an 3 by N array of position vectors w/ respect to the world frame w/ origin at the shoulder

% Define function: quat2Euler (q -> ZYZ Euler Angles) - Can write the function for Nathan
    
    eul_ZYZ = rad2deg(quat2eul(q_e_A, 'ZYZ')); 

    R_A_FA = quat2rotm(q_A_FA); % 3x3xN
    R_A_FA(4,4,:) = 1; % Make this a 4x4xN transformation
    T_A_FA = R_A_FA;
    
    N = length(q_e_A); 
    
    % Store the relevant DH variables
    theta1 = eul_ZYZ(:,1); % First rotation, Z
    theta2 = eul_ZYZ(:,2); % Second rotation, Y
    theta3 = eul_ZYZ(:,3); % Third rotation, Z
    d1 = 0;
    d2 = 0;
    a1 = 0;
    a2 = 0;
    a3 = Length_A; 
    a5 = Length_FA; 
    alpha1 = -90; % Degrees
    alpha2 = 90;
    alpha3 = 0; 
    
    T_A = eye(4); 
    T_A(3,4) = a3; % Along the sensor z axis (the dot) 
        
    T_FA = eye(4);
    T_FA(3,4) = a5; % Along the sensor z axis (the dot)

    c_t1 = cosd(theta1); % cos(theta1) 
    s_t1 = sind(theta1); % sin(theta1)
    c_a1 = cosd(alpha1); % cos(alpha1) 
    s_a1 = sind(alpha1); % sin(alpha1)
    
    c_t2 = cosd(theta2); % cos(theta2)
    s_t2 = sind(theta2); % sin(theta2)
    c_a2 = cosd(alpha2); % cos(alpha2)
    s_a2 = sind(alpha2); % sin(alpha2) 
    
    c_t3 = cosd(theta3); % cos(theta3) 
    s_t3 = sind(theta3); % sin(theta3)
    c_a3 = cosd(alpha3); % cos(alpha3) 
    s_a3 = sind(alpha3); % sin(alpha3)
    
          
    % Define the transformation matrices
    T_e_1 = zeros(4,4,N); % First Euler angle
    T_1_2 = zeros(4,4,N); % Second Euler angle
    T_2_3 = zeros(4,4,N); % Third Euler angle and movement along arm
    T_3_4 = zeros(4,4,N); % Movement along arm and rotate to forearm's orientation
    T_4_5 = zeros(4,4,N); % Movement along forearm
    T_e_5 = zeros(4,4,N); % Transformation from earth (on shoulder) to wrist
    pos_earthShoulder2Wrist = zeros(N,3); % Position of wrist w/ respect to shoulder (earth frame attached to shoulder)
    
    
    for t = 1:N
        T_e_1(:,:,t) = [c_t1(t), -s_t1(t)*c_a1,  s_t1(t)*s_a1, a1*c_t1(t); 
                        s_t1(t),  c_t1(t)*c_a1, -c_t1(t)*s_a1, a1*s_t1(t); 
                              0,          s_a1,          c_a1,         d1;
                              0,             0,             0,          1 ]; 
        
        T_1_2(:,:,t) = [c_t2(t), -s_t2(t)*c_a2,  s_t2(t)*s_a2, a2*c_t2(t); 
                        s_t2(t),  c_t2(t)*c_a2, -c_t2(t)*s_a2, a2*s_t2(t); 
                              0,          s_a2,          c_a2,         d2;
                              0,             0,             0,          1 ];
                          
        T_2_3(:,:,t) = [c_t3(t), -s_t3(t)*c_a3,  s_t3(t)*s_a3, 0; 
                        s_t3(t),  c_t3(t)*c_a3, -c_t3(t)*s_a3, 0; 
                              0,          s_a3,          c_a3, 0;
                              0,             0,             0, 1 ]; 
        
        T_3_4(:,:,t) = T_A * T_A_FA(:,:,t); % Translate along the arm from the shoulder to the elbow, then align orientation to the forearm
        
        T_4_5(:,:,t) = T_FA;

        T_e_5(:,:,t) = T_e_1(:,:,t) * T_1_2(:,:,t) * T_2_3(:,:,t) * T_3_4(:,:,t) * T_4_5(:,:,t); 
        
        p_e_5 = T_e_5(1:3,4,t);        
        pos_earthShoulder2Wrist(t,:) = p_e_5;

    end
              
    
  
end

