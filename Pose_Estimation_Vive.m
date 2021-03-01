% % Pose estimation script for Vive
% % Written by Jeffrey Z. Nie and James W. Nie 2/27/21
%% Extract raw data 
clc
clear
% close all

filenameA = 'nh_sral_1-25-2021_TrackerA_calib_reaches_seq'; % Arm  
filenameFA = 'nh_sral_1-25-2021_TrackerFA_calib_reaches_seq'; % Forearm 

% % Import the data from Unity
dataA = readtable(filenameA, 'ReadVariableNames', false); % Arm
dataA.Properties.VariableNames = {'State', 'posX', 'posY', 'posZ', 'rotZ', 'rotX', 'rotY'}; % Arm
dataFA = readtable(filenameFA, 'ReadVariableNames', false); % Forearm
dataFA.Properties.VariableNames = {'State', 'posX', 'posY', 'posZ', 'rotZ', 'rotX', 'rotY', 'Unity_Ang', 'time'}; % Forearm
pos_rot_A_raw = table2array(dataA(:,2:end)); 
pos_rot_FA_raw = table2array(dataFA(:,2:end)); 
time = pos_rot_FA_raw(:,end); % in ms 

start_index = 1;
end_index = length(pos_rot_FA_raw); % default is length(pos_rot_FA_raw);, but change this if want to cut out data

ZXY_EulerAngles_A = pos_rot_A_raw(start_index:end_index,4:6); 
ZXY_EulerAngles_FA = pos_rot_FA_raw(start_index:end_index,4:6); 
pos_A = pos_rot_A_raw(start_index:end_index,1:3);
pos_FA = pos_rot_FA_raw(start_index:end_index,1:3); 

q_e_s_A_itsRaw = Euler2quat_ZXY(ZXY_EulerAngles_A); 
q_e_s_FA_itsRaw = Euler2quat_ZXY(ZXY_EulerAngles_FA); 

time_final = time(start_index:end_index); % for calculation of vel, acc, etc.


%% Compute the relative orientation between the arm and forearm trackers
q_start = start_index;
q_e_A_raw = q_e_s_A_itsRaw(q_start:end,:); 
q_e_FA_raw = q_e_s_FA_itsRaw(q_start:end,:); 
N = min(length(q_e_A_raw), length(q_e_FA_raw));
q_e_A = zeros(N,4); 
q_e_FA = zeros(N,4); 

% % Redefine rotations from -180 to 180 degrees rather than 0 to 360 degrees 
uDiff_A = zeros(N,4); % check unit difference s.t. comparison makes sense
switchP_A = zeros(N,1); % range that quaternions flipcd

uDiff_FA = zeros(N,4); % check unit difference s.t. comparison makes sense
switchP_FA = zeros(N,1); % range that quaternions flip

thresholdFA = 0.9; % The threshold difference to determine whether there was a flip -> look at uDiff_A or uDiff_FA to determine a good threshold 
thresholdA = 0.4; % The threshold difference to determine whether there was a flip -> look at uDiff_A or uDiff_FA to determine a good threshold
for t = 1:N
    if t+1 < N
       uDiff_A(t,:) = abs(q_e_A_raw(t,:) - q_e_A_raw(t+1,:)); 
       if max(uDiff_A(t,:)) > thresholdA
            switchP_A(t) = t;
        else
            switchP_A(t) = NaN;
        end
    end
    
    if t+1 < N
       uDiff_FA(t,:) = abs(q_e_FA_raw(t,:) - q_e_FA_raw(t+1,:)); 
       if max(uDiff_FA(t,:)) > thresholdFA
            switchP_FA(t) = t;
       else
            switchP_FA(t) = NaN;
       end
    end
end

% % Add large values at the end of the array to ensure that that last switchpoint index is accounted for
switchP_A(N-1:N) = N+1; 
switchP_FA(N-1:N) = N+1; 

% % SwitchP_A and SwitchP_FA have 0s at the end of their vector, probably doesn't affect anything, but just fyi 
switchP_A = switchP_A(~isnan(switchP_A)); % finding regions that flip
switchP_A = switchP_A(1:length(switchP_A)); % switch point indices (should give even # of entries)
j = 1;

switchP_FA = switchP_FA(~isnan(switchP_FA)); % finding regions that flip
switchP_FA = switchP_FA(1:length(switchP_FA)); % switch point indices (should give even # of entries)
i = 1;

% switch sign based on index range 
for t = 1:N
    if switchP_A(j) == 0 % no switch points, so no need to switch sign
        q_e_A(t,:) = q_e_A_raw(t,:);
    elseif t > switchP_A(j) && t <= switchP_A(j+1) % in range to switch sign
        q_e_A(t,:) = -q_e_A_raw(t,:); 
        if t == switchP_A(j+1) && (j+2) < length(switchP_A) % jumps to next set of switch values
            j = j+2;
        end
    else % not in switch point -> don't need to switch sign
        q_e_A(t,:) = q_e_A_raw(t,:);
    end

    if switchP_FA(i) == 0 % no switch points, so no need to switch sign
        q_e_FA(t,:) = q_e_FA_raw(t,:);
    elseif t > switchP_FA(i) && t <= switchP_FA(i+1) % in range to switch sign
        q_e_FA(t,:) = -q_e_FA_raw(t,:); 
        if t == switchP_FA(i+1) && (i+2) < length(switchP_FA) % jumps to next set of switch values
            i = i+2;
        end
    else % not in switch point -> don't need to switch sign
        q_e_FA(t,:) = q_e_FA_raw(t,:);
    end
end

% % Flip the whole quaternion if the real component starts negative (q = -q i.e. same rotation) 
if q_e_A(1,1) < 0
    q_e_A = -q_e_A;
end
if q_e_FA(1,1) < 0
    q_e_FA = -q_e_FA;
end

q_A_e = quatconj(q_e_A); 
q_FA_e = quatconj(q_e_FA); 
q_A_FA = quatmultiply(q_A_e, q_e_FA); 
q_FA_A = quatconj(q_A_FA); 

% % Troubleshoot raw data extraction
% % If it doesn't fix, take a look at the variables uDiff_A and uDiff_FA, as well as the threshold
% % As well as the component of the quaternion that's switching a lot
figure;
subplot(5,1,1)
plot(q_e_FA_raw)
title('q e FA raw')
subplot(5,1,2)
plot(q_e_FA)
title('q e FA')
subplot(5,1,3)
plot(q_e_A_raw)
title('q e A raw')
subplot(5,1,4)
plot(q_e_A)
title('q e A')
subplot(5,1,5)
plot(pos_FA)
title('pos FA')


%% Wrist position w/ respect to the shoulder
index_start = 1;
index_end = length(q_e_A);
time_final = time_final(index_start:index_end); % for calculation of vel, acc, etc.

length_A = 29; % cm
length_FA = 23; % cm
wristEPP = WristPosition_Vive(q_e_A(index_start:index_end,:), q_A_FA(index_start:index_end,:), length_A, length_FA);

norm_2Dxy_EPP = zeros(length(wristEPP),1); 
norm_3D_EPP = zeros(length(wristEPP),1); 

for t = 1:length(wristEPP)
    norm_2Dxy_EPP(t) = norm(wristEPP(t,1:2));
    norm_3D_EPP(t) = norm(wristEPP(t,:)); 
end

figure('units', 'normalized', 'outerposition', [0 0 1 1])
subplot(511)
plot(wristEPP(:,1))
title('x position'); ylabel('cm')
subplot(512)
plot(wristEPP(:,2))
title('y position'); ylabel('cm')
subplot(513)
plot(wristEPP(:,3))
title('z position'); ylabel('cm')
subplot(514)
plot(norm_2Dxy_EPP)
title('2D norm - xy (horizontal) plane'); ylabel('cm')
subplot(515)
plot(norm_3D_EPP)
title('3D norm'); xlabel('Sample'); ylabel('cm') 

