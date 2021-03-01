function VisualizeMagCal(Mag_cal, Mag_cal_norm, Mag_debiased, Mag_raw_normalized, Mag, IMU_ID, flag_visualize)
% Written by Jeff Nie
% Takes in the calibrated/uncalibrated mag data, debiased (i.e. only hard iron distortions corrected for) raw mag data, and the normalized calibrated/uncalibrated mag data
% Takes in the IMU ID for labeling the plots
% Takes in a flag specifying whether the plot should be visualized 

    if flag_visualize == 1 % Want to visualize
        % % Plot the completely raw magnetometer data
        figure
        plot3(Mag(:,1), Mag(:,2), Mag(:,3), '.');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        title(strcat('Raw uncalibrated non-normalized magnetometer data -', IMU_ID)) 
        daspect([1 1 1])

        % % Plot the raw uncalibrated normalized magnetometer data onto a 3D plot
        figure
        plot3(Mag_raw_normalized(:,1), Mag_raw_normalized(:,2), Mag_raw_normalized(:,3), '.' , 'Color', 'g');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        title(strcat('Raw uncalibrated normalized magnetometer data -', IMU_ID)) 
        daspect([1 1 1])

        % % Plot the debiased magnetometer data
        figure
        plot3(Mag_debiased(:,1), Mag_debiased(:,2),Mag_debiased(:,3), '.', 'Color', 'm');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        title(strcat('Raw debiased non-normalized magnetometer data -', IMU_ID)) 
        daspect([1 1 1])

        % % Plot the raw calibrated normalized magnetometer data onto a 3D plot
        figure
        plot3(Mag_cal_norm(:,1), Mag_cal_norm(:,2), Mag_cal_norm(:,3) ,'.', 'Color', 'r');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        title(strcat('Raw calibrated normalized magnetometer data -', IMU_ID)) 
        daspect([1 1 1])

        % % Plot the raw calibrated non-normalized magnetometer data onto a 3D plot
        figure
        plot3(Mag_cal(:,1), Mag_cal(:,2), Mag_cal(:,3) ,'.', 'Color', 'b');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        title(strcat('Raw calibrated non-normalized magnetometer data -', IMU_ID)) 
        daspect([1 1 1])

        % % Plot the raw normalized uncalibrated and calibrated magnetometer data onto a 3D plot
        figure
        plot3(Mag_raw_normalized(:,1), Mag_raw_normalized(:,2), Mag_raw_normalized(:,3), '.' , 'Color', 'g');
        hold on
        plot3(Mag_cal_norm(:,1), Mag_cal_norm(:,2), Mag_cal_norm(:,3) ,'.', 'Color', 'r');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        legend('Green = Raw uncalibrated, normalized', 'Red = Raw calibrated', 'Location', 'North')
        title(strcat('Normalized raw magnetometer data before and after calibration -', IMU_ID)) 
        daspect([1 1 1])
        % xlim([-3 3])
        % ylim([-3 3])
        % zlim([-3 3])

        % % Plot the raw non-normalized uncalibrated and calibrated magnetometer data onto a 3D plot
        figure
        plot3(Mag(:,1), Mag(:,2), Mag(:,3), '.' , 'Color', 'g');
        hold on
        plot3(Mag_cal(:,1), Mag_cal(:,2), Mag_cal(:,3) ,'.', 'Color', 'r');
        plot3(Mag_debiased(:,1), Mag_debiased(:,2),Mag_debiased(:,3), '.', 'Color', 'm');
        xlabel('uT_x'); ylabel('uT_y'); zlabel('uT_z')
        legend('Green = Completely raw', 'Red = Raw calibrated', 'Magenta = Raw debiased', 'Location', 'North')
        title(strcat('Non-normalized raw magnetometer data before and after calibration, also w/ debiased step in between -', IMU_ID)) 
        daspect([1 1 1])
        % xlim([-200 200])
        % ylim([-200 200])
        % zlim([-200 200])

        figure
        subplot(311) 
        plot(Mag_cal(:,1)) 
        title(strcat('x Mag - Calibrated -', IMU_ID)) 
        xlabel('Sample'); ylabel('uT') 

        subplot(312) 
        plot(Mag_cal(:,2)) 
        title(strcat('y Mag - Calibrated -', IMU_ID)) 
        xlabel('Sample'); ylabel('uT') 

        subplot(313) 
        plot(Mag_cal(:,3)) 
        title(strcat('z Mag - Calibrated -', IMU_ID)) 
        xlabel('Sample'); ylabel('uT') 
    else % Don't want to visualize
        return 
    end


end

