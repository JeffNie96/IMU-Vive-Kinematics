function [ q ] = Euler2quat_ZXY(ZXY_Eulers)
% Inputs the ZXY Euler angles in degrees
    % N by 3 
% Outputs the corresponding quaternion 

    z = ZXY_Eulers(:,1); % psi
    x = ZXY_Eulers(:,2); % theta
    y = ZXY_Eulers(:,3); % phi

    N = length(ZXY_Eulers); 

    q = zeros(N,4); 

    for t = 1:N 
        q0 = cosd(x(t)/2)*cosd(y(t)/2)*cosd(z(t)/2) - sind(x(t)/2)*sind(y(t)/2)*sind(z(t)/2);
        q1 = sind(x(t)/2)*cosd(y(t)/2)*cosd(z(t)/2) - cosd(x(t)/2)*sind(y(t)/2)*sind(z(t)/2); 
        q2 = cosd(x(t)/2)*sind(y(t)/2)*cosd(z(t)/2) + sind(x(t)/2)*cosd(y(t)/2)*sind(z(t)/2); 
        q3 = cosd(x(t)/2)*cosd(y(t)/2)*sind(z(t)/2) + sind(x(t)/2)*sind(y(t)/2)*cosd(z(t)/2); 

        q(t,:) = [q0, q1, q2, q3]; 

    end

end

