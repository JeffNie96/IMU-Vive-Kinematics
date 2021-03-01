function [q] = OQE(b1, b2, r1, r2)
% Written by Jeff Nie 7/12/19
% Performs the Optimal Quaternion Estimation algorithm developed by Markley (similar to the TRIAD algorithm) 
% Can also perform the Suboptimal Quaternion Estimation algorithm, which gives the same estimate as the TRIAD algorithm 
% Inputs are:
    % Vector observation 1 (column vector) expressed in the body frame (usually sensor acc measurement) -> dominant vector
    % Vector observation 2 (column vector) expressed in the sensor frame (usually sensor mag measurement)
    % Vector observation 1 (column vector) expressed in the reference frame (usually the global acc) -> dominant vector
    % Vector observation 2 (column vector) expressed in the reference frame (usually the global mag)
% Output is the quaternion estimation expressing the body frame w/ respect to the world frame 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    weight = 1; % Choose a weight between 0 and 1, but doesn't seem to affect the fused gravity/mag quaternion estimation 

    rx = cross(r1, r2)/norm(cross(r1, r2));
    bx = cross(b1, b2)/norm(cross(b1, b2));

    % % Optimal quaternion estimator
    rho = (1+dot(bx,rx))*(weight*dot(b1,r1) + (1-weight)*dot(b2,r2)) + dot(cross(bx,rx),(weight*cross(b1,r1) + (1-weight)*cross(b2,r2)));
    sigma = dot((bx+rx),(weight*cross(b1,r1) + (1-weight)*cross(b2,r2)));
    zeta = sqrt(rho^2 + sigma^2);

    if rho >= 0
        scalar = 1/(2*sqrt(zeta * (zeta+rho) * (1+dot(bx,rx))));
        q_real = scalar * (zeta+rho) * (1+dot(bx,rx));
        q_imag = scalar * (((zeta+rho)*cross(bx,rx)) + sigma*(bx+rx));
    else
        scalar = 1/(2*sqrt(zeta * (zeta-rho) * (1+dot(bx,rx))));
        q_real = scalar * sigma*(1 + dot(bx,rx));
        q_imag = scalar * ((sigma*cross(bx,rx)) + ((zeta-rho)*(bx+rx)));
    end
    q = [q_real, q_imag'];

    % % Suboptimal quaternion estimator AKA SOQE (usually produces the same output as the OQE)
    %     rho = (1+dot(b1,r1))*dot(cross(b1,b2),cross(r1,r2)) - dot(b1,cross(r1,r2))*dot(r1,cross(b1,b2));
    %     sigma = dot((b1+r1),cross(cross(b1,b2),cross(r1,r2)));
    %     zeta = sqrt(rho^2 + sigma^2);
    %
    %     if rho(t) >= 0
    %         scalar = 1/(2*sqrt(zeta * (zeta+rho) * (1+dot(b1,r1))));
    %         q_real = scalar * (zeta+rho) * (1+dot(b1,r1));
    %         q_imag = scalar * (((zeta+rho)*cross(b1,r1)) + sigma*(b1+r1));
    %     else
    %         scalar = 1/(2*sqrt(zeta * (zeta-rho) * (1+dot(b1,r1))));
    %         q_real = scalar * sigma*(1 + dot(b1,r1));
    %         q_imag = scalar * ((sigma*cross(b1,r1)) + ((zeta-rho)*(b1+r1)));
    %     end
    %     q_e_b_GM = [q_real, q_imag'];

end

