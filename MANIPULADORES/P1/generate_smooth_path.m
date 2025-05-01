function [P, Q]=generate_smooth_path(P1, P2, P3, tau, T, t)
    % Function that calculates the transformation (P - position, and Q - orientation) from P1 to P3 smoothing in P2 with Taylor method (quaternions)

    if (t<-T || t>T)
        % Out of allowed range
        disp('Parameter t out of range');
    else

        if (t<=-tau) % First segment (lineal)
            [P, Q] = qpinter(P1, P2, (t+T)/T);
        elseif (t>=tau) % Third segment (lineal)
            [P, Q] = qpinter(P2, P3, t/T);            
        else % Second segment (smoothing)
            p1 = P1(1:3,4);
            p2 = P2(1:3,4);
            p3 = P3(1:3,4);

            dp1 = p2-p1;
            dp2 = p3-p2;
            
            q1 = tr2q(P1);
            q2 = tr2q(P2);
            q3 = tr2q(P3);

            % Position interpolation
            P = p2 - dp1 * (tau-t)^2 / (4*tau*T)  + dp2 * (tau+t)^2 / (4*tau*T); % EQUATION 8

            % Orientation interpolation
            q12 = qqmul(qinv(q1), q2); % EQUATION 11
            theta12 = 2*acos(q12(1));
            n12 = q12(2:4)/sin(theta12/2);

            q23 = qqmul(qinv(q2), q3); % EQUATION 12
            theta23 = 2*acos(q23(1));
            n23 = q23(2:4)/sin(theta23/2);
            
            thetak1 = -(tau-t)^2 / (4*tau*T)*theta12; % EQUATION 13
            qk1 = [cos(thetak1/2), n12*sin(thetak1/2)];

            thetak2 = (tau+t)^2 / (4*tau*T)*theta23; % EQUATION 14
            qk2 = [cos(thetak2/2), n23*sin(thetak2/2)];
            
            % EQUATION 10
            Q = qqmul(q2, qqmul(qk1,qk2)); % EQUATION 9

        end

    end
end