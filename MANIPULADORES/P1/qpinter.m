function [pr,qr]=qpinter(Pa,Pb,lambda)

    PA = Pa(1:3,4);
    PB = Pb(1:3,4);

    RA = Pa(1:3,1:3);
    QA = tr2q(RA, 1);

    RB = Pb(1:3,1:3);
    QB = tr2q(RB,1);
    QC = qqmul(qinv(QA),QB);
    
    % Interpolate the position
    % [x;y;z]
    
    pr = PA + lambda*(PB-PA); % EQUATION 2

    % Interpolate the orientation 
    % [w,x,y,z]

    theta = 2*acos(QC(1)); %ecuación 3
    n = QC(2:4)/sin(theta/2);
    thetal = lambda*theta; % ecuación 4

    wrot = cos(thetal/2); 
    vrot = n*sin(thetal/2); 
    
    Qrot = [wrot, vrot];%ecuación 5

    qr = qqmul(QA, Qrot); %ecuación 6
end