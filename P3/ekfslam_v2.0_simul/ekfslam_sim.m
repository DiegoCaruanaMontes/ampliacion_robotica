function data= ekfslam_sim(lm, wp)
%function data= ekfslam_sim(lm, wp)
%
% INPUTS: 
%   lm - set of landmarks
%   wp - set of waypoints
%
% OUTPUTS:
%   data - a data structure containing:
%       data.true: the vehicle 'true'-path (ie, where the vehicle *actually* went)
%       data.path: the vehicle path estimate (ie, where SLAM estimates the vehicle went)
%       data.state(k).x: the SLAM state vector at time k
%       data.state(k).P: the diagonals of the SLAM covariance matrix at time k
%
% NOTES:
%   This program is a SLAM simulator. To use, create a set of landmarks and 
%   vehicle waypoints (ie, waypoints for the desired vehicle path). The program
%   'frontend.m' may be used to create this simulated environment - type
%   'help frontend' for more information.
%       The configuration of the simulator is managed by the script file
%   'configfile.m'. To alter the parameters of the vehicle, sensors, etc
%   adjust this file. There are also several switches that control certain
%   filter options.
%
% Modified by Ricado Vazquez-Martin (2020/03/30) - changes:
%   - odometry (prediction) is included in the data structure
% Tim Bailey and Juan Nieto 2004.
% Version 2.0


format compact
configfile; % ** USE THIS FILE TO CONFIGURE THE EKF-SLAM **

% Setup plots
% fig=figure;
% plot(lm(1,:),lm(2,:),'b*')
% hold on, axis equal
% plot(wp(1,:),wp(2,:), 'g', wp(1,:),wp(2,:),'g.')
% xlabel('metres'), ylabel('metres')
% set(fig, 'name', 'EKF-SLAM Simulator')
h= setup_animations(lm,wp); % arguments for plots (added by Ricardo Vázquez)
veh= [0 -WHEELBASE -WHEELBASE; 0 -2 2]; % vehicle animation
plines=[]; % for laser line animation
pcount=0;

% Initialise states and other global variables
global XX PX DATA odom % included odom as global variable (by Ricardo Vazquez)
xtrue= zeros(3,1);
XX= zeros(3,1);
PX= zeros(3);
odom= zeros(3,1);
DATA= initialise_store(XX,odom,PX,XX); % stored data for off-line
                                  % included odom (modified by Ricardo Vázquez)
                                  
% Initialise other variables and constants
dt= DT_CONTROLS; % change in time between predicts
dtsum= 0; % change in time since last observation
ftag= 1:size(lm,2); % identifier for each landmark
da_table= zeros(1,size(lm,2)); % data association table 
iwp= 1; % index to first waypoint 
G= 0; % initial steer angle
QE= Q; RE= R; if SWITCH_INFLATE_NOISE, QE= 2*Q; RE= 8*R; end % inflate estimated noises (ie, add stabilising noise)
if SWITCH_SEED_RANDOM, randn('state',SWITCH_SEED_RANDOM), end

if SWITCH_PROFILE, profile on -detail builtin, end

% Main loop 
while iwp ~= 0
    
    % Compute true data
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    if iwp==0 && NUMBER_LOOPS > 1, pack; iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % perform loops: if final waypoint reached, go back to first
    xtrue= vehicle_model(xtrue, V,G, WHEELBASE,dt);
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    
    % EKF predict step
    predict (Vn,Gn,QE, WHEELBASE,dt);
    % store odometry in predict for data structure (included by Ricardo Vázquez)
    
    % If heading known, observe heading
    observe_heading(xtrue(3), SWITCH_HEADING_KNOWN);
    
    % Incorporate observation, (available every DT_OBSERVE seconds)
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        
        % Compute true data
        [z,ftag_visible]= get_observations(xtrue, lm, ftag, MAX_RANGE);
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
    
        % EKF update step
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(XX,z,ftag_visible, da_table);
        else
            [zf,idf, zn]= data_associate(XX,PX,z,RE, GATE_REJECT, GATE_AUGMENT); 
        end

        if SWITCH_USE_IEKF == 1
            update_iekf(zf,RE,idf, 5);
        else
            update_EKF(zf,RE,idf, SWITCH_BATCH_UPDATE); 
        end
        augment(zn,RE); 
    end
    
    % Offline data store
    store_data(XX, odom, PX, xtrue); % included odom (by Ricardo Vázquez)
    
    % Plots
    xt= transformtoglobal(veh, xtrue);
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
    
    if SWITCH_GRAPHICS
        xv= transformtoglobal(veh, XX(1:3));
        pvcov= make_vehicle_covariance_ellipse(XX,PX);
        set(h.xv, 'xdata', xv(1,:), 'ydata', xv(2,:))
        set(h.vcov, 'xdata', pvcov(1,:), 'ydata', pvcov(2,:))     
        
        pcount= pcount+1;
        if pcount == 120 % plot path infrequently
            pcount=0;
            set(h.pth, 'xdata', DATA.path(1,1:DATA.i), 'ydata', DATA.path(2,1:DATA.i)) 
            % plot vehicle odometry (added by Ricardo)
            set(h.odom, 'xdata', DATA.odom(1,1:DATA.i), 'ydata', DATA.odom(2,1:DATA.i))
        end            
        
        if dtsum==0 && ~isempty(z) % plots related to observations
            set(h.xf, 'xdata', XX(4:2:end), 'ydata', XX(5:2:end))
            plines= make_laser_lines (z,XX(1:3));
            set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:))
            pfcov= make_feature_covariance_ellipses(XX,PX);
            set(h.fcov, 'xdata', pfcov(1,:), 'ydata', pfcov(2,:)) 
        end
    end
    drawnow % Added update by Ricardo
    
end % end of main loop

if SWITCH_PROFILE, profile report, end

data = finalise_data(DATA);
set(h.pth, 'xdata', data.path(1,:), 'ydata', data.path(2,:))    

clear global DATA 
clear global XX 
clear global PX

% 
%
% All plots intialization in this function (modified by Ricardo)
function h= setup_animations(lm,wp)
% Setup plots
fig=figure;
h.rfeat=plot(lm(1,:),lm(2,:),'b*','DisplayName','real features'); % legend name (added by Ricardo)
hold on, axis equal
h.gt=plot(wp(1,:),wp(2,:), 'g','DisplayName','ground truth'); % legend name (added by Ricardo)
h.gtp=plot(wp(1,:),wp(2,:),'g.');
xlabel('metres'), ylabel('metres')
set(fig, 'name', 'EKF-SLAM Simulator')
h.xt= patch(0,0,'b'); % vehicle true
h.xt.FaceAlpha = 0.5;
h.xv= patch(0,0,'r'); % vehicle estimate
h.xv.FaceAlpha = 0.5;
% vehicle path estimate
% legend name (added by Ricardo)
h.pth= plot(0,0,'k-','markersize',2,'DisplayName','vehicle path estimate'); 
% vehicle odometry (added by Ricardo)
h.odom= plot(0,0,'r-','markersize',2,'DisplayName','vehicle odometry'); 
h.obs= plot(0,0,'y'); % observations
% estimated features
% legend name (added by Ricardo)
h.xf= plot(0,0,'r+','DisplayName','estimated features'); 
h.vcov= plot(0,0,'r'); % vehicle covariance ellipses
h.fcov= plot(0,0,'r'); % feature covariance ellipses
legend([h.rfeat h.gt h.pth h.odom h.xf]); % included lengend (added by Ricardo)


% function h= setup_animations()
% h.xt= patch(0,0,'b','erasemode','xor'); % vehicle true
% h.xv= patch(0,0,'r','erasemode','xor'); % vehicle estimate
% h.pth= plot(0,0,'k.','markersize',2,'erasemode','background'); % vehicle path estimate
% h.obs= plot(0,0,'y','erasemode','xor'); % observations
% h.xf= plot(0,0,'r+','erasemode','xor'); % estimated features
% h.vcov= plot(0,0,'r','erasemode','xor'); % vehicle covariance ellipses
% h.fcov= plot(0,0,'r','erasemode','xor'); % feature covariance ellipses

%
%

function p= make_laser_lines (rb,xv)
% compute set of line segments for laser range-bearing measurements
if isempty(rb), p=[]; return, end
len= size(rb,2);
lnes(1,:)= zeros(1,len)+ xv(1);
lnes(2,:)= zeros(1,len)+ xv(2);
lnes(3:4,:)= transformtoglobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
p= line_plot_conversion (lnes);

%
%

function p= make_vehicle_covariance_ellipse(x,P)
% compute ellipses for plotting vehicle covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];

p= make_ellipse(x(1:2), P(1:2,1:2), circ);

function p= make_feature_covariance_ellipses(x,P)
% compute ellipses for plotting feature covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];

lenx= length(x);
lenf= (lenx-3)/2;
p= zeros (2, lenf*(N+2));

ctr= 1;
for i=1:lenf
    ii= ctr:(ctr+N+1);
    jj= 2+2*i; jj= jj:jj+1;
    
    p(:,ii)= make_ellipse(x(jj), P(jj,jj), circ);
    ctr= ctr+N+2;
end

%
%

function p= make_ellipse(x,P,circ)
% make a single 2-D ellipse 
r= sqrtm_2by2(P);
a= r*circ;
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];

%
%

function data= initialise_store(x,odom,P, xtrue)
% offline storage initialisation
data.i=1;
data.path= x;
data.odom= odom;    % included odom (by Ricardo Vázquez)
data.true= xtrue;
data.state(1).x= x;
%data.state(1).P= P;
data.state(1).P= diag(P);

%
%

function store_data(x, odom, P, xtrue)
% add current data to offline storage
global DATA
CHUNK= 5000;
len= size(DATA.path,2);
if DATA.i == len % grow array exponentially to amortise reallocation
    if len < CHUNK, len= CHUNK; end
    DATA.path= [DATA.path zeros(3,len)];
    DATA.odom= [DATA.odom zeros(3,len)]; % included odom (by Ricardo Vázquez)
    DATA.true= [DATA.true zeros(3,len)];
    %pack
end
i= DATA.i + 1;
DATA.i= i;
DATA.path(:,i)= x(1:3);
DATA.odom(:,i)= odom(1:3);  % included odom (by Ricardo Vázquez)
DATA.true(:,i)= xtrue;
DATA.state(i).x= x;
%DATA.state(i).P= P;
DATA.state(i).P= diag(P);

%
%

function data = finalise_data(data)
% offline storage finalisation
data.path= data.path(:,1:data.i);
data.odom= data.odom(:,1:data.i);   % included odom (by Ricardo Vázquez)
data.true= data.true(:,1:data.i);
