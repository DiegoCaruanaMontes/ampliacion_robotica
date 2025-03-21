
% Modelo P2
clc
close

R = 0.1;
K = 0.4;

v = 0.3; % constante
dmin = 20; % Separación mínima entre puntos
T = 0.3; % Tiempo de muestreo GPS
T_telemetro = 0.5;
epsilon = 1; % Distancia de alcance al waypoint
Tm= 0.12; %Constante de tiempo
v_max = 15; %velocidad máxima
tsim = 0.1; %salto en la simulación
puntos_pasillo = [0 0;30 0;30 10;0 10;0 0];

timer = 0;

% Probar varios valores de ganancia (Controlador P)
G = 1;

% Inicializar variables
x = 0;
y = 0;
theta = 0;
d = 1000;
wi_ant = 0;
wd_ant = 0;
j_ant = 0;

for j=2:size(trayectoria,1) % Each point in the trayectory
    p = puntos_pasillo(1, :);
    d = norm(p - [x y])
    while d>epsilon
        % Calcular w con control proporcional
        vector = p-[x y]
        d = norm(vector);
        %if d>100
        %    break;
        %end
        theta_r = atan2(vector(2),vector(1));
        theta_g = mod(theta_r-theta,2*pi);
        w = G*theta_g;

        % MCI
        [wi,wd] = MCI2(v,w,K,R);

        % Simulate a step
        [dx,dy,dtheta] = step(wi, wi_ant, wd, wd_ant, tsim, Tm,theta, R, K, v_max);
        timer = timer + tsim;

        % Recalculate position
        x = x+dx; 
        y = y+dy;
        theta = theta+dtheta;

        % Crear variables intermedias x',y',z' para la pose real del robot
        % y la actualización del GPS
        wi_ant = wi;
        wd_ant = wd;

    end
end