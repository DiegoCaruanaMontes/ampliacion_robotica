
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
a = 30;
b = 10;
puntos_pasillo = [0 0;a 0;a b;0 b;0 0];

timer = 0;

% Probar varios valores de ganancia (Controlador P)
G = 1;

% Inicializar variables
x = 1;
y = 5;
theta = 0;
x_real = 1;
y_real = 5;
theta_real = 0;
contorno_x = [0 a a 0 0];
contorno_y = [0 0 b b 0];
d = 1000;
wi_ant = 0;
wd_ant = 0;

dx = 1;
dy = 0;

p = puntos_pasillo(1, :);
while true
    % Calcular w con control proporcional
    vector = [dx dy];
    d = norm(vector);

    theta_r = atan2(vector(2),vector(1));
    theta_g = mod(theta_r-theta,2*pi);
    w = G*theta_g;

    % MCI
    [wi,wd] = MCI2(v,w,K,R);

    % Simulate a step
    [dx,dy,dtheta] = step(wi, wi_ant, wd, wd_ant, tsim, Tm,theta_real, R, K, v_max);
    timer = timer + tsim;

    % Recalculate position
    x_real = x_real+dx; 
    y_real = y_real+dy;
    theta_real = theta_real+dtheta;

    % Crear variables intermedias x',y',z' para la pose real del robot
    if timer/T>1
        timer = mod(timer,T);
        rangos= laser2D(contorno_x, contorno_y, x_real, y_real, theta_real);

        dist_mpm = (rangos(18)+rangos(54))/2 % distancia media paredes medida
        dist_mp = b/2; % distancia media de la pared
        rate = dist_mp/dist_mpm;
        theta = acos(dist_mp/dist_mpm);
        if rangos(17) > rangos(18) %Está a la derecha
            theta = -theta;
        end
        
        dl = (cos(theta)*rangos(18))-(b/2);
        
        dx = dl*sin(theta)+d*cos(theta);
        dy = dl*cos(theta)-d*sin(theta);
    end

    wi_ant = wi;
    wd_ant = wd;
end