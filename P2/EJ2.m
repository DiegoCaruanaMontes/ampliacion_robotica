
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
G = 0.1;

% Inicializar variables

x_real = 5;
y_real = 5;
theta_real = 0;

contorno_x = [0 a a 0 0];
contorno_y = [0 0 b b 0];
d = 1000;
wi_ant = 0;
wd_ant = 0;

x = 1;
y = 0;
theta = 0;

delta_x = 1;
delta_y = 0;
trayectoria_x = [];
trayectoria_y = [];

p = puntos_pasillo(1, :);
while (x_real>=0 && x_real<a && y_real>=0 && y_real<(b))
    % Guardar posición en la trayectoria
    trayectoria_x = [trayectoria_x, x_real];
    trayectoria_y = [trayectoria_y, y_real];
    
    % Calcular w con control proporcional
    vector = [delta_x delta_y];
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
        d = dibujaBarrido(contorno_x, contorno_y, x_real, y_real, theta_real, rangos);
        % ...
        dist_mpm = (rangos(18)+rangos(54))/2; % distancia media paredes medida
        dist_mp = b/2; % distancia media de la pared
        rate = dist_mp/dist_mpm;
        if rate>1
            rate = 1;
        end
        theta = (acos(rate));
        if rangos(17) > rangos(18) %Está a la derecha
            theta = -theta;
        end
        
        dl = (cos(theta)*rangos(18))-(b/2);
        
        delta_x = dl*sin(theta)+d*cos(theta);
        delta_y = dl*cos(theta)-d*sin(theta);
    end
    if x<0
       x = 1;
    end
    wi_ant = wi;
    wd_ant = wd;


close
figure;
hold on;
plot(contorno_x, contorno_y, 'k', 'LineWidth', 3); % Pasillo
plot(trayectoria_x, trayectoria_y, 'b', 'LineWidth', 2); % Trayectoria del robot
scatter(trayectoria_x(1), trayectoria_y(1), 50, 'g', 'filled'); % Punto inicial
scatter(trayectoria_x(end), trayectoria_y(end), 50, 'r', 'filled'); % Punto final
xlabel('X (m)');
ylabel('Y (m)');
title('Trayectoria del Robot en el Pasillo');
grid on;
axis equal;
legend('Pasillo', 'Trayectoria', 'Inicio', 'Fin');
hold off;

end
% Graficar el contorno del pasillo y la trayectoria del robot
