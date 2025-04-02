% Modelo P1
clc
close all

R = 0.1;
K = 0.4;

v = 1.2; % constante
%dmin = 20; % Separación mínima entre puntos
T = 0.3; % Tiempo de muestreo GPS
epsilon = 1; % Distancia de alcance al waypoint
Tm= 0.12; %Constante de tiempo
v_max = 15; %velocidad máxima
tsim = 0.1; %salto en la simulación
trayectoria = [0 0;20 0;20 20;-10 30;-20 -10;0 -30;0 0];

timer = 0;

% Probar varios valores de ganancia (Controlador P)
G = 1;

% Inicializar variables
x = 0;
y = 0;
theta = 0;

x_real = 0;
y_real = 0;
theta_real = 0;

d = 1000;
wi_ant = 0;
wd_ant = 0;
j_ant = 0;

% Vectores para guardar la trayectoria recorrida
x_hist = [];
y_hist = [];
theta_hist = [];
w_hist = [];
v_hist = [];


for j=2:size(trayectoria,1) % Each point in the trayectory
    p = trayectoria(j,:);
    d = norm(p - [x y]);
    while d>epsilon
        % Calcular w con control proporcional
        vector = p-[x y];% globales-> mejor locales
        d = norm(vector);
        if d>100
            break;
        end
        theta_r = atan2(vector(2),vector(1));
        theta_g = mod(theta_r-theta,2*pi);
        w = G*theta_g;

        % MCI
        [wi,wd] = MCI2(v,w,K,R);

        % Simulate a step
        [dx,dy,dtheta] = step(wi, wi_ant, wd, wd_ant, tsim, Tm,theta, R, K, v_max);
        timer = timer + tsim;

        % Recalculate position
        x_real = x_real+dx; 
        y_real = y_real+dy;
        theta_real = theta_real+dtheta;

        % Crear variables intermedias x',y',z' para la pose real del robot
        % y la actualización del GPS

        % Add noise
        if timer/T>1
            timer = mod(timer,T);
            pos = DGPS(x_real,y_real,theta_real);
            x = pos(1);
            y = pos(2);
            theta = pos(3);
        end
        
        wi_ant = wi;
        wd_ant = wd;

        % Guardar datos para la gráfica
        x_hist = [x_hist, x_real];
        y_hist = [y_hist, y_real];
        theta_hist = [theta_hist, theta_real];
        w_hist = [w_hist, w];
        v_hist = [v_hist, v];

        
        
    end
end

% ---- Graficar la trayectoria ----
figure;
plot(trayectoria(:,1), trayectoria(:,2), 'ro-', 'LineWidth', 2); % Trayectoria deseada
hold on;
plot(x_hist, y_hist, 'b-', 'LineWidth', 1.5); % Trayectoria seguida
legend('Trayectoria deseada', 'Trayectoria seguida');
xlabel('Posición en X (m)');
ylabel('Posición en Y (m)');
title('Trayectoria del Robot');
grid on;
axis equal;

% ---- Graficar velocidad angular en función del tiempo ----
figure;
plot(w_hist, 'r-', 'LineWidth', 1.5);
xlabel('Tiempo (iteraciones)');
ylabel('Velocidad angular w (rad/s)');
title('Evolución de la Velocidad Angular');
grid on;

% ---- Graficar velocidad lineal en función del tiempo ----
figure;
plot(v_hist, 'b-', 'LineWidth', 1.5);
xlabel('Tiempo (iteraciones)');
ylabel('Velocidad lineal v (m/s)');
title('Evolución de la Velocidad Lineal');
grid on;