% Modelo P1
R = 0;
K = 0;

v = 1.2; % constante
dmin = 20; % Separación mínima entre puntos
T = 0.3; % Tiempo de muestreo GPS
epsilon = 1; % Distancia de alcance al waypoint
Tm= 0.12; %Constante de tiempo
tsim = 1; %salto en la simulación
trayectoria = [0 0;20 0;20 20;-10 30;-20 -10;0 -30;0 0];

% Probar varios valores de ganancia (Controlador P)
G = [1 2 3];

% Inicializar variables
x = 0;
y = 0;
theta = 0;
d = 1000;

% Vectores para guardar la trayectoria recorrida
x_hist = [];
y_hist = [];
theta_hist = [];
w_hist = [];
v_hist = [];


for j=2:size(trayectoria,1) % Each point in the trayectory
    p = trayectoria(j,:);
    % Go to point until d<epsilon
    while d>epsilon
        % Calcular w con control proporcional
        vector = p-[x y];
        d = norm(vector);
        theta_r = atan2(vector(2),vector(1));
        theta_g = theta_r-theta;
        w = G*theta_g;

        % MCI
        [wi,wd] = MCI2(v,w,K,R);

        % Simulate a step
        [dx,dy,dtheta] = step(wi, wd, tsim, Tm, theta);

        % Recalculate position
        x = x+dx; 
        y = y+dy;
        theta = theta+dtheta;

        % Guardar datos para la gráfica
        x_hist = [x_hist, x];
        y_hist = [y_hist, y];
        theta_hist = [theta_hist, theta];
        w_hist = [w_hist, w];
        v_hist = [v_hist, v];

        % Add noise
        pos = DGPS(x,y,theta);
        x = pos(1);
        y = pos(2);
        theta = pos(3);
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